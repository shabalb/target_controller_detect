#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "target_controller_detect/msg/detection2_d.hpp"
#include "target_controller_detect/msg/target_state.hpp"
#include "target_controller_detect/shm_image_ring.hpp"

namespace {

float median(std::vector<float> &values) {
  if (values.empty()) {
    return 0.0f;
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  float med = values[mid];
  if (values.size() % 2 == 0) {
    const auto max_it = std::max_element(values.begin(), values.begin() + mid);
    med = 0.5f * (med + *max_it);
  }
  return med;
}

}  // namespace

class ShmDepthFusionNode : public rclcpp::Node {
public:
  ShmDepthFusionNode() : Node("shm_depth_fusion_node") {
    shm_name_ = declare_parameter<std::string>("shm_name", "oak_depth");
    const auto detection_topic =
      declare_parameter<std::string>("detection_topic", "/target/detection2d");
    state_topic_ = declare_parameter<std::string>("state_topic", "/target/state");
    oper_topic_ = declare_parameter<std::string>("oper_topic", "/target/follow");
    image_width_ = declare_parameter<int>("image_width", 1920);
    image_height_ = declare_parameter<int>("image_height", 1080);
    camera_fov_ = declare_parameter<double>("camera_fov", 1.466);
    detection_hold_sec_ = declare_parameter<double>("detection_hold_sec", 0.6);
    process_fps_ = declare_parameter<double>("process_fps", 30.0);


    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    oper_sub_ = create_subscription<std_msgs::msg::String>(
      oper_topic_, qos, std::bind(&ShmDepthFusionNode::onOper, this, std::placeholders::_1));

    detection_sub_ = create_subscription<target_controller_detect::msg::Detection2D>(
      detection_topic, qos,
      std::bind(&ShmDepthFusionNode::onDetection, this, std::placeholders::_1));
    state_pub_ = create_publisher<target_controller_detect::msg::TargetState>(state_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, process_fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ShmDepthFusionNode::onTimer, this));

    RCLCPP_INFO(get_logger(), "Depth SHM: %s", shm_name_.c_str());
    RCLCPP_INFO(get_logger(), "Detection topic: %s", detection_topic.c_str());
    RCLCPP_INFO(get_logger(), "State topic: %s", state_topic_.c_str());
  }

private:
  bool ensureReader() {
    if (reader_) {
      return true;
    }

    try {
      reader_ = std::make_unique<target_controller_detect::ShmImageRingReader>(shm_name_);
      RCLCPP_INFO(get_logger(), "Opened depth shared memory: %s", shm_name_.c_str());
      return true;
    } catch (const std::exception &exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for depth SHM '%s': %s",
        shm_name_.c_str(), exc.what());
      return false;
    }
  }

  void onDetection(const target_controller_detect::msg::Detection2D::SharedPtr msg) {
    last_detection_ = *msg;
    last_detection_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void onOper(const std_msgs::msg::String::ConstPtr& msg){
    if(std::strcmp(msg->data.c_str(),"switch control mode to follow")==0){
      is_oper_follow_mode = true;
    }else{
      is_oper_follow_mode = false;
    }
  }

  void onTimer() {
    if (!is_oper_follow_mode){
      return;
    }
    if (!ensureReader()) {
      return;
    }

    target_controller_detect::ShmImageView view;
    if (!reader_->latest(view) || view.seq == last_seq_) {
      return;
    }
    last_seq_ = view.seq;

    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = "oak_depth_camera_optical_frame";
    auto state = buildInvalidState(header);

    const double det_age = std::abs((now() - last_detection_stamp_).seconds());
    const bool detection_fresh = last_detection_.found && det_age <= detection_hold_sec_;
    if (!detection_fresh) {
      state_pub_->publish(state);
      return;
    }

    if (view.encoding != "16UC1" || view.step < view.width * 2U) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Unsupported depth SHM format: %s step=%u",
        view.encoding.c_str(), view.step);
      state_pub_->publish(state);
      return;
    }

    std::vector<float> depth_values;
    extractDepthValues(view, last_detection_, depth_values);
    if (depth_values.empty()) {
      state_pub_->publish(state);
      return;
    }

    const float distance = median(depth_values);
    const float image_cx = 0.5f * static_cast<float>(image_width_);
    const float fx_pixels =
      static_cast<float>(image_width_) /
      (2.0f * std::tan(static_cast<float>(camera_fov_) / 2.0f));
    const float pixel_offset = last_detection_.center_x - image_cx;
    const float angle = std::atan2(pixel_offset, fx_pixels);

    state.valid = true;
    state.lost = false;
    state.distance = distance;
    state.angle = angle;
    state.rel_x = distance * std::sin(angle);
    state.rel_y = distance * std::cos(angle);
    state_pub_->publish(state);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "SHM TargetState valid=%d dist=%.3f angle=%.3f samples=%zu",
      state.valid, state.distance, state.angle, depth_values.size());
  }

  target_controller_detect::msg::TargetState buildInvalidState(
    const std_msgs::msg::Header &header) const {
    target_controller_detect::msg::TargetState state;
    state.header = header;
    state.valid = false;
    state.lost = true;
    return state;
  }

  void extractDepthValues(
    const target_controller_detect::ShmImageView &view,
    const target_controller_detect::msg::Detection2D &detection,
    std::vector<float> &depth_values) const {
    const int depth_w = static_cast<int>(view.width);
    const int depth_h = static_cast<int>(view.height);
    if (depth_w <= 0 || depth_h <= 0) {
      return;
    }

    const float sx = static_cast<float>(depth_w) / static_cast<float>(std::max(1, image_width_));
    const float sy = static_cast<float>(depth_h) / static_cast<float>(std::max(1, image_height_));
    const float shrink = 0.2f;
    const float inner_x = detection.x + detection.width * shrink;
    const float inner_y = detection.y + detection.height * shrink;
    const float inner_w = std::max(1.0f, detection.width * (1.0f - 2.0f * shrink));
    const float inner_h = std::max(1.0f, detection.height * (1.0f - 2.0f * shrink));

    const int x0 = std::clamp(static_cast<int>(inner_x * sx), 0, depth_w - 1);
    const int y0 = std::clamp(static_cast<int>(inner_y * sy), 0, depth_h - 1);
    const int roi_w = std::clamp(static_cast<int>(inner_w * sx), 1, depth_w - x0);
    const int roi_h = std::clamp(static_cast<int>(inner_h * sy), 1, depth_h - y0);

    depth_values.reserve(static_cast<std::size_t>(roi_w * roi_h));
    for (int y = y0; y < y0 + roi_h; ++y) {
      const auto *row = reinterpret_cast<const std::uint16_t *>(view.data + y * view.step);
      for (int x = x0; x < x0 + roi_w; ++x) {
        const std::uint16_t depth_mm = row[x];
        if (depth_mm < 100 || depth_mm > 20000) {
          continue;
        }
        depth_values.push_back(static_cast<float>(depth_mm) * 0.001f);
      }
    }
  }

  std::string shm_name_;
  std::string state_topic_;
  int image_width_{1920};
  int image_height_{1080};
  double camera_fov_{1.466};
  double detection_hold_sec_{0.6};
  double process_fps_{30.0};
  std::uint64_t last_seq_{0};

  target_controller_detect::msg::Detection2D last_detection_;
  rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

  std::unique_ptr<target_controller_detect::ShmImageRingReader> reader_;
  rclcpp::Subscription<target_controller_detect::msg::Detection2D>::SharedPtr detection_sub_;
  rclcpp::Publisher<target_controller_detect::msg::TargetState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr oper_sub_;
  std::string oper_topic_;
  bool is_oper_follow_mode{false};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShmDepthFusionNode>());
  rclcpp::shutdown();
  return 0;
}
