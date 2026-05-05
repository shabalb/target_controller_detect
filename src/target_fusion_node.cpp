#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "target_controller_detect/msg/detection2_d.hpp"
#include "target_controller_detect/msg/target_state.hpp"

namespace {

struct CloudPoint {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float range = 0.0f;
};

float median(std::vector<float> &values) {
  if (values.empty()) {
    return 0.0f;
  }
  const size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  float med = values[mid];
  if (values.size() % 2 == 0) {
    const auto max_it = std::max_element(values.begin(), values.begin() + mid);
    med = 0.5f * (med + *max_it);
  }
  return med;
}

}  // namespace

class TargetFusionNode : public rclcpp::Node {
public:
  TargetFusionNode() : Node("target_fusion_node") {
    const auto detection_topic =
      declare_parameter<std::string>("detection_topic", "/target/detection2d");
    const auto point_cloud_topic =
      declare_parameter<std::string>("point_cloud_topic", "/oak/stereo/depth");
    state_topic_ = declare_parameter<std::string>("state_topic", "/target/state");
    image_width_ = declare_parameter<int>("image_width", 640);
    image_height_ = declare_parameter<int>("image_height", 480);
    camera_fov_ = declare_parameter<double>("camera_fov", 1.466);
    detection_hold_sec_ = declare_parameter<double>("detection_hold_sec", 0.6);

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    detection_sub_ = create_subscription<target_controller_detect::msg::Detection2D>(
      detection_topic, qos, std::bind(&TargetFusionNode::onDetection, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      point_cloud_topic, qos, std::bind(&TargetFusionNode::onPointCloud, this, std::placeholders::_1));
    state_pub_ = create_publisher<target_controller_detect::msg::TargetState>(state_topic_, 10);

    RCLCPP_INFO(get_logger(), "Detection topic: %s", detection_topic.c_str());
    RCLCPP_INFO(get_logger(), "Point cloud topic: %s", point_cloud_topic.c_str());
    RCLCPP_INFO(get_logger(), "State topic: %s", state_topic_.c_str());
  }

private:
  void onDetection(const target_controller_detect::msg::Detection2D::SharedPtr msg) {
    last_detection_ = *msg;
    last_detection_stamp_ = rclcpp::Time(msg->header.stamp);
  }

  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
    auto state = buildInvalidState(msg->header);

    const auto cloud_t = rclcpp::Time(msg->header.stamp);
    const double det_age = std::abs((cloud_t - last_detection_stamp_).seconds());
    const bool detection_fresh = last_detection_.found && det_age <= detection_hold_sec_;
    if (!detection_fresh) {
      state_pub_->publish(state);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fusion lost target: stale detection");
      return;
    }

    auto object_points = extractObjectPoints(*msg, last_detection_);
    if (object_points.empty()) {
      state_pub_->publish(state);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "Fusion lost target: no cloud points in ROI");
      return;
    }

    std::vector<float> depth_values;
    depth_values.reserve(object_points.size());
    float mean_x = 0.0f;
    float mean_y = 0.0f;
    for (const auto &point : object_points) {
      if (!std::isfinite(point.range) || point.range < 0.1f || point.range > 20.0f) {
        continue;
      }
      depth_values.push_back(point.range);
      mean_x += point.x;
      mean_y += point.y;
    }

    if (depth_values.empty()) {
      state_pub_->publish(state);
      return;
    }

    mean_x /= static_cast<float>(depth_values.size());
    mean_y /= static_cast<float>(depth_values.size());

    state.valid = true;
    state.lost = false;
    state.distance = median(depth_values);
    state.rel_x = mean_x;
    state.rel_y = mean_y;

    const float image_cx = 0.5f * static_cast<float>(image_width_);
    const float fx_pixels =
      static_cast<float>(image_width_) /
      (2.0f * std::tan(static_cast<float>(camera_fov_) / 2.0f));
    const float pixel_offset = last_detection_.center_x - image_cx;
    state.angle = std::atan2(pixel_offset, fx_pixels);

    state_pub_->publish(state);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "TargetState valid=%d dist=%.3f angle=%.3f samples=%zu",
      state.valid, state.distance, state.angle, depth_values.size());
  }

  target_controller_detect::msg::TargetState buildInvalidState(const std_msgs::msg::Header &header) const {
    target_controller_detect::msg::TargetState state;
    state.header = header;
    state.valid = false;
    state.lost = true;
    return state;
  }

  std::vector<CloudPoint> extractObjectPoints(
    const sensor_msgs::msg::PointCloud2 &msg,
    const target_controller_detect::msg::Detection2D &detection) const {
    std::vector<CloudPoint> points;

    const int src_w = std::max(1, image_width_);
    const int src_h = std::max(1, image_height_);
    const int cloud_w = static_cast<int>(msg.width);
    const int cloud_h = static_cast<int>(msg.height);
    const bool organized = cloud_w > 0 && cloud_h > 1;

    cv::Rect roi;
    if (organized) {
      const float sx = static_cast<float>(cloud_w) / static_cast<float>(src_w);
      const float sy = static_cast<float>(cloud_h) / static_cast<float>(src_h);

      const float shrink = 0.2f;
      const float inner_x = detection.x + detection.width * shrink;
      const float inner_y = detection.y + detection.height * shrink;
      const float inner_w = std::max(1.0f, detection.width * (1.0f - 2.0f * shrink));
      const float inner_h = std::max(1.0f, detection.height * (1.0f - 2.0f * shrink));

      roi.x = std::clamp(static_cast<int>(inner_x * sx), 0, cloud_w - 1);
      roi.y = std::clamp(static_cast<int>(inner_y * sy), 0, cloud_h - 1);
      roi.width = std::clamp(static_cast<int>(inner_w * sx), 1, cloud_w - roi.x);
      roi.height = std::clamp(static_cast<int>(inner_h * sy), 1, cloud_h - roi.y);
    }

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
      size_t idx = 0;
      points.reserve(5000);

      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        if (organized) {
          const int u = static_cast<int>(idx % static_cast<size_t>(cloud_w));
          const int v = static_cast<int>(idx / static_cast<size_t>(cloud_w));
          if (u < roi.x || u >= roi.x + roi.width || v < roi.y || v >= roi.y + roi.height) {
            continue;
          }
        }

        CloudPoint p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.range = std::sqrt(x * x + y * y + z * z);
        if (!std::isfinite(p.range) || p.range <= 0.05f || p.range > 20.0f) {
          continue;
        }
        points.push_back(p);
      }
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(get_logger(), "PointCloud2 parse error: %s", e.what());
    }

    return points;
  }

  std::string state_topic_;
  int image_width_{640};
  int image_height_{480};
  double camera_fov_{1.466};
  double detection_hold_sec_{0.6};

  target_controller_detect::msg::Detection2D last_detection_;
  rclcpp::Time last_detection_stamp_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<target_controller_detect::msg::Detection2D>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<target_controller_detect::msg::TargetState>::SharedPtr state_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetFusionNode>());
  rclcpp::shutdown();
  return 0;
}
