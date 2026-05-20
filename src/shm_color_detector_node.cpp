#include <chrono>
#include <memory>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>

#include "target_controller_detect/detector_utils.hpp"
#include "target_controller_detect/msg/detection2_d.hpp"
#include "target_controller_detect/shm_image_ring.hpp"

using namespace std::chrono_literals;

class ShmColorDetectorNode : public rclcpp::Node {
public:
  ShmColorDetectorNode() : Node("shm_color_detector_node") {
    shm_name_ = declare_parameter<std::string>("shm_name", "oak_rgb");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/target/detection2d");
    oper_topic_ = declare_parameter<std::string>("oper_topic", "/target/follow");
    process_fps_ = declare_parameter<double>("process_fps", 15.0);
    show_windows_ = declare_parameter<bool>("show_windows", false);

    detection_pub_ = create_publisher<target_controller_detect::msg::Detection2D>(
      detection_topic_, 10);

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    oper_sub_ = create_subscription<std_msgs::msg::String>(
      oper_topic_, qos, std::bind(&ShmColorDetectorNode::onOper, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, process_fps_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ShmColorDetectorNode::onTimer, this));

    if (show_windows_) {
      cv::namedWindow("shm_detector_camera", cv::WINDOW_NORMAL);
    }

    RCLCPP_INFO(get_logger(), "RGB SHM: %s", shm_name_.c_str());
    RCLCPP_INFO(get_logger(), "Detection topic: %s", detection_topic_.c_str());
  }

  ~ShmColorDetectorNode() override {
    if (show_windows_) {
      cv::destroyWindow("shm_detector_camera");
    }
  }

private:
  bool ensureReader() {
    if (reader_) {
      return true;
    }

    try {
      reader_ = std::make_unique<target_controller_detect::ShmImageRingReader>(shm_name_);
      RCLCPP_INFO(get_logger(), "Opened RGB shared memory: %s", shm_name_.c_str());
      return true;
    } catch (const std::exception &exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for RGB SHM '%s': %s",
        shm_name_.c_str(), exc.what());
      return false;
    }
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
    header.frame_id = "oak_rgb_camera_optical_frame";

    target_controller_detect::msg::Detection2D detection;
    detection.header = header;
    detection.cell_x = -1;
    detection.cell_y = -1;

    try {
      cv::Mat bgr;
      if (view.encoding == "nv12") {
        cv::Mat nv12(
          static_cast<int>(view.height * 3 / 2), static_cast<int>(view.width), CV_8UC1,
          const_cast<std::uint8_t *>(view.data));
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
      } else if (view.encoding == "bgr8") {
        cv::Mat src(
          static_cast<int>(view.height), static_cast<int>(view.width), CV_8UC3,
          const_cast<std::uint8_t *>(view.data), view.step);
        bgr = src.clone();
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Unsupported RGB SHM encoding: %s",
          view.encoding.c_str());
        detection_pub_->publish(detection);
        return;
      }

      detection = target_controller_detect::detectRedTarget(bgr, header);
      detection_pub_->publish(detection);

      if (show_windows_) {
        if (detection.found) {
          cv::rectangle(
            bgr, cv::Rect(detection.x, detection.y, detection.width, detection.height),
            cv::Scalar(0, 255, 0), 2);
          cv::circle(
            bgr, cv::Point2f(detection.center_x, detection.center_y), 3,
            cv::Scalar(255, 255, 255), -1);
        }
        cv::imshow("shm_detector_camera", bgr);
        cv::waitKey(1);
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "SHM detection found=%d center=(%.1f, %.1f)",
        detection.found, detection.center_x, detection.center_y);
    } catch (const cv::Exception &exc) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", exc.what());
      detection_pub_->publish(detection);
    }
  }

  bool is_oper_follow_mode{false};
  std::string shm_name_;
  std::string detection_topic_;
  std::string oper_topic_;
  double process_fps_{15.0};
  bool show_windows_{false};
  std::uint64_t last_seq_{0};
  std::unique_ptr<target_controller_detect::ShmImageRingReader> reader_;
  rclcpp::Publisher<target_controller_detect::msg::Detection2D>::SharedPtr detection_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr oper_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShmColorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
