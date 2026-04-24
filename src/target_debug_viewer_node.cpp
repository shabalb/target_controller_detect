#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "target_controller_detect/detector_utils.hpp"
#include "target_controller_detect/msg/detection2_d.hpp"
#include "target_controller_detect/msg/target_state.hpp"

class TargetDebugViewerNode : public rclcpp::Node {
public:
  TargetDebugViewerNode() : Node("target_debug_viewer_node") {
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/oak/rgb/image_raw");
    const auto depth_topic = declare_parameter<std::string>("depth_topic", "/oak/stereo/depth");
    const auto detection_topic =
      declare_parameter<std::string>("detection_topic", "/target/detection2d");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/target/state");

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, qos, std::bind(&TargetDebugViewerNode::onImage, this, std::placeholders::_1));
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, qos, std::bind(&TargetDebugViewerNode::onDepth, this, std::placeholders::_1));
    detection_sub_ = create_subscription<target_controller_detect::msg::Detection2D>(
      detection_topic, 10,
      std::bind(&TargetDebugViewerNode::onDetection, this, std::placeholders::_1));
    state_sub_ = create_subscription<target_controller_detect::msg::TargetState>(
      state_topic, 10, std::bind(&TargetDebugViewerNode::onState, this, std::placeholders::_1));

    cv::namedWindow("debug_rgb", cv::WINDOW_NORMAL);
    cv::namedWindow("debug_depth", cv::WINDOW_NORMAL);

    RCLCPP_INFO(get_logger(), "Debug RGB topic: %s", camera_topic.c_str());
    RCLCPP_INFO(get_logger(), "Debug depth topic: %s", depth_topic.c_str());
  }

  ~TargetDebugViewerNode() override {
    cv::destroyWindow("debug_rgb");
    cv::destroyWindow("debug_depth");
  }

private:
  void onDetection(const target_controller_detect::msg::Detection2D::SharedPtr msg) {
    last_detection_ = *msg;
    have_detection_ = true;
  }

  void onState(const target_controller_detect::msg::TargetState::SharedPtr msg) {
    last_state_ = *msg;
    have_state_ = true;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      if (msg->encoding == sensor_msgs::image_encodings::RGB8 || msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
      } else if (
        msg->encoding == sensor_msgs::image_encodings::BGR8 || msg->encoding == "bgr8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } else if (
        msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "mono8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
      } else {
        cv_ptr = cv_bridge::toCvShare(msg);
      }

      cv::Mat bgr = target_controller_detect::ensureBgrImage(cv_ptr->image, msg->encoding);
      if (have_detection_ && last_detection_.found) {
        const cv::Rect rect(
          last_detection_.x, last_detection_.y, last_detection_.width, last_detection_.height);
        cv::rectangle(bgr, rect, cv::Scalar(0, 255, 0), 2);
        cv::circle(
          bgr, cv::Point2f(last_detection_.center_x, last_detection_.center_y), 4,
          cv::Scalar(255, 255, 255), -1);
      }

      const std::string state_text = have_state_ && last_state_.valid
        ? "valid dist=" + formatFloat(last_state_.distance) + " angle=" + formatFloat(last_state_.angle)
        : "lost";
      cv::putText(
        bgr, state_text, cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX, 0.7,
        cv::Scalar(0, 255, 255), 2);
      cv::imshow("debug_rgb", bgr);
      cv::waitKey(1);

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "Debug image %ux%u encoding=%s detection=%d state=%d",
        msg->width, msg->height, msg->encoding.c_str(), have_detection_ && last_detection_.found,
        have_state_ && last_state_.valid);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "RGB debug error: %s", e.what());
    }
  }

  void onDepth(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      cv::Mat depth_m;
      if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 || msg->encoding == "32FC1") {
        depth_m = cv_ptr->image;
      } else if (
        msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 || msg->encoding == "16UC1")
      {
        cv_ptr->image.convertTo(depth_m, CV_32FC1, 0.001);
      } else {
        cv_ptr->image.convertTo(depth_m, CV_32FC1);
      }

      cv::Mat finite_mask = (depth_m > 0.05f) & (depth_m < 20.0f);
      double min_v = 0.0;
      double max_v = 0.0;
      cv::minMaxLoc(depth_m, &min_v, &max_v, nullptr, nullptr, finite_mask);
      if (!std::isfinite(min_v) || !std::isfinite(max_v) || max_v <= min_v) {
        min_v = 0.0;
        max_v = 5.0;
      }

      cv::Mat normalized(depth_m.size(), CV_8UC1, cv::Scalar(0));
      depth_m.convertTo(normalized, CV_8UC1, 255.0 / (max_v - min_v), -min_v * 255.0 / (max_v - min_v));
      normalized.setTo(0, ~finite_mask);

      cv::Mat depth_color;
      cv::applyColorMap(normalized, depth_color, cv::COLORMAP_TURBO);
      cv::putText(
        depth_color, "depth " + formatFloat(static_cast<float>(min_v)) + ".." +
                       formatFloat(static_cast<float>(max_v)) + " m",
        cv::Point(10, 26), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
      cv::imshow("debug_depth", depth_color);
      cv::waitKey(1);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Depth debug error: %s", e.what());
    }
  }

  std::string formatFloat(float value) const {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return std::string(buffer);
  }

  bool have_detection_{false};
  bool have_state_{false};
  target_controller_detect::msg::Detection2D last_detection_;
  target_controller_detect::msg::TargetState last_state_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<target_controller_detect::msg::Detection2D>::SharedPtr detection_sub_;
  rclcpp::Subscription<target_controller_detect::msg::TargetState>::SharedPtr state_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetDebugViewerNode>());
  rclcpp::shutdown();
  return 0;
}
