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

namespace {
constexpr bool kShowWindows = true;
}

class ColorDetectorNode : public rclcpp::Node {
public:
  ColorDetectorNode() : Node("color_detector_node") {
    const auto camera_topic = declare_parameter<std::string>("camera_topic", "/camera/image");
    detection_topic_ = declare_parameter<std::string>("detection_topic", "/target/detection2d");

    rclcpp::QoS qos(rclcpp::KeepLast(20));
    qos.best_effort();

    detection_pub_ = create_publisher<target_controller_detect::msg::Detection2D>(
      detection_topic_, 10);
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      camera_topic, qos, std::bind(&ColorDetectorNode::onImage, this, std::placeholders::_1));

    if (kShowWindows) {
      cv::namedWindow("detector_camera", cv::WINDOW_NORMAL);
    }

    RCLCPP_INFO(get_logger(), "Camera topic: %s", camera_topic.c_str());
    RCLCPP_INFO(get_logger(), "Detection topic: %s", detection_topic_.c_str());
  }

  ~ColorDetectorNode() override {
    if (kShowWindows) {
      cv::destroyWindow("detector_camera");
    }
  }

private:
  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    target_controller_detect::msg::Detection2D empty_detection;
    empty_detection.header = msg->header;

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
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      }

      cv::Mat bgr = target_controller_detect::ensureBgrImage(cv_ptr->image, msg->encoding);
      auto detection = target_controller_detect::detectRedTarget(bgr, msg->header);
      detection_pub_->publish(detection);

      if (kShowWindows) {
        if (detection.found) {
          cv::rectangle(
            bgr, cv::Rect(detection.x, detection.y, detection.width, detection.height),
            cv::Scalar(0, 255, 0), 2);
          cv::circle(
            bgr, cv::Point2f(detection.center_x, detection.center_y), 3,
            cv::Scalar(255, 255, 255), -1);
          const std::string text =
            "cell=(" + std::to_string(detection.cell_x) + "," + std::to_string(detection.cell_y) + ")";
          cv::putText(
            bgr, text, cv::Point(detection.x, std::max(0, detection.y - 8)),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }

        cv::putText(
          bgr, "detector", cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.7,
          cv::Scalar(0, 255, 255), 2);
        cv::imshow("detector_camera", bgr);
        cv::waitKey(1);
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000, "Detection found=%d center=(%.1f, %.1f)",
        detection.found, detection.center_x, detection.center_y);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      detection_pub_->publish(empty_detection);
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
      detection_pub_->publish(empty_detection);
    }
  }

  std::string detection_topic_;
  rclcpp::Publisher<target_controller_detect::msg::Detection2D>::SharedPtr detection_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
