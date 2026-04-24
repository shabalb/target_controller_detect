#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "target_controller_detect/controller_utils.hpp"
#include "target_controller_detect/msg/target_state.hpp"

class TargetControllerNode : public rclcpp::Node {
public:
  TargetControllerNode() : Node("target_controller_node") {
    const auto state_topic = declare_parameter<std::string>("state_topic", "/target/state");
    const auto cmd_topic = declare_parameter<std::string>("cmd_topic", "/cmd_vel");

    target_controller_detect::ControllerConfig config;
    config.kd = declare_parameter<double>("kd", config.kd);
    config.ka = declare_parameter<double>("ka", config.ka);
    config.max_linear = declare_parameter<double>("max_linear", config.max_linear);
    config.max_angular = declare_parameter<double>("max_angular", config.max_angular);
    config.desired_distance =
      declare_parameter<double>("desired_distance", config.desired_distance);
    config.dist_deadband = declare_parameter<double>("dist_deadband", config.dist_deadband);
    config.angle_align_deadband =
      declare_parameter<double>("angle_align_deadband", config.angle_align_deadband);
    config.angle_follow_deadband =
      declare_parameter<double>("angle_follow_deadband", config.angle_follow_deadband);
    config.lost_search_angular =
      declare_parameter<double>("lost_search_angular", config.lost_search_angular);
    config_ = config;

    state_sub_ = create_subscription<target_controller_detect::msg::TargetState>(
      state_topic, 10, std::bind(&TargetControllerNode::onState, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

    RCLCPP_INFO(get_logger(), "State topic: %s", state_topic.c_str());
    RCLCPP_INFO(get_logger(), "Cmd topic: %s", cmd_topic.c_str());
  }

private:
  void onState(const target_controller_detect::msg::TargetState::SharedPtr msg) {
    const auto mode = target_controller_detect::decideMode(*msg, config_);
    const auto cmd = target_controller_detect::computeCommand(*msg, mode, config_);

    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.linear;
    twist.angular.z = cmd.angular;
    cmd_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 500, "Control valid=%d dist=%.3f angle=%.3f cmd=(%.3f, %.3f)",
      msg->valid, msg->distance, msg->angle, twist.linear.x, twist.angular.z);
  }

  target_controller_detect::ControllerConfig config_;
  rclcpp::Subscription<target_controller_detect::msg::TargetState>::SharedPtr state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetControllerNode>());
  rclcpp::shutdown();
  return 0;
}
