#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class DriveRobot : public rclcpp::Node {
public:
  DriveRobot() : Node("drive_robot"), t_(0.0) {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(50ms, std::bind(&DriveRobot::tick, this));
  }

private:
  void tick() {
    geometry_msgs::msg::Twist msg;
    
    msg.linear.x = 0.05;
    msg.angular.z = 0.05 * std::sin(t_);
    t_ += 0.05;
    pub_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double t_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveRobot>());
  rclcpp::shutdown();
  return 0;
}