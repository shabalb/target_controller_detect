#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class MotionValidator : public rclcpp::Node {
public:
  MotionValidator() : Node("motion_validator") {
    declare_parameter<double>("min_stop_distance", 0.90);
    declare_parameter<double>("slowdown_distance", 1.30);
    declare_parameter<double>("front_sector_half_angle", 0.35);
    declare_parameter<double>("rear_sector_half_angle", 0.35);
    declare_parameter<double>("turn_sector_half_angle", 0.45);
    declare_parameter<double>("turn_sector_center_angle", 1.57);
    declare_parameter<double>("scan_timeout_sec", 1.50);
    declare_parameter<double>("lidar_angle_shift", 0.0);

    const auto scan_qos = rclcpp::SensorDataQoS();

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", scan_qos,
        std::bind(&MotionValidator::onScan, this, std::placeholders::_1));

    raw_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel_raw", 10,
        std::bind(&MotionValidator::onRawCommand, this, std::placeholders::_1));

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(get_logger(),
                "motion_validator started: cmd_vel_raw -> /cmd_vel");
  }

private:
  struct SectorDistances {
    double front = std::numeric_limits<double>::infinity();
    double rear = std::numeric_limits<double>::infinity();
    double left_turn = std::numeric_limits<double>::infinity();
    double right_turn = std::numeric_limits<double>::infinity();
  };

  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    last_scan_stamp_ = rclcpp::Time(msg->header.stamp);
    last_sector_distances_ = computeSectorDistances(*msg);
  }

  void onRawCommand(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    geometry_msgs::msg::Twist safe_cmd = *msg;

    const double scan_timeout_sec = get_parameter("scan_timeout_sec").as_double();
    const double min_stop_distance =
        get_parameter("min_stop_distance").as_double();
    const double slowdown_distance =
        get_parameter("slowdown_distance").as_double();

    const bool scan_fresh = true;
        //last_scan_stamp_.nanoseconds() != 0 &&
        //std::abs((now() - last_scan_stamp_).seconds()) <= scan_timeout_sec;

    if (safe_cmd.linear.x > 0.0) {
      if (!scan_fresh ) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No fresh lidar scan, blocking forward motion");
      } else if (last_sector_distances_.front <= min_stop_distance) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Obstacle too close: front distance %.3f m, stopping",
            last_sector_distances_.front);
      } else if (last_sector_distances_.front < slowdown_distance) {
        const double ratio =
            (last_sector_distances_.front - min_stop_distance) /
            std::max(1e-6, slowdown_distance - min_stop_distance);
        safe_cmd.linear.x *= std::clamp(ratio, 0.0, 1.0);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Reducing forward speed, front distance %.3f m",
            last_sector_distances_.front);
      }
    }

    if (safe_cmd.linear.x < 0.0) {
      if (!scan_fresh ) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No fresh lidar scan, blocking reverse motion");
      } else if (last_sector_distances_.rear <= min_stop_distance) {
        safe_cmd.linear.x = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Obstacle too close: rear distance %.3f m, stopping reverse",
            last_sector_distances_.rear);
      } else if (last_sector_distances_.rear < slowdown_distance) {
        const double ratio =
            (last_sector_distances_.rear - min_stop_distance) /
            std::max(1e-6, slowdown_distance - min_stop_distance);
        safe_cmd.linear.x *= std::clamp(ratio, 0.0, 1.0);
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Reducing reverse speed, rear distance %.3f m",
            last_sector_distances_.rear);
      }
    }

    if (safe_cmd.angular.z > 0.0) {
      if (!scan_fresh ) {
        safe_cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No fresh lidar scan, blocking left turn");
      } else if (last_sector_distances_.left_turn <= min_stop_distance) {
        safe_cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Obstacle too close on the left: %.3f m, blocking turn",
            last_sector_distances_.left_turn);
      }
    } else if (safe_cmd.angular.z < 0.0) {
      if (!scan_fresh ) {
        safe_cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "No fresh lidar scan, blocking right turn");
      } else if (last_sector_distances_.right_turn <= min_stop_distance) {
        safe_cmd.angular.z = 0.0;
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 500,
            "Obstacle too close on the right: %.3f m, blocking turn",
            last_sector_distances_.right_turn);
      }
    }

    safe_cmd_pub_->publish(safe_cmd);
  }

  SectorDistances computeSectorDistances(
      const sensor_msgs::msg::LaserScan &scan) const {
    const double front_half_angle =
        get_parameter("front_sector_half_angle").as_double();
    const double rear_half_angle =
        get_parameter("rear_sector_half_angle").as_double();
    const double turn_half_angle =
        get_parameter("turn_sector_half_angle").as_double();
    const double turn_center_angle =
        get_parameter("turn_sector_center_angle").as_double();
    const double angle_shift = get_parameter("lidar_angle_shift").as_double();

    SectorDistances distances;
    double angle = scan.angle_min + angle_shift;

    for (float range : scan.ranges) {
      if (std::isfinite(range) && range >= scan.range_min &&
          range <= scan.range_max) {
        const double distance = static_cast<double>(range);

        if (std::abs(angle) <= front_half_angle) {
          distances.front = std::min(distances.front, distance);
        }

        if (std::abs(normalizeAngle(angle - M_PI)) <= rear_half_angle) {
          distances.rear = std::min(distances.rear, distance);
        }

        if (std::abs(normalizeAngle(angle - turn_center_angle)) <=
            turn_half_angle) {
          distances.left_turn = std::min(distances.left_turn, distance);
        }

        if (std::abs(normalizeAngle(angle + turn_center_angle)) <=
            turn_half_angle) {
          distances.right_turn = std::min(distances.right_turn, distance);
        }
      }
      angle += scan.angle_increment;
    }

    return distances;
  }

  static double normalizeAngle(double angle) {
    while (angle > M_PI) {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr raw_cmd_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;

  rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
  SectorDistances last_sector_distances_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionValidator>());
  rclcpp::shutdown();
  return 0;
}
