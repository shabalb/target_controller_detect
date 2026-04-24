#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdint>


#include <rclcpp/rclcpp.hpp>

#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/boolean.pb.h>


using namespace std::chrono_literals;

class MoveTargetNode : public rclcpp::Node
{
public:
  MoveTargetNode()
  : Node("move_target"),
    t_(0.0)
  {
    
    this->declare_parameter<std::string>("world", "default");
    this->declare_parameter<std::string>("model", "target_cube");
    this->declare_parameter<double>("radius", 4);
    this->declare_parameter<double>("omega", 0.6);     // рад/с
    this->declare_parameter<double>("z", 0.2);
    this->declare_parameter<int>("rate_hz", 5);

    auto rate_hz = this->get_parameter("rate_hz").as_int();
    auto hz = std::max<int64_t>(1, rate_hz);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / hz),
        std::bind(&MoveTargetNode::tick, this)
    );

    RCLCPP_INFO(this->get_logger(), "move_target started");
  }

private:
  void tick()
  {
    const auto world = this->get_parameter("world").as_string();
    const auto model = this->get_parameter("model").as_string();
    const auto radius = this->get_parameter("radius").as_double();
    const auto omega  = this->get_parameter("omega").as_double();
    const auto z      = this->get_parameter("z").as_double();

    
    const double x = radius * std::cos(t_);
    const double y = radius * std::sin(t_);
    t_ += omega * dt_;

    // Сервис Gazebo
    const std::string srv = "/world/" + world + "/set_pose";

    gz::msgs::Pose req;
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);

    
    req.set_name(model);

    // Можно задать ориентацию (кватернион). Здесь оставим (0,0,0,1)
    req.mutable_orientation()->set_w(1.0);
    req.mutable_orientation()->set_x(0.0);
    req.mutable_orientation()->set_y(0.0);
    req.mutable_orientation()->set_z(0.0);

    gz::msgs::Boolean rep;
    bool result = false;

    
    bool sent = gz_node_.Request(srv, req, 100, rep, result);

    if (!sent || !result || !rep.data())
    {
      
      static int c = 0;
      if ((c++ % 30) == 0)
      {
        RCLCPP_WARN(this->get_logger(),
          "Failed set_pose (sent=%d result=%d rep=%d) service=%s name=%s",
          (int)sent, (int)result, (int)rep.data(), srv.c_str(), model.c_str());
      }
    }
  }

  gz::transport::Node gz_node_;
  rclcpp::TimerBase::SharedPtr timer_;

  double t_;
  const double dt_ = 1.0 / 60.0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveTargetNode>());
  rclcpp::shutdown();
  return 0;
}
