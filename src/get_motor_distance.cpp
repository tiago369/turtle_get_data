#include <memory>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class GetMotorDistance : public rclcpp::Node
{
  public:
    GetMotorDistance()
    : Node("get_motor_distance")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&GetMotorDistance::velocity_callback, this, std::placeholders::_1));

    }

  private:
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
      sum_x_ += msg->linear.x;
      sum_y_ += msg->linear.y;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    float sum_x_ = 0.0;
    float sum_y_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetMotorDistance>());
  rclcpp::shutdown();
  return 0;
}