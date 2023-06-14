#include <memory>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class GetMotorEffort : public rclcpp::Node
{
  public:
    GetMotorEffort()
    : Node("get_motor_effort")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_state", 10, std::bind(&GetMotorEffort::velocity_callback, this, std::placeholders::_1));

    }

  private:
    void velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
      for(int i=0; i < msg->velocity.size(); i++){
        sum_velocities_.insert(sum_velocities_.begin() + i, 
          abs(msg->velocity.at(i)) + sum_velocities_.at(i));
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::vector<float> sum_velocities_ = {0.0f, 0.0f};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetMotorEffort>());
  rclcpp::shutdown();
  return 0;
}