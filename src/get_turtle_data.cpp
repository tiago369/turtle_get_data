#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

// #include "action_tutorials_cpp/visibility_control.h"

namespace action_get_data
{
class GetTurtleDataServer : public rclcpp::Node 
{
    public:
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandleAction = rclcpp_action::ServerGoalHandle<Action>;

    explicit GetTurtleDataServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("get_turtle_data_server", options)
    {
    using namespace std::placeholders;
    this->action_server_ = rclcpp_action::create_server<Action>(
        this,
        "turtle",
        std::bind(&GetTurtleDataServer::handle_goal, this, _1, _2),
        std::bind(&GetTurtleDataServer::handle_cancel, this, _1),
        std::bind(&GetTurtleDataServer::handle_accepted, this, _1));
    
    subscription_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_state", 10, std::bind(&GetTurtleDataServer::joint_callback, this, _1));

    subscription_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&GetTurtleDataServer::velocity_callback, this, std::placeholders::_1));

    publish_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("send_goal", 10);
    }
    
    private:
    rclcpp_action::Server<Action>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Action::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order");
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleAction> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&GetTurtleDataServer::execute, this, _1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleAction> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        // rclcpp::Rate loop_rate(1);

        record = 1;
        geometry_msgs::msg::PoseStamped msg;

        msg.header.stamp.sec = 0;
        msg.header.stamp.nanosec = 0;
        msg.header.frame_id = "map";
        msg.pose.position.x = 0.0;
        msg.pose.position.y = 0.0;
        msg.pose.position.z = 0.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = 0.0;
        msg.pose.orientation.w = 1.0;

        publish_goal_->publish(msg);
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if(record){
            for(int i=0; i < msg->velocity.size(); i++){
            sum_velocities_.insert(sum_velocities_.begin() + i, 
                abs(msg->velocity.at(i)) + sum_velocities_.at(i));
            }
        }
    }

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if(record){
            sum_x_ += msg->linear.x;
            sum_y_ += msg->linear.y;
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_;
    std::vector<float> sum_velocities_ = {0.0f, 0.0f};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_velocity_;
    float sum_x_ = 0.0;
    float sum_y_ = 0.0;
    int record = 0;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publish_goal_;

};
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_get_data::GetTurtleDataServer) 