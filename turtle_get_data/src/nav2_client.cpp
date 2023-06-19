#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <sstream>
#include <future>
#include <fstream>
#include <yaml-cpp/yaml.h>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"


namespace action_get_data
{
class Nav2Client : public rclcpp::Node
{
    public:
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandleAction = rclcpp_action::ClientGoalHandle<Action>;

    explicit Nav2Client(const rclcpp::NodeOptions & options)
    : Node("nav2_action_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<Action>(
        this,
        "nav2");

        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Nav2Client::send_goal, this));

        subscription_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_state", 10, std::bind(&Nav2Client::joint_callback, this, std::placeholders::_1));

        subscription_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Nav2Client::velocity_callback, this, std::placeholders::_1));
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

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

        auto goal_msg = Action::Goal();
        goal_msg.pose = msg;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Action>::SendGoalOptions();
        send_goal_options.goal_response_callback =
        std::bind(&Nav2Client::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = 
        std::bind(&Nav2Client::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = 
        std::bind(&Nav2Client::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

        
    }

    private:
    rclcpp_action::Client<Action>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_;
    std::vector<float> sum_velocities_ = {0.0f, 0.0f};
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_velocity_;
    rclcpp::Time initial_time, last_time;
    float sum_x_ = 0.0;
    float sum_y_ = 0.0;
    bool record = false;

    void goal_response_callback(const GoalHandleAction::SharedPtr future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            initial_time = this->now();
            record = true;
        }
    }

    void feedback_callback(
        GoalHandleAction::SharedPtr,
        const std::shared_ptr<const Action::Feedback> feedback) {
    }

    void result_callback(const GoalHandleAction::WrappedResult & result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        last_time = this->now();
        save_data();
        record = false;
        rclcpp::shutdown();
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

    void save_data() {
        // Create a YAML::Emitter object
        YAML::Emitter emitter;

        // Add initial time (when the goal is accepted)
        emitter << YAML::Key << "initial_time";
        emitter << YAML::Value << initial_time.seconds();

        // Add last time (when revieve the result)
        emitter << YAML::Key << "last_time";
        emitter << YAML::Value << last_time.seconds();

        // Add joint effort
        // X
        emitter << YAML::Key << "joint_effort_x";
        emitter << YAML::Value << sum_velocities_.at(0);

        // Y
        emitter << YAML::Key << "joint_effort_y";
        emitter << YAML::Value << sum_velocities_.at(1);

        // Add distance
        // X
        emitter << YAML::Key << "distance_x";
        emitter << YAML::Value << sum_x_;

        // Y
        emitter << YAML::Key << "distance_y";
        emitter << YAML::Value << sum_y_;

        // End emitting YAML
        emitter << YAML::EndMap;

        // Write YAML to file
        std::ofstream file("data.yaml");
        file << emitter.c_str();
        file.close();
    }

};
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_get_data::Nav2Client)
