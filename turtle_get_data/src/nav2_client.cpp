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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace action_get_data
{
class Nav2Client : public rclcpp::Node
{
    public:
    using Action = nav2_msgs::action::NavigateToPose;
    using GoalHandleAction = rclcpp_action::ClientGoalHandle<Action>;

    explicit Nav2Client(const rclcpp::NodeOptions & options)
    : Node("nav2_action_client", options) {
        this->client_ptr_ = rclcpp_action::create_client<Action>(
        this,
        "navigate_to_pose");

        this->timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Nav2Client::send_goal, this));
        // send_goal();

        subscription_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&Nav2Client::joint_callback, this, std::placeholders::_1));

        subscription_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Nav2Client::velocity_callback, this, std::placeholders::_1));

        tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    void send_goal() {
        using namespace std::placeholders;
        
        file_name_ = declare_parameter<std::string>("file_name", file_name_);

        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
        }

        geometry_msgs::msg::PoseStamped msg;

        msg.header.stamp.sec = 0;
        msg.header.stamp.nanosec = 0;
        msg.header.frame_id = "map";
        msg.pose.position.x = 1.3;
        msg.pose.position.y = 0.8;
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
    std::string file_name_{"data.yaml"};
    rclcpp::Time initial_time, last_time;
    float sum_x_ = 0.0;
    float sum_y_ = 0.0;
    bool record = false;
    std::vector<float> cpu_usage_;
    bool success_ = false;
    // TF
    geometry_msgs::msg::TransformStamped transform_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    void goal_response_callback(const GoalHandleAction::SharedPtr goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            initial_time = this->get_clock()->now();
            record = true;
        }
    }

    void feedback_callback(
        GoalHandleAction::SharedPtr,
        const std::shared_ptr<const Action::Feedback> feedback) {
            // Get cpu percentage use in the feedback clock
            cpu_usage_.push_back(GetCPUUsage());
    }

    void result_callback(const GoalHandleAction::WrappedResult & result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            success_ = true;
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        last_time = this->get_clock()->now();

        try {
          transform_ = tf_buffer_->lookupTransform(
            "map", "base_footprint",
            tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform");
          return;
        }

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
        std::ofstream file(file_name_);

        // See if trajectory was sucessfull
        file << "sucess: " << success_ << std::endl;
        
        // Add initial time (when the goal is accepted)
        file << "initial_time: " << initial_time.seconds() << std::endl;

        // Add last time (when revieve the result)
        file << "last_time: " << last_time.seconds() << std::endl;

        // Add time variation
        file << "dt: " << last_time.seconds() - initial_time.seconds() << std::endl;

        // Add joint effort
        // X
        file << "joint_effort_x: " << sum_velocities_.at(0) << std::endl;

        // Y
        file << "joint_effort_y: " << sum_velocities_.at(1) << std::endl;

        // Add distance
        // X
        file << "distance_x: " << sum_x_ << std::endl;

        // Y
        file << "distance_y: " << sum_y_ << std::endl;

        //pose
        file << "pose: " << std::endl;

        //x
        file << "   x: " << transform_.transform.translation.x << std::endl;
        //y
        file << "   y: " << transform_.transform.translation.y << std::endl;
        //z
        file << "   z: " << transform_.transform.translation.y << std::endl;
        //x

        //rot
        file << "rot: " << std::endl;
        file << "   qx: " << transform_.transform.rotation.x << std::endl;
        file << "   qy: " << transform_.transform.rotation.y << std::endl;
        file << "   qz: " << transform_.transform.rotation.z << std::endl;
        file << "   qw: " << transform_.transform.rotation.w << std::endl;

        file << "test: ";
        for (double cpu : cpu_usage_) {
            file << cpu << ", ";
        }

        file << std::endl;

        file.close();
        std::cout << "Data saved to " << file_name_ << std::endl;
    }

    // Function to measure CPU usage
    double GetCPUUsage() {
        // Record initial CPU time
        auto start = std::chrono::high_resolution_clock::now();
        auto startCPU = clock();

        // Sleep for the desired time interval
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Change the duration as needed

        // Record final CPU time
        auto end = std::chrono::high_resolution_clock::now();
        auto endCPU = clock();

        // Calculate CPU usage as a percentage
        double cpuUsage = static_cast<double>(endCPU - startCPU) / CLOCKS_PER_SEC;
        double elapsedTime = std::chrono::duration<double>(end - start).count();
        double cpuUsagePercentage = (cpuUsage / elapsedTime) * 100.0;

        return cpuUsagePercentage;
    }

};
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_get_data::Nav2Client)
