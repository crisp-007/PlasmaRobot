#ifndef RM_CONTROL_H
#define RM_CONTROL_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <iostream>
#include <chrono>
#include <mutex>
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>

//RM Robot msg
#include "rm_ros_interfaces/msg/jointpos.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
//#include "rm_ros_interfaces/msg/jointpos75.hpp"

/* 使用变长数组 */
#include <vector>
#include <algorithm>

using namespace std;

class Rm_Control : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    explicit Rm_Control(std::string name);
    ~Rm_Control(){}

    void timer_callback();

private:
    rm_ros_interfaces::msg::Jointpos joint_msg;
    // rm_ros_interfaces::msg::Jointpos75 joint7_msg;
    int arm_type_ = 75;
    bool follow_ = false;
    bool blocking_movej_mode_ = false;
    // 实例化样条
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // 声明话题发布者
    rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos>::SharedPtr joint_pos_publisher;
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_publisher_;
    // rclcpp::Publisher<rm_ros_interfaces::msg::Jointpos75>::SharedPtr joint_pos_publisher_75;

    //声明话题订阅者
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr Get_Move_Stop_Cmd;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_result_subscription_;

    rclcpp::TimerBase::SharedPtr State_Timer;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void get_move_stop_callback(std_msgs::msg::Bool::SharedPtr msg);
    void joint_state_callback(sensor_msgs::msg::JointState::SharedPtr msg);
    void movej_result_callback(std_msgs::msg::Bool::SharedPtr msg);

    std::mutex joint_state_mutex_;
    std::vector<double> latest_joint_positions_;
    std::chrono::steady_clock::time_point latest_joint_state_time_;
    double point_reached_tolerance_rad_ = 0.008726646259971648;
    double max_command_lead_rad_ = 0.03490658503988659;
    int max_joint_state_age_ms_ = 200;
    double final_settle_max_error_rad_ = 0.03490658503988659;
    int final_settle_speed_percent_ = 5;
    int final_settle_timeout_ms_ = 30000;
    std::atomic_bool final_settle_sent_{false};
    std::atomic_bool final_settle_result_received_{false};
    std::atomic_bool final_settle_succeeded_{false};
    std::atomic_bool execution_failed_{false};
    std::chrono::steady_clock::time_point final_settle_start_time_;
    std::string execution_error_;
};

#endif // Rm_Control_H
