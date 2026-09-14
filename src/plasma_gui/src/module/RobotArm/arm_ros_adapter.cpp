#include "arm_ros_adapter.h"

#include <algorithm>
#include <utility>

ArmRosAdapter::ArmRosAdapter(rclcpp::Node &node)
{
    m_moveJPublisher = node.create_publisher<rm_ros_interfaces::msg::Movej>(
        "/rm_driver/movej_cmd", rclcpp::ParametersQoS());
    m_moveLOffsetPublisher = node.create_publisher<rm_ros_interfaces::msg::Moveloffset>(
        "/rm_driver/movel_offset_cmd", rclcpp::ParametersQoS());
    m_moveStopPublisher = node.create_publisher<std_msgs::msg::Empty>(
        "/rm_driver/move_stop_cmd", rclcpp::ParametersQoS());
    m_toolFrameQueryPublisher = node.create_publisher<std_msgs::msg::Empty>(
        "/rm_driver/get_current_tool_frame_cmd", rclcpp::ParametersQoS());

    m_moveJResultSubscription = node.create_subscription<std_msgs::msg::Bool>(
        "/rm_driver/movej_result", rclcpp::ParametersQoS(),
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
            if (m_moveJResultCallback)
                m_moveJResultCallback(message->data);
        });
    m_moveLOffsetResultSubscription = node.create_subscription<std_msgs::msg::Bool>(
        "/rm_driver/movel_offset_result", rclcpp::ParametersQoS(),
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
            if (m_moveLOffsetResultCallback)
                m_moveLOffsetResultCallback(message->data);
        });
    m_moveStopResultSubscription = node.create_subscription<std_msgs::msg::Bool>(
        "/rm_driver/move_stop_result", rclcpp::ParametersQoS(),
        [this](const std_msgs::msg::Bool::ConstSharedPtr message) {
            if (m_moveStopResultCallback)
                m_moveStopResultCallback(message->data);
        });
    m_toolFrameResultSubscription = node.create_subscription<std_msgs::msg::String>(
        "/rm_driver/get_current_tool_frame_result", rclcpp::ParametersQoS(),
        [this](const std_msgs::msg::String::ConstSharedPtr message) {
            if (m_currentToolFrameCallback)
                m_currentToolFrameCallback(message->data);
        });
}

void ArmRosAdapter::setMoveJResultCallback(ResultCallback callback)
{
    m_moveJResultCallback = std::move(callback);
}

void ArmRosAdapter::setMoveStopResultCallback(ResultCallback callback)
{
    m_moveStopResultCallback = std::move(callback);
}

void ArmRosAdapter::setMoveLOffsetResultCallback(ResultCallback callback)
{
    m_moveLOffsetResultCallback = std::move(callback);
}

void ArmRosAdapter::setCurrentToolFrameCallback(TextCallback callback)
{
    m_currentToolFrameCallback = std::move(callback);
}

void ArmRosAdapter::publishMoveJ(const std::array<double, 6> &jointsRad,
                                 std::uint8_t speed)
{
    rm_ros_interfaces::msg::Movej message;
    message.joint.assign(jointsRad.begin(), jointsRad.end());
    message.speed = speed;
    message.block = true;
    message.trajectory_connect = 0;
    message.dof = 6;
    m_moveJPublisher->publish(message);
}

void ArmRosAdapter::publishToolOffset(int toolAxis, double distanceMeters,
                                      std::uint8_t speed)
{
    rm_ros_interfaces::msg::Moveloffset message;
    const int axis = std::clamp(toolAxis, 0, 2);
    if (axis == 0)
        message.pose.position.x = distanceMeters;
    else if (axis == 1)
        message.pose.position.y = distanceMeters;
    else
        message.pose.position.z = distanceMeters;
    message.pose.orientation.w = 1.0;
    message.speed = speed;
    message.r = 0;
    message.trajectory_connect = false;
    message.frame_type = true;
    message.block = true;
    m_moveLOffsetPublisher->publish(message);
}

void ArmRosAdapter::publishMoveStop()
{
    m_moveStopPublisher->publish(std_msgs::msg::Empty{});
}

void ArmRosAdapter::requestCurrentToolFrame()
{
    m_toolFrameQueryPublisher->publish(std_msgs::msg::Empty{});
}

bool ArmRosAdapter::moveJReady() const
{
    return m_moveJPublisher && m_moveJResultSubscription &&
           m_moveJPublisher->get_subscription_count() > 0 &&
           m_moveJResultSubscription->get_publisher_count() > 0;
}

bool ArmRosAdapter::moveLOffsetReady() const
{
    return m_moveLOffsetPublisher && m_moveLOffsetResultSubscription &&
           m_moveLOffsetPublisher->get_subscription_count() > 0 &&
           m_moveLOffsetResultSubscription->get_publisher_count() > 0;
}

bool ArmRosAdapter::toolFrameQueryReady() const
{
    return m_toolFrameQueryPublisher && m_toolFrameResultSubscription &&
           m_toolFrameQueryPublisher->get_subscription_count() > 0 &&
           m_toolFrameResultSubscription->get_publisher_count() > 0;
}
