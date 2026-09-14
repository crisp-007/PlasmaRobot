#ifndef ARM_ROS_ADAPTER_H
#define ARM_ROS_ADAPTER_H

#include <array>
#include <cstdint>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <rm_ros_interfaces/msg/movej.hpp>
#include <rm_ros_interfaces/msg/moveloffset.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

class ArmRosAdapter
{
public:
    using ResultCallback = std::function<void(bool)>;
    using TextCallback = std::function<void(const std::string &)>;

    explicit ArmRosAdapter(rclcpp::Node &node);

    void setMoveJResultCallback(ResultCallback callback);
    void setMoveLOffsetResultCallback(ResultCallback callback);
    void setMoveStopResultCallback(ResultCallback callback);
    void setCurrentToolFrameCallback(TextCallback callback);

    void publishMoveJ(const std::array<double, 6> &jointsRad, std::uint8_t speed);
    void publishToolOffset(int toolAxis, double distanceMeters, std::uint8_t speed);
    void publishMoveStop();
    void requestCurrentToolFrame();
    bool moveJReady() const;
    bool moveLOffsetReady() const;
    bool toolFrameQueryReady() const;

private:
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr m_moveJPublisher;
    rclcpp::Publisher<rm_ros_interfaces::msg::Moveloffset>::SharedPtr m_moveLOffsetPublisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_moveStopPublisher;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_toolFrameQueryPublisher;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_moveJResultSubscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_moveLOffsetResultSubscription;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_moveStopResultSubscription;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_toolFrameResultSubscription;
    ResultCallback m_moveJResultCallback;
    ResultCallback m_moveLOffsetResultCallback;
    ResultCallback m_moveStopResultCallback;
    TextCallback m_currentToolFrameCallback;
};

#endif // ARM_ROS_ADAPTER_H
