#ifndef GUI_CONNECT_H
#define GUI_CONNECT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class GuiConnect : public rclcpp::Node
{
public:
    GuiConnect():Node("gui_connect")
    {
        //创建发布者与订阅者
        RCLCPP_INFO(this->get_logger(), "GuiConnect node has been started.");
        con_sub=this->create_subscription<std_msgs::msg::String>("gui_connect", 10, std::bind(&GuiConnect::connect_callback, this, std::placeholders::_1));
        con_pub=this->create_publisher<std_msgs::msg::String>("ros_connect", 10);
        
        //创建定时器
        timer=this->create_wall_timer(100ms, std::bind(&GuiConnect::timer_callback, this));

    };
    ~GuiConnect()=default;


    void connect_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "received from qt: %s", msg->data.c_str());
    }
    //组织消息并发布
    void timer_callback()
    {
        static int count;
        std_msgs::msg::String msg=std_msgs::msg::String();
        msg.data="this is ros2"+std::to_string(count++);
        RCLCPP_INFO(this->get_logger(), "Send to qt: %s", msg.data.c_str());
        con_pub->publish(msg);
    }
        
private:
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> con_pub;
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> con_sub;
    rclcpp::TimerBase::SharedPtr timer;
};




#endif
