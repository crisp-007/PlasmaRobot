#include "gui_connect.h"

int main(int argc,char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuiConnect>();
    rclcpp::spin(node);
    rclcpp::shutdown();//释放资源
    return 0;
}