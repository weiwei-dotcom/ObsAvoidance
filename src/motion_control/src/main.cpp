#include "motion_control.hpp"
#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<MotionControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}