#include "path_follow.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<PathFollow>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}