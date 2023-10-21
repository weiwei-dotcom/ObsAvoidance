#include "media.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mediaNode>();
    std::thread{std::bind(&mediaNode::initialization, node)}.detach();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}