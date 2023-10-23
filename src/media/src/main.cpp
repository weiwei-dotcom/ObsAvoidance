#include "media.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mediaNode>();
    std::thread thread_initializaiton(std::bind(&mediaNode::initialization, node));
    rclcpp::spin(node);
    thread_initializaiton.join();
    rclcpp::shutdown();
    return 0;
}