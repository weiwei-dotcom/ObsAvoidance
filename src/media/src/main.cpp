#include "media.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mediaNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // node->getScaleRequired();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}