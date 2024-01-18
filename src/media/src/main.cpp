#include "media.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mediaNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}