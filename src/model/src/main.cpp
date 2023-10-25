#include "model.hpp"


// 主程序
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelNode>();
    node->send_transform_request();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}