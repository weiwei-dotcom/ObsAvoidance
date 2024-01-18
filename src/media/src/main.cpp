#include "media.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<mediaNode>();
<<<<<<< HEAD
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // node->getScaleRequired();
    executor.spin();
=======
    rclcpp::spin(node);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    rclcpp::shutdown();
    return 0;
}