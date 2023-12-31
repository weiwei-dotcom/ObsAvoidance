#include "slam.hpp"

using namespace std;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    bool visualization = true;
    ORB_SLAM3::System slam("/home/weiwei/Desktop/thirdParty/ORB_SLAM3/Vocabulary/ORBvoc.txt", 
                            "/home/weiwei/Desktop/thirdParty/ORB_SLAM3/Vocabulary/small.yaml",
                            ORB_SLAM3::System::MONOCULAR, visualization);
    auto node = std::make_shared<slamNode>(&slam);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}