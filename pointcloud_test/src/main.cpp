#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "interface/msg/slam.hpp"
#include "pcl_conversions/pcl_conversions.h"

/*
这是一段测试代码，现在测试的是接收slam发布的点云消息，使用ros2的octomap消息接收并显示点云的八叉树模型
*/

class pointCloudToOctomap : public rclcpp::Node
{
private:
    rclcpp::Subscription<interface::msg::Slam>::SharedPtr pointCloudSub;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octoMapPub;
    void recv_callback(const interface::msg::Slam::SharedPtr slamMsg)
    {

        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(slamMsg->point_cloud, *pointCloud);
        
        // 测试代码
        // std::cout << pointCloud->at(1).x << " " << pointCloud->at(1).y << " " << pointCloud->at(1).z << std::endl;
        // std::cout << pointCloud->points.at(1).x<< " " << pointCloud->points[1].y << " " << pointCloud->points[1].z << std::endl;


        // 测试代码
        octomap::OcTree octree(0.1);
        octree.setResolution(0.1);
        float i = 0.01f;
        for (auto& point : pointCloud->points)
        {
            octree.updateNode(octomap::point3d(point.x, point.y, point.z),true);
            //测试代码，查看输出的
            // octomap::OcTreeNode * octotreeNode;
            std::cout << "point.data " << point.data << std::endl;
            // i+=1;
            // std::cout << "x coordinate " << point.x << "y coordinate " << point.y << "z coordinate " << point.z << std::endl;
        }
        octree.updateInnerOccupancy();
        // 将八叉树转换为OctoMap消息
        octomap_msgs::msg::Octomap octomap;
        octomap.header.frame_id = "camera_frame";
        octomap_msgs::fullMapToMsg(octree, octomap);
        octoMapPub->publish(octomap);
        std::cout << "octoMap published success" << std::endl;
        return;
    }
public:
    pointCloudToOctomap() : Node("test")
    {
        pointCloudSub = this->create_subscription<interface::msg::Slam>("slam", 10, std::bind(&pointCloudToOctomap::recv_callback, this, std::placeholders::_1));
        octoMapPub = this->create_publisher<octomap_msgs::msg::Octomap>("octo_map", 10);
    }
    ~pointCloudToOctomap(){};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointCloudToOctomap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
