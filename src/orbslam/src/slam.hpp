#include "rclcpp/rclcpp.hpp"
#include "interface/msg/slam.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/point_field_conversion.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sophus/se3.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.h"
#include <chrono>
#include <System.h>
#include <MapPoint.h>
#include <Eigen/Eigen>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace cv;


class slamNode:public rclcpp::Node
{
private:
    // 创建相机图像订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr imgSub;
    // 创建话题发布者指针,需要再初始化函数内进行实例化
    rclcpp::Publisher<interface::msg::Slam>::SharedPtr slamPub;
    ORB_SLAM3::System* visualOdom;
    // 声明订阅图像消息回调函数
    void grabImg_pubSlam(const sensor_msgs::msg::Image::SharedPtr img);
    // 创建图像消息转化为cv类型指针的接收变量
    cv_bridge::CvImagePtr img2cvPtr;
    // // slam系统初始化的时间
    // std::chrono::steady_clock::time_point slamInitTime;
    // // slam系统接收单目图像开始跟踪的时间
    // std::chrono::steady_clock::time_point slamTrackTime;
    // 用来接收orbslam3的获得数据以及转化为ros2消息接口的中间变量
    Sophus::SE3f Tci;  // 第一帧坐标系(world)相对于当前帧的位姿矩阵
    interface::msg::Slam slamMsg;  // 定义ros2消息接口变量
    std::vector<ORB_SLAM3::MapPoint*> pointCloud; // 接收orbslam3系统返回的当前帧的点云数据
    pcl::PointCloud<pcl::PointXYZ> pclPointCloud; // 中间变量
    pcl::PointXYZ point;
    int state;
    double scale_fact; // 尺度因子
public:
    slamNode(ORB_SLAM3::System* slam);
    // void distortionImg(cv_bridge::CvImagePtr imgPtr, sensor_msgs::msg::Image& img, std::string name);
    ~slamNode();
};