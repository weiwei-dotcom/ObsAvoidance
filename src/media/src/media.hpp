#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "interface/msg/slam.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "interface/srv/map_point.hpp"
#include "interface/srv/cam_pose.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <mutex>

class mediaNode : public rclcpp :: Node
{
private:
    // 是否获得了slam到真实世界的尺度标志
    bool getScale;

    // slam系统初始化之后的尺度到真实尺度的尺度变换因子
    // TODO: 后面需要一个标定尺度的因子的方法来更新该变量
    float scaleFact_slamToCam;
    // 创建互斥锁变量
    std::mutex mtx;
    // 创建回调函数组
    rclcpp::CallbackGroup::SharedPtr mapPointCallbackGroup;
    // 创建订阅slam话题的消息订阅指针
    rclcpp::Subscription<interface::msg::Slam>::SharedPtr slamSub;
    // 声明订阅slam话题的回调函数
    void slam_callback(const interface::msg::Slam::SharedPtr slamMsg);
    // 定义接收slam话题消息的相机位姿、点云消息、关键点图像坐标变量
    interface::msg::Slam::SharedPtr camPose_pointCloud_img;
    sensor_msgs::msg::PointCloud2 m_pointCloud;
    geometry_msgs::msg::Pose m_camPose;
    // 创建 获取原图像、相机位姿、点云、关键点坐标 服务端指针 
    rclcpp::Service<interface::srv::MapPoint>::SharedPtr mapPointSer;
    // 创建 获取相机位姿 服务端指针 
    rclcpp::Service<interface::srv::CamPose>::SharedPtr camPoseSer;
    // 创建 获取原图像、相机位姿、点云、关键点坐标 服务端回调函数
    void mapPoint_callback(const interface::srv::MapPoint::Request::SharedPtr request,
                            const interface::srv::MapPoint::Response::SharedPtr response);
    // // 创建 获取相机位姿 服务端回调函数
    void camPose_callback(const interface::srv::CamPose::Request::SharedPtr request,
                            const interface::srv::CamPose::Response::SharedPtr response);
    // 获取到真实世界mm单位制的尺度因子的回调函数
    void getScale_callback();
    
public:
    // 初始化函数
    mediaNode();
    // 析构函数
    ~mediaNode();
};