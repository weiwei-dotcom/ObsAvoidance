#include "rclcpp/rclcpp.hpp"
#include "joint.hpp"
#include "Eigen/Eigen"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "interface/srv/path_points.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <fstream>

class PathFollow:public rclcpp::Node
{
private:
    // 判断是否已经获取了离散跟跟随路径点标志，判断发起服务后是否响应标志位防止重复发起服务
    bool flag_get_first_path, flag_called_not_response;
    // 判断是否靠近目标点标志位
    bool flag_close_to_target_point;
    // 世界到参考坐标系下的转换
    Eigen::Matrix4d world_to_ref;
    // 行程始终点
    Eigen::Vector3d stroke_start,stroke_end;
    // 存储路径离散点
    std::vector<Eigen::Vector3d> path_points;
    // 声明定时器，定时启动获取新的离散路径点服务
    rclcpp::TimerBase::SharedPtr get_path_timer;
    // 声明客户端
    rclcpp::Client<interface::srv::PathPoints>::SharedPtr get_path_client;
public:
    PathFollow();
    void fitPathCallback();
    void getNewPathCall();
    void getNewPathCallback(rclcpp::Client<interface::srv::PathPoints>::SharedFuture response);
    
};