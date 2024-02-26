#include "rclcpp/rclcpp.hpp"
#include "joint.hpp"
#include "Eigen/Eigen"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "interface/srv/path_points.hpp"
#include "interface/srv/base_joint_motor_value.hpp"
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
    /*  判断是否已经获取了离散跟跟随路径点标志变量
            在初始化函数中置为假；
            在getNewPathCallback()函数中，每次成功获取新的离散路径点后置为真;
    */
    bool flag_get_first_path;

    /*  判断服务已经发起过但是未被响应的标志变量
            在初始化函数中置为假；
            在每次发起获取新离散路径点服务请求后置为真;
            在进入获取新离散路径点响应函数后置为假;
    */
    bool flag_called_not_response;
    
    /*  判断是否靠近目标点标志位
            在初始化函数中置为假；
            计算每次拟合路径后的距离骨架末端点最近离散路径点与离散路径点序列末端点之间路径长度，如果长度小于设定阈值，那么该标志位置为真；
            如果该标志位为真，那么不用再发起获取新离散路径点服务；
    */
    bool flag_close_to_target_point;
    // 世界到参考坐标系下的转换
    Eigen::Matrix4d world_to_ref;
    // 行程始终点
    Eigen::Vector3d stroke_start,stroke_end;
    // 存储路径离散点
    std::vector<Eigen::Vector3d> path_points;
    // 声明获取离散路径点定时器
    rclcpp::TimerBase::SharedPtr get_path_timer;
    // 声明拟合路径服务端
    ///TODO: here
    rclcpp::Service<interface::srv::BaseJointMotorValue>::SharedPtr fit_path_server;
    // 声明获取新离散路径客户端
    rclcpp::Client<interface::srv::PathPoints>::SharedPtr get_path_client;
    // 路径规划点在离散路径点序列上的索引值
    int replan_start_id;
    // 路径规划的起点速度值，同样是路径跟随推进速度值
    double speed_value;

public:
    PathFollow();
    void fitPathCallback();
    void getNewPathCall();
    void getNewPathCallback(rclcpp::Client<interface::srv::PathPoints>::SharedFuture response);
    
};