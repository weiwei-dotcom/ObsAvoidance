#include "rclcpp/rclcpp.hpp"
#include "joint.hpp"
#include "Eigen/Eigen"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "nav_msgs/msg/path.hpp"
#include "uniform_bspline.hpp"
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
    bool flag_get_first_path, flag_called_not_response;
    // 世界到参考坐标系下的转换
    Eigen::Matrix4d world_to_ref;
    // 行程始终点
    Eigen::Vector3d stroke_start,stroke_end;
    // 存储路径离散点
    std::vector<Eigen::Vector3d> path_points;
    
public:
    PathFollow();
    fitPathCallback();
    getNewPathCall();
    getNewPathCallback();
    
};