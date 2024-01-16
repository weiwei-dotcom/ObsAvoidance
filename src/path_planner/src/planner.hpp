#include <rclcpp/rclcpp.hpp>
#include <Eigen/Eigen>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include "pcl/point_cloud.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
// #include <ceres/ceres.h>


class path_planner:public rclcpp::Node
{
private:

public:
    path_planner();
};