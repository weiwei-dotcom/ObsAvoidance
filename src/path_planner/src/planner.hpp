#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include "pcl/point_cloud.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
// #include <ceres/ceres.h>


class PathPlanner:public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_obs_sub;
    pcl::PointCloud<pcl::PointXYZ> pcl_obs;
public:
    PathPlanner();
    void pclObsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_obs_msg);
};