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
#include <ceres/ceres.h>


class PathPlanner:public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_obs_sub;

    rclcpp::TimerBase::SharedPtr replan_timer;

    pcl::PointCloud<pcl::PointXYZ> pcl_obs;

    int grid_map_x_size,grid_map_y_size,grid_map_z_size;

    std::vector<std::vector<std::vector<bool>>> occupy_status;

    double resolution;

    int inflation_radius;

    bool flag_get_grid_map,flag_get_plan_start;

    Eigen::Vector3d grid_map_origin_point;

    double replan_period;

    Eigen::Vector3d start_position,start_direction;

    double average_speed;

    int max_num_proj_line, max_num_proj;

    // if the project_center_point = [0,0,0], which means the there is no project center point
    std::vector<std::vector<Eigen::Vector3d>> project_center_points;

    std::vector<std::vector<Eigen::Vector3d>> control_points_list;

    double init_proj_map_view_angle, opt_proj_map_view_angle;
    double init_proj_map_lati_step, init_proj_map_longi_step;
    double opt_proj_map_lati_step, opt_proj_map_longi_step;

public:
    PathPlanner();
    void pclObsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_obs_msg);

    void replanPathCallback();
    
    void boundCorrect(int &x, int &y, int &z);
    
};