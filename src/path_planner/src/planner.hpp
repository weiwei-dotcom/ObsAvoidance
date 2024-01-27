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
#include "polynomial_traj.hpp"
#include "uniform_bspline.hpp"


class PathPlanner:public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_obs_sub;

    rclcpp::TimerBase::SharedPtr collision_check_timer;

    pcl::PointCloud<pcl::PointXYZ> pcl_obs;

    int grid_map_x_size,grid_map_y_size,grid_map_z_size;

    std::vector<std::vector<std::vector<bool>>> grid_map;

    double resolution;

    int inflation_radius;

    bool flag_get_grid_map,flag_get_plan_start, flag_finish_planning;

    Eigen::Vector3d grid_map_origin_point;

    double replan_period;

    Eigen::Vector3d start_pos,start_direction,start_vel,end_pos;

    std::vector<Eigen::Vector3d> start_end_derivatives;

    Eigen::Vector3d current_position, current_direction;

    double average_speed;

    std::vector<Eigen::Vector3d> bspline_interp_pt;

    // init_polynomial path
    PolynomialTraj init_poly_path;

    double interp_dist_thresh, control_point_dist;

    // std::vector<Eigen::Vector3d> ctrl_pt;
    Eigen::MatrixXd ctrl_pts;

    double extension_ratio;

    double min_plan_dist, max_control_point_dist;

    int order;

    std::vector<std::pair<int, int>> segment_ids;

    std::vector<std::vector<Eigen::Vector3d>> base_pts;
    std::vector<std::vector<Eigen::Vector3d>> esc_directions;
    std::vector<bool> temp_flags;

public:
    PathPlanner();
    void pclObsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_obs_msg);

    void replanPath();

    void collisionCheckCallback();
    
    void boundCorrect(int &x, int &y, int &z);
    
    bool PathPlanner::planInitTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initControlPoints();

    void getCollisionSegId();

    bool checkCollision(const Eigen::Vector3d &coor);

    Eigen::Vector3i coorToIndex(const Eigen::Vector3d &coor);

    void reboundSegId();
};