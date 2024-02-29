#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <Eigen/Eigen>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"
#include "pcl/point_cloud.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include <ceres/ceres.h>
#include "polynomial_traj.hpp"
#include "uniform_bspline.hpp"

constexpr double inf = 1 >> 20;
struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
    enum enum_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0}; // Distinguish every call
    enum enum_state state
    {
        UNDEFINED
    };
    Eigen::Vector3i index;

    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{NULL};
};

class NodeComparator
{
public:
    bool operator()(GridNodePtr node1, GridNodePtr node2)
    {
        return node1->fScore > node2->fScore;
    }
};

class PathPlanner:public rclcpp::Node
{
private:
    pcl::PointCloud<pcl::PointXYZ> pcl_obs;

    int grid_map_x_size,grid_map_y_size,grid_map_z_size;

    std::vector<std::vector<std::vector<bool>>> grid_map;

    double resolution;

    int inflation_radius;

    bool flag_finish_planning;

    Eigen::Vector3d grid_map_origin_point;

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

    int rounds_; // distinguish the round of a_star search;

    std::vector<std::pair<int, int>> segment_ids;

    std::vector<std::vector<Eigen::Vector3d>> base_pts;
    std::vector<std::vector<Eigen::Vector3d>> esc_directions;
    std::vector<bool> temp_flags;

    double tie_breaker_;

    Eigen::Vector3i POOL_SIZE_, CENTER_IDX_;

    GridNodePtr ***GridNodeMap_;

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;

    std::vector<GridNodePtr> gridPath_;

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

public:
    PathPlanner();
    void pclToGridMap(const pcl::PointCloud<pcl::PointXYZ> &obs_pcl);

    void replanPath();
    
    void boundCorrect(int &x, int &y, int &z);
    
    bool PathPlanner::planInitTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                   const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc);

    void initControlPoints();

    void resize(const int size_set);

    void getBasePointsAndDirection();

    void Optimize();

    bool checkCollision(const Eigen::Vector3d &coor);

    bool coorToIndex(const Eigen::Vector3d &coor, Eigen::Vector3i &index);

    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    std::vector<Eigen::Vector3d> getPath();

    bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx);

    void initGridMap(/* GridMap::Ptr occ_map, */const Eigen::Vector3i pool_size);

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);

    vector<GridNodePtr> retrievePath(GridNodePtr current);

    bool optFirstSeg();

    bool check_collision_and_rebound();

    inline Eigen::Vector3d PathPlanner::Index2Coord_a_star(const Eigen::Vector3i &index) const
    {
        return ((index - CENTER_IDX_).cast<double>() * step_size_) + center_;
    };

    inline bool PathPlanner::Coord2Index_a_star(const Eigen::Vector3d &pt, Eigen::Vector3i &idx) const
    {
        idx = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + CENTER_IDX_;

        if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) || idx(1) < 0 || idx(1) >= POOL_SIZE_(1) || idx(2) < 0 || idx(2) >= POOL_SIZE_(2))
        {
            RCLCPP_ERROR(this->get_logger(), "Ran out of pool, index=%d %d %d", idx(0), idx(1), idx(2));
            return false;
        }
        return true;
    };

    inline double PathPlanner::getHeu(GridNodePtr node1, GridNodePtr node2)
    {
        return tie_breaker_ * getDiagHeu(node1, node2);
    }

};