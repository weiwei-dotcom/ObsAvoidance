#include "planner.hpp"

PathPlanner::PathPlanner():Node("path_planner")
{
    this->pcl_obs_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pcl_obstacle",10,std::bind(&PathPlanner::pclObsCallback,this,std::placeholders::_1));
    return;
}

// This function will get the pcl of obstacle and change to the 3d grid map that used in collision detection.
void PathPlanner::pclObsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_obs_msg)
{
    pcl::fromROSMsg(*pcl_obs_msg, this->pcl_obs);

    return;
}