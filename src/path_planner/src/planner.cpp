#include "planner.hpp"

PathPlanner::PathPlanner():Node("path_planner")
{
    flag_get_grid_map = false;
    flag_get_plan_start = false;
    this->pcl_obs_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pcl_obstacle",10,std::bind(&PathPlanner::pclObsCallback,this,std::placeholders::_1));

    this->declare_parameter<std::double_t>("replan_period", 0.5);
    int ms = floor(replan_period * 1000.0);
    this->collision_check_timer = this->create_wall_timer(std::chrono::milliseconds(ms), std::bind(&PathPlanner::collisionCheckCallback,this));

    this->declare_parameter<std::double_t>("grid_map_origin_point_x", 0.0);
    this->declare_parameter<std::double_t>("grid_map_origin_point_y", 0.0);
    this->declare_parameter<std::double_t>("grid_map_origin_point_z", 0.0);
    this->grid_map_origin_point.x() = this->get_parameter("grid_map_origin_point_x").as_double();
    this->grid_map_origin_point.y() = this->get_parameter("grid_map_origin_point_y").as_double();
    this->grid_map_origin_point.z() = this->get_parameter("grid_map_origin_point_z").as_double();

    this->declare_parameter<std::int16_t>("grid_map_x_size", 200);
    this->declare_parameter<std::int16_t>("grid_map_y_size", 400);
    this->declare_parameter<std::int16_t>("grid_map_z_size", 200);
    this->grid_map_x_size = this->get_parameter("grid_map_x_size").as_int();
    this->grid_map_y_size = this->get_parameter("grid_map_y_size").as_int();
    this->grid_map_z_size = this->get_parameter("grid_map_z_size").as_int();
    std::vector<bool> temp_occupy(this->grid_map_z_size, false);
    std::vector<std::vector<bool>> temp_occupy_2d(this->grid_map_y_size, temp_occupy);
    this->grid_map.resize(this->grid_map_x_size, temp_occupy_2d);

    this->declare_parameter<double_t>("resolution",10.0);
    this->resolution = this->get_parameter("resolution").as_double();

    this->declare_parameter<int16_t>("inflation_radius",5);
    this->inflation_radius = this->get_parameter("inflation_radius").as_int();

    this->declare_parameter<double_t>("start_position_x", 1000.0);
    this->declare_parameter<double_t>("start_position_y", 1900.0);
    this->declare_parameter<double_t>("start_position_z", 1000.0);
    this->declare_parameter<double_t>("start_speed_x", 0.0);
    this->declare_parameter<double_t>("start_speed_y", 20.0);
    this->declare_parameter<double_t>("start_speed_z", 0.0);
    this->declare_parameter<double_t>("average_speed", 20.0);
    this->start_position.x() = this->get_parameter("start_position_x").as_double();
    this->start_position.y() = this->get_parameter("start_position_y").as_double();
    this->start_position.z() = this->get_parameter("start_position_z").as_double();
    this->start_direction.x() = this->get_parameter("start_speed_x").as_double();
    this->start_direction.y() = this->get_parameter("start_speed_y").as_double();
    this->start_direction.z() = this->get_parameter("start_speed_z").as_double();
    this->average_speed = this->get_parameter("average_speed").as_double();

    return;
}

// This function will get the pcl of obstacle and change to the 3d grid map that used in collision detection.
void PathPlanner::pclObsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pcl_obs_msg)
{
    if (flag_get_grid_map)
    {
        return;        
    }
    pcl::fromROSMsg(*pcl_obs_msg, this->pcl_obs);
    int pcl_obs_size = pcl_obs.points.size();
    int temp_index_x,temp_index_y,temp_index_z;
    for (int i=0;i<pcl_obs_size;i++)
    {
        temp_index_x = floor((pcl_obs.points[i].x-this->grid_map_origin_point.x())/this->resolution);
        temp_index_y = floor((pcl_obs.points[i].y-this->grid_map_origin_point.y())/this->resolution);
        temp_index_z = floor((pcl_obs.points[i].z-this->grid_map_origin_point.z())/this->resolution);
        this->grid_map[temp_index_x][temp_index_y][temp_index_z] = true;
        for (int x=temp_index_x-inflation_radius;x<=temp_index_x+inflation_radius;x++)
        {
            for (int y=temp_index_y-inflation_radius;y<=temp_index_y+inflation_radius;y++)
            {
                for (int z=temp_index_z-inflation_radius;z<=temp_index_z+inflation_radius;z++)
                {
                    boundCorrect(x,y,z);
                    grid_map[x][y][z] = true;
                } 
            }
        }
    }
    flag_get_grid_map = true;
    return;
}

void PathPlanner::collisionCheckCallback()
{
    // TODO:
    return ;
}

// init the straight line path that for the 
void PathPlanner::replanPath()
{
    //TODO: 
    if (!this->flag_get_grid_map || !flag_get_plan_start) return; // this plan start position and velocity is got from the 
                                                                  // decoder of the linear mechanism's motor
    // # STEP 1 #: Initializing global polynomial path.
    // TIP: set distance thresh 200mm, 
    
    // # STEP 2 #: Initializing control point on polynomial path.


    // # STEP 3 #: Collision checkout and optimization until the control points set free with collision.


    return;
}

// Corrects points outside the boundary
void PathPlanner::boundCorrect(int &x, int &y, int &z)
{
    x = std::min(std::max(0,x), grid_map_x_size-1);
    y = std::min(std::max(0,y), grid_map_y_size-1);
    z = std::min(std::max(0,z), grid_map_z_size-1);
    return;
}

