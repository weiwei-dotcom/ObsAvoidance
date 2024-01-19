#include "planner.hpp"

PathPlanner::PathPlanner():Node("path_planner")
{
    flag_get_grid_map = false;
    flag_get_plan_start = false;
    this->pcl_obs_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("pcl_obstacle",10,std::bind(&PathPlanner::pclObsCallback,this,std::placeholders::_1));

    this->declare_parameter<std::double_t>("replan_period", 0.5);
    int ms = floor(replan_period * 1000.0);
    this->replan_timer = this->create_wall_timer(std::chrono::milliseconds(ms), std::bind(&PathPlanner::replanPathCallback,this));

    this->declare_parameter<std::double_t>("grid_map_origin_point_x", 0.0);
    this->declare_parameter<std::double_t>("grid_map_origin_point_y", 0.0);
    this->declare_parameter<std::double_t>("grid_map_origin_point_z", 0.0);
    this->grid_map_origin_point.x() = this->get_parameter("grid_map_origin_point_x").as_double();
    this->grid_map_origin_point.y() = this->get_parameter("grid_map_origin_point_y").as_double();
    this->grid_map_origin_point.z() = this->get_parameter("grid_map_origin_point_z").as_double();

    this->declare_parameter<std::int16_t>("grid_map_x_size", 2000);
    this->declare_parameter<std::int16_t>("grid_map_y_size", 4000);
    this->declare_parameter<std::int16_t>("grid_map_z_size", 2000);
    this->grid_map_x_size = this->get_parameter("grid_map_x_size").as_int();
    this->grid_map_y_size = this->get_parameter("grid_map_y_size").as_int();
    this->grid_map_z_size = this->get_parameter("grid_map_z_size").as_int();
    std::vector<bool> temp_occupy(this->grid_map_z_size, false);
    std::vector<std::vector<bool>> temp_occupy_2d(this->grid_map_y_size, temp_occupy);
    this->occupy_status.resize(this->grid_map_x_size, temp_occupy_2d);

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
    this->start_position.x() = this->get_parameter("start_position_x").as_double();
    this->start_position.y() = this->get_parameter("start_position_y").as_double();
    this->start_position.z() = this->get_parameter("start_position_z").as_double();
    this->start_speed.x() = this->get_parameter("start_speed_x").as_double();
    this->start_speed.y() = this->get_parameter("start_speed_y").as_double();
    this->start_speed.z() = this->get_parameter("start_speed_z").as_double();

    this->declare_parameter<double_t>("init_proj_map_lati_step", 1.0);
    this->declare_parameter<double_t>("init_proj_map_longi_step", 2.0);
    this->declare_parameter<double_t>("opt_proj_map_lati_step", 1.0);
    this->declare_parameter<double_t>("opt_proj_map_longi_step", 2.0);
    this->init_proj_map_lati_size = std::ceil(90.0/this->get_parameter("init_proj_map_lati_step").as_double());
    this->init_proj_map_longi_size = std::ceil(360.0/this->get_parameter("init_proj_map_longi_step").as_double());
    this->opt_proj_map_lati_size = std::ceil(90.0/this->get_parameter("opt_proj_map_lati_step").as_double());
    this->opt_proj_map_longi_size = std::ceil(360.0/this->get_parameter("opt_proj_map_longi_step").as_double());

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
        this->occupy_status[temp_index_x][temp_index_y][temp_index_z] = true;
        for (int x=temp_index_x-inflation_radius;x<=temp_index_x+inflation_radius;x++)
        {
            for (int y=temp_index_y-inflation_radius;y<=temp_index_y+inflation_radius;y++)
            {
                for (int z=temp_index_z-inflation_radius;z<=temp_index_z+inflation_radius;z++)
                {
                    boundCorrect(x,y,z);
                    occupy_status[x][y][z] = true;
                } 
            }
        }
    }
    flag_get_grid_map = true;
    return;
}

// init the straight line path that for the 
void PathPlanner::replanPathCallback()
{
    //TODO: 
    if (!this->flag_get_grid_map || !flag_get_plan_start) return; // this plan start position and velocity is got from the 
                                                                  // decoder of the linear mechanism's motor
    // initializing the straight line path
    // start to project the obstacle to the project map
    for (int i=0; i<this->init_proj_map_lati_size;i++)

    // initializing the B spline control point on the straight line according to the preconfigured interval distance 

    // use the sample point of B spline to detect collision,
        // if collision
        // record the time knot and position of start & end sample point of the every collision segment 
        // use the vector<vector<int>> collision_indexs_list record the index of control point list of every collision 
        // segment accord the time knot,
        // (each vector<int> of collision_indexs_list store a set of index of control point of a collision segment) 
    
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