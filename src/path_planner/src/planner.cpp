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
    this->declare_parameter<double_t>("average_speed", 20.0);
    this->declare_parameter<int16_t>("max_num_proj_line", 3);
    this->declare_parameter<int16_t>("max_num_proj", 2);
    this->start_position.x() = this->get_parameter("start_position_x").as_double();
    this->start_position.y() = this->get_parameter("start_position_y").as_double();
    this->start_position.z() = this->get_parameter("start_position_z").as_double();
    this->start_direction.x() = this->get_parameter("start_speed_x").as_double();
    this->start_direction.y() = this->get_parameter("start_speed_y").as_double();
    this->start_direction.z() = this->get_parameter("start_speed_z").as_double();
    this->max_num_proj_line = this->get_parameter("max_num_proj_line").as_int();
    this->max_num_proj = this->get_parameter("max_num_proj").as_int();
    this->average_speed = this->get_parameter("average_speed").as_double();
    Eigen::Vector3d temp_project_point = Eigen::Vector3d::Zero();
    std::vector<Eigen::Vector3d> temp_project_points((int)std::pow(max_num_proj_line, max_num_proj),temp_project_point);
    std::vector<double> temp_start_proj_step((int)std::pow(max_num_proj_line, max_num_proj),0);
    init_proj_center_points.resize(this->max_num_proj+1, temp_project_points);
    init_proj_start_proj_step.resize(this->max_num_proj+1, temp_start_proj_step);
    init_proj_center_points[0][0] = this->start_position + this->start_direction.normalized()*average_speed;
    init_proj_start_proj_step[0][0] = 0.0; 

    // debug
    RCLCPP_INFO(this->get_logger(), "init_proj_center_points[0][0]: [%f,%f,%f]", init_proj_center_points[0][0].x(),init_proj_center_points[0][0].y(),init_proj_center_points[0][0].z());
    RCLCPP_INFO(this->get_logger(), "init_proj_center_points[0].size: %d", this->init_proj_center_points[0].size());
    RCLCPP_INFO(this->get_logger(), "init_proj_start_proj_step[0][0]: %f", init_proj_start_proj_step[0][0]);
    RCLCPP_INFO(this->get_logger(), "init_proj_start_proj_step[0].size: %d", this->init_proj_start_proj_step[0].size());

    this->declare_parameter<double_t>("init_proj_map_view_angle", 160.0);
    this->declare_parameter<double_t>("opt_proj_map_view_angle", 160.0);
    this->init_proj_map_view_angle = this->get_parameter("init_proj_map_view_angle").as_double();
    this->opt_proj_map_view_angle = this->get_parameter("opt_proj_map_view_angle").as_double();

    this->declare_parameter<double_t>("init_proj_map_lati_step", 1.0);
    this->declare_parameter<double_t>("init_proj_map_longi_step", 2.0);
    this->declare_parameter<double_t>("opt_proj_map_lati_step", 1.0);
    this->declare_parameter<double_t>("opt_proj_map_longi_step", 2.0);
    this->init_proj_map_lati_step = this->get_parameter("init_proj_map_lati_step").as_double();
    this->init_proj_map_longi_step = this->get_parameter("init_proj_map_longi_step").as_double();
    this->opt_proj_map_lati_step = this->get_parameter("opt_proj_map_lati_step").as_double();
    this->opt_proj_map_longi_step = this->get_parameter("opt_proj_map_longi_step").as_double();

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
    // # STEP 1 #: initializing the straight line path
    for (int proj_phase=0;proj_phase<this->max_num_proj;proj_phase++)
    {
        int init_line_size = this->init_proj_center_points[0].size();

        //debug
        RCLCPP_INFO(this->get_logger(), "init_line_size: %d", init_line_size);

        for (int proj_line_index=0; proj_line_index<init_line_size; proj_line_index+=(int)std::pow(max_num_proj_line, max_num_proj-proj_phase))
        {
            
            // start to project each pixel of project map;
            // lati which is mean latitude;
            int init_latis_size = std::ceil(init_proj_map_view_angle/init_proj_map_lati_step/2.0);
            double temp_init_lati_step = init_proj_map_view_angle/2.0/(double)init_latis_size;
            int init_longis_size = std::ceil(360.0/init_proj_map_longi_step);
            double temp_init_longi_step = 360.0/(double)init_longis_size;         

            //debug
            RCLCPP_INFO(this->get_logger(), "init_longis_size: %d", init_longis_size);
            //debug
            RCLCPP_INFO(this->get_logger(), "init_latis_size: %d", init_latis_size);

            // init list for check the longitude list that needed to traverse
            std::vector<bool> longi_check_list(init_longis_size, false);
            // init list for record info longitude of 
            std::vector<double> in_proj_step_num_list(init_longis_size, 0.0),out_proj_step_num_list(init_longis_size, 0.0);
            std::vector<double> max_proj_depth_list(init_longis_size, 0.0);

            for (int lati_id=0; lati_id<init_latis_size;lati_id++)
            {
                // longi which is mean longitude
                int longi_step = 1;
                if (lati_id < 2*init_latis_size/3)
                {
                    longi_step = lati_id < init_latis_size/3 ? 3 : 2;
                }
                for (int longi_id=0; longi_id<init_longis_size; longi_id+=longi_step)
                {
                    
                }
            }               
        }
 
    }

    // # STEP 2 #: initializing B spline control point on the straight line and optimize.

    // # STEP 3 #: collision check and optimize.
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