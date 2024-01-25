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
    this->start_pos.x() = this->get_parameter("start_position_x").as_double();
    this->start_pos.y() = this->get_parameter("start_position_y").as_double();
    this->start_pos.z() = this->get_parameter("start_position_z").as_double();
    this->start_direction.x() = this->get_parameter("start_direction_x").as_double();
    this->start_direction.y() = this->get_parameter("start_direction_y").as_double();
    this->start_direction.z() = this->get_parameter("start_direction_z").as_double();
    this->average_speed = this->get_parameter("average_speed").as_double();
    this->start_vel = this->start_direction.normalized() * average_speed;

    this->declare_parameter<double_t>("interp_dist_thresh", 200.0);
    this->interp_dist_thresh = this->get_parameter("interp_dist_thresh").as_double();

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
    // TIP: set distance thresh 200mm, use end_front_index become the path plan start point.
    bool succes = planInitTraj(start_pos, start_vel, Eigen::Vector3d::Zero(), end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (!succes)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to generate Init Path !");

        rclcpp::shutdown();
    }
    
    // # STEP 2 #: Initializing control point on polynomial path.


    // # STEP 3 #: Collision checkout and optimization until the control points set free with collision.


    return;
}

bool PathPlanner::planInitTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                               const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
    // generate global reference trajectory
    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        inter_points.push_back(points.at(i));
        double dist = (points.at(i + 1) - points.at(i)).norm();

        if (dist > this->interp_dist_thresh)
        {
            int id_num = floor(dist / interp_dist_thresh) + 1;

            for (int j = 1; j < id_num; ++j)
            {
            Eigen::Vector3d inter_pt =
                points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
            inter_points.push_back(inter_pt);
            }
        }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
        pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    // Allocate time
    for (int i = 0; i < pt_num - 1; ++i)
    {
        time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (this->average_speed);
    }

    // Assume that the speed of the beginning and end stages needs to be increased and decreased
    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    // Calculate the time span and the coefficients of each poly segment, return to the PolynomialTraj class variant.
    if (pos.cols() >= 3)
        this->init_poly_path = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
        this->init_poly_path = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
        return false;
    return true;
}

// Corrects points outside the boundary
void PathPlanner::boundCorrect(int &x, int &y, int &z)
{
    x = std::min(std::max(0,x), grid_map_x_size-1);
    y = std::min(std::max(0,y), grid_map_y_size-1);
    z = std::min(std::max(0,z), grid_map_z_size-1);
    return;
}

