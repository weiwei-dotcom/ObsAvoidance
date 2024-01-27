#include "planner.hpp"

PathPlanner::PathPlanner():Node("path_planner")
{
    flag_get_grid_map = false;
    flag_get_plan_start = false;
    flag_finish_planning = false;

    this->order = 3;

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
    this->declare_parameter<double_t>("control_point_dist", 20.0);
    this->control_point_dist = this->get_parameter("control_point_dist").as_double();

    this->declare_parameter<double_t>("extension_ratio", 1.4);
    this->extension_ratio = this->get_parameter("extension_ratio").as_double();

    this->declare_parameter<double_t>("min_plan_dist", 60.0);
    this->min_plan_dist = this->get_parameter("min_plan_dist").as_double();  

    this->declare_parameter<double_t>("max_control_point_dist", 40.0);
    this->max_control_point_dist = this->get_parameter("max_control_point_dist").as_double(); 
      
    
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
    bool success = planInitTraj(start_pos, start_vel, Eigen::Vector3d::Zero(), end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to generate Init Path !");

        rclcpp::shutdown();
    }
    if (flag_finish_planning)
    {
        return;
    }
    
    // # STEP 2 #: Initializing control point on polynomial path.
    initControlPoints();

    // # STEP 3 #: Collision checkout and optimization until the control points set free with collision.
    getCollisionSegId();
    
    /// TODO:

    return;
}

bool PathPlanner::planInitTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                               const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
    if ((start_pos - end_pos).norm() < min_plan_dist)
    {
        RCLCPP_INFO(this->get_logger(), "the start position is closed to target position, don't need plan !");
        this->flag_finish_planning = true;
        return true;
    }
    
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

void PathPlanner::initControlPoints()
{
    double ts = control_point_dist/average_speed/extension_ratio;
    ts*=1.5;
    bool flag_too_far = false;
    bspline_interp_pt.clear();
    start_end_derivatives.clear();
    do 
    {
        ts/=1.5;
        bspline_interp_pt.clear();
        flag_too_far = false;
        double time_sum = init_poly_path.getTimeSum();
        Eigen::Vector3d last_pt = init_poly_path.evaluate(0.0);
        bspline_interp_pt.push_back(last_pt);
        for (double t=ts;t<time_sum;t+=ts)
        {
            Eigen::Vector3d pt = this->init_poly_path.evaluate(ts);
            if ((pt - last_pt).norm() > max_control_point_dist)
            {
                flag_too_far = true;
                break;
            }
            this->bspline_interp_pt.push_back(pt);
            last_pt = pt;
        }
    }while(flag_too_far || this->bspline_interp_pt.size()<7);

    bspline_interp_pt.push_back(end_pos);
    if ((bspline_interp_pt.back()-bspline_interp_pt[bspline_interp_pt.size()-2]).norm() < 10.0)
    {
        bspline_interp_pt[bspline_interp_pt.size()-2] = bspline_interp_pt.back();
        bspline_interp_pt.pop_back();
    }

    start_end_derivatives.push_back(this->start_vel);
    start_end_derivatives.push_back(Eigen::Vector3d(0,0,0));
    start_end_derivatives.push_back(Eigen::Vector3d(0,0,0));
    start_end_derivatives.push_back(Eigen::Vector3d(0,0,0));
    UniformBspline::parameterizeToBspline(ts, bspline_interp_pt, start_end_derivatives, ctrl_pts);

    return;
}

void PathPlanner::getCollisionSegId()
{
    /*** Segment the initial trajectory according to obstacles ***/
    constexpr int ENOUGH_INTERVAL = 2;
    // step_size is a ratio value
    double step_size = resolution / ((ctrl_pts.col(0) - ctrl_pts.rightCols(1)).norm() / (ctrl_pts.cols() - 1)) / 2;
    int in_id, out_id;
    int same_occ_state_times = ENOUGH_INTERVAL + 1;
    bool occ, last_occ = false;
    bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
    int i_end = (int)ctrl_pts.cols() - order;
    for (int i = order; i <= i_end; ++i)
    {
        for (double a = 1.0; a >= 0.0; a -= step_size)
        {
            occ = checkCollision(a * ctrl_pts.col(i - 1) + (1 - a) * ctrl_pts.col(i));

            if (occ && !last_occ)
            {
                if (same_occ_state_times > ENOUGH_INTERVAL || i == order)
                {
                    in_id = i - 1;
                    flag_got_start = true;
                }
                same_occ_state_times = 0;
                flag_got_end_maybe = false; // terminate in advance
            }
            else if (!occ && last_occ)
            {
                out_id = i;
                flag_got_end_maybe = true;
                same_occ_state_times = 0;
            }
            else
            {
                ++same_occ_state_times;
            }

            if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)ctrl_pts.cols() - order)))
            {
                flag_got_end_maybe = false;
                flag_got_end = true;
            }

            last_occ = occ;
            if (flag_got_start && flag_got_end)
            {
                flag_got_start = false;
                flag_got_end = false;
                segment_ids.push_back(std::pair<int, int>(in_id, out_id));
            }
        }
    }

    /*** a star search ***/
    vector<vector<Eigen::Vector3d>> a_star_pathes;
    for (size_t i = 0; i < segment_ids.size(); ++i)
    {
        //cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
        Eigen::Vector3d in(ctrl_pts.col(segment_ids[i].first)), out(ctrl_pts.col(segment_ids[i].second));
        /// TODO:
        if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
        {
            a_star_pathes.push_back(a_star_->getPath());
        }
        else
        {
            ROS_ERROR("a star error, force return!");
            return a_star_pathes;
        }
    }

    /*** calculate bounds ***/
    int id_low_bound, id_up_bound;
    vector<std::pair<int, int>> bounds(segment_ids.size());
    for (size_t i = 0; i < segment_ids.size(); i++)
    {

        if (i == 0) // first segment
        {
            id_low_bound = this->order;
            if (segment_ids.size() > 1)
            {
                id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
            }
            else
            {
                id_up_bound = ctrl_pts.cols() - this->order - 1;
            }
        }
        else if (i == segment_ids.size() - 1) // last segment, i != 0 here
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
            id_up_bound = ctrl_pts.cols() - order - 1;
        }
        else
        {
            id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
            id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
        }

        bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
    }

    // cout << "+++++++++" << endl;
    // for ( int j=0; j<bounds.size(); ++j )
    // {
    //   cout << bounds[j].first << "  " << bounds[j].second << endl;
    // }

    /*** Adjust segment length ***/
    vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
    constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
    int minimum_points = round(ctrl_pts.cols() * MINIMUM_PERCENT), num_points;
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
        /*** Adjust segment length ***/
        num_points = segment_ids[i].second - segment_ids[i].first + 1;
        //cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
        if (num_points < minimum_points)
        {
            double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

            final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

            final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
        }
        else
        {
            final_segment_ids[i].first = segment_ids[i].first;
            final_segment_ids[i].second = segment_ids[i].second;
        }

      //cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
    }

    /*** Assign data to each segment ***/
    for (size_t i = 0; i < segment_ids.size(); i++)
    {
        // step 1
        for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
            this->temp_flags[j] = false;

        // step 2
        int got_intersection_id = -1;
        for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
        {
            Eigen::Vector3d ctrl_pts_law(ctrl_pts.col(j + 1) - ctrl_pts.col(j - 1)), intersection_point;
            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            double val = (a_star_pathes[i][Astar_id] - ctrl_pts.col(j)).dot(ctrl_pts_law), last_val = val;
            while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
            {
                last_Astar_id = Astar_id;

                if (val >= 0)
                    --Astar_id;
                else
                    ++Astar_id;

                val = (a_star_pathes[i][Astar_id] - ctrl_pts.col(j)).dot(ctrl_pts_law);

                if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                {
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(ctrl_pts.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                        );

                    //cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

                    got_intersection_id = j;
                    break;
                }
            }

            if (got_intersection_id >= 0)
            {
                this->temp_flags[j] = true;
                double length = (intersection_point - ctrl_pts.col(j)).norm();
                if (length > 1e-5)
                {
                    for (double a = length; a >= 0.0; a -= resolution)
                    {
                        occ = checkCollision((a / length) * intersection_point + (1 - a / length) * ctrl_pts.col(j));

                        if (occ || a < resolution)
                        {
                            if (occ)
                            a += resolution;
                            this->base_pts[j].push_back((a / length) * intersection_point + (1 - a / length) * ctrl_pts.col(j));
                            this->esc_directions[j].push_back((intersection_point - ctrl_pts.col(j)).normalized());
                            break;
                        }
                    }
                }
            }
        }

      /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
        if (segment_ids[i].second - segment_ids[i].first == 1)
        {
            Eigen::Vector3d ctrl_pts_law(ctrl_pts.col(segment_ids[i].second) - ctrl_pts.col(segment_ids[i].first)), intersection_point;
            Eigen::Vector3d middle_point = (ctrl_pts.col(segment_ids[i].second) + ctrl_pts.col(segment_ids[i].first)) / 2;
            int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
            double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
            while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
            {
                last_Astar_id = Astar_id;

                if (val >= 0)
                    --Astar_id;
                else
                    ++Astar_id;

                val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                {
                    intersection_point =
                        a_star_pathes[i][Astar_id] +
                        ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                        (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                        );

                    if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
                    {

                        /// FLAG: You should consider if you use the ControlPoint class.
                        this->temp_flags[segment_ids[i].first] = true;
                        this->base_pts[segment_ids[i].first].push_back(ctrl_pts.col(segment_ids[i].first));
                        this->esc_directions[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

                        got_intersection_id = segment_ids[i].first;
                    }
                    break;
                }
            }
        }

        //step 3
        if (got_intersection_id >= 0)
        {
            for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
            if (!this->temp_flags[j])
            {
                this->base_pts[j].push_back(this->base_pts[j - 1].back());
                this->esc_directions[j].push_back(this->esc_directions[j - 1].back());
            }

            for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
            if (!this->temp_flags[j])
            {
                this->base_pts[j].push_back(this->base_pts[j + 1].back());
                this->esc_directions[j].push_back(this->esc_directions[j + 1].back());
            }
        }
        else
        {
            // Just ignore, it does not matter ^_^.
            // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
        }
    }
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

bool PathPlanner::checkCollision(const Eigen::Vector3d &coor)
{
    Eigen::Vector3i temp_index = this->coorToIndex(coor);
    return this->grid_map[temp_index.x()][temp_index.y()][temp_index.z()];
}

Eigen::Vector3i PathPlanner::coorToIndex(const Eigen::Vector3d &coor)
{
    Eigen::Vector3i temp_index;
    temp_index.x() = std::floor((coor.x()-this->grid_map_origin_point.x())/resolution);
    temp_index.y() = std::floor((coor.y()-this->grid_map_origin_point.y())/resolution);
    temp_index.z() = std::floor((coor.z()-this->grid_map_origin_point.z())/resolution);
    return temp_index;
}
