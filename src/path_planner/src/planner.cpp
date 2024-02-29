#include "planner.hpp"

PathPlanner::PathPlanner():Node("path_planner")
{
    cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance/src/path_planner/config.yaml", cv::FileStorage::READ);

    ///TODO:
    this->order = 3;

    this->grid_map_origin_point.x() = fileRead["grid_map_origin_point_x"];
    this->grid_map_origin_point.y() = fileRead["grid_map_origin_point_y"];
    this->grid_map_origin_point.z() = fileRead["grid_map_origin_point_z"];

    this->grid_map_x_size = fileRead["grid_map_x_size"];
    this->grid_map_y_size = fileRead["grid_map_y_size"];
    this->grid_map_z_size = fileRead["grid_map_z_size"];
    std::vector<bool> temp_occupy(this->grid_map_z_size, false);
    std::vector<std::vector<bool>> temp_occupy_2d(this->grid_map_y_size, temp_occupy);
    this->grid_map.resize(this->grid_map_x_size, temp_occupy_2d);
    initGridMap(Eigen::Vector3i(grid_map_x_size,grid_map_y_size,grid_map_z_size));

    this->resolution = fileRead["resolution"];

    this->inflation_radius = fileRead["inflation_radius"];

    this->start_pos.x() = fileRead["start_position_x"];
    this->start_pos.y() = fileRead["start_position_y"];
    this->start_pos.z() = fileRead["start_position_z"];
    this->start_direction.x() = fileRead["start_direction_x"];
    this->start_direction.y() = fileRead["start_direction_y"];
    this->start_direction.z() = fileRead["start_direction_z"];
    this->average_speed = fileRead["average_speed"];
    this->start_vel = this->start_direction.normalized() * average_speed;

    this->interp_dist_thresh = fileRead["interp_dist_thresh"];
    this->control_point_dist = fileRead["control_point_dist"];

    this->extension_ratio = fileRead["extension_ratio"];

    this->min_plan_dist = fileRead["min_plan_dist"];  

    this->max_control_point_dist = fileRead["max_control_point_dist"]; 
      
    
    return;
}

// This function will get the pcl of obstacle and change to the 3d grid map that used in collision detection.
void PathPlanner::pclToGridMap(const pcl::PointCloud<pcl::PointXYZ> &obs_pcl)
{
    int pcl_obs_size = obs_pcl.points.size();
    int temp_index_x,temp_index_y,temp_index_z;
    for (int i=0;i<pcl_obs_size;i++)
    {
        temp_index_x = floor((obs_pcl.points[i].x-this->grid_map_origin_point.x())/this->resolution);
        temp_index_y = floor((obs_pcl.points[i].y-this->grid_map_origin_point.y())/this->resolution);
        temp_index_z = floor((obs_pcl.points[i].z-this->grid_map_origin_point.z())/this->resolution);
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
    return;
}

void PathPlanner::collisionCheckCallback()
{
    // TODO:
    return;
}

// init the straight line path that for the 
void PathPlanner::replanPath()
{   
    // # STEP 1 #: Initializing global polynomial path.
    // TIP: set distance thresh 200mm, use end_front_index become the path plan start point.
    bool success = planInitTraj(start_pos, start_vel, Eigen::Vector3d::Zero(), end_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    if (!success)
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to generate Init Path !");
        rclcpp::shutdown();
    }
    
    initControlPoints();

    resize(ctrl_pts.size());
    
    getBasePointsAndDirection();

    Optimize();



    return;
}

bool PathPlanner::planInitTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                               const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
    if ((start_pos - end_pos).norm() < min_plan_dist)
    {
        RCLCPP_INFO(this->get_logger(), "the start position is closed to target position, don't need plan !");
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

void PathPlanner::resize(const int size_set)
{
    this->base_pts.clear();
    this->esc_directions.clear();
    this->temp_flags.clear();
    // occupancy.clear();

    this->base_pts.resize(size_set);
    this->esc_directions.resize(size_set);
    this->temp_flags.resize(size_set);
    // occupancy.resize(size);
    return;
}

void PathPlanner::getBasePointsAndDirection()
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
        if (AstarSearch(/*(in-out).norm()/10+0.05*/ 10.0, in, out))
        {
            a_star_pathes.push_back(this->getPath());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "a star error, force return!");
            return;
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

void PathPlanner::Optimize()
{

    return;
}

bool PathPlanner::check_collision_and_rebound()
{

    int end_idx = this->ctrl_pts.size() - order;

    /*** Check and segment the initial trajectory according to obstacles ***/
    int in_id, out_id;
    vector<std::pair<int, int>> segment_ids;
    bool flag_new_obs_valid = false;
    int i_end = end_idx - (end_idx - order) / 3;
    for (int i = order - 1; i <= i_end; ++i)
    {

        bool occ = checkCollision(ctrl_pts.col(i));

        /*** check if the new collision will be valid ***/
        if (occ)
        {
            for (size_t k = 0; k < this->esc_directions[i].size(); ++k)
            {
                cout.precision(2);
                if ((ctrl_pts.col(i) - this->base_pts[i][k]).dot(esc_directions[i][k]) < 1 * resolution) // current point is outside all the collision_points.
                {
                    occ = false; // Not really takes effect, just for better hunman understanding.
                    break;
                }
            }
        }

        if (occ)
        {
            flag_new_obs_valid = true;

            int j;
            for (j = i - 1; j >= 0; --j)
            {
                occ = checkCollision(ctrl_pts.col(j));
                if (!occ)
                {
                    in_id = j;
                    break;
                }
            }
            if (j < 0) // fail to get the obs free point
            {
                RCLCPP_ERROR(this->get_logger(), "ERROR! the drone is in obstacle. This should not happen.");
                in_id = 0;
            }

            for (j = i + 1; j < ctrl_pts.size(); ++j)
            {
                occ = checkCollision(ctrl_pts.col(j));

                if (!occ)
                {
                    out_id = j;
                    break;
                }
            }
            if (j >= ctrl_pts.size()) // fail to get the obs free point
            {
                RCLCPP_ERROR(this->get_logger(), "WARN! terminal point of the current trajectory is in obstacle, skip this planning.");
                force_stop_type_ = STOP_FOR_ERROR;
                return false;
            }

            i = j + 1;

            segment_ids.push_back(std::pair<int, int>(in_id, out_id));
        }
    }

    if (flag_new_obs_valid)
    {
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            /*** a star search ***/
            Eigen::Vector3d in(ctrl_pts.col(segment_ids[i].first)), out(ctrl_pts.col(segment_ids[i].second));
            if (this->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
            {
                a_star_pathes.push_back(this->getPath());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "a star error");
                segment_ids.erase(segment_ids.begin() + i);
                i--;
            }
        }

        /*** Assign parameters to each segment ***/
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            // step 1
            for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
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

                    // cout << val << endl;

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                    {
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                            (ctrl_pts_law.dot(ctrl_pts.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                            );

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
                            bool occ = this->checkCollision((a / length) * intersection_point + (1 - a / length) * ctrl_pts.col(j));

                            if (occ || a < resolution)
                            {
                                if (occ)
                                a += resolution;
                                base_pts[j].push_back((a / length) * intersection_point + (1 - a / length) * ctrl_pts.col(j));
                                esc_directions[j].push_back((intersection_point - ctrl_pts.col(j)).normalized());
                                break;
                            }
                        }
                    }
                    else
                    {
                        got_intersection_id = -1;
                    }
                }
            }

            //step 3
            if (got_intersection_id >= 0)
            {
                for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                    if (!this->temp_flags[j])
                    {
                        this->base_pts[j].push_back(this->base_pts[j - 1].back());
                        this->esc_directions[j].push_back(this->esc_directions[j - 1].back());
                    }

                for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                    if (!this->temp_flags[j])
                    {
                        this->base_pts[j].push_back(this->base_pts[j + 1].back());
                        this->esc_directions[j].push_back(this->esc_directions[j + 1].back());
                    }
            }
            else
                RCLCPP_ERROR(this->get_logger(), "Failed to generate direction. It doesn't matter.");
        }

        force_stop_type_ = STOP_FOR_REBOUND;
        return true;
    }

    return false;
}

std::vector<Eigen::Vector3d> PathPlanner::getPath()
{
    std::vector<Eigen::Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord_a_star(ptr->index));

    std::reverse(path.begin(), path.end());
    return path;
}

void PathPlanner::initGridMap(/* GridMap::Ptr occ_map, */const Eigen::Vector3i pool_size)
{
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    // grid_map_ = occ_map;
}

bool PathPlanner::AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt)
{
    rclcpp::Time time_1 = this->now();
    ++rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2;

    Eigen::Vector3i start_idx, end_idx;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
    {
        RCLCPP_ERROR(this->get_logger(), "Unable to handle the initial or end point, force return!");
        return false;
    }

    // if ( start_pt(0) > -1 && start_pt(0) < 0 )
    //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

    GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    openSet_.swap(empty);

    GridNodePtr neighborPtr = NULL;
    GridNodePtr current = NULL;

    startPtr->index = start_idx;
    startPtr->rounds = rounds_;
    startPtr->gScore = 0;
    startPtr->fScore = getHeu(startPtr, endPtr);
    startPtr->state = GridNode::OPENSET; //put start node in open set
    startPtr->cameFrom = NULL;
    openSet_.push(startPtr); //put start in open set

    endPtr->index = end_idx;

    double tentative_gScore;

    int num_iter = 0;
    while (!openSet_.empty())
    {
        num_iter++;
        current = openSet_.top();
        openSet_.pop();

        // if ( num_iter < 10000 )
        //     cout << "current=" << current->index.transpose() << endl;

        if (current->index(0) == endPtr->index(0) && current->index(1) == endPtr->index(1) && current->index(2) == endPtr->index(2))
        {
            // ros::Time time_2 = ros::Time::now();
            // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
            // if((time_2 - time_1).toSec() > 0.1)
            //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
            gridPath_ = retrievePath(current);
            return true;
        }
        current->state = GridNode::CLOSEDSET; //move current node from open set to closed set.

        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++)
                for (int dz = -1; dz <= 1; dz++)
                {
                    if (dx == 0 && dy == 0 && dz == 0)
                        continue;

                    Eigen::Vector3i neighborIdx;
                    neighborIdx(0) = (current->index)(0) + dx;
                    neighborIdx(1) = (current->index)(1) + dy;
                    neighborIdx(2) = (current->index)(2) + dz;

                    if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
                    {
                        continue;
                    }

                    neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
                    neighborPtr->index = neighborIdx;

                    bool flag_explored = neighborPtr->rounds == rounds_;

                    if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighborPtr->rounds = rounds_;

                    if (checkCollision(Index2Coord_a_star(neighborPtr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighborPtr->state = GridNode::OPENSET;
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                        openSet_.push(neighborPtr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighborPtr->gScore)
                    { //in open set and need update
                        neighborPtr->cameFrom = current;
                        neighborPtr->gScore = tentative_gScore;
                        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
                    }
                }
        rclcpp::Time time_2 = this->now();
        if ((double)(time_2 - time_1).nanoseconds()/1e9 > 0.2)
        {
            RCLCPP_WARN(this->get_logger(), "Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }

    rclcpp::Time time_2 = this->now();

    if ((time_2 - time_1).nanoseconds()/1e9 > 0.1)
        RCLCPP_WARN(this->get_logger(), "Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).seconds(), num_iter);

    return false;
}

bool PathPlanner::ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, Eigen::Vector3i &start_idx, Eigen::Vector3i &end_idx)
{
    if (!Coord2Index_a_star(start_pt, start_idx) || !Coord2Index_a_star(end_pt, end_idx))
        return false;

    if (checkCollision(Index2Coord_a_star(start_idx)))
    {
        //ROS_WARN("Start point is inside an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index_a_star(start_pt, start_idx))
                return false;
        } while (checkCollision(Index2Coord_a_star(start_idx)));
    }

    if (checkCollision(Index2Coord_a_star(end_idx)))
    {
        //ROS_WARN("End point is inside an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index_a_star(end_pt, end_idx))
                return false;
        } while (checkCollision(Index2Coord_a_star(end_idx)));
    }

    return true;
}

inline double PathPlanner::getHeu(GridNodePtr node1, GridNodePtr node2)
{
	return tie_breaker_ * getDiagHeu(node1, node2);
}

double PathPlanner::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

vector<GridNodePtr> PathPlanner::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool PathPlanner::optFirstSeg()
{
    ceres::Problem pathOptimizer;

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
    Eigen::Vector3i temp_index;
    bool flag = this->coorToIndex(coor, temp_index);
    if(!flag)
    {
        RCLCPP_ERROR(this->get_logger(), "exceed the bound of map");
    }
    return this->grid_map[temp_index.x()][temp_index.y()][temp_index.z()];
}

bool PathPlanner::coorToIndex(const Eigen::Vector3d &coor, Eigen::Vector3i &index)
{
    Eigen::Vector3i index;
    if (coor.x() < grid_map_origin_point.x() || coor.y() < grid_map_origin_point.y() ||coor.z() < grid_map_origin_point.z()
        || coor.x() > grid_map_origin_point.x()+(double)grid_map_x_size*resolution 
        || coor.y() > grid_map_origin_point.y()+(double)grid_map_y_size*resolution
        || coor.z() > grid_map_origin_point.z()+(double)grid_map_z_size*resolution)
    {
        RCLCPP_ERROR(this->get_logger(), "exceed the bound of map");
        return false;
    }
    index.x() = std::floor((coor.x()-this->grid_map_origin_point.x())/resolution);
    index.y() = std::floor((coor.y()-this->grid_map_origin_point.y())/resolution);
    index.z() = std::floor((coor.z()-this->grid_map_origin_point.z())/resolution);
    return true;
}
