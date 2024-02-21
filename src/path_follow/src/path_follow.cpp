#include "path_follow.hpp"

PathFollow::PathFollow():rclcpp::Node("path_follow")
{
    // 初始化判断标志变量
    this->flag_get_first_path = false;
    this->flag_called_not_response = false;

    // 获取运捕坐标系系统转换到障碍物参考坐标系的变换矩阵
    this->declare_parameter<double_t>("world_to_ref_pos_x",0.0);
    this->declare_parameter<double_t>("world_to_ref_pos_y",0.0);
    this->declare_parameter<double_t>("world_to_ref_pos_z",0.0);
    this->declare_parameter<double_t>("world_to_ref_orien_x",0.0);
    this->declare_parameter<double_t>("world_to_ref_orien_y",0.0);
    this->declare_parameter<double_t>("world_to_ref_orien_z",0.0);
    this->declare_parameter<double_t>("world_to_ref_orien_w",1.0);
    Eigen::Quaterniond temp_q(this->get_parameter("world_to_ref_orien_w").as_double(),
                              this->get_parameter("world_to_ref_orien_x").as_double(),
                              this->get_parameter("world_to_ref_orien_y").as_double(),
                              this->get_parameter("world_to_ref_orien_z").as_double());
    Sophus::Vector3d temp_trans(this->get_parameter("world_to_ref_pos_x").as_double(),
                                this->get_parameter("world_to_ref_pos_y").as_double(),
                                this->get_parameter("world_to_ref_pos_z").as_double());
    Sophus::SE3d world_to_ref_sophus(temp_q,temp_trans);
    this->world_to_ref = world_to_ref_sophus.matrix();
    
    // 初始化直线离散点
    Eigen::Vector4d temp_stroke_start, temp_stroke_end;
    this->declare_parameter<double_t>("stroke_start_x", 0.0);
    this->declare_parameter<double_t>("stroke_start_y", 0.0);
    this->declare_parameter<double_t>("stroke_start_z", 0.0);
    this->declare_parameter<double_t>("stroke_end_x", 0.0);
    this->declare_parameter<double_t>("stroke_end_y", 0.0);
    this->declare_parameter<double_t>("stroke_end_z", 0.0);
    temp_stroke_start(0) = this->get_parameter("stroke_start_x").as_double();
    temp_stroke_start(1) = this->get_parameter("stroke_start_y").as_double();
    temp_stroke_start(2) = this->get_parameter("stroke_start_z").as_double();
    temp_stroke_start(3) = 1.0;
    temp_stroke_end(0) = this->get_parameter("stroke_end_x").as_double();
    temp_stroke_end(1) = this->get_parameter("stroke_end_y").as_double();
    temp_stroke_end(2) = this->get_parameter("stroke_end_z").as_double();
    temp_stroke_end(3) = 1.0;
    this->stroke_end = (this->world_to_ref * temp_stroke_end).block(0,0,3,1);
    this->stroke_start = (this->world_to_ref * temp_stroke_start).block(0,0,3,1);
    int step_num = std::ceil((stroke_end-stroke_start).norm()/1.0);
    if (step_num == 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Stroke Error !!!");
        rclcpp::shutdown();
        return;
    }
    for(int i=0; i<=step_num; i++)
    {
        Eigen::Vector3d temp_path_point = stroke_start*((double)step_num-(double)i)/(double)step_num + stroke_end*(double)i/(double)step_num;
        this->path_points.push_back(temp_path_point);
    } 
    // 
    return;
}


