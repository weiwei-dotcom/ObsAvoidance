#include "path_follow.hpp"

PathFollow::PathFollow():rclcpp::Node("path_follow")
{
    // 初始化判断标志变量
    this->flag_get_first_path = false;
    this->flag_called_not_response = false;
    this->flag_close_to_target_point = false;

    // 创建读取配置文件对象
    cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance/src/path_follow/path_follow_config.yaml", cv::FileStorage::READ);
    
    // 获取运捕坐标系系统转换到障碍物参考坐标系的变换矩阵
    Eigen::Quaterniond temp_q(fileRead["world_to_ref_orien_w"],
                              fileRead["world_to_ref_orien_x"],
                              fileRead["world_to_ref_orien_y"],
                              fileRead["world_to_ref_orien_z"]);
    Sophus::Vector3d temp_trans(fileRead["world_to_ref_pos_x"],
                                fileRead["world_to_ref_pos_y"],
                                fileRead["world_to_ref_pos_z"]);
    Sophus::SE3d world_to_ref_sophus(temp_q,temp_trans);
    this->world_to_ref = world_to_ref_sophus.matrix();
    
    // 初始化直线离散点
    Eigen::Vector4d temp_stroke_start, temp_stroke_end;
    temp_stroke_start(0) = fileRead["stroke_start_x"];
    temp_stroke_start(1) = fileRead["stroke_start_y"];
    temp_stroke_start(2) = fileRead["stroke_start_z"];
    temp_stroke_start(3) = 1.0;
    temp_stroke_end(0) = fileRead["stroke_end_x"];
    temp_stroke_end(1) = fileRead["stroke_end_y"];
    temp_stroke_end(2) = fileRead["stroke_end_z"]; 
    temp_stroke_end(3) = 1.0;
    // // 如果在提前预先标定的程序中没将行程位置坐标转换为参考坐标系下，则执行下两句
    // this->stroke_end = (this->world_to_ref * temp_stroke_end).block(0,0,3,1);
    // this->stroke_start = (this->world_to_ref * temp_stroke_start).block(0,0,3,1);
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

    // 声明路径规划客户端
    this->get_path_client = this->create_client<interface::srv::PathPoints>("get_path_points");
    
    // 声明获取离散路径点定时器后，创建定时器
    this->get_path_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&PathFollow::getNewPathCall,this));
    
    return;
}

void PathFollow::fitPathCallback()
{

    return;
}

void PathFollow::getNewPathCall()
{
    // 如果当前末端已接近目标点，不再需要规划新路径，return
    if (flag_close_to_target_point)
    {
        std::cout << "Closed to target point, no need to get new path" << std::endl;
        return;
    }
    // 如果上一次请求还没有处理完毕，则不需要再次发起，return
    if (flag_called_not_response)
    {
        std::cout << "The response to the last request was not completed" << std::endl;
        return;
    }
    // 等待服务端上线
    while(!this->get_path_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "等待路径规划服务端上线......");
    }
    RCLCPP_INFO(this->get_logger(), "路径规划服务端已上线！");
    // 发起获取新离散路径点服务
    auto request = std::make_shared<interface::srv::PathPoints_Request>();
    get_path_client->async_send_request(request, std::bind(&PathFollow::getNewPathCallback, this, std::placeholders::_1));
    // 发起服务未获得响应标志位置为真，待响应回调函数响应且正确获取了离散路径点后复位为假
    this->flag_called_not_response = true;
    return;
}

void PathFollow::getNewPathCallback(rclcpp::Client<interface::srv::PathPoints>::SharedFuture response)
{
    RCLCPP_INFO(this->get_logger(), "已收到来自路径规划节点的离散路径点数据，插入中......")
    auto result = response.get();
    ///TODO:
    // 获取离散路径点得到响应，可以开始下一次服务发起,将发起但未得到响应标志复位为假。
    flag_called_not_response = false;
    // 正确获取了离散路径点后，第一次获取离散路径点判断标志位置为真。
    flag_get_first_path = true;
    return;
}


