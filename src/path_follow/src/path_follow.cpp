#include "path_follow.hpp"

PathFollow::PathFollow():rclcpp::Node("path_follow")
{
    // 初始化判断标志变量
    this->flag_get_first_path = false;
    this->flag_called_not_response = false;
    this->flag_no_need_to_replan = false;
    this->flag_arrived_target = false;

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

    // 开始时，将路径规划起点索引置为倒数path_points序列倒数第二
    this->replan_start_id = path_points.size()-2;
    // 路径跟随的推进速度大小
    speed_value = fileRead["speed_value"];
    
    // 通过输入动捕标记坐标，获取基座相对于参考坐标系姿态并计算其在行程上的位置，来确定初始基座初始位姿（注意是在行程上的位置而不是直接位置）
    std::vector<Eigen::Vector3d> flag_points(3,Eigen::Vector3d(0,0,0));
    std::cout << "请输入上方标记小球的位置坐标: " << std::endl;
    std::cin >> flag_points[0].x();
    std::cin >> flag_points[0].y();
    std::cin >> flag_points[0].z();
    std::cout << "请输入右下方标记小球的位置坐标: " << std::endl;
    std::cin >> flag_points[1].x();
    std::cin >> flag_points[1].y();
    std::cin >> flag_points[1].z();
    std::cout << "请输入左下方标记小球的位置坐标: " << std::endl;
    std::cin >> flag_points[2].x();
    std::cin >> flag_points[2].y();
    std::cin >> flag_points[2].z();
    this->trans_base_ref = calFrame(flag_points);
    Eigen::Vector3d temp_base_position = trans_base_ref.block(0,3,3,1);
    this->init_base_position = getIntervalPoint(temp_base_position, stroke_end,stroke_start);
    
    // 创建路径规划客户端
    this->get_path_client = this->create_client<interface::srv::PathPoints>("get_path_points");
    // 创建通知enable_follow客户端
    this->enable_follow_client = this->create_client<interface::srv::EnableFollow>("enable_follow");
    // 创建获取离散路径点定时器后，创建定时器
    this->get_path_timer = this->create_wall_timer(std::chrono::seconds(2), std::bind(&PathFollow::getNewPathCall,this));
    // 创建路径拟合服务端
    this->fit_path_server = this->create_service<interface::srv::BaseJointMotorValue>("fit_path", std::bind(&PathFollow::fitPathCallback,this));

    // 

    return;
}

void PathFollow::fitPathCallback()
{
    ///TODO: 编写拟合路径服务回调函数函数
    // 未获得第一条离散路径点，直接返回
    if (!flag_get_first_path)
    {
        RCLCPP_INFO(this->get_logger(),"还未得到第一条离散化路径点，不能开始拟合！！");
        return;
    }
    // 已经到达目标终点附近，直接返回
    if (flag_arrived_target)
    {
        RCLCPP_INFO(this->get_logger(), "已经到达目标点，停止拟合！！");
        return;
    }

    return;
}

void PathFollow::getNewPathCall()
{
    // 如果当前末端已接近目标点，不再需要规划新路径，return
    if (flag_no_need_to_replan)
    {
        RCLCPP_WARN(this->get_logger(), "末端靠近目标点，不需要规划新路径！！");
        return;
    }
    // 如果当前末端已抵达目标点，不在需要规划新路径，return
    if (flag_arrived_target)
    {
        RCLCPP_WARN(this->get_logger(), "已抵达目标点，不需要规划新路径！！");
        return;
    }
    // 如果上一次请求还没有处理完毕，则不需要再次发起，return
    if (flag_called_not_response)
    {
        RCLCPP_WARN(this->get_logger(), "上次规划新路径请求未被响应，不能重复发起！！");
        return;
    }
    // 等待服务端上线
    while(!this->get_path_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "等待路径规划服务端上线......");
    }
    // 发起获取新离散路径点服务
    auto request = std::make_shared<interface::srv::PathPoints_Request>();
    // 根据离散路径点序列对路径规划起始位置以及速度进行赋值
    request->start_position.x = this->path_points[replan_start_id].x();
    request->start_position.y = this->path_points[replan_start_id].y();
    request->start_position.z = this->path_points[replan_start_id].z();
    request->start_speed.x = (this->path_points[replan_start_id+1]- path_points[replan_start_id-1]).x()*speed_value;
    request->start_speed.y = (this->path_points[replan_start_id+1]- path_points[replan_start_id-1]).y()*speed_value;
    request->start_speed.z = (this->path_points[replan_start_id+1]- path_points[replan_start_id-1]).z()*speed_value;
    get_path_client->async_send_request(request, std::bind(&PathFollow::getNewPathCallback, this, std::placeholders::_1));
    // 发起服务未获得响应标志位置为真，待响应回调函数响应且正确获取了离散路径点后复位为假
    this->flag_called_not_response = true;
    return;
}

void PathFollow::getNewPathCallback(rclcpp::Client<interface::srv::PathPoints>::SharedFuture response)
{
    // 获取离散路径点得到响应，可以开始下一次服务发起，将发起但未得到响应标志复位
    flag_called_not_response = false;
    auto result = response.get();
    // 如果服务消息接口success标志位为假，那么没能成功收到新离散路径点，直接返回等待下一次的请求
    if (!result->success) 
    {
        RCLCPP_WARN(this->get_logger(), "路径规划节点出错，未能成功获取新离散路径点！！！");
        return;
    }
    RCLCPP_INFO(this->get_logger(), "成功收到来自路径规划节点的离散路径点数据，插入中......");
    // 根据呼叫服务时的路径点序列索引对path_points进行插入删除
    this->path_points.erase(path_points.begin()+this->replan_start_id+1, path_points.end());
    // 从path_points中路径规划点索引值开始将收到的离散路径点逐个插入
    
    ///DEBUG:
    geometry_msgs::msg::Point temp_point = result->path_points[0];
    RCLCPP_INFO(this->get_logger(), "获取到的第一个离散路径点为：%f, %f, %f", temp_point.x, temp_point.y,temp_point.z);
    RCLCPP_INFO(this->get_logger(), "当前离散路径点序列最后一个点为：%f, %f, %f",
                path_points.back().x(),path_points.back().y(),path_points.back().z());
    
    for (geometry_msgs::msg::Point point:result->path_points)
    {   
        Eigen::Vector3d temp_path_point(point.x, point.y, point.z);
        this->path_points.emplace_back(temp_path_point);        
    }
    if (!flag_get_first_path) 
    {
        RCLCPP_INFO(this->get_logger(),"已获取第一条规划路径,通知motor_control节点可以开始请求获取跟随动作了");
        this->enableFollowCall();
    }
    // 正确获取了离散路径点后，第一次获取离散路径点判断标志位置为真
    flag_get_first_path = true;
    return;
}

void PathFollow::enableFollowCall()
{
    while (!this->enable_follow_client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(),"等待motor_control的enable_follow服务端上线");
    }
    RCLCPP_INFO(this->get_logger(), "enable_follow服务端已上线");
    auto request = std::make_shared<interface::srv::EnableFollow_Request>();
    enable_follow_client->async_send_request(request, std::bind(&PathFollow::enableFollowCallback,this,std::placeholders::_1));
    return;
}

void PathFollow::enableFollowCallback(rclcpp::Client<interface::srv::EnableFollow>::SharedFuture response)
{
    RCLCPP_INFO(this->get_logger(),"motor_control节点已收到通知");
    return;
}


