
// TODO: 修改配置文件让获取尺度因子的时间更长一点，精确度更高点;
#include "media.hpp"

using MoveAction = interface::action::Move;
using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;

mediaNode::mediaNode() : Node("media")
{
	flag_trouble=false;
	// the flag of finish the process of initialization
	flag_haveInitialized=false;
	flag_getScaleFactor = false;
	flag_getTransformToBase = false;
	flag_slamInitialized=false;
	flag_timeout=false;
	scaleFactor_slamToWorld = 1.0;

	cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance/src/config.yaml", cv::FileStorage::READ);
	scaleFact_mmToTarget = fileRead["scaleFact_mmToTarget"];
	cv::Mat Tmat_camera_to_tool(4,4,CV_64F);
	fileRead["T_camera_to_tool"] >> Tmat_camera_to_tool;
	cv::cv2eigen(Tmat_camera_to_tool, T_camera_to_tool);
	//TODO: The measure unit of T_camera_to_tool's translation is unknown, it should be transformed to the target measure unit
	T_camera_to_tool.block(0,3,3,1) = T_camera_to_tool.block(0,3,3,1)*scaleFact_mmToTarget;
	TimeOut = fileRead["TimeOut"];
	double fx = fileRead["Camera.fx"];
	double fy = fileRead["Camera.fy"];
	double cx = fileRead["Camera.cx"];
	double cy = fileRead["Camera.cy"];
	m_projectMatrix << fx,  0, cx,
						0, fy, cy,
						0,  0,  1; 

	// TODO: Transform the target vector to robot msg;
	//     	 The goal position and direction is describe at base frame;
	Eigen::Vector3d goal_direction, goal_position;
	goal_direction(0)=fileRead["goal_direction.x"];
	goal_direction(1)=fileRead["goal_direction.y"];
	goal_direction(2)=fileRead["goal_direction.z"];
	m_goal_pose.position.x=fileRead["goal_position.x"];
	m_goal_pose.position.x*=scaleFact_mmToTarget;
	m_goal_pose.position.y=fileRead["goal_position.y"];
	m_goal_pose.position.y*=scaleFact_mmToTarget;
	m_goal_pose.position.z=fileRead["goal_position.z"];
	m_goal_pose.position.z*=scaleFact_mmToTarget;
	Eigen::Matrix3d goalRotationMatrix = Eigen::AngleAxisd(acos(goal_direction.normalized().dot(Eigen::Vector3d(0,1,0))),
															 Eigen::Vector3d(0,1,0).cross(goal_direction).normalized()).toRotationMatrix();
	Eigen::Quaterniond goalQuaternion(goalRotationMatrix);
	m_goal_pose.orientation.w = goalQuaternion.w();
	m_goal_pose.orientation.x = goalQuaternion.x();
	m_goal_pose.orientation.y = goalQuaternion.y();
	m_goal_pose.orientation.z = goalQuaternion.z();

	T_base_to_world = Eigen::Matrix4d::Identity();
	T_base_to_world(0,3)=fileRead["T_base_to_world.position.x"];
	T_base_to_world(0,3)*=scaleFact_mmToTarget;
	T_base_to_world(1,3)=fileRead["T_base_to_world.position.y"];
	T_base_to_world(1,3)*=scaleFact_mmToTarget;
	T_base_to_world(2,3)=fileRead["T_base_to_world.position.z"];
	T_base_to_world(2,3)*=scaleFact_mmToTarget;

	this->slamInitializedFlag_cli = this->create_client<interface::srv::SlamInitialized>("slam_initialization");
	this->move_cdcr_cli = rclcpp_action::create_client<interface::action::Move>(this,"move_cdcr");
	slam_sub = this->create_subscription<interface::msg::Slam>("slam", 10, std::bind(&mediaNode::slam_callback, this, std::placeholders::_1));
	pub_T_init_to_cur=this->create_publisher<geometry_msgs::msg::PoseStamped>("transform_initToCur", 5);
	pub_T_cur_to_init=this->create_publisher<geometry_msgs::msg::PoseStamped>("transform_curToInit", 5);
	pub_point_cloud=this->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud_initFrame", 5);
}

void mediaNode::initialization()
{
	rclcpp::Rate timer(1);
	initializeSlam();
	while(!flag_slamInitialized)
	{
		if(flag_trouble||flag_timeout)
		{
			rclcpp::shutdown();
			return; 
		}    
		timer.sleep();
	}
	getSlamToWorldScaleFact();
	while(!flag_getScaleFactor);
	{
		if(flag_trouble||flag_timeout)
		{
			rclcpp::shutdown();
			return; 
		}      
		timer.sleep();
	}
	timer.sleep();
	getTransformInitToBaseAndInitToWorld();
	while(!flag_getTransformToBase);
	{   
		timer.sleep();
	}
	flag_haveInitialized=true;
	return;
}

void mediaNode::slamInitialzed_callback(rclcpp::Client<interface::srv::SlamInitialized>::SharedFuture response)
{
	auto result = response.get();
	if (!result->flag_slam_initialized)
	{
		this->flag_trouble = true;
		RCLCPP_INFO(this->get_logger(), "Cdcr controller can't finish slam initialization.");
		return;
	}
	this->flag_slamInitialized = true;
	RCLCPP_INFO(this->get_logger(), "Cdcr controller finish slam initialization.");
	return;
}
void mediaNode::initializeSlam()
{
	std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t_now;
	double time_spend;
	while (!slamInitializedFlag_cli->wait_for_service(std::chrono::seconds(1)))
	{
		t_now = std::chrono::steady_clock::now();
		time_spend = std::chrono::duration_cast<std::chrono::seconds>(t_now-t_start).count();
		if (time_spend > TimeOut)
		{
			RCLCPP_ERROR(this->get_logger(), "Waiting slamInitialzation server TIME_OUT!!");
			flag_timeout=true;
			return; 
		}
		RCLCPP_INFO(this->get_logger(), "Waiting slamInitialzation server online");
	}
	auto request = std::make_shared<interface::srv::SlamInitialized_Request>();
	this->slamInitializedFlag_cli->async_send_request(request, 
		std::bind(&mediaNode::slamInitialzed_callback,this,std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "slamInitialization service is online, wait the process of slamInitialization finish!");
	return;
}

void mediaNode::goalPose_callback(std::shared_future<GoalHandleMoveAction::SharedPtr> future)
{
	auto goal_handle = future.get();
	if (!goal_handle)
	{
		RCLCPP_ERROR(this->get_logger(), "Goal was rejected by move cdcr server ");
		flag_trouble = true;
	}
	else
	{
		RCLCPP_INFO(this->get_logger(), "Goal was accepted by move cdcr server");
	}
	return;
};
void mediaNode::feedbackPose_callback(GoalHandleMoveAction::SharedPtr, 
							const std::shared_ptr<const MoveAction::Feedback> feedback)
{
	geometry_msgs::msg::Pose feedbackPose = feedback->feedback_pose;
	Eigen::Matrix3d feedbackR(feedbackPose.orientation.x, feedbackPose.orientation.y,feedbackPose.orientation.z,feedbackPose.orientation.w);
	RCLCPP_INFO(this->get_logger(), "feedback_direction: %f, %f, %f", feedbackR(0,1),feedbackR(1,1),feedbackR(2,1));
	RCLCPP_INFO(this->get_logger(), "feedback_position: %f, %f, %f", 
				feedbackPose.position.x,feedbackPose.position.y,feedbackPose.position.z);
	return;
}

void mediaNode::calSlamToWorldScaleFactor()
{	

	Eigen::Matrix4d T_tool_begin,T_tool_end;
	calTransformMatrixFromPoints(start_frame_points, T_tool_begin);
	calTransformMatrixFromPoints(end_frame_points, T_tool_end);

	//TODO: remember the extrinsic here is using the frame system of camera which means the xyz order is not consistent with world frame.     
	Eigen::Vector3d camera_start_position_real = (T_tool_begin*T_camera_to_tool.col(3)).block(0,0,3,1);
	Eigen::Vector3d camera_end_position_real = (T_tool_end*T_camera_to_tool.col(3)).block(0,0,3,1);

	double real_translation = (camera_end_position_real-camera_start_position_real).norm();
	double slam_translation = (end_camera_position-start_camera_position).norm();
	scaleFactor_slamToWorld = real_translation/slam_translation;
	return ;
}
void mediaNode::resultPose_callback(const GoalHandleMoveAction::WrappedResult & result)
{
	switch (result.code)
	{
	case rclcpp_action::ResultCode::SUCCEEDED:
		RCLCPP_INFO(this->get_logger(), "Move cdcr action succeed!");
		break;
	case rclcpp_action::ResultCode::ABORTED:
		RCLCPP_ERROR(this->get_logger(), "Move cdcr action aborted!");
		flag_trouble=true;
		return;
	case rclcpp_action::ResultCode::CANCELED:
		RCLCPP_ERROR(this->get_logger(), "Move cdcr action canceled!");
		flag_trouble=true;
		return;
	default:
		RCLCPP_ERROR(this->get_logger(), "UNKNOWN ERROR !");
		flag_trouble=true;
		return;
	}

	inputFlagPoints(end_frame_points);
	end_camera_position=Eigen::Vector3d(transform_cur2init_msg.pose.position.x,
										transform_cur2init_msg.pose.position.y,
										transform_cur2init_msg.pose.position.z);

	calSlamToWorldScaleFactor();
	flag_getScaleFactor=true;
	return;
}

void mediaNode::getSlamToWorldScaleFact()
{
    // 
	std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point t_now;
	double t_spend ;
    RCLCPP_INFO(this->get_logger(), "Start to get the scale factor of slam to world");
	while (!this->move_cdcr_cli->wait_for_action_server(std::chrono::seconds(1)))
	{
		t_now = std::chrono::steady_clock::now();
		t_spend = std::chrono::duration_cast<std::chrono::seconds>(t_now-t_start).count();
		if (t_spend > TimeOut)
		{
			RCLCPP_ERROR(this->get_logger(), "Waiting cdcr_movement service TIME_OUT!");
			flag_timeout=true;
			return;
		}
		RCLCPP_INFO(this->get_logger(), "Waiting cdcr_movement service online!");
	}
	auto goal = MoveAction::Goal();

	inputFlagPoints(start_frame_points);
	start_camera_position=Eigen::Vector3d(transform_cur2init_msg.pose.position.x,
										  transform_cur2init_msg.pose.position.y,
										  transform_cur2init_msg.pose.position.z);

	goal.goal_pose = m_goal_pose;
	auto send_goal_options = rclcpp_action::Client<MoveAction>::SendGoalOptions();
	send_goal_options.goal_response_callback = std::bind(&mediaNode::goalPose_callback,this,std::placeholders::_1);
	send_goal_options.feedback_callback = std::bind(&mediaNode::feedbackPose_callback,this,std::placeholders::_1,std::placeholders::_2);
	send_goal_options.result_callback = std::bind(&mediaNode::resultPose_callback,this,std::placeholders::_1);
	this->move_cdcr_cli->async_send_goal(goal, send_goal_options);
	RCLCPP_INFO(this->get_logger(), "Send the cdcr movement request secceed!");
    return;
}

void mediaNode::getTransformInitToBaseAndInitToWorld()
{
	// from the quaternion to rotation matrix 
	Eigen::Quaterniond q_init_to_cur(transform_init2cur_msg.pose.orientation.w, 
								  transform_init2cur_msg.pose.orientation.x, 
								  transform_init2cur_msg.pose.orientation.y, 
								  transform_init2cur_msg.pose.orientation.z);
	Eigen::Vector3d t_init_to_cur(transform_init2cur_msg.pose.position.x,
								  transform_init2cur_msg.pose.position.y,
								  transform_init2cur_msg.pose.position.z);
	Eigen::Matrix4d T_init_to_cur = Eigen::Matrix4d::Identity();
	T_init_to_cur.block(0,0,3,3) = q_init_to_cur.matrix();
	T_init_to_cur.block(0,3,3,1) = t_init_to_cur;
	Eigen::Matrix3d tool_flag_points, base_flag_points;
	inputFlagPoints(tool_flag_points);
	inputFlagPoints(base_flag_points);

	Eigen::Matrix4d T_tool;
	Eigen::Matrix4d T_base;
	calTransformMatrixFromPoints(tool_flag_points, T_tool);
	calTransformMatrixFromPoints(base_flag_points, T_base);

	Eigen::Matrix4d T_tool_to_base = T_base.inverse()*T_tool;
	T_init_to_base = T_tool_to_base * T_camera_to_tool * T_init_to_cur;
	T_init_to_world = T_base_to_world * T_init_to_base;
	flag_getTransformToBase = true;
	return;
}

void mediaNode::inputFlagPoints(Eigen::Matrix3d & input_flag_points)
{
	for (int i=0;i<3;i++)
	{
		std::vector<std::string> xyz = {"x","y","z"};
		for(int j=0;j<3;j++)
		{
			RCLCPP_INFO(this->get_logger(), " point[%d].%s: ", i, xyz[j].c_str());
			std::cin >> input_flag_points(i,j);
			input_flag_points(i,j) *= scaleFact_mmToTarget;
		}
	}
	return;
}

// Calculate the transform matrix from three flag points
void mediaNode::calTransformMatrixFromPoints(const Eigen::Matrix3d points, Eigen::Matrix4d &output_T) 
{
	output_T = Eigen::Matrix4d::Identity();
	Eigen::Vector3d t;
    // 由三个坐标点取平均获得坐标原点
    output_T(0,3) = (points(0,0) + points(1,0) + points(2,0)) / 3; 
    output_T(1,3) = (points(0,1) + points(1,1) + points(2,1)) / 3; 
    output_T(2,3) = (points(0,2) + points(1,2) + points(2,2)) / 3; 
    Eigen::Vector3d arr21,arr20;
    arr20 = points.row(0)-points.row(2);
    arr21 = points.row(1)-points.row(2);
    Eigen::Vector3d axisY = arr21.cross(arr20);
    axisY.normalize();
    Eigen::Vector3d axisZ;
    axisZ.x() =points(0,0)-output_T(0,3);
    axisZ.y() =points(0,1)-output_T(1,3);
    axisZ.z() =points(0,2)-output_T(2,3);
    axisZ.normalize();
    Eigen::Vector3d axisX = axisY.cross(axisZ);
    output_T.block(0,0,3,1) = axisX;
    output_T.block(0,1,3,1) = axisY;
    output_T.block(0,2,3,1) = axisZ;
}

void mediaNode::slam_callback(const interface::msg::Slam::SharedPtr slam_msg)
{
	transform_init2cur_msg = slam_msg->transform_init2cur;
	transform_init2cur_msg.pose.position.x *= scaleFactor_slamToWorld;
	transform_init2cur_msg.pose.position.y *= scaleFactor_slamToWorld;
	transform_init2cur_msg.pose.position.z *= scaleFactor_slamToWorld;

	Eigen::Quaterniond q_init_to_cur(transform_init2cur_msg.pose.orientation.w,
								  	transform_init2cur_msg.pose.orientation.x,
								  	transform_init2cur_msg.pose.orientation.y,
								  	transform_init2cur_msg.pose.orientation.z);
	Eigen::Vector3d t_init_to_cur(transform_init2cur_msg.pose.position.x,
								  transform_init2cur_msg.pose.position.y,
								  transform_init2cur_msg.pose.position.z);
	Eigen::Matrix4d tempT_init_to_cur=Eigen::Matrix4d::Identity();
	tempT_init_to_cur.block(0,0,3,3) = q_init_to_cur.matrix();
	tempT_init_to_cur.block(0,3,3,1) = t_init_to_cur;
	Eigen::Matrix4d tempT_cur_to_init=tempT_init_to_cur.inverse();
	transform_cur2init_msg.pose.orientation.w = q_init_to_cur.inverse().w();
	transform_cur2init_msg.pose.orientation.x = q_init_to_cur.inverse().x();
	transform_cur2init_msg.pose.orientation.y = q_init_to_cur.inverse().y();
	transform_cur2init_msg.pose.orientation.z = q_init_to_cur.inverse().z();
	transform_cur2init_msg.pose.position.x = tempT_cur_to_init(0,3);
	transform_cur2init_msg.pose.position.y = tempT_cur_to_init(1,3);
	transform_cur2init_msg.pose.position.z = tempT_cur_to_init(2,3);
	if (!flag_haveInitialized) return;
	
	// 如果初始化获得了到真实世界的尺度因子，将slam尺度下的点云坐标以及相机的位移量乘上尺度因子就获得了真实世界下的点云数据以及相机位移
	pcl::PointCloud<pcl::PointXYZ> temp_pointCloud;
	pcl::fromROSMsg(slam_msg->point_cloud, temp_pointCloud);
	// 调试代码3
	// std::cout << "temp_pointCloud.size(): " << temp_pointCloud.points.size() << std::endl;
	for (size_t i=0;i<temp_pointCloud.points.size();i++) {
		temp_pointCloud.points[i].x *= scaleFactor_slamToWorld;
		temp_pointCloud.points[i].y *= scaleFactor_slamToWorld;
		temp_pointCloud.points[i].z *= scaleFactor_slamToWorld;
	}
	
	// transform the pointcloud of init frame to base frame and publish;

	// get the transform of cur to base frame and publish;

	sensor_msgs::msg::PointCloud2 point_cloud_pub_msg;
	pcl::toROSMsg(temp_pointCloud, point_cloud_pub_msg);
	point_cloud_pub_msg.header = slam_msg->point_cloud.header;
	// std::cout << "point_cloud_pub_msg.header.frame_id: " << point_cloud_pub_msg.header.frame_id << std::endl;
	pub_point_cloud->publish(point_cloud_pub_msg);
	// Put the publish operation in here is for the posible delay result of pcl operation, 
	// that may increase the message filter's load.
	pub_T_init_to_cur->publish(transform_init2cur_msg);
	transform_cur2init_msg.header = slam_msg->point_cloud.header;
	pub_T_cur_to_init->publish(transform_cur2init_msg);

	return;
}

// 返回给消息点云和关键点图像坐标信息的消息
mediaNode::~mediaNode() {}