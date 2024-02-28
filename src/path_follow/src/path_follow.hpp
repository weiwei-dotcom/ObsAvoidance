#include "rclcpp/rclcpp.hpp"
#include "joint.hpp"
#include "Eigen/Eigen"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "interface/srv/path_points.hpp"
#include "interface/srv/base_joint_motor_value.hpp"
#include "interface/srv/enable_follow.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/float64.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <ceres/ceres.h>
#include <iostream>
#include <fstream>

class PathFollow:public rclcpp::Node
{
private:
    /*  判断是否已经获取了离散跟跟随路径点标志变量
            在初始化函数中置为假；
            在getNewPathCallback()函数中，每次成功获取新的离散路径点后置为真;
    */
    bool flag_get_first_path;

    /*  判断服务已经发起过但是未被响应的标志变量
            在初始化函数中置为假；
            在每次发起获取新离散路径点服务请求后置为真;
            在进入获取新离散路径点响应函数后置为假;
    */
    bool flag_called_not_response;
    
    /*  判断是否靠近目标点标志位
            在初始化函数中置为假；
            计算每次拟合路径后的距离骨架末端点最近离散路径点与离散路径点序列末端点之间路径长度，如果长度小于设定阈值，那么该标志位置为真；
            如果该标志位为真，那么不用再发起获取新离散路径点服务；
    */
    bool flag_no_need_to_replan;

    /*  判断是否到达目标终点标志变量
			在初始化函数中置为假；
			当拟合后计算末端点距离目标终点之间距离小于一定的阈值时，该标志位置为真；
			当拟合时机器人末端的拟合点索引靠近目标终点小于一定的阈值，但是没有超过终点索引时，该标志位置为真；
			当拟合时机器人末端的拟合点索引超过目标终点时，停止拟合，该标志位置为真；
			如果该标志位为真时，停止拟合服务，并将服务消息中的 response->flag_arrived_target = this->flag_arrived_target;
    */
    bool flag_arrived_target;
	// 路径离散采样间隔
	double sample_interval;
    // 世界到参考坐标系下的转换
    Eigen::Matrix4d world_to_ref;
    // 行程始终点
    Eigen::Vector3d stroke_start,stroke_end;
    // 存储路径离散点
    std::vector<Eigen::Vector3d> path_points;
    // 声明获取离散路径点定时器
    rclcpp::TimerBase::SharedPtr get_path_timer;
    // 声明拟合路径服务端
    rclcpp::Service<interface::srv::BaseJointMotorValue>::SharedPtr fit_path_server;
	// 负责通知motor_control节点可以开始进行 path follow 的通知发起客户端
	rclcpp::Client<interface::srv::EnableFollow>::SharedPtr enable_follow_client;
    // 声明获取新离散路径客户端
    rclcpp::Client<interface::srv::PathPoints>::SharedPtr get_path_client;
    // 路径规划点在离散路径点序列上的索引值
    int replan_start_id;
    // 路径规划的起点速度值，同样是路径跟随推进速度值
    double speed_value;
	// 当前基座到参考坐标系变换矩阵
	Eigen::Matrix4d trans_base_ref;
	// 基座在行程上的初始位置
	Eigen::Vector3d init_stroke_position;
	// 路径跟随时的基座跟随点在离散路径点序列中的索引值
	int base_position_id;
	// 路径跟随关节数量
	int joint_number;
	// 路径跟随机器人所有关节对象
	std::vector<Joint> joints;
	// 拟合关节角上下边界
	double alpha_lower_bound,alpha_upper_bound,theta_lower_bound,theta_upper_bound;
	// 拟合权重
	double weight_direction,weight_position;

	// 路径拟合位置误差结构体
	struct position_residual{
		position_residual(double weight_position,
					double length_continuum,
					double length_rigid2,
					double length_rigid1,
					Eigen::Vector3d position
					):
						length_continuum_(length_continuum),
						length_rigid2_(length_rigid2),
						length_rigid1_(length_rigid1),
						position_(position),
						weight_position_(weight_position) {}
		template <typename T> bool operator()(const T* const alpha, const T* const theta, T* residual) const {
			residual[0] = weight_position_*(length_continuum_/theta[0]*(1.0-cos(theta[0]))*cos(alpha[0])+length_rigid2_*sin(theta[0])*cos(alpha[0])-position_.x());
			residual[1] = weight_position_*(length_continuum_/theta[0]*(1.0-cos(theta[0]))*sin(alpha[0])+length_rigid2_*sin(theta[0])*sin(alpha[0])-position_.y());
			residual[2] = weight_position_*(length_continuum_/theta[0]*sin(theta[0])+length_rigid1_+length_rigid2_*cos(theta[0])-position_.z());
			return true;
		}
	private: 
		const double weight_position_;
		const double length_continuum_,length_rigid2_,length_rigid1_;
		const Eigen::Vector3d position_; 
	};
	// 路径拟合角度误差结构体
	struct angle_residual{
		angle_residual(double weight_direction,
					double length_rigid1,
					double length_continuum,
					double length_rigid2,
					Eigen::Vector3d tangent_vector):
						length_rigid1_(length_rigid1), 
						length_continuum_(length_continuum),
						length_rigid2_(length_rigid2),
						tangent_vector_(tangent_vector),
						weight_direction_(weight_direction){}
		template <typename T> bool operator()(const T* const alpha, const T* const theta, T* residual) const {
			residual[0] void find_closed_path_point(const int& start_path_point_id,const Eigen::Vector3d& joint_end_position, int& segment_start_path_point_id);= weight_direction_*180.0/M_PI*acos(sin(theta[0])*cos(alpha[0])*tangent_vector_(0)
															+sin(theta[0])*sin(alpha[0])*tangent_vector_(1)
															+cos(theta[0])*tangent_vector_(2));
			return true;
		}
	private: 
		const double weight_direction_;
		const double length_rigid1_,length_continuum_,length_rigid2_;
		const Eigen::Vector3d tangent_vector_; 
	};

public:
    PathFollow();
    void fitPathCallback();
    void getNewPathCall();
    void getNewPathCallback(rclcpp::Client<interface::srv::PathPoints>::SharedFuture response);
	
	void enableFollowCall();
    void enableFollowCallback(rclcpp::Client<interface::srv::EnableFollow>::SharedFuture response);
	void find_closed_path_point(const int& start_path_point_id,const Eigen::Vector3d& joint_end_position, int& segment_start_path_point_id);
	// 根据三个标记点坐标计算坐标系
	Eigen::Matrix4d calFrame(const std::vector<Eigen::Vector3d> &flag_points) 
	{
		Eigen::Matrix4d result_frame = Eigen::Matrix4d::Identity();
		// 由三点计算圆心
		Eigen::Vector3d centor = calCenter(flag_points[0],flag_points[1],flag_points[2]);
		result_frame.block(0,3,3,1) = centor;
		// 计算坐标轴
		Eigen::Vector3d arr21, arr20;
		arr20[0] = flag_points[2][0] - flag_points[0][0];
		arr20[1] = flag_points[2][1] - flag_points[0][1];
		arr20[2] = flag_points[2][2] - flag_points[0][2];
		arr21[0] = flag_points[2][0] - flag_points[1][0];
		arr21[1] = flag_points[2][1] - flag_points[1][1];
		arr21[2] = flag_points[2][2] - flag_points[1][2];
		Eigen::Vector3d axisZ = arr21.cross(arr20);
		axisZ.normalize();
		Eigen::Vector3d axisX;
		axisX[0] = result_frame(0,3) - flag_points[0][0];
		axisX[1] = result_frame(1,3) - flag_points[0][1];
		axisX[2] = result_frame(2,3) - flag_points[0][2];
		axisX.normalize();
		Eigen::Vector3d axisY = axisZ.cross(axisX);
		result_frame.block(0,0,3,3) << axisX,axisY,axisZ;
	}

	// 获得线段外一点投影到该线段上的点
	Eigen::Vector3d getIntervalPoint(const Eigen::Vector3d& inter_point,const Eigen::Vector3d& line_end1,const Eigen::Vector3d& line_end2)
	{
		double temp_proj_value=calVecProjValue(inter_point, line_end1,line_end2);
		double temp_norm_value = (line_end1-line_end2).norm();
		return (temp_proj_value/temp_norm_value * line_end2 + (temp_norm_value-temp_proj_value)/temp_norm_value*line_end1);
	}

	// 获得media_point指向first_point向量向media_point指向second_point向量投影的长度值
	double calVecProjValue(const Eigen::Vector3d& first_point, const Eigen::Vector3d& media_point, const Eigen::Vector3d& second_point)
	{
		return (first_point-media_point).dot((second_point-media_point).normalized());
	}

	// 函数定义：计算圆心
	Eigen::Vector3d calCenter(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3) 
	{
		Eigen::Vector3d center;
		// 构造系数矩阵
		Eigen::Vector3d a21 = p2-p1;
		Eigen::Vector3d a21_2 = (p2+p1)*0.5;
		Eigen::Vector3d a31 = p3-p1;
		Eigen::Vector3d a31_2 = (p3+p1)*0.5;
		Eigen::Matrix3d A;
		A << a21[0], a21[1], a21[2],
			a31[0], a31[1], a31[2],
			a21.cross(a31)[0], a21.cross(a31)[1], a21.cross(a31)[2];
		// 构造常数向量
		Eigen::Vector3d b;
		b << a21.dot(a21_2),
			a31.dot(a31_2),
			(a21.cross(a31).dot(p1));
		// 解线性方程组
		center = A.fullPivHouseholderQr().solve(b);
		return center;
	}
};