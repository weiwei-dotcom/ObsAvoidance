#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "Eigen/Eigen"
#include "sophus/se3.hpp"

class MotionControl:public rclcpp::Node
{
private:
    // 平台半径
    double plat_radius;
    // 关节数量，绳索数量，同一根绳索牵拉的最少关节数量
    int joints_number, cables_number, segs_number;
    // 初始化视觉里程计尺度时的动作前后末端工具、相机位姿
    Eigen::Matrix4d tool_pose_before_move, tool_pose_after_move, cam_pose_before_move, cam_pose_after_move;
    // 记录当前工具坐标系在参考坐标系下的位姿，通过订阅 camera_pose 话题利用手眼标定矩阵转化而来。
    Eigen::Matrix4d tool_pose;

    // // 记录初始化机器人位姿下的从下位机发来的各电机初始编码器位置
    // std::vector<Eigen::Vector3d> motor_init_encodes;

    // // 记录每次从 path_follow 接收的关节变量
    // std::vector<Eigen::Vector2d> target_joint_variables;

    // 记录每个关节在各个系绳点（总共24个系绳点，本研究使用六关节18个系绳点),Eigen::VectorXd记录单个关节每个id的绳长
    std::vector<Eigen::Vector3d> cable_lengths;
    // 记录各个关节系绳点索引号
    std::vector<Eigen::Vector3i> cable_ids;
    // 记录各个关节的柔性骨架长度、刚性头尾两端刚体长度
    std::vector<double> continuum_lengths,rigid1_lengths,rigid2_lengths;
    
public:
    // 初始化函数
    MotionControl();
    // 计算整体机器人每根绳长的
    void calRobotCableLength(std::vector<Eigen::Vector3d> &dst_cable_lengths,
                            const std::vector<Eigen::Vector3i> &src_cable_ids,
                            const std::vector<Eigen::Vector2d> &src_joint_values,
                            const std::vector<double> &src_continuum_lengths,
                            const std::vector<double> &src_rigid1_lengths,
                            const std::vector<double> &src_rigid2_lengths);
    // 计算第src_joint_id关节模块末端动平台所系的绳子长度
    Eigen::Vector3d calJointCableLength(const int& src_joint_id,const Eigen::Vector3i& src_cable_ids, const std::vector<Eigen::Matrix4d>& src_transforms);
    // 该函数根据关节几何参数计算动平台相对于静平台转换矩阵
    Eigen::Matrix4d getJointTransform(const Eigen::Vector2d &joint_value, const double &continuum_length, const double &rigid1_length, const double &rigid2_length);
    // 析构函数
    ~MotionControl();
      
};