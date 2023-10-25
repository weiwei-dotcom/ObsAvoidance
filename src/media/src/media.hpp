#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "interface/msg/slam.hpp"
#include "interface/srv/slam_initialized.hpp"
#include "interface/srv/transform.hpp"
#include "interface/action/move.hpp"
#include "pcl/conversions.h"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl_conversions/pcl_conversions.h"
#include "sophus/se3.hpp"
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include <mutex>
#include <numeric>
#include <cmath>
#include <thread>
#include <rclcpp_action/rclcpp_action.hpp>

class mediaNode : public rclcpp :: Node
{
using MoveAction = interface::action::Move;
using GoalHandleMoveAction = rclcpp_action::ClientGoalHandle<MoveAction>;
private:
    // the flag of slam system been initialized
    bool flag_slamInitialized;
    // the flag of be initialized
    bool flag_initialized;
    
    // 针孔相机投影矩阵
    Eigen::Matrix3d m_projectMatrix; 

    // 是否获得了slam到真实世界的尺度标志,是否获得将世界坐标系转换到机器人基座标系转换矩阵标志
    bool flag_getScaleFactor;
    bool flag_getTransformToWorld;

    // slam系统初始化之后的尺度到真实尺度的尺度变换因子
    double scaleFactor_slamToWorld;
    // 创建互斥锁变量
    std::mutex mtx;
    // 声明发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_T_init_to_cur;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_T_cur_to_init;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_point_cloud;
    // 声明订阅slam话题的回调函数
    void slam_callback(const interface::msg::Slam::SharedPtr slamMsg);
    rclcpp::Subscription<interface::msg::Slam>::SharedPtr slam_sub;
    // 相机到末端工具坐标系的转换矩阵
    Eigen::Matrix4d T_camera_to_tool;
    // flag of recieve state relation to initialization process;
    bool flag_trouble;
    // time out value;
    double TimeOut_initialization;
    // recieve the msg of slam system that contain the info of current camera pose;
    geometry_msgs::msg::PoseStamped transform_init2cur_msg, transform_cur2init_msg;
    // the scale factor of mm unit to target measurement Unit
    double scaleFact_mmToTarget;
    // 
    bool flag_timeout;
    // The target pose of process getting the scale factor
    geometry_msgs::msg::Pose m_goal_pose;
    // start frame flag point and end frame flag point at process of getting scale factor;
    Eigen::Matrix3d start_frame_points, end_frame_points;
    // start and end position of camera at process of getting scale factor
    Eigen::Vector3d start_camera_position, end_camera_position;
    // Save the transform matrix from init to cur and init to world
    Eigen::Matrix4d T_init_to_base;
    Eigen::Matrix4d T_init_to_world;
    // Transform matrix of base to world. The rotation matrix is identity matrix,
    // which means direction is same as the base frame, position part will be calculated
    // based on config file value(self set).
    Eigen::Matrix4d T_base_to_world;

    // Declare the server of request for variant T
    rclcpp::CallbackGroup::SharedPtr transform_callback_group;
    rclcpp::Service<interface::srv::Transform>::SharedPtr service_transform;
    // 
    void slamInitialzed_callback(rclcpp::Client<interface::srv::SlamInitialized>::SharedFuture response);

    // TODO: Before completing the function as follow, we should know the data interface transmit on corresponding node.
    // Client of sending the request to get the state of slam system initialization 
    rclcpp::Client<interface::srv::SlamInitialized>::SharedPtr slamInitializedFlag_cli;
    // Action client of sending the request to get the scale fact of slam to realworld
    rclcpp_action::Client<interface::action::Move>::SharedPtr move_cdcr_cli;
    void goalPose_callback(std::shared_future<GoalHandleMoveAction::SharedPtr> future);
    void feedbackPose_callback(GoalHandleMoveAction::SharedPtr, 
                               const std::shared_ptr<const MoveAction::Feedback> feedback);
    void resultPose_callback(const GoalHandleMoveAction::WrappedResult & result);

public:
    // 构造函数
    mediaNode();
    // 从运动捕捉数据得到真实世界的尺度转换因子函数
    void getSlamToWorldScaleFact();
    // calculate the transform matrix from three points
    void calTransformMatrixFromPoints(const Eigen::Matrix3d points, Eigen::Matrix4d &output_T) ;
    // 获取从相机当前坐标系到基座标系下的转换矩阵
    void getTransformInitToBaseAndInitToWorld();
    // 初始化slam动作发送程序
    void initializeSlam();
    // The main function of initialization whole system;
    void initialization();
    // calculate the scale factor of transformation the measure value from slam to world
    void calSlamToWorldScaleFactor();
    // input flag point coordinate from keyboard 
    void inputFlagPoints(Eigen::Matrix3d & input_flag_points);
    // 
    void transform_callback(const interface::srv::Transform::Request::SharedPtr request,
                            const interface::srv::Transform::Response::SharedPtr response);


    // 析构函数
    ~mediaNode();
};