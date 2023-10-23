#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "interface/msg/slam.hpp"
#include "interface/srv/slam_initialized.hpp"
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
#include <sophus/se3.hpp>
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
    enum ERROR_TYPE
    {
        Circle_detection_is_not_stable = 0,
        Distance_of_circle_is_too_much = 1,
        Not_found_the_countour = 2,
        Point_cloud_is_little = 3,
        OK = 4,
        Image_recieve_error = 5,
        The_camera_is_not_straight_on_the_plane = 6
    };
    // 错误状态
    ERROR_TYPE error_type;
    // the flag of slam system been initialized
    bool flag_slamInitialized;
    // the flag of be initialized
    bool flag_haveInitialized;
    // 用来判断相机是否正视于检测平面的余弦角度阈值
    double cosValue_thresh;
    // 尺度因子单个样本好点概率
    float inlier_probability_;
    // 相机焦距计算尺度因子时要用
    double fx_;
    // 尺度因子随机采样的长度
    int sample_length;
    // ransac内点阈值
    double inlier_thresh_scaleFact;
    // 尺度因子列表及其长度阈值
    std::vector<double> scaleFactList_;
    int scaleFactList_size_thresh;
    // 平面slam尺度距离
    std::vector<double> distance_;
    // 针孔相机投影矩阵
    Eigen::Matrix3f m_projectMatrix; 
    // 相机初始帧到机器人基座标系的转换矩阵
    Eigen::Matrix4d m_transformToBase;
    // 腐蚀膨胀操作结构体
    cv::Mat structure_erode;
    cv::Mat structure_dilate;
    // 蓝色平面分割阈值
    cv::Scalar blueLowerThresh, blueUpperThresh;
    // 圆形检测函数参数
    double minDist, dp, cannyUpThresh, circleThresh;
    int minRadius, maxRadius;
    // 两帧圆之间的半径差阈值、圆心距离阈值
    float difference_radius_thresh, distance_center_thresh;
    // 是否获得了slam到真实世界的尺度标志,是否获得将世界坐标系转换到机器人基座标系转换矩阵标志
    bool flag_getScaleFact;
    bool flag_getTransformToBase;
    // 多次圆形检测记录的圆
    std::vector<cv::Vec3f> circles_;
    // 实际入口圆形半径
    double circleRadius;
    // 圆形列表长度阈值
    int circle_size_thresh;
    // slam系统初始化之后的尺度到真实尺度的尺度变换因子
    double scaleFact_slamToWorld;
    // 创建互斥锁变量
    std::mutex mtx;
    // 声明发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformInit2Cur_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformCurToInit_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // 声明订阅slam话题的回调函数
    void slam_callback(const interface::msg::Slam::SharedPtr slamMsg);
    rclcpp::Subscription<interface::msg::Slam>::SharedPtr slam_sub;
    // 相机到末端工具坐标系的转换矩阵
    Eigen::Matrix4d extrinsicMatrix;
    // 相机初始帧到基座标系的转换矩阵
    Eigen::Matrix4d transform_initToBase;
    // flag of recieve state relation to initialization process;
    bool flag_trouble;
    // time out value;
    double TimeOut;
    // 
    bool flag_timeout;
    // The target pose of process getting the scale factor
    geometry_msgs::msg::Pose m_goal_pose;
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
    // 
    // 从运动捕捉数据得到真实世界的尺度转换因子函数
    void getSlamToWorldScaleFact();
    // 获取从相机当前坐标系到基座标系下的转换矩阵
    void getTransformToBase();
    // 判断是否正对检测平面函数
    bool detectPoseCorrect(const cv::Mat img, const interface::msg::Slam::ConstSharedPtr slamMsg, double &distance_plane, const cv::Vec3f tempCircle);
    // 提取最大连通域轮廓函数
    bool getMaxAreaContour(const cv::Mat img_bin, std::vector<cv::Point> &contour);
    // 拟合点云平面获取平面参数函数
    Eigen::Vector4d calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    // 对随机采样一致计算尺度因子函数
    void ransacScaleFact();
    // 根据好点概率计算ransac迭代次数函数
    int calRansacIterNum();
    // 打印检测错误状态信息
    void changeErrorType(ERROR_TYPE newError);
    // 初始化slam动作发送程序
    void initializeSlam();
    // The main function of initialization whole system;
    void initialization();
    
    // 析构函数
    ~mediaNode();
};