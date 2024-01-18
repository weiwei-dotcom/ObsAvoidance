#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "interface/msg/slam.hpp"
<<<<<<< HEAD
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "interface/srv/map_point.hpp"
#include "interface/srv/cam_pose.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <mutex>
=======
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
#include "sophus/so3.hpp"
#include <Eigen/Eigen>
#include <mutex>
#include <numeric>
#include <cmath>
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09

class mediaNode : public rclcpp :: Node
{
private:
<<<<<<< HEAD
    // 是否获得了slam到真实世界的尺度标志
    bool getScale;

    // slam系统初始化之后的尺度到真实尺度的尺度变换因子
    // TODO: 后面需要一个标定尺度的因子的方法来更新该变量
    float scaleFact_slamToCam;
    // 创建互斥锁变量
    std::mutex mtx;
    // 创建回调函数组
    rclcpp::CallbackGroup::SharedPtr mapPointCallbackGroup;
    // 创建订阅slam话题的消息订阅指针
    rclcpp::Subscription<interface::msg::Slam>::SharedPtr slamSub;
    // 声明订阅slam话题的回调函数
    void slam_callback(const interface::msg::Slam::SharedPtr slamMsg);
    // 定义接收slam话题消息的相机位姿、点云消息、关键点图像坐标变量
    interface::msg::Slam::SharedPtr camPose_pointCloud_img;
    sensor_msgs::msg::PointCloud2 m_pointCloud;
    geometry_msgs::msg::Pose m_camPose;
    // 创建 获取原图像、相机位姿、点云、关键点坐标 服务端指针 
    rclcpp::Service<interface::srv::MapPoint>::SharedPtr mapPointSer;
    // 创建 获取相机位姿 服务端指针 
    rclcpp::Service<interface::srv::CamPose>::SharedPtr camPoseSer;
    // 创建 获取原图像、相机位姿、点云、关键点坐标 服务端回调函数
    void mapPoint_callback(const interface::srv::MapPoint::Request::SharedPtr request,
                            const interface::srv::MapPoint::Response::SharedPtr response);
    // // 创建 获取相机位姿 服务端回调函数
    void camPose_callback(const interface::srv::CamPose::Request::SharedPtr request,
                            const interface::srv::CamPose::Response::SharedPtr response);
    // 获取到真实世界mm单位制的尺度因子的回调函数
    void getScale_callback();
    
public:
    // 初始化函数
    mediaNode();
=======
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
    // motionControl功能包根据别的功能包输出的位姿信息来通过逆运动学或者其他方法来计算绳索绳索拉伸量

    // 判断motionControl功能包程序是否编写完成的标志
    int flag_motionControlProgramHaveDone;
    // 判断slam系统是否完成一段运动实现初始化标志
    bool flag_slamInited;
    // 用来判断相机是否正视于检测平面的余弦角度阈值
    double cosValue_thresh;
    // 尺度因子单个样本好点概率
    float inlier_probability_;
    Eigen::Matrix4d transform_init_to_world;
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
    // 是否获得了slam到真实世界的尺度标志
    bool flag_getScaleFact;
    // 多次圆形检测记录的圆
    std::vector<cv::Vec3f> circles_;
    // 实际入口圆形半径
    double circleRadius;
    // 圆形列表长度阈值
    int circle_size_thresh;
    // slam系统初始化之后的尺度到真实尺度的尺度变换因子
    float scaleFact_slamToWorld;
    // 创建互斥锁变量
    std::mutex mtx;
    // message_filters创建话题消息接收器
    message_filters::Subscriber<interface::msg::Slam> slam_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> image_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_init_to_world_sub;
    // 声明发布器
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformInit2Cur_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformCurToInit_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr transformCurToWorld_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud_pub;
    // 声明时间对齐器
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<interface::msg::Slam, sensor_msgs::msg::Image>>> sync1;
    // 声明订阅slam话题的回调函数
    void slam_callback(const interface::msg::Slam::ConstSharedPtr &slamMsg,const sensor_msgs::msg::Image::ConstSharedPtr &imageMsg);

public:
    // 构造函数
    mediaNode();
    // 初始化获得尺度转换因子函数
    void getScaleSlamToWorld(const sensor_msgs::msg::Image::ConstSharedPtr &imageMsg, const interface::msg::Slam::ConstSharedPtr &slamMsg);
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
    void transform_init_to_world_callback(const geometry_msgs::msg::PoseStamped::SharedPtr transform_init_to_world);
>>>>>>> 9ce6bb423e552849a267afd38d866d6092578e09
    // 析构函数
    ~mediaNode();
};