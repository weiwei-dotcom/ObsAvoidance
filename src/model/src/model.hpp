#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "cv_bridge/cv_bridge.h"
#include "sophus/se3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"
#include "opencv2/core/eigen.hpp"
#include "pcl/common/transforms.h"
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ceres/ceres.h>
#include <fstream>
#include <algorithm>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"


class ModelNode : public rclcpp::Node
{
private:
    // 錯誤類型結構體
    enum ERROR_TYPE
    {
        Circle_detection_is_not_stable = 0,
        Distance_of_circle_is_too_much = 1,
        Not_found_the_countour = 2,
        Point_cloud_is_little = 3,
        OK = 4,
        Image_recieve_error = 5,
        The_camera_is_not_straight_on_the_plane = 6,
        TooClose = 7,
        No_detect_line = 8,
        Line_mismatch_condition = 9,
        Can_not_find_circle = 10
    };
    // 定义蓝色平面法向量残差函数结构体
    struct normVecResidual {
        normVecResidual(double x, double y, double z)
                : x_(x), y_(y), z_(z) {}
        template <typename T> bool operator()(const T* const a, const T* const b, const T* const c, T* residual) const {
            residual[0] = acos(x_*a[0]+y_*b[0]+z_*c[0])+(pow(a[0],2)+pow(b[0],2)+pow(c[0],2)-1.0);
            return true;
        }
    private:
        const double x_;
        const double y_;
        const double z_;
    };
    // 利用message_filters订阅者声明消息订阅者
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub;
    message_filters::Subscriber<sensor_msgs::msg::Image> img_sub;
    message_filters::Subscriber<geometry_msgs::msg::PoseStamped> pose_sub;
    // 映射消息同步器需要的类型参数名
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::PointCloud2,sensor_msgs::msg::Image,geometry_msgs::msg::PoseStamped> synMsgType1;
    typedef message_filters::Synchronizer<synMsgType1> synType1;

    std::shared_ptr<synType1> sync1;

    // 是否完成建模的flag变量
    bool flag_pubModel;

    cv::Scalar blueLower, blueUpper;
    // 检测平面筛选出来的点云数量阈值，小于该阈值则不进行平面拟合
    int mapPointNumThresh;

    // 直线检测函数相关参数变量
    int lineThresh;
    double minLineLength,maxLineGap;
    // 存储连续检测得到的三维圆心位置
    std::vector<Eigen::Vector3d> centrePositions;
    // 存储连续检测得到的三维直线方向向量
    std::vector<Eigen::Vector3d> vecs_direction;
    // 存储连续检测得到的平面法向量变量
    std::vector<Eigen::Vector3d> vecs_norm;
    // 存储连续检测当前帧到初始帧的变换矩阵
    std::vector<Eigen::Matrix4d> transforms_curToInit;

    // 边沿直线最小共线阈值与最大垂直阈值
    double minCosValueThresh_collineation, maxCosValueThresh_vertical;
    double distanceThresh_plandAndPoint;
    
       
    // // 存储每次检测得到的三维直线参数变量
    // std::vector<Eigen::Matrix<double, 6, 1>> upLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> underLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> leftLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> rightLine3d;
    
    // 存储每次有效检测得到的从圆心位置出发竖直向上的三维向量变量
    std::vector<Eigen::Vector3d> global_vecs_direction;
    // 存储每次有效检测得到的三维圆心位置
    std::vector<Eigen::Vector3d> global_centrePostions;
    // 存储每次有效检测得到的平面法向量
    std::vector<Eigen::Vector3d> global_vecs_norm;
    // 最终参数
    // 水平向右
    Eigen::Vector3d Xaxis;
    // 垂直平面远离光心
    Eigen::Vector3d Yaxis;
    // 垂直向上
    Eigen::Vector3d Zaxis;
    // 入口圆心位置
    Eigen::Vector3d centrePosition;
    // 用来建模的结构点位置
    Eigen::Vector3d frontLeftUnder; 
    Eigen::Vector3d frontRightUnder;
    Eigen::Vector3d frontLeftUp;
    Eigen::Vector3d backLeftUnder; 
    Eigen::Vector3d structureLeftUnder1;
    Eigen::Vector3d structureLeftUnder2;


    // 圆形列表长度阈值
    int circle_size_thresh;

    // 圆形检测函数参数
    double minDist, dp, cannyUpThresh, circleThresh;
    int minRadius, maxRadius;
    // 两帧圆之间的半径差阈值、圆心距离阈值
    float difference_radius_thresh, distance_center_thresh;
    // 多次圆形检测记录的圆
    std::vector<cv::Vec3f> circles_;
    // 检测平面与相机之间的距离阈值
    double distance_thresh;

    // 随机采样的长度
    int sample_length;
    // 单个样本好点概率
    float inlier_probability;
    // ransac内点阈值
    double inlier_thresh_centre;
    double inlier_thresh_normVec;
    double inlier_thresh_dirVec;
    // 开始ransac准备建模的参数列表长度阈值；
    int modelThresh;

    // 圆检测时圆周上的点向圆外以及圆内的像素值
    float outCircleVal,inCircleVal;

    int canny_threshLow,canny_threshUp;

    // 直线端点掩码判断时，线段端点的向内的平移量
    double pixelNum_translate;

    // 消息的header
    std_msgs::msg::Header m_header_initFrame;

    //开始改变阈值的循环次数占总迭代次数的比例
    double scale_startLoopCountTochangeInlierThresh;

    // 錯誤狀態變量
    ERROR_TYPE error_type;

    // 掩码膨胀结构体
    cv::Mat dilateStructure_mask;

    // 腐蚀膨胀操作结构体
    cv::Mat structure_erode1;
    cv::Mat structure_erode2;

    // 大核闭运算结构体
    cv::Mat kernelBigClose;

    // 用来判断相机是否正视于检测平面的余弦角度阈值
    double cosValueThresh_planeNormAndCameraZaxis;

    // 定义接收服务端 图像、成员变量、相机位姿、点云 的成员变量；
    cv::Mat m_img;
    Eigen::Matrix4d m_transform_curToInit;
    Eigen::Matrix4d m_transform_initToCur;
    pcl::PointCloud<pcl::PointXYZ> m_pointCloud;
    float fx, fy, cx, cy;
    Eigen::Matrix3f m_projectMatrix;

    // 存储障碍物点云
    pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;
    // 调试代码3
    int successNum;

    void mapPoint_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);

public:
    class GreaterLength
    {
    public:
        bool operator()(cv::Vec4i first, cv::Vec4i second) 
        {
            double lengthFirst = pow((double)first[0]-(double)first[2], 2)+pow((double)first[1]-(double)first[3], 2);
            double lengthSecond = pow((double)second[0]-(double)second[2], 2)+pow((double)second[1]-(double)second[3], 2);
            return lengthFirst > lengthSecond;
        }
    };
    bool recieveMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);
    // 构造函数 init function()
    ModelNode();

    // 障碍物建模主函数
    bool Model();
    // Publish the message of pointcloud of obstacle model;
    void PubModel();
    void changeErrorType(ERROR_TYPE newError);
    bool detectPoseCorrect(Eigen::Vector4d &param,const cv::Mat mask,cv::Mat &img_erode);
    Eigen::Vector4d calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    void from2dTo3dPlane(const Eigen::Vector2d inputPoint, Eigen::Vector3d &outputPoint, Eigen::Vector4d paramPlane);
    void ransacModelParam();
    int calRansacIterNum();
    void houghCircleDetect(std::vector<cv::Vec3f> &outputCircles, cv::Mat &img_gaussian);
    bool selectCircle(cv::Vec3f &temp_circle, const std::vector<cv::Vec3f> temp_circles, cv::Mat &mask);
    bool getMaxArea(cv::Mat &maxArea);
    void detectLine(const cv::Mat img_gaussian, std::vector<cv::Vec4i> &tempLines);
    bool selectLine(const cv::Mat img_erode, std::vector<cv::Vec4i> tempLines, cv::Vec4i &line,const cv::Vec3f circle);
    void calModelParam(const cv::Vec3f circle,const cv::Vec4i line,const Eigen::Vector4d param, 
                Eigen::Vector3d &planeNormalVec,Eigen::Vector3d &verticalVec, Eigen::Vector3d &circleCentreInInit);
    void buildFront();
    void buildBack();
    void buildSide();
    void buildStructure1();
    void buildStructure2();
    ~ModelNode();
};