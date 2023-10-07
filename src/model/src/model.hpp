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
        TooClose = 7
    };
    // 定义蓝色平面法向量残差函数结构体
    struct normVecResidual {
        normVecResidual(float x, float y, float z)
                : x_((double)x), y_((double)y), z_((double)z) {}
        template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const {
            residual[0] = 1.0-x_*a[0]-y_*b[0]-z_*sqrt(1.0-pow(a[0],2)-pow(b[0],2));
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

    // 点云列表的最大长度
    int maxLengthPointCloudList;

    // 直线拟合参数向量的数量阈值
    int threshToOptBlue;
    int threshToOptGreen;
    int threshToOptRed;

    // 是否完成建模的flag变量
    bool flag_pubModel;

    // ransac迭代次数
    int timesRansacIterBlue;
    int timesRansacIterGreen;
    int timesRansacIterRed;

    //每次ransac迭代选择的子集所包含的参数数量
    int numPlaneParamSubSet;

    // 存储每次检测得到的二维直线方向向量
    std::vector<Eigen::Vector2d> horizontalLine;
    std::vector<Eigen::Vector2d> vertialLine;
    // // 存储每次检测得到的三维直线参数变量
    // std::vector<Eigen::Matrix<double, 6, 1>> upLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> underLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> leftLine3d;
    // std::vector<Eigen::Matrix<double, 6, 1>> rightLine3d;
    
    // 存储每次检测得到的从圆心位置出发竖直向上的三维向量变量
    std::vector<Eigen::Vector3d> vec_upward;
    // 存储每次检测得到的平面法向量变量
    std::vector<Eigen::Vector3d> vec_norm;
    // 存储每次检测得到的三维圆心位置
    std::vector<Eigen::Vector3d> center_circle;
    // 每次优化更新的标准参数
    Eigen::Vector3d standardParamBlue;
    Eigen::Vector3d standardParamGreen;
    Eigen::Vector3d standardParamRed;

    // 圆形检测函数参数
    double minDist, dp, cannyUpThresh, circleThresh;
    int minRadius, maxRadius;
    // 两帧圆之间的半径差阈值、圆心距离阈值
    float difference_radius_thresh, distance_center_thresh;
    // 多次圆形检测记录的圆
    std::vector<cv::Vec3f> circles_;
    // 检测平面与相机之间的距离阈值
    double distance_thresh;

    // 法向量参数的cos阈值
    double cosValueTresh;

    // 拟合蓝色障碍物平面获得的法向量系数 A B C(Ax+By+Cz+D=0),系数D根据不同平面上的点计算
    std::vector<Eigen::Vector4d> paramListBlue;
    std::vector<Eigen::Vector4d> paramListGreen;
    std::vector<Eigen::Vector4d> paramListRed;
    Eigen::Vector4d paramBlue;
    Eigen::Vector4d paramGreen;
    Eigen::Vector4d paramRed;
    double param_D_bluePlaneInCamFrame;
    double param_D_greenPlaneInCamFrame;
    double param_D_redPlaneInCamFrame;
    std::ofstream ofs;

    // 能够进行点云拟合平面的最小点云数量值
    int minNumBlueKeyPoint;
    int minNumGreenKeyPoint;
    int minNumRedKeyPoint;

    // octoTree分辨率
    double octreeResolution;
    // 消息的header
    std_msgs::msg::Header m_header_initFrame;

    // 錯誤狀態變量
    ERROR_TYPE error_type;

    // octoMap发布者指针创建
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octoMapPub;

    // 图像处理阶段需要用到的变量
    std::vector<cv::Point> contours_blue, contours_blue_ex, contours_blue_in, contours_green, contours_red;
    cv::Mat redSegImg;
    cv::Mat blueSegImg;
    cv::Mat greenSegImg;
    cv::Mat imgGaussian, imageHSV;
    cv::Mat imageBin_blue, imageBin_green, imageBin_red;
    cv::Mat imgBlueOpen, imgGreenOpen, imgRedOpen;
    cv::Mat imgBlueClose, imgGreenClose, imgRedClose;
    cv::Scalar blueUpper,greenUpper,redUpper, redPlusUpper, whiteLower, whiteUpper;
    cv::Scalar blueLower, greenLower, redLower, redPlusLower;
    cv::Mat kernelOpen_blue, kernelOpen_green, kernelOpen_red;
    cv::Mat kernelClose_blue, kernelClose_green, kernelClose_red;

    // 定义存储各平面的点云的队列
    std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleBlueList;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleGreenList;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleRedList;

    // 腐蚀膨胀操作结构体
    cv::Mat structure_erode;
    cv::Mat structure_dilate;

    // 用来判断相机是否正视于检测平面的余弦角度阈值
    double cosValue_thresh;

    // // 点云转化为当前帧相机像素点坐标
    // std::vector<cv::KeyPoint> keyPoints;
    // 定义接收服务端 图像、成员变量、相机位姿、点云 的成员变量；
    cv::Mat m_img;
    Eigen::Matrix4d m_transform_curToInit;
    Eigen::Matrix4d m_transform_initToCur;
    cv::Mat m_worldToCamCV;
    pcl::PointCloud<pcl::PointXYZ> m_pointCloud;
    float fx, fy, cx, cy;
    Eigen::Matrix3f m_projectMatrix;
    pcl::PointCloud<pcl::PointXYZ> pointCloudInCamFrame_blue;
    pcl::PointCloud<pcl::PointXYZ> pointCloudInCamFrame_green;
    pcl::PointCloud<pcl::PointXYZ> pointCloudInCamFrame_red;

    pcl::PointCloud<pcl::PointXYZ> pointCloud_blue;
    pcl::PointCloud<pcl::PointXYZ> pointCloud_green;
    pcl::PointCloud<pcl::PointXYZ> pointCloud_red;

    // 当前拟合得到的平面系数
    Eigen::Vector4d coefficientBlue;
    Eigen::Vector4d coefficientGreen;
    Eigen::Vector4d coefficientRed;
    // 各颜色平面是否优化过的标志
    bool flag_optimized_blue;
    bool flag_optimized_green;
    bool flag_optimized_red;
    // 当前蓝色、绿色、红色障碍物点云
    pcl::PointCloud<pcl::PointXYZ> obstacleBlue;
    pcl::PointCloud<pcl::PointXYZ> obstacleGreen;
    pcl::PointCloud<pcl::PointXYZ> obstacleRed;

    // 历史蓝绿红色障碍物点云之和
    pcl::PointCloud<pcl::PointXYZ> obstacle;

    pcl::PointCloud<pcl::PointXYZ> pcl_obstacle;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub;

    // 相机坐标系下的障碍物点云
    pcl::PointCloud<pcl::PointXYZ> obstacleBlueInCam;
    pcl::PointCloud<pcl::PointXYZ> obstacleGreenInCam;
    pcl::PointCloud<pcl::PointXYZ> obstacleRedInCam;

    // 当前蓝绿红色障碍物点云之和
    pcl::PointCloud<pcl::PointXYZ> obstacle_current;

    void mapPoint_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);

public:
    bool recieveMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg);
    // 构造函数 init function()
    ModelNode();
    // 该函数用来图像预处理，同时对障碍物进行二值化分割
    void preProcAndThresh();

    // 分别对红绿蓝色障碍物区域进行分割
    void threshBlue();
    void threshGreen();
    void threshRed();

    void getBlueContour();
    void getGreenContour();

    void getRedContour() ;

    void selMaxArea();
    void invProjAndSel();

    void calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud, Eigen::Vector4d &param);

    // 获取当前用来投影的平面参数值
    void fitPlaneBlue();
    void fitPlaneGreen();
    void fitPlaneRed();

    void transformParamToCamFrame(Eigen::Vector4d &paramInWorldFrame, Eigen::Vector4d &paramInCamFrame);
    void pixelToPlane(cv::Mat &imgClose, Eigen::Vector4d &paramInCamFrame, pcl::PointCloud<pcl::PointXYZ> &obstacle);
    void updateObstacle(std::vector<pcl::PointCloud<pcl::PointXYZ>> &obstacleList, pcl::PointCloud<pcl::PointXYZ> &obstacle) ;
    void pixelToBluePlane();
    void pixelToGreenPlane();
    void pixelToRedPlane();
    void optimizeParam(int timesRansacIter, std::vector<Eigen::Vector4d> &paramList, Eigen::Vector3d &standardParam, bool &flag_optimized,int threshToOpt);
    // 发布八叉树模型
    void octoTreeCurPub();
    // 障碍物建模主函数
    bool Model();
    // Publish the message of pointcloud of obstacle model;
    void PubModel();
    void changeErrorType(ERROR_TYPE newError);
    bool detectPoseCorrect(double &distance_plane, const cv::Vec3f tempCircle, cv::Mat &mask);
    bool getMaxAreaContour(cv::Mat &img_bin, std::vector<cv::Point> &contour);
    Eigen::Vector4d calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud);
    ~ModelNode();
};