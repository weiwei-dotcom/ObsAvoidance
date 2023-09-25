#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "cv_bridge/cv_bridge.h"
#include "sophus/se3.hpp"
#include <Eigen/Eigen>
#include "interface/srv/map_point.hpp"
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

class ModelNode : public rclcpp::Node
{
private:
// 最小二乘各颜色平面D参数的次数
int timesIteration_BlueD,timesIteration_GreenD,timesIteration_RedD;
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

// 点云列表的最大长度
int maxLengthPointCloudList;

// 直线拟合参数向量的数量阈值
int threshToOptBlue;
int threshToOptGreen;
int threshToOptRed;

// 平面D参数拟合的ransac迭代次数
int timesRansacIterBlue;
int timesRansacIterGreen;
int timesRansacIterRed;

//每次ransac迭代选择的子集所包含的参数数量
int numPlaneParamSubSet;

// 每次优化更新的标准参数
Eigen::Vector3d standardParamBlue;
Eigen::Vector3d standardParamGreen;
Eigen::Vector3d standardParamRed;

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
cv::Scalar blueUpper,greenUpper,redUpper, redPlusUpper;
cv::Scalar  blueLower, greenLower, redLower, redPlusLower;
cv::Mat kernelOpen_blue, kernelOpen_green, kernelOpen_red;
cv::Mat kernelClose_blue, kernelClose_green, kernelClose_red;

// 定义存储各平面的点云的队列
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleBlueList;
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleGreenList;
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleRedList;

// // 点云转化为当前帧相机像素点坐标
// std::vector<cv::KeyPoint> keyPoints;
// 定义接收服务端 图像、成员变量、相机位姿、点云 的成员变量；
cv::Mat m_img;
Eigen::Matrix4d m_camPose;
Eigen::Matrix4d m_world2cam;
cv::Mat m_worldToCamCV;
pcl::PointCloud<pcl::PointXYZ> m_pointCloud;
rclcpp::Client<interface::srv::MapPoint>::SharedPtr mapPointCli;
double fx, fy, cx, cy;
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

// 相机坐标系下的障碍物点云
pcl::PointCloud<pcl::PointXYZ> obstacleBlueInCam;
pcl::PointCloud<pcl::PointXYZ> obstacleGreenInCam;
pcl::PointCloud<pcl::PointXYZ> obstacleRedInCam;

// 当前蓝绿红色障碍物点云之和
pcl::PointCloud<pcl::PointXYZ> obstacle_current;

void mapPoint_callback(rclcpp::Client<interface::srv::MapPoint>::SharedFuture response) {
    auto result = response.get();
    // 图像赋值给成员变量；
    cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(result->img, "bgr8");
    imgPtr->image.copyTo(m_img);
    // cv::imwrite("/home/weiwei/Desktop/test/opencvTest/build/imagebgr.jpg", m_img);
    // 稀疏点云赋值给成员变量；
    pcl::fromROSMsg(result->point_cloud, m_pointCloud);
    // std::cout << "m_pointCloud size = " << m_pointCloud.points.size() << std::endl;
    // 相机位姿赋值给成员变量
    Eigen::Quaterniond tempQuaCamPose(result->cam_pose.orientation.w,
                        result->cam_pose.orientation.x,
                        result->cam_pose.orientation.y,
                        result->cam_pose.orientation.z);
    Sophus::Vector3d tempTranslattionCamPos(result->cam_pose.position.x,
                                    result->cam_pose.position.y,
                                    result->cam_pose.position.z);
    Sophus::SE3d tempCamPose(tempQuaCamPose, tempTranslattionCamPos);
    m_camPose = tempCamPose.matrix();
    // 第一帧位姿在当前帧位姿下的变换赋值给成员变量
    Eigen::Quaterniond tempQua(result->world2cam.orientation.w,
                        result->world2cam.orientation.x,
                        result->world2cam.orientation.y,
                        result->world2cam.orientation.z);
    Sophus::Vector3d tempTranslattion(result->world2cam.position.x,
                                    result->world2cam.position.y,
                                    result->world2cam.position.z);
    Sophus::SE3d tempWorld2Cam(tempQua, tempTranslattion);
    m_world2cam = tempWorld2Cam.matrix();
    int q=cv::waitKey(20);
    if (q!=27){
        // 开始对障碍物进行建模
        Model();
        subscribMapPoint();
    }
    return;
}

public:
    // 初始化函数 init function()
    ModelNode():Node("model")
    {
        ofs.open("/home/weiwei/Desktop/project/ObsAvoidance/src/model/ParamD.txt");
        octoMapPub = this->create_publisher<octomap_msgs::msg::Octomap>("obstacle", 10);
        mapPointCli = this->create_client<interface::srv::MapPoint>("mappoint");
        cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance/src/config.yaml", cv::FileStorage::READ);
        // 读取拟合平面最小点云数参数
        minNumBlueKeyPoint = fileRead["minNumBlueKeyPoint"];
        minNumGreenKeyPoint = fileRead["minNumGreenKeyPoint"];
        minNumRedKeyPoint = fileRead["minNumRedKeyPoint"];
        int blueLower1 = fileRead["blueLower.1"];
        int blueLower2 = fileRead["blueLower.2"];
        int blueLower3 = fileRead["blueLower.3"];
        int blueUpper1 = fileRead["blueUpper.1"];
        int blueUpper2 = fileRead["blueUpper.2"];
        int blueUpper3 = fileRead["blueUpper.3"];
        int greenLower1 = fileRead["greenLower.1"];
        int greenLower2 = fileRead["greenLower.2"];
        int greenLower3 = fileRead["greenLower.3"];
        int greenUpper1 = fileRead["greenUpper.1"];
        int greenUpper2 = fileRead["greenUpper.2"];
        int greenUpper3 = fileRead["greenUpper.3"];
        int redLower1 = fileRead["redLower.1"];
        int redLower2 = fileRead["redLower.2"];
        int redLower3 = fileRead["redLower.3"];
        int redUpper1 = fileRead["redUpper.1"];
        int redUpper2 = fileRead["redUpper.2"];
        int redUpper3 = fileRead["redUpper.3"];
        int redPlusUpper1 = fileRead["redPlusUpper.1"];
        int redPlusUpper2 = fileRead["redPlusUpper.2"];
        int redPlusUpper3 = fileRead["redPlusUpper.3"];
        int redPlusLower1 = fileRead["redPlusLower.1"];
        int redPlusLower2 = fileRead["redPlusLower.2"];
        int redPlusLower3 = fileRead["redPlusLower.3"];
        blueLower = cv::Scalar(blueLower1, blueLower2, blueLower3);
        blueUpper = cv::Scalar(blueUpper1, blueUpper2, blueUpper3);
        greenLower = cv::Scalar(greenLower1, greenLower2, greenLower3);
        greenUpper = cv::Scalar(greenUpper1, greenUpper2, greenUpper3);
        redLower = cv::Scalar(redLower1, redLower2, redLower3);
        redUpper = cv::Scalar(redUpper1, redUpper2, redUpper3);
        redPlusLower = cv::Scalar(redPlusLower1, redPlusLower2, redPlusLower3);
        redPlusUpper = cv::Scalar(redPlusUpper1, redPlusUpper2, redPlusUpper3);
        int open_blue_kernel_size = fileRead["open_blue_kernel_size"];
        int close_blue_kernel_size = fileRead["close_blue_kernel_size"];
        int open_green_kernel_size = fileRead["open_green_kernel_size"];
        int close_green_kernel_size = fileRead["close_green_kernel_size"];
        int open_red_kernel_size = fileRead["open_red_kernel_size"];
        int close_red_kernel_size = fileRead["close_red_kernel_size"];
        cv::Mat kernelOpen_blue = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_blue_kernel_size, open_blue_kernel_size));
        cv::Mat kernelClose_blue = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_blue_kernel_size, close_blue_kernel_size));
        cv::Mat kernelOpen_green = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_green_kernel_size, open_green_kernel_size));
        cv::Mat kernelClose_green = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_green_kernel_size, close_green_kernel_size));
        cv::Mat kernelOpen_red = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_red_kernel_size, open_red_kernel_size));
        cv::Mat kernelClose_red = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_red_kernel_size, close_red_kernel_size));
        fx = fileRead["Camera.fx"];
        fy = fileRead["Camera.fy"];
        cx = fileRead["Camera.cx"];
        cy = fileRead["Camera.cy"];
        m_projectMatrix << fx, 0, cx,
                           0, fy, cy,
                           0,  0,  1; 
        octreeResolution = fileRead["octoTree.resolution"];
        obstacleBlueInCam.clear();
        obstacleGreenInCam.clear();
        obstacleRedInCam.clear();
        threshToOptBlue = fileRead["threshToOptBlue"];
        threshToOptGreen = fileRead["threshToOptGreen"];
        threshToOptRed = fileRead["threshToOptRed"];
        numPlaneParamSubSet = fileRead["numPlaneParamSubSet"];
        timesRansacIterBlue = fileRead["timesRansacIterBlue"];
        timesRansacIterGreen = fileRead["timesRansacIterGreen"];
        timesRansacIterRed = fileRead["timesRansacIterRed"];
        cosValueTresh = fileRead["cosValueTresh"];
        maxLengthPointCloudList = fileRead["maxLengthPointCloudList"];
    }

    // 该函数用来图像预处理，同时对障碍物进行二值化分割
    void preProcAndThresh()
    {
        threshBlue();
        threshGreen();
        threshRed();
    }

    // 分别对红绿蓝色障碍物区域进行分割
    void threshBlue() {
        cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
        cv::inRange(imageHSV, blueLower, blueUpper, imageBin_blue);
        cv::morphologyEx(imageBin_blue, imgBlueOpen, cv::MORPH_OPEN, kernelOpen_blue);
        cv::morphologyEx(imgBlueOpen, imgBlueClose, cv::MORPH_CLOSE, kernelClose_blue);  
    }
    void threshGreen() {
        cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
        cv::inRange(imageHSV, greenLower, greenUpper, imageBin_green);
        cv::morphologyEx(imageBin_green, imgGreenOpen, cv::MORPH_OPEN, kernelOpen_green);
        cv::morphologyEx(imgGreenOpen, imgGreenClose, cv::MORPH_CLOSE, kernelClose_green);   
    }
    void threshRed() {
        cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
        cv::Mat tempImgBin_red1, tempImgBin_red2;
        cv::inRange(imageHSV, redLower, redUpper, tempImgBin_red1);
        cv::inRange(imageHSV, redPlusLower, redPlusUpper, tempImgBin_red2);
        imageBin_red = tempImgBin_red1 + tempImgBin_red2;
        cv::morphologyEx(imageBin_red, imgRedOpen, cv::MORPH_OPEN, kernelOpen_red);
        cv::morphologyEx(imgRedOpen, imgRedClose, cv::MORPH_CLOSE, kernelClose_red);  
    }

    void getBlueContour() {
        cv::Mat labels_blue, stats_blue, centroids_blue;
        int num_labels_blue = connectedComponentsWithStats(imgBlueClose, labels_blue, stats_blue, centroids_blue, 8, CV_16U);
        // 获取连通域的面积
        contours_blue_ex.clear(), contours_blue_in.clear();
        // std::cout << 263 << " " ;
        std::vector<std::vector<cv::Point>> temp_contours_blue;
        std::vector<int> areas_blue;
        // 如果大于两个轮廓，也就是包括背景在内的至少有三个轮廓，那么可能存在圆孔的轮廓输出障碍物以及圆孔轮廓
        if (num_labels_blue>2)
        {
            for (int i = 1; i < num_labels_blue; i++) // 忽略背景标签0
            {
                areas_blue.push_back(stats_blue.at<int>(i, cv::CC_STAT_AREA));
            }
            // std::cout << "outofrange" << std::endl; // ......................... 
            int max_area_label_blue = max_element(areas_blue.begin(), areas_blue.end()) - areas_blue.begin() + 1;
            cv::findContours((labels_blue == max_area_label_blue), temp_contours_blue, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);   
            contours_blue_ex = temp_contours_blue[0];
            int num_contour = temp_contours_blue.size();
            int index_maxLength=0;
            double maxLength = 0;
            for (int i = 1; i<num_contour;i++)
            {
                double length = cv::arcLength(temp_contours_blue[i], close);
                if (length>maxLength) {
                    maxLength=length;
                    index_maxLength = i;
                }
            }
            contours_blue_in=temp_contours_blue[index_maxLength];
        }
        // 如果只有两个轮廓那么可能只有外层的障碍物轮廓
        else if(num_labels_blue == 2)
        {
            // std::cout << "183" << std::endl; // .........................
            cv::findContours((labels_blue == 1), temp_contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
            contours_blue_ex = temp_contours_blue[0];
            // std::cout << "191" << std::endl; // .........................
        }
    }
    void getGreenContour() {
        cv::Mat labels_green, stats_green, centroids_green;
        // std::cout << 302<< " "  ;
        int num_labels_green = connectedComponentsWithStats(imgGreenClose, labels_green, stats_green, centroids_green, 8, CV_16U);
        contours_green.clear();
        // std::cout << 305 << " " ;
        std::vector<std::vector<cv::Point>> temp_contours_green;
        std::vector<int> areas_green;
        if (num_labels_green>1)
        {
            for (int i = 1; i < num_labels_green; i++) // 忽略背景标签0
            {
                areas_green.push_back(stats_green.at<int>(i, cv::CC_STAT_AREA));
            }
            int max_area_label_green = max_element(areas_green.begin(), areas_green.end()) - areas_green.begin() + 1;
            cv::findContours((labels_green == max_area_label_green), temp_contours_green, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);            
            contours_green = temp_contours_green[0];
        }  
    }
    void getRedContour() {
        cv::Mat labels_red, stats_red, centroids_red;
        // std::cout << 320 << " ";
        int num_labels_red = connectedComponentsWithStats(imgRedClose, labels_red, stats_red, centroids_red, 8, CV_16U);
        contours_red.clear();
        // std::cout << 324 <<" " ;
        std::vector<std::vector<cv::Point>> temp_contours_red;
        std::vector<int> areas_red;
        if (num_labels_red>1)
        {
            for (int i = 1; i < num_labels_red; i++) // 忽略背景标签0
            {
                areas_red.push_back(stats_red.at<int>(i, cv::CC_STAT_AREA));
            }
            int max_area_label_red = max_element(areas_red.begin(), areas_red.end()) - areas_red.begin() + 1;
            cv::findContours((labels_red == max_area_label_red), temp_contours_red, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);             
            contours_red = temp_contours_red[0];
        }
    }

    void selMaxArea()
    {
        // std::cout << 342 << " ";
        getBlueContour();
        // std::cout << 344 << " ";
        getRedContour();
        // std::cout << 346<<" ";
        getGreenContour();
        // FIXME : 这个函数最好是用多线程来分别分割红绿蓝三种颜色的区域，节省时间；
        return;
    }

    void invProjAndSel()
    {
        // 将世界坐标系下的点云投影到相机坐标系下
        pcl::PointCloud<pcl::PointXYZ> pointCloudInCamFrame;
        pcl::transformPointCloud(m_pointCloud, pointCloudInCamFrame, m_world2cam);

        std::vector<cv::Point2f> projectPointsInImg;

        // 调试查看特征点的筛选情况
        std::vector<cv::Point2f> projectPointsInImg_blue; // .....................
        std::vector<cv::Point2f> projectPointsInImg_green;
        std::vector<cv::Point2f> projectPointsInImg_red;

        int contour_blue_ex_size = contours_blue_ex.size();
        int contour_blue_in_size = contours_blue_in.size();
        int contour_green_size = contours_green.size();
        int contour_red_size = contours_red.size();
        bool flag_find_contour_blue = (contour_blue_ex_size > 0);
        bool flag_find_contour_green = (contour_green_size > 0);
        bool flag_find_contour_red = (contour_red_size > 0);
        pointCloudInCamFrame_blue.clear();
        pointCloudInCamFrame_green.clear();
        pointCloudInCamFrame_red.clear();

        pointCloud_blue.clear();
        pointCloud_green.clear();
        pointCloud_red.clear();

        if (!(flag_find_contour_blue || flag_find_contour_red || flag_find_contour_green))
        {
            std::cout << "并没有找到轮廓" << std::endl;
            return;
        }
        int index = 0;
        std::vector<int> indexPointCloudBlue;
        std::vector<int> indexPointCloudGreen;
        std::vector<int> indexPointCloudRed;
        for(auto point:pointCloudInCamFrame.points)
        {
            Eigen::Vector3f tempProjectPoint = m_projectMatrix * point.getVector3fMap();
            // std::cout << tempProjectPoint << std::endl; //.........................
            cv::Point2f imagePoint(tempProjectPoint[0]/tempProjectPoint[2], tempProjectPoint[1]/tempProjectPoint[2]);
            // std::cout << imagePoint << std::endl; //.........................
            projectPointsInImg.push_back(imagePoint);
            if (flag_find_contour_blue && contour_blue_in_size > 0)
            {
                // std::cout << "310" << std::endl; // .........................
                double distance_blue_ex = cv::pointPolygonTest(contours_blue_ex, imagePoint, false);
                double distance_blue_in = cv::pointPolygonTest(contours_blue_in, imagePoint, false);
                if (distance_blue_ex > 0 && distance_blue_in < 0 )
                {
                    // std::cout << "315" << std::endl; // .........................
                    pointCloudInCamFrame_blue.points.push_back(point);
                    indexPointCloudBlue.push_back(index);
                    projectPointsInImg_blue.push_back(imagePoint); // ...........................
                }
            }
            else if(flag_find_contour_blue)
            {
                // std::cout << "321" << std::endl; // .........................
                double distance_blue_ex = cv::pointPolygonTest(contours_blue_ex, imagePoint, false);
                if (distance_blue_ex > 0)
                {

                    // std::cout << "325" << std::endl; // .........................
                    pointCloudInCamFrame_blue.points.push_back(point);
                    indexPointCloudBlue.push_back(index);
                    projectPointsInImg_blue.push_back(imagePoint); // ...........................
                }
            }
            if(flag_find_contour_green){
                // std::cout << "330" << std::endl; // .........................
                double distance_green = cv::pointPolygonTest(contours_green, imagePoint, false);  
                if (distance_green > 0) {
                    // std::cout << "333" << std::endl; // .........................
                    pointCloudInCamFrame_green.points.push_back(point);
                    indexPointCloudGreen.push_back(index);
                    projectPointsInImg_green.push_back(imagePoint); // ...........................
                }              
            }
            if (flag_find_contour_red) {
                // std::cout << "338" << std::endl; // .........................
                double distance_red = cv::pointPolygonTest(contours_red, imagePoint, false); 
                if (distance_red > 0) {
                    // std::cout << "341" << std::endl; // .........................
                    pointCloudInCamFrame_red.points.push_back(point);
                    indexPointCloudRed.push_back(index);
                    projectPointsInImg_red.push_back(imagePoint); // .................
                }                
            }
            index++;
        }
        for(int i=0; i<indexPointCloudBlue.size();i++)
        {
            pointCloud_blue.points.push_back(m_pointCloud.points.at(indexPointCloudBlue[i]));
        }
        for(int i=0; i<indexPointCloudGreen.size();i++)
        {
            pointCloud_green.points.push_back(m_pointCloud.points.at(indexPointCloudGreen[i]));
        }
        for(int i=0; i<indexPointCloudRed.size();i++)
        {
            pointCloud_red.points.push_back(m_pointCloud.points.at(indexPointCloudRed[i]));
        }
        return;
    }

    void calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud, Eigen::Vector4d &param) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_Ptr=pointCloud_blue.makeShared();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 设置分割系数
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01);
        // 从点云中分割最有可能的平面
        seg.setInputCloud(pointCloud_Ptr);
        pcl::ModelCoefficients coefficient;
        seg.segment(*inliers, coefficient);        
        param(0) = coefficient.values[0];
        param(1) = coefficient.values[1];
        param(2) = coefficient.values[2];
        param(3) = coefficient.values[3];
    }

    // 获取当前用来投影的平面参数值
    void fitPlaneBlue() {
        // 根据当前点云数量计算当前平面参数
        calParam(pointCloud_blue, coefficientBlue);
        // 如果优化过了，与优化过的标准参数中的法向量进行比较
        if (flag_optimized_blue){
            double cosValue=coefficientBlue.block(0,0,3,1).dot(standardParamBlue);
            // 如果在阈值范围内
            if (cosValue > cosValueTresh){
                // 将满足要求的参数付给当前参数
                paramBlue=coefficientBlue;
            }
        }
        // 如果没有优化过则直接使用当前参数变量作为拟合平面变量
        else
            paramBlue = coefficientBlue;
        // 将当前参数变量添加到蓝色参数列表中
        paramListBlue.push_back(coefficientBlue);
    }
    void fitPlaneGreen() {
        // 根据当前点云数量计算当前平面参数
        calParam(pointCloud_green, coefficientGreen);
        // 如果优化过了，与优化过的标准参数中的法向量进行比较
        if (flag_optimized_green){
            double cosValue=coefficientGreen.block(0,0,3,1).dot(standardParamGreen);
            // 如果在阈值范围内
            if (cosValue > cosValueTresh){
                // 将满足要求的参数付给当前参数
                paramGreen=coefficientGreen;
            }
        }
        // 如果没有优化过则直接使用当前参数变量作为拟合平面变量
        else
            paramGreen = coefficientGreen;
        // 将当前参数变量添加到蓝色参数列表中
        paramListGreen.push_back(coefficientGreen);
    }
    void fitPlaneRed() {
        // 根据当前点云数量计算当前平面参数
        calParam(pointCloud_red, coefficientRed);
        // 如果优化过了，与优化过的标准参数中的法向量进行比较
        if (flag_optimized_red){
            double cosValue=coefficientRed.block(0,0,3,1).dot(standardParamRed);
            // 如果在阈值范围内
            if (cosValue > cosValueTresh){
                // 将满足要求的参数付给当前参数
                paramRed=coefficientRed;
            }
        }
        // 如果没有优化过则直接使用当前参数变量作为拟合平面变量
        else
            paramRed = coefficientRed;
        // 将当前参数变量添加到蓝色参数列表中
        paramListRed.push_back(coefficientRed);
    }

    void transformParamToCamFrame(Eigen::Vector4d &paramInWorldFrame, Eigen::Vector4d &paramInCamFrame) {
        paramInCamFrame.block(0,0,3,1) = m_world2cam.block(0,0,3,3)*paramInWorldFrame.block(0,0,3,1);
        Eigen::Vector3d tempCamPosition = m_camPose.block(0,3,3,1);
        paramInCamFrame(3) = paramInWorldFrame(3) + tempCamPosition.dot(paramInWorldFrame.block(0,0,3,1));
    }
    void pixelToPlane(cv::Mat &imgClose, Eigen::Vector4d &paramInCamFrame, pcl::PointCloud<pcl::PointXYZ> &obstacle) {
        double a = paramInCamFrame[0];
        double b = paramInCamFrame[1];
        double c = paramInCamFrame[2];
        double d = paramInCamFrame[3];
        pcl::PointCloud<pcl::PointXYZ> obstacleInCamFrame;
        int rows = imgClose.rows;
        int cols = imgClose.cols;
        cv::Mat temp_imgClose;
        imgClose.convertTo(temp_imgClose, CV_8UC1);
        for (int u=0;u<rows;u+2) {
            for (int v=0;v<cols;v+2) {
                if (temp_imgClose.at<uchar>(u,v) == 255) {
                    pcl::PointXYZ tempPoint;
                    tempPoint.z = -d/(a*(v-cx)/fx+b*(u-cy)/fy+c);
                    tempPoint.x = tempPoint.z*(v-cx)/fx;
                    tempPoint.y = tempPoint.z*(u-cy)/fy;
                    obstacleInCamFrame.points.push_back(tempPoint);
                }
            }
        };
        pcl::transformPointCloud(obstacleInCamFrame, obstacle, m_camPose);
    };
    void updateObstacle(std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleList, pcl::PointCloud<pcl::PointXYZ> obstacle) {
        obstacleList.push_back(obstacle);
        if (obstacleList.size() > maxLengthPointCloudList) {
            std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator start=obstacleList.begin()+1;
            std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator end=obstacleList.end();
            std::vector<pcl::PointCloud<pcl::PointXYZ>> tempPointCloudList(start,end);
            obstacleList=tempPointCloudList;
        }
    }
    void pixelToBluePlane(){
        Eigen::Vector4d paramInCamFrame;
        transformParamToCamFrame(paramBlue, paramInCamFrame);
        pixelToPlane(imgBlueClose, paramInCamFrame, obstacleBlue);
        updateObstacle(obstacleBlueList, obstacleBlue);
    }
    void pixelToGreenPlane() {
        Eigen::Vector4d paramInCamFrame;
        transformParamToCamFrame(paramGreen, paramInCamFrame);
        pixelToPlane(imgGreenClose, paramInCamFrame, obstacleGreen);
        updateObstacle(obstacleGreenList, obstacleGreen);
    }
    void pixelToRedPlane() {
        Eigen::Vector4d paramInCamFrame;
        transformParamToCamFrame(paramRed, paramInCamFrame);
        pixelToPlane(imgRedClose, paramInCamFrame, obstacleRed);
        updateObstacle(obstacleRedList, obstacleRed);
    }
    void optimizeParam(int timesRansacIter, std::vector<Eigen::Vector4d> &paramList, Eigen::Vector3d &standardParam, bool &flag_optimized,int threshToOpt) {
        if (paramList.size() == threshToOpt) {
            int maxNumInlier=0;
            for (int i=0;i<timesRansacIter;i++) {
                std::random_shuffle(paramList.begin(), paramList.end());
                ceres::Problem problem;
                double a=0,b=0;
                for (int j=0;j<numPlaneParamSubSet;j++) {
                    problem.AddResidualBlock(
                        new ceres::AutoDiffCostFunction<normVecResidual,1,1,1>(
                            new normVecResidual(paramList[j].x(), paramList[j].y(), paramList[j].z())
                        ),
                        NULL,
                        &a,&b
                    );           
                }
                ceres::Solver::Options options;
                options.max_num_iterations = 25;
                options.linear_solver_type = ceres::DENSE_QR;
                options.minimizer_progress_to_stdout = true;
                options.logging_type = ceres::SILENT;
                ceres::Solver::Summary summary;
                Solve(options, &problem, &summary);
                double c = sqrt(1-pow(a,2)-pow(b,2));
                std::cout << "A: " << a << "  B: " << b << "  C: " << c <<endl;
                // 得到优化的变量以后使用该参数模型利用与样点参数之间的夹角余弦是否在夹角阈值范围内来筛选内点，并统计内点数量
                int tempNumInlier=0;
                for(int n=0;n<paramList.size();n++) {
                    if (a*paramList[n](0)+b*paramList[n](1)+c*paramList[n](2) > cosValueTresh) {
                        tempNumInlier++;
                    }
                }
                if (maxNumInlier < tempNumInlier) {
                    maxNumInlier = tempNumInlier;
                    standardParam(0)=a;
                    standardParam(1)=b;
                    standardParam(2)=c;
                }
            }
            flag_optimized = true; 
            paramList.clear();
        }
    }
    // 发布八叉树模型
    void octoTreeCurPub()
    {
        pcl::PointCloud<pcl::PointXYZ> tempObstacle;
        for (int i=0;i<obstacleBlueList.size();i++) {
            tempObstacle+=obstacleBlueList[i];
        }
        for (int i=0;i<obstacleGreenList.size();i++) {
            tempObstacle+=obstacleGreenList[i];
        }
        for (int i=0;i<obstacleRedList.size();i++) {
            tempObstacle+=obstacleRedList[i];
        }
        if (tempObstacle.points.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "当前八叉树模型为空,请对准障碍物平台");
            return ;
        }
        octomap::OcTree octree(octreeResolution);
        for (auto point:tempObstacle)
        {
            octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
        octree.updateInnerOccupancy();
        octomap_msgs::msg::Octomap octoMap;
        octoMap.header.frame_id = "world_frame";
        octomap_msgs::fullMapToMsg(octree, octoMap);
        octoMapPub->publish(octoMap);
        std::cout << "octoMap published success" << std::endl;
        return;
    }
    // 障碍物建模主函数
    void Model()
    {
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // std::cout << "preProcAndThresh" << std::endl;
        // 图像预处理以及图像阈值分割
        preProcAndThresh();
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::cout << "selMaxArea" << std::endl;
        // 对分割图像选取最大连通域的轮廓
        selMaxArea();
        // std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
        // std::cout << "invProjAndSel" << std::endl;
        // 特征点逆投影并使用轮廓筛选
        invProjAndSel();
        // std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
        // // 平面拟合函数
        // transformParamABC();
        if (pointCloudInCamFrame_blue.size() > minNumBlueKeyPoint) {
            // std::cout << "fitPlaneBlue" << std::endl;
            fitPlaneBlue();
            // std::cout << "pixelToBluePlane" << std::endl;
            pixelToBluePlane();
            // 参数列表长度到达设定长度 对其进行优化
            optimizeParam(timesRansacIterBlue, paramListBlue, standardParamBlue, flag_optimized_blue,threshToOptBlue);
        }
        // std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
        if(pointCloudInCamFrame_green.size() > minNumGreenKeyPoint){
            // std::cout << "fitPlaneGreen" << std::endl;
            fitPlaneGreen();
            // std::cout << "pixelToGreenPlane" << std::endl;
            pixelToGreenPlane();
            // std::cout << "greenPlaneToWorld" << std::endl;   
            optimizeParam(timesRansacIterGreen, paramListGreen, standardParamGreen, flag_optimized_green,threshToOptGreen);        
        }
        // std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
        if (pointCloudInCamFrame_red.size() > minNumRedKeyPoint){
            // std::cout << "fitPlaneRed" << std::endl;
            fitPlaneRed();
            // std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
            // std::cout << "pixelToRedPlane" << std::endl;
            pixelToRedPlane();
            // 点云拼接并转化为八叉树消息发布并在rviz中显示
            // std::cout << "redPlaneToWorld" << std::endl;
            optimizeParam(timesRansacIterRed, paramListRed, standardParamRed, flag_optimized_red,threshToOptRed);         
        }
        // std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
        // 测试代码
        // std::cout << "octoTreeCurPub" << std::endl;
        octoTreeCurPub();
    }

    // 服务端发起函数
    void subscribMapPoint()
    {
        //FIXME: 在等待服务端上线时，使用ctrl_c关闭该进程 导致终端一直执行while中的语句且之后不会再发起服务；
        while(!mapPointCli->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "waiting the mapPoint servier online !!!");
        }
        auto request = std::make_shared<interface::srv::MapPoint_Request>();
        mapPointCli->async_send_request(request, std::bind(&ModelNode::mapPoint_callback, this, std::placeholders::_1));
    }
    // 析构函数 delete function()
    ~ModelNode(){
        ofs.close();
    }
};

// 主程序
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModelNode>();
    node->subscribMapPoint();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}