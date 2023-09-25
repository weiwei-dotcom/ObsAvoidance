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
#include <queue>

class ModelNode : public rclcpp::Node
{
private:
// 定义平面参数二范数残差结构体
struct paramDResidual {
    paramDResidual(float a, float b, float c)
            : a_((double)a), b_((double)b), c_((double)c) {}
    template <typename T> bool operator()(const T* const A,const T* const B,const T* const C,const T* const D, T* residual) const {
        residual[0] = pow(A[0]-a,2)+pow(B[0]-b,2)+pow(C[0]-c,2)+pow(D[0]-d,2);
        return true;
    }
private:
    const double a_;
    const double b_;
    const double c_;
};

// 存储各平面前最后十次的点云
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleBluePlane;
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleGreenPlane;
std::vector<pcl::PointCloud<pcl::PointXYZ>> obstacleRedPlane;

// ransac优化平面参数样本点数量
int numSampleBlue;
int numSampleGreen;
int numSampleRed;

// 对平面参数优化时的ransac迭代次数
int numRansacIterBlue;
int numRansacIterGreen;
int numRansacIterRed;

// 对平面参数优化时的每次迭代的随机采样数量
int numSubSampleBlue;
int numSubSampleGreen;
int numSubSampleRed;

// 拟合蓝色障碍物平面获得的法向量系数 A B C(Ax+By+Cz+D=0),系数D根据不同平面上的点计算
// 每次拟合时的参数列表
std::vector<Eigen::Vector4d> paramListBlue;
std::vector<Eigen::Vector4d> paramListGreen;
std::vector<Eigen::Vector4d> paramListRed;

// 蓝绿红平面的优化后最终参数值
Eigen::Vector4d paramBlue;
Eigen::Vector4d paramGreen;
Eigen::Vector4d paramRed;

// 写文件流
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

pcl::ModelCoefficients coefficientBlue;
pcl::ModelCoefficients coefficientGreen;
pcl::ModelCoefficients coefficientRed;
bool flag_bluePlaneFit;
bool flag_redPlaneFit;
bool flag_greenPlaneFit;

// 当前蓝色、绿色、红色障碍物点云
pcl::PointCloud<pcl::PointXYZ> obstacleBlue;
pcl::PointCloud<pcl::PointXYZ> obstacleGreen;
pcl::PointCloud<pcl::PointXYZ> obstacleRed;

// 历史蓝绿红色障碍物点云之和
pcl::PointCloud<pcl::PointXYZ> obstacle;

// 相机坐标系下的障碍物点云
pcl::PointCloud<pcl::PointXYZ> obstacleBlueInCamFrame;
pcl::PointCloud<pcl::PointXYZ> obstacleGreenInCamFrame;
pcl::PointCloud<pcl::PointXYZ> obstacleRedInCamFrame;

// 当前蓝绿红色障碍物点云之和
pcl::PointCloud<pcl::PointXYZ> obstacle_current;

void mapPoint_callback(rclcpp::Client<interface::srv::MapPoint>::SharedFuture response)
{
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
    // 继续订阅点云话题接收数据；
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
        numSampleBlue=fileRead["numSampleBlue"];
        numSampleGreen=fileRead["numSampleGreen"];
        numSampleRed=fileRead["numSampleRed"];
        numSubSampleBlue=fileRead["numSubSampleBlue"];
        numSubSampleGreen=fileRead["numSubSampleGreen"];
        numSubSampleRed=fileRead["numSubSampleRed"];
        numRansacIterBlue=fileRead["numRansacIterBlue"];
        numRansacIterGreen=fileRead["numRansacIterGreen"];
        numRansacIterRed=fileRead["numRansacIterRed"];
    }

    // 该函数用来图像预处理，同时对障碍物进行二值化分割
    void preProcAndThresh()
    {
        /*
        对障碍物区域进行分割
        */
        threshBlue();
        threshGreen();
        threshRed();
        // std::cout << 230 << " " ;
        // // 调试代码
        // cv::imshow("imgBlueclose", imgBlueClose);
        // cv::imshow("imgGreenclose", imgGreenClose);
        // cv::imshow("imgRedclose", imgRedClose); 
        // std::cout << 235<<" "; 
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
        // std::cout << 260 << " " ;
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

    // 该函数对障碍物进行轮廓提取
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

    void fitPlaneBlue()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInCamFrameBlue_Ptr=pointCloudInCamFrame_blue.makeShared();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 设置分割系数
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01);
        // 从点云中分割最有可能的平面
        seg.setInputCloud(pointCloudInCamFrameBlue_Ptr);
        seg.segment(*inliers, coefficientBlue);
        flag_bluePlaneFit = true;
        Eigen::Vector4d tempParam(coefficientBlue.values[0],coefficientBlue.values[1],coefficientBlue.values[2],coefficientBlue.values[3]);
        paramListBlue.push_back(tempParam);
    }

    void fitPlaneGreen()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInCamFrameGreen_Ptr=pointCloudInCamFrame_green.makeShared();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 设置分割系数
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01);
        // 从点云中分割最有可能的平面
        seg.setInputCloud(pointCloudInCamFrameGreen_Ptr);
        seg.segment(*inliers, coefficientGreen);
        Eigen::Vector4d tempParam(coefficientGreen.values[0],coefficientGreen.values[1],coefficientGreen.values[2],coefficientGreen.values[3]);
        paramListGreen.push_back(tempParam);
    }

    void fitPlaneRed()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudInCamFrameRed_Ptr=pointCloudInCamFrame_red.makeShared();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 设置分割系数
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.01);
        // 从点云中分割最有可能的平面
        seg.setInputCloud(pointCloudInCamFrameRed_Ptr);
        seg.segment(*inliers, coefficientRed);
        Eigen::Vector4d tempParam(coefficientRed.values[0],coefficientRed.values[1],coefficientRed.values[2],coefficientRed.values[3]);
        paramListRed.push_back(tempParam);
    }

    // // ransac算法求解参数值；
    // void RANSACGetBlueParam()
    // {
    //     // 以百分之五十的好点概率以及十分之一样点数来计算迭代次数
    //     for(int n=0;n<timesRansacIterBlue;n++) {
    //         ceres::Problem problem;
    //         double tempParamD;
    //         std::cout << "PointNum of blue plane: " << pointCloud_blue.points.size()<<std::endl;
    //         for (int i=0;i<pointCloud_blue.points.size();i++){
    //             problem.AddResidualBlock(
    //                 new ceres::AutoDiffCostFunction<paramDResidual, 1, 1>(
    //                     new paramDResidual((double)pointCloud_blue.points.at(i).x, 
    //                                         (double)pointCloud_blue.points.at(i).y,
    //                                         (double)pointCloud_blue.points.at(i).z)),
    //                     NULL,
    //                     &tempParamD
    //             );
    //         }
    //         // 模型配置
    //         ceres::Solver::Options options;
    //         options.max_num_iterations = timesIteration_BlueD;
    //         options.linear_solver_type = ceres::DENSE_QR;
    //         options.minimizer_progress_to_stdout = true;
    //         options.logging_type = ceres::SILENT;
    //         // 总结
    //         ceres::Solver::Summary summary;
    //         Solve(options, &problem, &summary);
    //         // 用该参数计算与所有点云的距离;
    //         for (int i=0;i<pointCloud_blue.points.size();i++) {

    //         }
    //     }
    //     Eigen::Vector3f tempVec=m_camPose.block(0,3,3,1);
    //     param_D_bluePlaneInCamFrame=tempParamD+param_normVec.dot(tempVec);
    // }

    // // 将世界坐标系平面参数转化为相机坐标系
    // void transformParamToCamFrame(Eigen::)
  
    void pixelToBluePlane()
    {
        double a = paramBlue[0];
        double b = paramBlue[1];
        double c = paramBlue[2];
        double d = paramBlue[3];
        std::cout<<"RedPlane: "<<"a="<<a<<" b=" <<b<<" c="<<c<< " d="<<d<<std::endl;
        obstacleBlueInCam.points.clear();
        int rows = imgBlueClose.rows;
        int cols = imgBlueClose.cols;
        cv::Mat temp_imgBlueClose;
        imgBlueClose.convertTo(temp_imgBlueClose, CV_8UC1);
        for (int u=0;u<rows;u++)
        {
            for (int v=0;v<cols;v++)
            {
                if (temp_imgBlueClose.at<uchar>(u,v) == 255)
                {
                    pcl::PointXYZ tempPoint;
                    tempPoint.z = -d/(a*(v-cx)/fx+b*(u-cy)/fy+c);
                    tempPoint.x = tempPoint.z*(v-cx)/fx;
                    tempPoint.y = tempPoint.z*(u-cy)/fy;
                    obstacleBlueInCam.points.push_back(tempPoint);
                }
            }
        }
    }
    void pixelToGreenPlane()
    {
        obstacleGreenInCam.points.clear();
        int rows = imgGreenClose.rows;
        int cols = imgGreenClose.cols;
        cv::Mat temp_imgGreenClose;
        imgGreenClose.convertTo(temp_imgGreenClose, CV_8UC1);
        double a = paramGreen[0];
        double b = paramGreen[1];
        double c = paramGreen[2];
        double d = paramGreen[3];
        std::cout<<"RedPlane: "<<"a="<<a<<" b=" <<b<<" c="<<c<< " d="<<d<<std::endl;
        for (int u=0;u<rows;u++)
        {
            for (int v=0;v<cols;v++)
            {
                if (temp_imgGreenClose.at<uchar>(u,v) == 255)
                {
                    pcl::PointXYZ tempPoint;
                    tempPoint.z = -d/(b*(u-cy)/fy+a*(v-cx)/fx+c);
                    tempPoint.x = tempPoint.z*(v-cx)/fx;
                    tempPoint.y = tempPoint.z*(u-cy)/fy;
                    obstacleGreenInCam.points.push_back(tempPoint);
                }
            }
        }
    }
    void pixelToRedPlane()
    {
        obstacleRedInCam.points.clear();
        int rows = imgRedClose.rows;
        int cols = imgRedClose.cols;
        cv::Mat temp_imgRedClose;
        imgRedClose.convertTo(temp_imgRedClose, CV_8UC1);
        double a = paramRed[0];
        double b = paramRed[1];
        double c = paramRed[2];
        double d = paramRed[3];
        std::cout<<"RedPlane: "<<"a="<<a<<" b=" <<b<<" c="<<c<< " d="<<d<<std::endl;
        for (int u=0;u<rows;u++)
        {
            for (int v=0;v<cols;v++)
            {
                if (temp_imgRedClose.at<uchar>(u,v) == 255)
                {
                    pcl::PointXYZ tempPoint;
                    tempPoint.z = -d/(a*(v-cx)/fx+b*(u-cy)/fy+c);
                    tempPoint.x = tempPoint.z*(v-cx)/fx;
                    tempPoint.y = tempPoint.z*(u-cy)/fy;
                    obstacleRedInCam.points.push_back(tempPoint);
                }
            }
        }
    }

    // 相机坐标系下蓝色平面转换到世界坐标系下
    void bluePlaneToWorld()
    {
        obstacleBlue.points.clear();
        pcl::transformPointCloud(obstacleBlueInCam, obstacleBlue, m_camPose);
        // // 将新的障碍物平面点云添加到历史点云中
        // obstacle+=obstacleBlue;
    }
    // 相机坐标系下绿色平面转换到世界坐标系下
    void greenPlaneToWorld()
    {
        obstacleGreen.points.clear();
        pcl::transformPointCloud(obstacleGreenInCam, obstacleGreen, m_camPose);
    }
    // 相机坐标系下红色平面转换到世界坐标系下
    void redPlaneToWorld()
    {
        obstacleRed.points.clear();
        pcl::transformPointCloud(obstacleRedInCam, obstacleRed, m_camPose);
        // // 将新的障碍物平面点云添加到历史点云中
        // obstacle+=obstacleRed;
    }

    // 点云拼接
    void pclConcatenate()
    {    
        obstacle_current.points.clear();
        obstacle_current+=obstacleBlue;
        obstacle_current+=obstacleGreen;
        obstacle_current+=obstacleRed;
    }

    // 发布当前八叉树模型函数
    void octoTreeCurPub()
    {
        if (obstacle_current.points.size() == 0)
        {
            RCLCPP_INFO(this->get_logger(), "当前八叉树模型为空,请对准障碍物平台");
            return ;
        }
        octomap::OcTree octree(octreeResolution);
        for (auto point:obstacle_current)
        {
            octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
        }
        octree.updateInnerOccupancy();
        // while (true) {
        //     // 将八叉树转换为OctoMap消息
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
        // 平面拟合函数
        if (pointCloud_blue.size() > minNumBlueKeyPoint) {
            // std::cout << "fitPlaneBlue" << std::endl;
            fitPlaneBlue();
            
            // if (paramListBlue.size() == numSampleBlue) {
            //     // std::cout << "pixelToBluePlane" << std::endl;
            //     pixelToBluePlane();
            //     // std::cout << "bluePlaneToWorld" << std::endl;
            //     bluePlaneToWorld();                
            // }
        }
        // std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
        if(pointCloud_green.size() > minNumGreenKeyPoint){
            // std::cout << "fitPlaneGreen" << std::endl;
            fitPlaneGreen();
            // if(paramListGreen.size()==numSampleGreen) {
            //     // std::cout << "pixelToGreenPlane" << std::endl;
            //     pixelToGreenPlane();
            //     // std::cout << "greenPlaneToWorld" << std::endl;   
            //     greenPlaneToWorld();                   
            // }
        }
        // std::chrono::steady_clock::time_point t6 = std::chrono::steady_clock::now();
        if (pointCloud_red.size() > minNumRedKeyPoint){
            // std::cout << "fitPlaneRed" << std::endl;
            fitPlaneRed();
            // // std::chrono::steady_clock::time_point t7 = std::chrono::steady_clock::now();
            // if (paramListRed.size()==numSampleRed) {
            //     // std::cout << "pixelToRedPlane" << std::endl;
            //     pixelToRedPlane();
            //     // 点云拼接并转化为八叉树消息发布并在rviz中显示
            //     // std::cout << "redPlaneToWorld" << std::endl;
            //     redPlaneToWorld();                  
            // }
        }
        // std::chrono::steady_clock::time_point t8 = std::chrono::steady_clock::now();
        // std::cout << "pclConcatenate" << std::endl;
        pclConcatenate();
        // std::chrono::steady_clock::time_point t9 = std::chrono::steady_clock::now();
        // 测试代码
        // std::cout << "octoTreeCurPub" << std::endl;
        octoTreeCurPub();
        // std::cout << "octoTreePub" << std::endl;
        // octoTreePub();
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