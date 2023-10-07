#include "model.hpp"

void ModelNode::mapPoint_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
{
    // If have complete publish the pointcloud of model, don't need model again;
    if (flag_pubModel)
        return;
    // Recieve the message data;
    if (!recieveMsg(pcl_msg, img_msg, pose_msg))
        return;   
    // Start modeling;
    if (!Model())
        return;
    // Publish pointcloud of obstacle model;
    PubModel();        
    return;
}

bool ModelNode::recieveMsg(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
{
    // 图像赋值给成员变量；
    cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(img_msg, "bgr8");
    if (imgPtr->image.empty())
    {
        RCLCPP_INFO(this->get_logger(), "Recieve img failed !");
        return false;
    }
    imgPtr->image.copyTo(m_img);
    // 稀疏点云赋值给成员变量；
    if (pcl_msg->width==0)
    {
        RCLCPP_INFO(this->get_logger(), "Recieve pcl_msg failed !");
        return false;
    }
    pcl::fromROSMsg(*pcl_msg, m_pointCloud);
    // 第一帧位姿在当前帧位姿下的变换赋值给成员变量
    Eigen::Quaterniond tempQua(pose_msg->pose.orientation.w,
                        pose_msg->pose.orientation.x,
                        pose_msg->pose.orientation.y,
                        pose_msg->pose.orientation.z);
    Sophus::Vector3d tempTranslattion(pose_msg->pose.position.x,
                                    pose_msg->pose.position.y,
                                    pose_msg->pose.position.z);
    Sophus::SE3d temp_transform(tempQua, tempTranslattion);
    m_transform_initToCur = temp_transform.matrix();
    m_transform_curToInit = m_transform_initToCur.inverse();
    m_header_initFrame = pcl_msg->header;
    return true;
}
// 构造函数 init function()
ModelNode::ModelNode():Node("model")
{
    error_type = OK;
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_obstacle", 10);
    flag_pubModel = false;
    octoMapPub = this->create_publisher<octomap_msgs::msg::Octomap>("obstacle", 10);
    // 定义消息订阅者
    pcl_sub.subscribe(this, "pointCloud_initFrame");
    img_sub.subscribe(this, "image_raw");
    pose_sub.subscribe(this, "transform_initToCur");
    // 定义时间同步器
    sync1.reset(new synType1(synMsgType1(10), pcl_sub, img_sub, pose_sub));
    // 时间同步器注册回调函数
    sync1->registerCallback(std::bind(&ModelNode::mapPoint_callback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3));

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
    int erodeStructure_size = fileRead["model_erodeStructure_size"];
    int dilateStructure_size = fileRead["model_dilateStructure_size"];
    structure_erode=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeStructure_size, erodeStructure_size));
    structure_dilate=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilateStructure_size, dilateStructure_size));
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
    minDist = fileRead["model_minDist"];
    dp = fileRead["model_dp"];
    cannyUpThresh = fileRead["model_cannyUpThresh"];
    circleThresh = fileRead["model_circleThresh"];
    minRadius = fileRead["model_minRadius"];
    maxRadius = fileRead["model_maxRadius"];
    difference_radius_thresh = fileRead["model_difference_radius_thresh"];
    distance_center_thresh = fileRead["model_distance_center_thresh"];
    cosValue_thresh = fileRead["cosValue_thresh"];
    distance_thresh = fileRead["distance_thresh"]; 
}

void ModelNode::changeErrorType(ERROR_TYPE newError)
{
  if (newError != error_type)
  {
    static std::string errorTypeString[8] = {
      "Circle detection is not stable",
      "Distance of circle is too much",
      "Not found the countour",
      "Point cloud is little",
      "OK",
      "Image recieve error",
      "The camera is not straight on the plane",
      "the distance between camera and detect plane is too close"
    };
    error_type = newError;
    RCLCPP_INFO(this->get_logger(), "Error Type: %s", errorTypeString[(int)error_type].c_str());
  }
}
// 障碍物建模主函数
bool ModelNode::Model()
{
    // 入口圓形檢測
    cv::Mat img_gray;
    cv::cvtColor(m_img, img_gray, CV_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_gray, cv::Size(7, 7),2,2);
    std::vector<cv::Vec3f> temp_circles;
    cv::HoughCircles(img_gray, temp_circles, cv::HOUGH_GRADIENT, dp, minDist, cannyUpThresh, circleThresh, minRadius, maxRadius);
    if (temp_circles.size()>1) {
        changeErrorType(Circle_detection_is_not_stable);
        return false;    
    }
    if (temp_circles.size()==0) {
        changeErrorType(Circle_detection_is_not_stable);
        return false;    
    }
    if (circles_.size() != 0) 
    {
        float distance_center, difference_radius;
        distance_center = std::sqrt(std::pow(temp_circles[0][0]-circles_.back()[0], 2) + std::pow(temp_circles[0][1]-circles_.back()[1], 2));
        difference_radius=std::abs(temp_circles[0][2] - circles_.back()[2]);    
        if (distance_center > distance_center_thresh || difference_radius > difference_radius_thresh) 
        {
            changeErrorType(Distance_of_circle_is_too_much);
            circles_.clear();
            return false;
        }
    }
    // 利用稀疏点云拟合平面获得平面距离，并判断当前平面是否足够正对相机
    double distance;
    cv::Mat mask; // 掩码图像，用以保存最大连通域区域
    bool flag_poseCorrect = detectPoseCorrect(distance,temp_circles[0],mask);
    if(!flag_poseCorrect) 
        return false;
    // 如果相機距離檢測平面太近，則很可能檢測不到實驗平臺邊沿輪廓直線
    if (distance < distance_thresh) {
        changeErrorType(TooClose);
        return false;
    }

    // TODO: 实验平台边沿直线检测...
    // 大阈值检测直线
    // 对直线列表按照长度从大到小排列
    // 膨胀mask图像得到mask_dilate;
    // 定义临时直线参数变量 tempLine
    // for 循环从列表中筛选直线
        // 如果当前全局直线参数列表不为空
            // 利用两个坐标点计算二维法向量
            // 与全局列表中最后一个不全为零的直线参数进行点乘输出绝对值
            // 如果绝对值小于共线阈值（0.98）并且大于(0.25)
                // continue
        // 利用mask_dilate掩码判断直线两端点是否都在最大连通域上
        // 不在
            // continue
        // 计算直线距离圆心距离
        // 圆心距离太近
            // continue
        // 利用直线两端点坐标计算中点坐标
        // 利用中点坐标计算圆心指向中点的单位向量
        // 利用中点坐标加上20倍的单位向量计算 flag_point(cv::Point)
        // 如果 mask.at<bool>(flag_point) == 255
            // continue;
        // 返回当前直线参数到 tempLine
    // 利用
    // 直线与圆心距离判断


    circles_.push_back(temp_circles[0]);
    // // 判断检测圆形的数量是否达到每次输出的上限
    // if (circles_.size() < circle_size_thresh) 
    //     return false;
    // circles_.clear();
    

    // Completing model;
    return true;
}

//TODO: 直接從media.cpp 裏面複製過來的，需要修改
bool ModelNode::detectPoseCorrect(double &distanceWithPlane, const cv::Vec3f tempCircle, cv::Mat &mask)
{
  // 阈值分割
  cv::Mat imgHsv, imgBin, img_erode, img_dilate;
  cv::cvtColor(m_img, imgHsv, CV_BGR2HSV);
  cv::inRange(imgHsv, blueLower, blueUpper, imgBin);
  // 较小核腐蚀
  cv::erode(imgBin, img_erode, structure_erode);
  
  // // 调试代码
  // cv::imshow("img_erode", img_erode);
  // cv::waitKey(20);

  // 最大连通域提取轮廓
  std::vector<cv::Point> contour;
  bool flag_getContour = getMaxAreaContour(img_erode, contour);
  if (!flag_getContour) 
  {
    changeErrorType(Not_found_the_countour);
    return false;
  }
  // 较大核膨胀
  cv::dilate(img_erode, img_dilate, structure_dilate);
  // 点云变换坐标系至当前帧
  pcl::PointCloud<pcl::PointXYZ> PointCloud_curFrame;
  pcl::transformPointCloud(m_pointCloud, PointCloud_curFrame, m_transform_initToCur);
  // 逐个点云逆投影至像素平面进行筛选
  pcl::PointCloud<pcl::PointXYZ> finalPointCloud;
  for (auto point:PointCloud_curFrame.points) {
    Eigen::Vector3f tempPixelPoint = m_projectMatrix * point.getVector3fMap();
    cv::Point2f pixelPoint(tempPixelPoint[0]/tempPixelPoint[2], tempPixelPoint[1]/tempPixelPoint[2]);
    double polyTestValue=cv::pointPolygonTest(contour, pixelPoint, false);
    // 特征点轮廓判断    
    if (polyTestValue<0) 
      continue;
    // 特征点掩码判断    
    if (img_dilate.at<bool>((int)pixelPoint.x, (int)pixelPoint.y) != 255) 
      continue; 
    // 半径距离判断，是否在圆孔附近 
    float tempPixelDistance = std::sqrt(std::pow(pixelPoint.x-tempCircle[0], 2)+std::pow(pixelPoint.y-tempCircle[1],2));
    if (tempPixelDistance < tempCircle[2]+(double)3)
      continue;
    
    // // 调试代码1
    // cv::drawMarker(img, cv::Point((int)pixelPoint.x, (int)pixelPoint.y), cv::Scalar(0,255,0), 2, 20, 3);

    finalPointCloud.points.push_back(point);
  }

  // // 调试代码1
  // cv::imshow("img", img);
  // cv::waitKey(10);

  // 用筛选后的点云拟合平面
  if (finalPointCloud.points.size()<90) {
    changeErrorType(Point_cloud_is_little);
    return false;
  }
  Eigen::Vector4d param = calParam(finalPointCloud);
  distanceWithPlane=-param[3];
  // 将拟合平面后的法向量点乘当前帧坐标系z轴向量
  Eigen::Vector3d zAxis(0,0,1);
  double cosValue = param.block(0,0,3,1).dot(zAxis);
  // 两向量余弦值判断
  if (cosValue < cosValue_thresh) {
    changeErrorType(The_camera_is_not_straight_on_the_plane);
    return false;    
  }
  // // 调试代码2
  // std::cout << "distance_plane: " << distance_plane << std::endl;
  mask=img_dilate;
  return true;
}

bool ModelNode::getMaxAreaContour(cv::Mat& img_bin, std::vector<cv::Point> &contour) {
    cv::Mat labels, stats, centroids;
    int num_labels = connectedComponentsWithStats(img_bin, labels, stats, centroids, 8, CV_16U);
    std::vector<std::vector<cv::Point>> temp_contours;
    std::vector<int> areas;
    if (num_labels>1)
    {
        for (int i = 1; i < num_labels; i++) // 忽略背景标签0
        {
            areas.push_back(stats.at<int>(i, cv::CC_STAT_AREA));
        }
        int max_area_label = max_element(areas.begin(), areas.end()) - areas.begin() + 1;
        img_bin = (labels == max_area_label);
        cv::findContours((labels == max_area_label), temp_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);            
        contour = temp_contours[0];
        return true;
    }
    return false;
}

void ModelNode::PubModel()
{
    sensor_msgs::msg::PointCloud2 pclMsg_obstacle;
    pcl::toROSMsg(pcl_obstacle, pclMsg_obstacle);
    pclMsg_obstacle.header = m_header_initFrame;
    pcl_pub->publish(pclMsg_obstacle);
    flag_pubModel = true;
}

// 该函数用来图像预处理，同时对障碍物进行二值化分割
void ModelNode::preProcAndThresh()
{
    threshBlue();
    threshGreen();
    threshRed();
}

// 分别对红绿蓝色障碍物区域进行分割
void ModelNode::threshBlue() {
    cv::Mat tempImgBin_white;
    cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
    // cv::Mat tempImgBin_white, tempImgBin_blue;
    cv::inRange(imageHSV, blueLower, blueUpper, imageBin_blue);
    // cv::inRange(imageHSV, whiteLower, whiteUpper, tempImgBin_white);
    // cv::inRange(imageHSV, blueLower, blueUpper, tempImgBin_blue);
    // imageBin_blue = tempImgBin_blue+tempImgBin_white;
    cv::morphologyEx(imageBin_blue, imgBlueOpen, cv::MORPH_OPEN, kernelOpen_blue);
    cv::morphologyEx(imgBlueOpen, imgBlueClose, cv::MORPH_CLOSE, kernelClose_blue);  
}
void ModelNode::threshGreen() {
    cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
    // cv::Mat tempImgBin_white, tempImgBin_green;
    cv::inRange(imageHSV, greenLower, greenUpper, imageBin_green);
    // cv::inRange(imageHSV, whiteLower, whiteUpper, tempImgBin_white);
    // cv::inRange(imageHSV, greenLower, greenUpper, tempImgBin_green);
    // imageBin_green = tempImgBin_green+tempImgBin_white;
    cv::morphologyEx(imageBin_green, imgGreenOpen, cv::MORPH_OPEN, kernelOpen_green);
    cv::morphologyEx(imgGreenOpen, imgGreenClose, cv::MORPH_CLOSE, kernelClose_green);   
}
void ModelNode::threshRed() {
    cv::cvtColor(m_img, imageHSV, cv::COLOR_BGR2HSV);
    cv::Mat tempImgBin_red1, tempImgBin_red2, tempImgBin_white;
    cv::inRange(imageHSV, redLower, redUpper, tempImgBin_red1);
    cv::inRange(imageHSV, redPlusLower, redPlusUpper, tempImgBin_red2);
    // cv::inRange(imageHSV, whiteLower, whiteUpper, tempImgBin_white);
    imageBin_red = tempImgBin_red1 + tempImgBin_red2;
    cv::morphologyEx(imageBin_red, imgRedOpen, cv::MORPH_OPEN, kernelOpen_red);
    cv::morphologyEx(imgRedOpen, imgRedClose, cv::MORPH_CLOSE, kernelClose_red);  
}

void ModelNode::getBlueContour() {
    cv::Mat labels_blue, stats_blue, centroids_blue;
    int num_labels_blue = connectedComponentsWithStats(imgBlueClose, labels_blue, stats_blue, centroids_blue, 8, CV_16U);
    // 获取连通域的面积
    contours_blue_ex.clear(), contours_blue_in.clear();
    std::vector<std::vector<cv::Point>> temp_contours_blue;
    std::vector<int> areas_blue;
    // 如果大于两个轮廓，也就是包括背景在内的至少有三个轮廓，那么可能存在圆孔的轮廓输出障碍物以及圆孔轮廓
    if (num_labels_blue>2)
    {
        for (int i = 1; i < num_labels_blue; i++) // 忽略背景标签0
        {
            areas_blue.push_back(stats_blue.at<int>(i, cv::CC_STAT_AREA));
        }
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
        cv::findContours((labels_blue == 1), temp_contours_blue, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  
        contours_blue_ex = temp_contours_blue[0];
    }
}
void ModelNode::getGreenContour() {
    cv::Mat labels_green, stats_green, centroids_green;
    int num_labels_green = connectedComponentsWithStats(imgGreenClose, labels_green, stats_green, centroids_green, 8, CV_16U);
    contours_green.clear();
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

void ModelNode::getRedContour() {
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

void ModelNode::selMaxArea()
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
void ModelNode::invProjAndSel()
{
    // 将世界坐标系下的点云投影到相机坐标系下
    pcl::PointCloud<pcl::PointXYZ> pointCloudInCamFrame;
    pcl::transformPointCloud(m_pointCloud, pointCloudInCamFrame, m_transform_initToCur);

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
        cv::Point2f imagePoint(tempProjectPoint[0]/tempProjectPoint[2], tempProjectPoint[1]/tempProjectPoint[2]);
        projectPointsInImg.push_back(imagePoint);
        if(flag_find_contour_green){
            double distance_green = cv::pointPolygonTest(contours_green, imagePoint, false);  
            if (distance_green > 0) {
                pointCloudInCamFrame_green.points.push_back(point);
                indexPointCloudGreen.push_back(index);
                projectPointsInImg_green.push_back(imagePoint); // ...........................
                index++;
                continue;
            }              
        }
        if (flag_find_contour_red) {
            double distance_red = cv::pointPolygonTest(contours_red, imagePoint, false); 
            if (distance_red > 0) {
                pointCloudInCamFrame_red.points.push_back(point);
                indexPointCloudRed.push_back(index);
                projectPointsInImg_red.push_back(imagePoint); // .................
                index++;
                continue;
            }    
        }
        if (flag_find_contour_blue && contour_blue_in_size > 0)
        {
            double distance_blue_ex = cv::pointPolygonTest(contours_blue_ex, imagePoint, false);
            double distance_blue_in = cv::pointPolygonTest(contours_blue_in, imagePoint, false);
            if (distance_blue_ex > 0 && distance_blue_in < 0 )
            {
                pointCloudInCamFrame_blue.points.push_back(point);
                indexPointCloudBlue.push_back(index);
                projectPointsInImg_blue.push_back(imagePoint); // ...........................
            }
        }
        else if(flag_find_contour_blue)
        {
            double distance_blue_ex = cv::pointPolygonTest(contours_blue_ex, imagePoint, false);
            if (distance_blue_ex > 0)
            {
                pointCloudInCamFrame_blue.points.push_back(point);
                indexPointCloudBlue.push_back(index);
                projectPointsInImg_blue.push_back(imagePoint); // ...........................
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

Eigen::Vector4d ModelNode::calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud) {
    Eigen::Vector4d param;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_Ptr=pointCloud.makeShared();
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 设置分割系数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.006);
    // 从点云中分割最有可能的平面
    seg.setInputCloud(pointCloud_Ptr);
    pcl::ModelCoefficients coefficient;
    seg.segment(*inliers, coefficient);        
    param(0) = coefficient.values[0];
    param(1) = coefficient.values[1];
    param(2) = coefficient.values[2];
    param(3) = coefficient.values[3];
    return param;
}

// 获取当前用来投影的平面参数值
void ModelNode::fitPlaneBlue() {
    std::cout<<"    fitPlaneBlue: "<<std::endl;
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
void ModelNode::fitPlaneGreen() {
    std::cout<<"    fitPlaneGreen: "<<std::endl;
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
void ModelNode::fitPlaneRed() {
    std::cout<<"    fitPlaneRed: "<<std::endl;
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

void ModelNode::transformParamToCamFrame(Eigen::Vector4d &paramInWorldFrame, Eigen::Vector4d &paramInCamFrame) {
    std::cout<<"        transformParamToCamFrame"<<std::endl;
    paramInCamFrame.block(0,0,3,1) = m_transform_initToCur.block(0,0,3,3)*paramInWorldFrame.block(0,0,3,1);
    Eigen::Vector3d tempCamPosition = m_transform_curToInit.block(0,3,3,1);
    paramInCamFrame(3) = paramInWorldFrame(3) + tempCamPosition.dot(paramInWorldFrame.block(0,0,3,1));
}
void ModelNode::pixelToPlane(cv::Mat &imgClose, Eigen::Vector4d &paramInCamFrame, pcl::PointCloud<pcl::PointXYZ> &obstacle) {
    std::cout<<"        pixelToPlane"<<std::endl;
    double a = paramInCamFrame(0);
    double b = paramInCamFrame(1);
    double c = paramInCamFrame(2);
    double d = paramInCamFrame(3);
    pcl::PointCloud<pcl::PointXYZ> obstacleInCamFrame;
    cv::Mat temp_imgClose;
    imgClose.convertTo(temp_imgClose, CV_8UC1);
    int rows = temp_imgClose.rows;
    int cols = temp_imgClose.cols;
    std::cout<<"rows: "<<rows<<" cols: "<<cols<<std::endl;
    for (int u=0;u<rows;u+=3) {
        for (int v=0;v<cols;v+=3) {
            if (temp_imgClose.at<uchar>(u,v) == 255) {
                pcl::PointXYZ tempPoint;
                tempPoint.z = -d/(a*(v-cx)/fx+b*(u-cy)/fy+c);
                tempPoint.x = tempPoint.z*(v-cx)/fx;
                tempPoint.y = tempPoint.z*(u-cy)/fy;
                obstacleInCamFrame.points.push_back(tempPoint);
            }
        }
    };
    pcl::transformPointCloud(obstacleInCamFrame, obstacle, m_transform_curToInit);
};
void ModelNode::updateObstacle(std::vector<pcl::PointCloud<pcl::PointXYZ>> &obstacleList, pcl::PointCloud<pcl::PointXYZ> &obstacle) {
    std::cout<<"        updateObstacle"<<std::endl;
    obstacleList.push_back(obstacle);
    if (obstacleList.size() > maxLengthPointCloudList) {
        std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator start=obstacleList.begin()+1;
        std::vector<pcl::PointCloud<pcl::PointXYZ>>::const_iterator end=obstacleList.end();
        std::vector<pcl::PointCloud<pcl::PointXYZ>> tempPointCloudList(start,end);
        obstacleList=tempPointCloudList;
    }
}
void ModelNode::pixelToBluePlane(){
    std::cout<<"    pixelToBluePlane: "<<std::endl;
    Eigen::Vector4d paramInCamFrame;
    transformParamToCamFrame(paramBlue, paramInCamFrame);
    pixelToPlane(imgBlueClose, paramInCamFrame, obstacleBlue);
    updateObstacle(obstacleBlueList, obstacleBlue);
}
void ModelNode::pixelToGreenPlane() {
    std::cout<<"    pixelToGreenPlane: "<<std::endl;
    Eigen::Vector4d paramInCamFrame;
    transformParamToCamFrame(paramGreen, paramInCamFrame);
    pixelToPlane(imgGreenClose, paramInCamFrame, obstacleGreen);
    updateObstacle(obstacleGreenList, obstacleGreen);
}
void ModelNode::pixelToRedPlane() {
    std::cout<<"    pixelToRedPlane: "<<std::endl;
    Eigen::Vector4d paramInCamFrame;
    transformParamToCamFrame(paramRed, paramInCamFrame);
    pixelToPlane(imgRedClose, paramInCamFrame, obstacleRed);
    updateObstacle(obstacleRedList, obstacleRed);
}
void ModelNode::optimizeParam(int timesRansacIter, std::vector<Eigen::Vector4d> &paramList, Eigen::Vector3d &standardParam, bool &flag_optimized,int threshToOpt) {
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
        std::cout<< "A:" << standardParam(0) << " B:" << standardParam(1) << " C:"<<standardParam(2)<<std::endl;
        paramList.clear();
    }
}
// 发布八叉树模型
void ModelNode::octoTreeCurPub()
{
    RCLCPP_INFO(this->get_logger(), "    octoTreeCurPub\n");
    pcl::PointCloud<pcl::PointXYZ> tempObstacle;
    std::cout<<"    obstacleBlueList.size: "<<obstacleBlueList.size()<<std::endl;
    for (int i=0;i<obstacleBlueList.size();i++) {
        tempObstacle+=obstacleBlueList[i];
    }
    std::cout<<"    obstacleGreenList.size: "<<obstacleGreenList.size()<<std::endl;
    for (int i=0;i<obstacleGreenList.size();i++) {
        tempObstacle+=obstacleGreenList[i];
    }
    std::cout<<"    obstacleRedList.size: "<<obstacleRedList.size()<<std::endl;
    for (int i=0;i<obstacleRedList.size();i++) {
        tempObstacle+=obstacleRedList[i];
    }
    if (tempObstacle.points.size() == 0)
    {
        RCLCPP_INFO(this->get_logger(), "当前八叉树模型为空,请对准障碍物平台");
        return ;
    }
    // std::cout<<"    tempObstacle.points.size()"<<std::endl;
    // std::cout<<"    "<<tempObstacle.points.size()<<std::endl;
    octomap::OcTree octree(octreeResolution);
    for (auto point:tempObstacle)
    {
        octree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    octree.updateInnerOccupancy();
    octomap_msgs::msg::Octomap octoMap;
    // std::cout<<"    octree.size() " <<std::endl;
    // std::cout<<"    "<<     octree.size()<<std::endl;
    octoMap.header = m_header_initFrame;
    octomap_msgs::fullMapToMsg(octree, octoMap);
    octoMapPub->publish(octoMap);
    std::cout << "octoMap published success" << std::endl;
    return;
}

// 析构函数 delete function()
ModelNode::~ModelNode() {
    ofs.close();
}