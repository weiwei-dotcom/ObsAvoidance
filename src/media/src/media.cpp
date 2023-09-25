
// TODO: 修改配置文件让获取尺度因子的时间更长一点，精确度更高点;
#include "media.hpp"

mediaNode::mediaNode():Node("media")
{
  flag_getScaleFact = false;
  slam_sub.subscribe(this, "slam");
  image_sub.subscribe(this, "image_raw");
  sync1.reset(new message_filters::Synchronizer<message_filters::sync_policies::ExactTime<interface::msg::Slam, sensor_msgs::msg::Image>>(
      message_filters::sync_policies::ExactTime<interface::msg::Slam, sensor_msgs::msg::Image>(10), slam_sub, image_sub)
  );
  sync1->registerCallback(std::bind(&mediaNode::slam_callback, this, std::placeholders::_1, std::placeholders::_2));
  transformInit2Cur_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("transform_init2cur", 5);
  pointCloud_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud_initFrame", 5);
  cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance_2.0/src/config.yaml", cv::FileStorage::READ);
  minDist = fileRead["minDist"];
  dp = fileRead["dp"];
  cannyUpThresh = fileRead["cannyUpThresh"];
  circleThresh = fileRead["circleThresh"];
  minRadius = fileRead["minRadius"];
  maxRadius = fileRead["maxRadius"];
  difference_radius_thresh = fileRead["difference_radius_thresh"];
  distance_center_thresh = fileRead["distance_center_thresh"];
  circle_size_thresh = fileRead["circle_size_thresh"];
  double blueUpperThreshH = fileRead["blueUpper.1"];
  double blueUpperThreshS = fileRead["blueUpper.2"];
  double blueUpperThreshV = fileRead["blueUpper.3"];
  double blueLowerThreshH = fileRead["blueLower.1"];
  double blueLowerThreshS = fileRead["blueLower.2"];
  double blueLowerThreshV = fileRead["blueLower.3"];
  blueLowerThresh = cv::Scalar(blueLowerThreshH,blueLowerThreshS,blueLowerThreshV);
  blueUpperThresh = cv::Scalar(blueUpperThreshH,blueUpperThreshS,blueUpperThreshV);
  int erodeStructure_size = fileRead["erodeStructure_size"];
  int dilateStructure_size = fileRead["dilateStructure_size"];
  structure_erode=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeStructure_size, erodeStructure_size));
  structure_dilate=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilateStructure_size, dilateStructure_size));
  float fx = fileRead["Camera.fx"];
  float fy = fileRead["Camera.fy"];
  float cx = fileRead["Camera.cx"];
  float cy = fileRead["Camera.cy"];
  m_projectMatrix << fx,  0, cx,
                      0, fy, cy,
                      0,  0,  1; 
  scaleFactList_size_thresh = fileRead["scaleFactList_size_thresh"];
  inlier_thresh_scaleFact = fileRead["inlier_thresh_scaleFact"];
  sample_length = fileRead["sample_length"];
  fx_ = fileRead["Camera.fx"];
  inlier_probability_=fileRead["inlier_probability"];
  cosValue_thresh = fileRead["cosValue_thresh"];
  flag_motionControlProgramHaveDone = fileRead["flag_motionControlProgramHaveDone"];
  flag_slamInited = false;
  // 在运动控制程序编写完成之前不执行该判断内部的代码
  if (flag_motionControlProgramHaveDone == 1) {

  }
}

void mediaNode::slam_callback(const interface::msg::Slam::ConstSharedPtr &slam_msg, 
                              const sensor_msgs::msg::Image::ConstSharedPtr &image_msg)
{
  std::cout << "----------------------------------" << std::endl;
  // // 在运动控制程序编写完成之前不执行该判断的内部代码
  // if (flag_motionControlProgramHaveDone == 1)
  // {
  //   if (!flag_slamInited) 
  //   {
  //     slamInitFromMotion()
  //     return;
  //   }
  //   if (!flag_getScale)
  //   {
  //     getScaleFromMotion();
  //     return;
  //   }  
  //   if (!flag_getCurToBase)    
  // }
  // 初始化获得到真实世界的尺度因子
  if (!flag_getScaleFact) {
    getScaleSlamToWorld(image_msg, slam_msg);
    return;
  }
  // 
  // 如果初始化获得了到真实世界的尺度因子，将slam尺度下的点云坐标以及相机的位移量乘上尺度因子就获得了真实世界下的点云数据以及相机位移
  pcl::PointCloud<pcl::PointXYZ> temp_pointCloud;
  pcl::fromROSMsg(slam_msg->point_cloud, temp_pointCloud);
  // 调试代码3
  // std::cout << "temp_pointCloud.size(): " << temp_pointCloud.points.size() << std::endl;
  for (size_t i=0;i<temp_pointCloud.points.size();i++) {
    temp_pointCloud.points[i].x *= scaleFact_slamToWorld;
    temp_pointCloud.points[i].y *= scaleFact_slamToWorld;
    temp_pointCloud.points[i].z *= scaleFact_slamToWorld;
  }
  sensor_msgs::msg::PointCloud2 point_cloud_pub_msg;
  pcl::toROSMsg(temp_pointCloud, point_cloud_pub_msg);
  point_cloud_pub_msg.header = slam_msg->point_cloud.header;
  std::cout << "point_cloud_pub_msg.header.frame_id: " << point_cloud_pub_msg.header.frame_id << std::endl;
  pointCloud_pub->publish(point_cloud_pub_msg);
  geometry_msgs::msg::PoseStamped transform_init2cur_pub_msg;
  transform_init2cur_pub_msg = slam_msg->transform_init2cur;
  transform_init2cur_pub_msg.pose.position.x *= scaleFact_slamToWorld;
  transform_init2cur_pub_msg.pose.position.y *= scaleFact_slamToWorld;
  transform_init2cur_pub_msg.pose.position.z *= scaleFact_slamToWorld;
  // //调试代码3
  // std::cout << "transform_init2cur_pub_msg.pose.position.x: " << transform_init2cur_pub_msg.pose.position.x << std::endl;
  // std::cout << "transform_init2cur_pub_msg.pose.position.y: " << transform_init2cur_pub_msg.pose.position.y << std::endl;
  // std::cout << "transform_init2cur_pub_msg.pose.position.z: " << transform_init2cur_pub_msg.pose.position.z << std::endl;
  std::cout << "transform_init2cur_pub_msg.header.frame_id: " << transform_init2cur_pub_msg.header.frame_id << std::endl;
  transformInit2Cur_pub->publish(transform_init2cur_pub_msg);
  return;
}

void mediaNode::getScaleSlamToWorld(const sensor_msgs::msg::Image::ConstSharedPtr &imageMsg, const interface::msg::Slam::ConstSharedPtr &slamMsg) 
{
  cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(imageMsg, "bgr8");
  cv::Mat img = img_ptr->image;
  if (img.empty()){
    RCLCPP_INFO(this->get_logger(), "图像数据接收失败");
    return;
  }
  cv::Mat img_gray;
  cv::cvtColor(img, img_gray, CV_BGR2GRAY);
  cv::GaussianBlur(img_gray, img_gray, cv::Size(7, 7),2,2);
  std::vector<cv::Vec3f> temp_circles;
  cv::HoughCircles(img_gray, temp_circles, cv::HOUGH_GRADIENT, dp, minDist, cannyUpThresh, circleThresh, minRadius, maxRadius);
  if (temp_circles.size()>1) {
    RCLCPP_INFO(this->get_logger(), "检测圆形数量过多");
    return;    
  }
  if (temp_circles.size()==0) {
    RCLCPP_INFO(this->get_logger(), "未检测到圆孔");
    return;    
  }
  float distance_center=0.0, difference_radius=0.0;
  if (circles_.size() != 0) 
  {
    distance_center = std::sqrt(std::pow(temp_circles[0][0]-circles_.back()[0], 2) + std::pow(temp_circles[0][1]-circles_.back()[1], 2));
    difference_radius=std::abs(temp_circles[0][2] - circles_.back()[2]);    
  }
  if (distance_center > distance_center_thresh || difference_radius > difference_radius_thresh) {
    RCLCPP_INFO(this->get_logger(), "前后圆形差异过大，圆形检测不稳定");
    distance_.clear();
    circles_.clear();
    return;
  }
  // 利用稀疏点云拟合平面获得slam尺度的平面距离，并判断当前平面是否足够正对相机
  double distance;
  bool flag_poseCorrect = detectPoseCorrect(img,slamMsg,distance,temp_circles[0]);
  if(!flag_poseCorrect) 
    return;
  circles_.push_back(temp_circles[0]);
  distance_.push_back(distance);
  // 判断检测圆形的数量是否达到每次输出的上限
  if (circles_.size() < circle_size_thresh) 
    return;
  for (int i=0;i<circles_.size();i++) {
    double scaleFact=(double)100.0 *  fx_ / ((double)circles_[i][2] * distance_[i]);
    scaleFactList_.push_back(scaleFact);
  }
  circles_.clear();
  distance_.clear();
  if (scaleFactList_.size()<scaleFactList_size_thresh)
    return;
  ransacScaleFact();
  flag_getScaleFact = true;
}

bool mediaNode::detectPoseCorrect(const cv::Mat img, const interface::msg::Slam::ConstSharedPtr slamMsg, double &distance_plane, const cv::Vec3f tempCircle)
{
  // 阈值分割
  cv::Mat imgHsv, imgBin, img_erode, img_dilate;
  cv::cvtColor(img, imgHsv, CV_BGR2HSV);
  cv::inRange(imgHsv, blueLowerThresh, blueUpperThresh, imgBin);
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
    std::cout<<"没有找到轮廓" << std::endl;
    return false;
  }
  // 较大核膨胀
  cv::dilate(img_erode, img_dilate, structure_dilate);
  // 点云变换坐标系至当前帧
  pcl::PointCloud<pcl::PointXYZ> PointCloud_InitFrame, PointCloud_curFrame;
  pcl::fromROSMsg(slamMsg->point_cloud, PointCloud_InitFrame);
  Eigen::Quaterniond q(slamMsg->transform_init2cur.pose.orientation.w,
                       slamMsg->transform_init2cur.pose.orientation.x,
                       slamMsg->transform_init2cur.pose.orientation.y,
                       slamMsg->transform_init2cur.pose.orientation.z);
  Sophus::Vector3d pos(slamMsg->transform_init2cur.pose.position.x,
                       slamMsg->transform_init2cur.pose.position.y,
                       slamMsg->transform_init2cur.pose.position.z);
  Sophus::SE3d transform_init2cur(q, pos);
  pcl::transformPointCloud(PointCloud_InitFrame, PointCloud_curFrame, transform_init2cur.matrix());
  // 逐个点云逆投影至像素平面进行筛选
  pcl::PointCloud<pcl::PointXYZ> finalPointCloud;
  for (auto point:PointCloud_curFrame.points) {
    Eigen::Vector3f tempPixelPoint=m_projectMatrix*point.getVector3fMap();
    cv::Point2f pixelPoint(tempPixelPoint[0]/tempPixelPoint[2], tempPixelPoint[1]/tempPixelPoint[2]);
    double polyTestValue=cv::pointPolygonTest(contour, pixelPoint, false);
    if (polyTestValue<0) 
      continue;
    // 特征点掩码判断    
    if (img_dilate.at<bool>((int)pixelPoint.x, (int)pixelPoint.y) != 255) 
      continue; 
    // 半径距离判断，是否在圆孔附近 
    float tempPixelDistance = std::sqrt(std::pow(pixelPoint.x-tempCircle[0], 2)+std::pow(pixelPoint.y-tempCircle[1],2));
    if (tempPixelDistance < tempCircle[2]+(double)2)
      continue;
    
    // // 调试代码1
    // cv::drawMarker(img, cv::Point((int)pixelPoint.x, (int)pixelPoint.y), cv::Scalar(0,255,0), 2, 20, 3);

    // 特征点轮廓判断
    finalPointCloud.points.push_back(point);
  }

  // // 调试代码1
  // cv::imshow("img", img);
  // cv::waitKey(10);

  // 用筛选后的点云拟合平面
  if (finalPointCloud.points.size()<80) {
    std::cout<<"finalPointCloud.points.size() : " << finalPointCloud.points.size()<<std::endl;
    RCLCPP_INFO(this->get_logger(), "拟合点云数量过少, 无法进行正视判断");
    return false;
  }
  Eigen::Vector4d param = calParam(finalPointCloud);
  distance_plane=-param[3];
  // 将拟合平面后的法向量点乘当前帧坐标系z轴向量
  Eigen::Vector3d zAxis(0,0,1);
  double cosValue = param.block(0,0,3,1).dot(zAxis);
  // 两向量余弦值判断
  if (cosValue < cosValue_thresh) {
    std::cout << std::acos(cosValue)/M_PI * (double)180 << "°" << std::endl;
    RCLCPP_INFO(this->get_logger(), "检测到相机偏离检测平面角度过大，请将相机正视于检测平面");
    return false;    
  }
  // // 调试代码2
  // std::cout << "distance_plane: " << distance_plane << std::endl;
  return true;
}

bool mediaNode::getMaxAreaContour(cv::Mat img_bin, std::vector<cv::Point> &contour) {
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
        cv::findContours((labels == max_area_label), temp_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);            
        contour = temp_contours[0];
        return true;
    }
    return false;
}

Eigen::Vector4d mediaNode::calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud) {
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

void mediaNode::ransacScaleFact()
{
  int iterNum = calRansacIterNum();
  int maxInlierNum = 0;
  for (int i=0;i<iterNum;i++) 
  {
    int inlierNum = 0;
    std::random_shuffle(scaleFactList_.begin(), scaleFactList_.end());
    std::vector<double>::const_iterator front = scaleFactList_.begin();
    std::vector<double>::const_iterator back = scaleFactList_.begin()+sample_length;
    std::vector<double> tempScaleFactList(front, back);
    double tempSum = std::accumulate(tempScaleFactList.begin(), tempScaleFactList.end(), 0);
    double result = tempSum/(double)sample_length;
    for (int j = 0;j<scaleFactList_.size();j++)
    {
      double value = std::abs(scaleFactList_[j]-result);
      if (value <= inlier_thresh_scaleFact) {
        inlierNum++;
      }
    }
    if (inlierNum > maxInlierNum) 
    {
      maxInlierNum = inlierNum;
      scaleFact_slamToWorld = (float)result;
    }
  }
  std::cout << "the final result of scale fact: " << scaleFact_slamToWorld <<std::endl;
}

int mediaNode::calRansacIterNum()
{
  float tempFloat = 1.0;
  for (int i=0;i<sample_length;i++) 
    tempFloat*=(inlier_probability_ * (float)scaleFactList_size_thresh-(float)i)/(float)(scaleFactList_size_thresh-i);
  int iterNum = std::ceil((float)1.0/tempFloat);
  std::cout<<"ransac 迭代次数为： " << iterNum <<std::endl;
  return iterNum;
}

// 返回给消息点云和关键点图像坐标信息的消息
mediaNode::~mediaNode() {}