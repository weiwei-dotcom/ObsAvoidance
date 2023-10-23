
// TODO: 修改配置文件让获取尺度因子的时间更长一点，精确度更高点;
#include "media.hpp"

mediaNode::mediaNode():Node("media")
{
  error_type = OK;
  flag_trouble=false;
  // the flag of finish the process of initialization
  flag_haveInitialized=false;
  flag_getScaleFact = false;
  flag_getTransformToBase = false;
  flag_slamInitialized=false;
  flag_timeout=false;

  slam_sub = this->create_subscription<interface::msg::Slam>("slam", 10, std::bind(&mediaNode::slam_callback, this, std::placeholders::_1));
  transformInit2Cur_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("transform_initToCur", 5);
  transformCurToInit_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("transform_curToInit", 5);
  pointCloud_pub=this->create_publisher<sensor_msgs::msg::PointCloud2>("pointCloud_initFrame", 5);
  cv::FileStorage fileRead("/home/weiwei/Desktop/project/ObsAvoidance/src/config.yaml", cv::FileStorage::READ);
  
  cv::Mat tempExtrinsicMatrix(4,4,CV_64F);
  fileRead["extrinsicMatrix"] >> tempExtrinsicMatrix;
  cv::cv2eigen(tempExtrinsicMatrix, extrinsicMatrix);
  //TODO: The measure unit of extrinsicMatrix's translation is unknown, it should be transformed to the target measure unit
  TimeOut = fileRead["TimeOut"];
  double scaleFact_mmToTarget = fileRead["scaleFact_mmToTarget"];
  scaleFact_slamToWorld = 1;
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
  inlier_thresh_scaleFact*=scaleFact_mmToTarget;
  sample_length = fileRead["sample_length"];
  fx_ = fileRead["Camera.fx"];
  inlier_probability_=fileRead["inlier_probability"];
  cosValue_thresh = fileRead["cosValue_thresh"];
  circleRadius=fileRead["circleRadius"];
  circleRadius*=scaleFact_mmToTarget;
  m_transformToBase = Eigen::Matrix4d::Identity();
}

void mediaNode::changeErrorType(ERROR_TYPE newError)
{
  if (newError != error_type)
  {
    static std::string errorTypeString[7] = {
      "Circle detection is not stable",
      "Distance of circle is too much",
      "Not found the countour",
      "Point cloud is little",
      "OK",
      "Image recieve error",
      "The camera is not straight on the plane"
    };
    error_type = newError;
    RCLCPP_INFO(this->get_logger(), "Error Type: %s", errorTypeString[error_type].c_str());
  }
}

void mediaNode::initialization()
{
  //TODO:
  rclcpp::Rate timer(1);
  initializeSlam();
  while(!flag_slamInitialized)
  {
    if(flag_trouble||flag_timeout)
    {
      rclcpp::shutdown();
      return; 
    }    
    timer.sleep();
  }
  getSlamToWorldScaleFact();
  while(!flag_getScaleFact);
  {
    if(flag_trouble||flag_timeout)
    {
      rclcpp::shutdown();
      return; 
    }      
    timer.sleep();
  }

  getTransformToBase();
  while(!flag_getTransformToBase);
  {
    if(flag_trouble||flag_timeout)
    {
      rclcpp::shutdown();
      return; 
    }      
    timer.sleep();
  }
  flag_haveInitialized=true;
  return;
}
void mediaNode::slamInitialzed_callback(rclcpp::Client<interface::srv::SlamInitialized>::SharedFuture response)
{
  auto result = response.get();
  if (!result->flag_slam_initialized)
  {
    this->flag_trouble = true;
    RCLCPP_INFO(this->get_logger(), "Cdcr controller can't finish slam initialization.");
    return;
  }
  this->flag_slamInitialized = true;
  RCLCPP_INFO(this->get_logger(), "Cdcr controller finish slam initialization.");
  return;
}
void mediaNode::initializeSlam()
{
  std::chrono::steady_clock::time_point t_start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point t_now;
  double time_spend;
  while (!slamInitializedFlag_cli->wait_for_service(std::chrono::seconds(1)))
  {
    t_now = std::chrono::steady_clock::now();
    time_spend = std::chrono::duration_cast<std::chrono::seconds>(t_now-t_start).count();
    if (time_spend > TimeOut)
    {
      RCLCPP_ERROR(this->get_logger(), "Wait Timeout");
      flag_timeout=true;
      return;      
    }
    RCLCPP_INFO(this->get_logger(), "Waiting for the slamInitialzation server online");
  }
  auto request = std::make_shared<interface::srv::SlamInitialized_Request>();
  this->slamInitializedFlag_cli->async_send_request(request, 
      std::bind(&mediaNode::slamInitialzed_callback,this,std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "slamInitialization service is online, wait the process of slamInitialization finish!");
  return;
}
void mediaNode::getSlamToWorldScaleFact()
{
  //TODO: These two function (with getTransformToBase()) may use a action mechanism to realization
  // 

  return ;
}

void mediaNode::getTransformToBase()
{
  //TODO:
  // 
  return;
}
// Calculate the Frame from three flag points
void calFrameFromPoints(Eigen::Matrix3d points, Eigen::Matrix3d &R, Eigen::Matrix<double, 3, 1> &t) 
{
    // 由三个坐标点取平均获得坐标原点
    t(0) = (points(0,0) + points(1,0) + points(2,0)) / 3; 
    t(1) = (points(0,1) + points(1,1) + points(2,1)) / 3; 
    t(2) = (points(0,2) + points(1,2) + points(2,2)) / 3; 
    Eigen::Vector3d arr21,arr20;
    arr20 = points.row(0)-points.row(2);
    arr21 = points.row(1)-points.row(2);
    Eigen::Vector3d axisY = arr21.cross(arr20);
    axisY.normalize();
    Eigen::Vector3d axisZ;
    axisZ.x() =points(0,0)-t(0);
    axisZ.y() =points(0,1)-t(1) ;
    axisZ.z() =points(0,2)-t(2);
    axisZ.normalize();
    Eigen::Vector3d axisX = axisY.cross(axisZ);
    R.col(0) = axisX;
    R.col(1) = axisY;
    R.col(2) = axisZ;
}

void mediaNode::slam_callback(const interface::msg::Slam::SharedPtr slam_msg)
{
  geometry_msgs::msg::PoseStamped transform_init2cur_pub_msg, transform_cur2init_pub_msg;
  transform_init2cur_pub_msg = slam_msg->transform_init2cur;
  transform_init2cur_pub_msg.pose.position.x *= scaleFact_slamToWorld;
  transform_init2cur_pub_msg.pose.position.y *= scaleFact_slamToWorld;
  transform_init2cur_pub_msg.pose.position.z *= scaleFact_slamToWorld;

  Eigen::Quaterniond tempQua(transform_init2cur_pub_msg.pose.orientation.w,
                      transform_init2cur_pub_msg.pose.orientation.x,
                      transform_init2cur_pub_msg.pose.orientation.y,
                      transform_init2cur_pub_msg.pose.orientation.z);
  Eigen::Quaterniond tempQ(tempQua.inverse());
  transform_cur2init_pub_msg.header = slam_msg->point_cloud.header;
  transform_cur2init_pub_msg.pose.orientation.w = tempQ.w();
  transform_cur2init_pub_msg.pose.orientation.x = tempQ.x();
  transform_cur2init_pub_msg.pose.orientation.y = tempQ.y();
  transform_cur2init_pub_msg.pose.orientation.z = tempQ.z();
  transform_cur2init_pub_msg.pose.position.x = -transform_init2cur_pub_msg.pose.position.x;
  transform_cur2init_pub_msg.pose.position.y = -transform_init2cur_pub_msg.pose.position.y;
  transform_cur2init_pub_msg.pose.position.z = -transform_init2cur_pub_msg.pose.position.z;
  // //调试代码3
  // std::cout << "transform_init2cur_pub_msg.pose.position.x: " << transform_init2cur_pub_msg.pose.position.x << std::endl;
  // std::cout << "transform_init2cur_pub_msg.pose.position.y: " << transform_init2cur_pub_msg.pose.position.y << std::endl;
  // std::cout << "transform_init2cur_pub_msg.pose.position.z: " << transform_init2cur_pub_msg.pose.position.z << std::endl;
  // std::cout << "transform_init2cur_pub_msg.header.frame_id: " << transform_init2cur_pub_msg.header.frame_id << std::endl;
  
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
  
  // Put the publish operation in here is for the posible delay result of pcl operation, 
  // that may increase the message filter's load.
  transformInit2Cur_pub->publish(transform_init2cur_pub_msg);
  transformCurToInit_pub->publish(transform_cur2init_pub_msg);
  if (!flag_haveInitialized) return;

  point_cloud_pub_msg.header = slam_msg->point_cloud.header;
  // std::cout << "point_cloud_pub_msg.header.frame_id: " << point_cloud_pub_msg.header.frame_id << std::endl;
  pointCloud_pub->publish(point_cloud_pub_msg);

  return;
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
    changeErrorType(Not_found_the_countour);
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
    // 特征点轮廓判断    
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

    finalPointCloud.points.push_back(point);
  }

  // // 调试代码1
  // cv::imshow("img", img);
  // cv::waitKey(10);

  // 用筛选后的点云拟合平面
  if (finalPointCloud.points.size()<80) {
    changeErrorType(Point_cloud_is_little);
    return false;
  }
  Eigen::Vector4d param = calParam(finalPointCloud);
  distance_plane=-param[3];
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
    seg.setDistanceThreshold(3);
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
  for (int i=0;i<scaleFactList_.size();i++)
  {
    std::cout.precision(12);
    std::cout << scaleFactList_[i] << std::endl;
  }
  for (int i=0;i<iterNum;i++) 
  {
    int inlierNum = 0;
    std::random_shuffle(scaleFactList_.begin(), scaleFactList_.end());
    // std::vector<double>::const_iterator front = scaleFactList_.begin();
    // std::vector<double>::const_iterator back = scaleFactList_.begin()+sample_length;
    // std::vector<double> tempScaleFactList(front, back);
    // double tempSum = std::accumulate(tempScaleFactList.begin(), tempScaleFactList.end(), 0);
    double tempSum=0;
    for (int j=0;j<sample_length;j++)
    {
      tempSum+=scaleFactList_[j];
    }
    double result = tempSum/(double)sample_length;
    for (int j = 0;j<scaleFactList_.size();j++)
    {
      double value = std::abs(scaleFactList_[j]-result);
      if (value <= inlier_thresh_scaleFact) {
        inlierNum++;
      }
    }
    std::cout << "result: " << result <<std::endl;
    if (inlierNum >= maxInlierNum) 
    {
      maxInlierNum = inlierNum;
      scaleFact_slamToWorld = result;
    }
  }
  std::cout << "******************************" << std::endl;
  std::cout.precision(12);
  std::cout << "The final result of scale fact: " << scaleFact_slamToWorld <<std::endl;
}

int mediaNode::calRansacIterNum()
{
  float tempFloat = 1.0;
  for (int i=0;i<sample_length;i++) 
    tempFloat*=(inlier_probability_ * (float)scaleFactList_size_thresh-(float)i)/(float)(scaleFactList_size_thresh-i);
  int iterNum = std::ceil((float)1.0/tempFloat);
  std::cout << "***************************" << std::endl;
  std::cout<<"Ransac iteration time is: " << iterNum <<std::endl;
  return iterNum;
}

// 返回给消息点云和关键点图像坐标信息的消息
mediaNode::~mediaNode() {}