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
    int dilateSize_mask = fileRead["dilateSize_mask"];
    dilateStructure_mask = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(dilateSize_mask,dilateSize_mask));
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
    lineThresh = fileRead["lineThresh"];
    minLineLength = fileRead["minLineLength"];
    maxLineGap=fileRead["maxLineGap"];
    minCosValueThresh_collineation=fileRead["minCosValueThresh_collineation"];
    maxCosValueThresh_vertical=fileRead["maxCosValueThresh_vertical"];
    circle_size_thresh = fileRead["circle_size_thresh"];
    modelThresh=fileRead["modelThresh"];
    sample_length=fileRead["model_sample_length"];
    inlier_probability=fileRead["model_inlier_probability"];
    inlier_thresh_centre=fileRead["inlier_thresh_centre"];
    inlier_thresh_dirVec=fileRead["inlier_thresh_dirVec"];
    inlier_thresh_normVec=fileRead["inlier_thresh_normVec"];
}

void ModelNode::changeErrorType(ERROR_TYPE newError)
{
  if (newError != error_type)
  {
    static std::string errorTypeString[10] = {
      "Circle detection is not stable",
      "Distance of circle is too much",
      "Not found the countour",
      "Point cloud is little",
      "OK",
      "Image recieve error",
      "The camera is not straight on the plane",
      "the distance between camera and detect plane is too close",
      "No_detect_line",
      "Line_mismatch_condition"
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
            vecs_direction.clear();
            vecs_norm.clear();
            transforms_curToInit.clear();
            return false;
        }
    }
    // 利用稀疏点云拟合平面获得平面距离，并判断当前平面是否足够正对相机
    Eigen::Vector4d temp_param;
    cv::Mat mask; // 掩码图像，用以保存最大连通域区域
    bool flag_poseCorrect = detectPoseCorrect(temp_param,temp_circles[0],mask);
    if(!flag_poseCorrect) 
        return false;
    Eigen::Vector3d vec_norm = temp_param.block(0,0,3,1);
    double distance = abs(temp_param[4]);
    // 如果相機距離檢測平面太近，則很可能檢測不到實驗平臺邊沿輪廓直線
    if (distance < distance_thresh) {
        changeErrorType(TooClose);
        return false;
    }

    // TODO: 实验平台边沿直线检测...
    // 大阈值检测直线
    std::vector<cv::Vec4i> tempLines;
    cv::Mat img_gaussian;
    cv::GaussianBlur(m_img, img_gaussian, cv::Size(7,7),2,2);
    cv::HoughLinesP(img_gaussian,tempLines, 1,CV_PI/180, lineThresh, minLineLength,maxLineGap);
    if (tempLines.size()==0)
    {
        changeErrorType(No_detect_line);
        return false;
    }
    // 对线段列表按照长度从大到小排列
    std::sort(tempLines.begin(), tempLines.end(), GreaterLength());
    // 膨胀mask图像得到mask_dilate;
    cv::Mat mask_dilate;
    cv::dilate(mask, mask_dilate, dilateStructure_mask);
    // 定义临时线段参数变量 tempLine
    cv::Vec4i templine(0,0,0,0);
    Eigen::Vector3d DirectionVec_line(0,0,0);
    Eigen::Vector3d centrePosition;
    // for 循环从列表中筛选直线
    for (int i=0;i<tempLines.size();i++)
    {
        int x1_i = tempLines[i][0],x2_i = tempLines[i][2],y1_i=tempLines[i][1],y2_i=tempLines[i][3];
        // 利用mask_dilate掩码判断线段两端点是否都在最大连通域上
        if (mask_dilate.at<bool>(x1_i, y1_i)!=255 || mask_dilate.at<bool>(x2_i, y2_i)!=255)
        {
            continue;
        }
        // 利用线段两端点坐标计算中点坐标
        double x1 = (double)tempLines[i][0],x2 = (double)tempLines[i][2],y1=(double)tempLines[i][1],y2=(double)tempLines[i][3];
        Eigen::Vector2d temp_centerPoint((x1+x2)/(double)2, (y1+y2)/(double)2);
        // 利用中点坐标计算圆心指向线段中点的单位向量
        // TODO: 这里最好改垂直于线段方向且远离圆心的单位向量
        Eigen::Vector2d temp_centreToCenter(temp_centerPoint.x()-(double)temp_circles[0][0], temp_centerPoint.y()-(double)temp_circles[0][1]);
        temp_centreToCenter.normalize();
        // 利用中点坐标加上20倍的单位向量计算 flag_point(cv::Point)
        Eigen::Vector2d flag_point(temp_centerPoint.x()+20*temp_centreToCenter.x(),temp_centerPoint.y()+20*temp_centreToCenter.y());
        // 如果 mask.at<bool>(flag_point) == 255
        if (mask.at<bool>((int)flag_point.x(), (int)flag_point.y()) == 255)
        {
            continue;            
        }

        // 将直线两端点以及圆心像素坐标投影到三维平面上
        Eigen::Vector2d temp_endPoint2d_1(x1,y1),temp_endPoint2d_2(x2,y2);
        Eigen::Vector3d temp_endPoint3d_1, temp_endPoint3d_2, temp_centrePoint3d;
        from2dTo3dPlane(temp_endPoint2d_1, temp_endPoint3d_1, temp_param);
        from2dTo3dPlane(temp_endPoint2d_2, temp_endPoint3d_2, temp_param);
        from2dTo3dPlane(temp_centerPoint, temp_centrePoint3d, temp_param);
        // 计算三维空间上圆心到直线的距离
        Eigen::Vector3d tempDirectionVec_line = (temp_endPoint3d_2-temp_endPoint3d_1).normalized();
        Eigen::Vector3d tempVec_endPoint1ToCentre = temp_centrePoint3d-temp_endPoint2d_1;
        double distance_lineToCentre = (tempDirectionVec_line.cross(tempVec_endPoint1ToCentre)).norm();
        // 如果距离 小于最小距离阈值(200)
        if (distance_lineToCentre < double(200))
        {
            continue;            
        }

        // 如果当前全局三维方向向量列表为空
        if (vecs_direction.size()==0)
        {
            centrePosition = temp_centrePoint3d;
            DirectionVec_line = tempDirectionVec_line;
            break;
        }
        // 与全局列表的方向向量最后一个元素进行点乘输出绝对值
        double cosValue_line = tempDirectionVec_line.dot(vecs_direction.back());
        // 如果绝对值 小于共线阈值(0.97）&& 大于垂直阈值(0.17)
        if (abs(cosValue_line) < minCosValueThresh_collineation && abs(cosValue_line) > maxCosValueThresh_vertical)
        {
            continue;         
        }
        // 满足上诉所有筛选条件
        DirectionVec_line = tempDirectionVec_line;
        centrePosition = temp_centrePoint3d;
        break;
    }
    // 如果当前templine参数全为零
    if (DirectionVec_line.norm() < 1.0e-4)
    {
        changeErrorType(Line_mismatch_condition);
        return false;
    } 
    centrePositions.push_back(centrePosition);
    vecs_direction.push_back(DirectionVec_line);
    vecs_norm.push_back(vec_norm);
    circles_.push_back(temp_circles[0]);
    transforms_curToInit.push_back(m_transform_curToInit);
    int tempSizeSum = circles_.size()+vecs_norm.size()+vecs_direction.size()+transforms_curToInit.size();
    if (tempSizeSum!=circles_.size()*4||tempSizeSum!=vecs_norm.size()*4||tempSizeSum!=vecs_direction.size()*4||tempSizeSum!=transforms_curToInit.size()*4)
    {
        RCLCPP_INFO(this->get_logger(), "UNKNOWN ERROR !!!");
        return false;
    }
    if (circles_.size() < circle_size_thresh) 
        return false;

    // TODO:
    // 将连续几次检测的存储变量列表添加到较长的全局模型参数列表(平面法向量、垂直方向向量、圆心位置)
    for (int i=0;i<circles_.size();i++)
    {
        Eigen::Vector4d centrePosition_init;
        Eigen::Vector3d vec_norm_init;

        centrePosition_init = transforms_curToInit[i] * Eigen::Vector4d(centrePositions[i].x(),centrePositions[i].y(),centrePositions[i].z(),1);
        vec_norm_init=transforms_curToInit[i].block(0,0,3,3) * vecs_norm[i];
        global_centrePostions.push_back(centrePosition_init.block(0,0,3,1));
        global_vecs_norm.push_back(vec_norm_init);
        pushGlobalDirectionVec(vecs_direction[i], vec_norm_init, transforms_curToInit[i]);
    }
    if (global_centrePostions.size()<modelThresh)
    {
        // 清除全局连续性判断变量列表
        circles_.clear();
        vecs_direction.clear();
        vecs_norm.clear();
        transforms_curToInit.clear();
        changeErrorType(OK);
        return false;
    }
    // 开始ransac优化平面法向量、圆心位置以及垂直向上方向参数
    ransacModelParam();
    
    return true;
}

void ModelNode::ransacModelParam()
{
  int iterNum = calRansacIterNum();
  int maxInlierNum_centre = 0;
  int maxInlierNum_dirVec = 0;
  int maxInlierNum_normVec = 0;
  for (int i=0;i<iterNum;i++) 
  {
    int inlierNum_centre = 0;
    int inlierNum_dirVec =0;
    int inlierNum_normVec =0;
    std::random_shuffle(global_centrePostions.begin(), global_centrePostions.end());
    std::random_shuffle(global_vecs_direction.begin(), global_vecs_direction.end());
    std::random_shuffle(global_vecs_norm.begin(), global_vecs_norm.end());
    double tempSum_x=0,tempSum_y=0,tempSum_z=0;
    Eigen::Vector3d result_centre;
    ceres::Problem problem_dirVec;
    ceres::Problem problem_normVec;
    double a_dirVec=0,b_dirVec=0,c_dirVec=0;
    double a_normVec=0,b_normVec=0,c_normVec=0;
    for (int j=0;j<sample_length;j++)
    {
        tempSum_x += global_centrePostions[j].x();
        tempSum_y += global_centrePostions[j].y();
        tempSum_z += global_centrePostions[j].z();
        if (j == sample_length-1)
        {
            result_centre.x() = tempSum_x/sample_length;
            result_centre.y() = tempSum_y/sample_length;
            result_centre.z() = tempSum_z/sample_length;
        }
        problem_dirVec.AddResidualBlock(
            new ceres::AutoDiffCostFunction<normVecResidual,1,1,1>(
                new normVecResidual(global_vecs_direction[j].x(), global_vecs_direction[j].y(), global_vecs_direction[j].z())
            ),
            NULL,
            &a_dirVec,&b_dirVec
        );
        problem_normVec.AddResidualBlock(
            new ceres::AutoDiffCostFunction<normVecResidual,1,1,1>(
                new normVecResidual(global_vecs_norm[j].x(), global_vecs_norm[j].y(), global_vecs_norm[j].z())
            ),
            NULL,
            &a_normVec,&b_normVec
        );
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 30;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::SILENT;
    ceres::Solver::Summary summary_dirVec;
    ceres::Solver::Summary summary_normVec;
    Solve(options, &problem_dirVec, &summary_dirVec);
    Solve(options, &problem_normVec, &summary_normVec);
    c_dirVec = sqrt(1-pow(a_dirVec,2)-pow(b_dirVec,2));
    c_normVec = sqrt(1-pow(a_normVec,2)-pow(b_normVec,2));
    // 得到优化的变量以后使用该参数模型利用与样点参数之间的夹角余弦是否在夹角阈值范围内来筛选内点，并统计内点数量
    int tempNumInlier_dirVec=0;
    int tempNumInlier_normVec=0;
    int tempNumInlier_centre=0;
    for(int n=0;n<global_vecs_direction.size();n++) {
        if (a_dirVec*global_vecs_direction[n](0)+b_dirVec*global_vecs_direction[n](1)+c_dirVec*global_vecs_direction[n](2) > inlier_thresh_dirVec) {
            tempNumInlier_dirVec++;
        }
        if (a_normVec*global_vecs_norm[n](0)+b_normVec*global_vecs_norm[n](1)+c_normVec*global_vecs_norm[n](2) > inlier_thresh_normVec) {
            tempNumInlier_normVec++;
        }
        if (std::sqrt(std::pow(global_centrePostions[n](0)-result_centre(0),2)
                     +std::pow(global_centrePostions[n](1)-result_centre(1),2)
                     +std::pow(global_centrePostions[n](2)-result_centre(2),2))<inlier_thresh_centre)
        {
            tempNumInlier_centre++;
        }
    }
    if (inlierNum_centre > maxInlierNum_centre) 
    {
      maxInlierNum_centre = inlierNum_centre;
      final_centrePosition = result_centre;
    }
    if (inlierNum_dirVec > maxInlierNum_dirVec) 
    {
      maxInlierNum_dirVec = inlierNum_dirVec;
      final_dirVec << a_dirVec,b_dirVec,c_dirVec;
    }
    if (inlierNum_normVec > maxInlierNum_normVec) 
    {
      maxInlierNum_normVec = inlierNum_normVec;
      final_normVec << a_normVec,b_normVec,c_normVec;
    }
  }
  final_dirVec = final_normVec.cross(final_dirVec).cross(final_normVec).normalized();
  std::cout << "******************************" << std::endl;
  std::cout << "The final result of centre position: " << final_centrePosition.transpose() <<std::endl;
  std::cout << "The final result of direction vector: " << final_dirVec.transpose() <<std::endl;
  std::cout << "The final result of plane norm vector: " << final_normVec.transpose() <<std::endl;
}

int ModelNode::calRansacIterNum()
{
  float tempFloat = 1.0;
  for (int i=0;i<sample_length;i++) 
    tempFloat*=(inlier_probability * (float)modelThresh-(float)i)/(float)(modelThresh-i);
  int iterNum = std::ceil((float)1.0/tempFloat);
  std::cout << "***************************" << std::endl;
  std::cout<<"Ransac iteration time is: " << iterNum <<std::endl;
  return iterNum;
}

void ModelNode::pushGlobalDirectionVec(const Eigen::Vector3d vec_direction,
                                       const Eigen::Vector3d vec_norm,
                                       const Eigen::Matrix4d transform_curToInit)
{
    Eigen::Vector3d vec_direction_init = (transform_curToInit.block(0,0,3,3) * vec_direction).normalized();
    if (vec_direction_init.dot(Eigen::Vector3d(0,-1,0)) > std::cos(CV_PI/4))
    {
        global_vecs_direction.push_back(vec_direction_init);
    }
    if (vec_direction_init.dot(Eigen::Vector3d(0,1,0)) > std::cos(CV_PI/4))
    {
        global_vecs_direction.push_back(-vec_direction_init);
    }
    if (vec_direction_init.dot(Eigen::Vector3d(-1,0,0)) > std::cos(CV_PI/4))
    {
        global_vecs_direction.push_back(vec_norm.cross(vec_direction_init).normalized());
    }
    if (vec_direction_init.dot(Eigen::Vector3d(1,0,0)) > std::cos(CV_PI/4))
    {
        global_vecs_direction.push_back(vec_direction_init.cross(vec_norm).normalized());
    }
    return ;
}

void ModelNode::from2dTo3dPlane(const Eigen::Vector2d inputPoint, Eigen::Vector3d &outputPoint, Eigen::Vector4d paramPlane)
{
    double a=paramPlane[0];
    double b=paramPlane[1];
    double c=paramPlane[2];
    double d=paramPlane[3];
    outputPoint.z() = -d/(a*(inputPoint.x()-cx)/fx+b*(inputPoint.y()-cy)/fy+c);
    outputPoint.x() = outputPoint.z()*(inputPoint.x()-cx)/fx;
    outputPoint.y() = outputPoint.z()*(inputPoint.y()-cy)/fy;    
}


bool ModelNode::detectPoseCorrect(Eigen::Vector4d &param, const cv::Vec3f tempCircle, cv::Mat &mask)
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
  param = calParam(finalPointCloud);

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
                Eigen::Vector3d Point;
                from2dTo3dPlane(Eigen::Vector2d(v,u), Point, paramInCamFrame);
                tempPoint.z = Point.z();
                tempPoint.x = Point.x();
                tempPoint.y = Point.y();
                obstacleInCamFrame.points.push_back(tempPoint);
            }
        }
    };
    pcl::transformPointCloud(obstacleInCamFrame, obstacle, m_transform_curToInit);
};

// 析构函数 delete function()
ModelNode::~ModelNode() {
    ofs.close();
}