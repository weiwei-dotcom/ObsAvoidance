#include "model.hpp"

void ModelNode::mapPoint_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcl_msg,
                       const sensor_msgs::msg::Image::ConstSharedPtr &img_msg,
                       const geometry_msgs::msg::PoseStamped::ConstSharedPtr &pose_msg)
{
    // If have complete publish the pointcloud of model, don't need model again;

    // // 调试代码3
    // std::cout << "-----------------------------------------" << std::endl;
    // std::cout << "Start recieving the msg" << std::endl;

    if (flag_pubModel)
    {
        PubModel(); 
        return;        
    }

    // Recieve the message data;
    if (!recieveMsg(pcl_msg, img_msg, pose_msg))
        return;   
    // Start modeling;
    if (!Model())
    {
        // 调试代码3
        cv::waitKey(10);

        return; 
    }
    cv::waitKey(10);
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
    std::cout << "Msg recieved success !" << std::endl;
    return true;
}

// 构造函数 init function()
ModelNode::ModelNode():Node("model")
{
    // 调试代码3
    successNum = 0;

    error_type = OK;
    pcl_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pcl_obstacle", 10);
    flag_pubModel = false;
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
    int blueLower1 = fileRead["blueLower.1"];
    int blueLower2 = fileRead["blueLower.2"];
    int blueLower3 = fileRead["blueLower.3"];
    int blueUpper1 = fileRead["blueUpper.1"];
    int blueUpper2 = fileRead["blueUpper.2"];
    int blueUpper3 = fileRead["blueUpper.3"];
    int bigCloseStructure_size = fileRead["model_bigCloseStructure_size"];
    int erodeStructure_size1 = fileRead["model_erodeStructure_size1"];
    int erodeStructure_size2 = fileRead["model_erodeStructure_size2"];
    structure_erode1=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeStructure_size1, erodeStructure_size1));
    structure_erode2=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erodeStructure_size2, erodeStructure_size2));
    blueLower = cv::Scalar(blueLower1, blueLower2, blueLower3);
    blueUpper = cv::Scalar(blueUpper1, blueUpper2, blueUpper3);
    pixelNum_translate = fileRead["pixelNum_translate"];
    distanceThresh_plandAndPoint=fileRead["distanceThresh_plandAndPoint"];
    fx = fileRead["Camera.fx"];
    fy = fileRead["Camera.fy"];
    cx = fileRead["Camera.cx"];
    cy = fileRead["Camera.cy"];
    m_projectMatrix << fx, 0, cx,
                        0, fy, cy,
                        0,  0,  1; 
    minDist = fileRead["model_minDist"];
    dp = fileRead["model_dp"];
    cannyUpThresh = fileRead["model_cannyUpThresh"];
    circleThresh = fileRead["model_circleThresh"];
    minRadius = fileRead["model_minRadius"];
    maxRadius = fileRead["model_maxRadius"];
    difference_radius_thresh = fileRead["model_difference_radius_thresh"];
    distance_center_thresh = fileRead["model_distance_center_thresh"];
    cosValueThresh_planeNormAndCameraZaxis = fileRead["model_cosValueThresh_planeNormAndCameraZaxis"];
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
    canny_threshLow = fileRead["canny_threshLow"];
    canny_threshUp=fileRead["canny_threshUp"];
    kernelBigClose = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(bigCloseStructure_size, bigCloseStructure_size));
    outCircleVal=fileRead["outCircleVal"];
    inCircleVal=fileRead["inCircleVal"];
    mapPointNumThresh=fileRead["mapPointNumThresh"];
}

void ModelNode::changeErrorType(ERROR_TYPE newError)
{
  if (newError != error_type)
  {
    static std::string errorTypeString[11] = {
      "Circle detection is not stable",
      "Distance of circle is too much",
      "Not found the countour",
      "Point cloud is little",
      "OK",
      "Image recieve error",
      "The camera is not straight on the plane",
      "the distance between camera and detect plane is too close",
      "No_detect_line",
      "Line_mismatch_condition",
      "Can't find circle"
    };
    error_type = newError;
    RCLCPP_INFO(this->get_logger(), "Error Type: %s", errorTypeString[(int)error_type].c_str());
  }
}

// hough圆形检测函数
void ModelNode::houghCircleDetect(std::vector<cv::Vec3f> &outputCircles, cv::Mat &img_gaussian)
{
    // 入口圓形檢測
    std::cout << "houghCircleDetect()" <<std::endl;
    cv::Mat img_gray;
    cv::cvtColor(m_img, img_gray, CV_BGR2GRAY);
    cv::GaussianBlur(img_gray, img_gaussian, cv::Size(7, 7),2,2);
    cv::HoughCircles(img_gaussian, outputCircles, cv::HOUGH_GRADIENT, dp, minDist, cannyUpThresh, circleThresh, minRadius, maxRadius);
}

bool ModelNode::getMaxArea(cv::Mat &maxArea)
{
    // 阈值分割    
    std::cout << "getMaxArea()" <<std::endl;
    cv::Mat imgHsv, imgBin, img_erode, img_dilate;
    cv::cvtColor(m_img, imgHsv, CV_BGR2HSV);
    cv::inRange(imgHsv, blueLower, blueUpper, imgBin);
    // 较小核腐蚀
    cv::erode(imgBin, img_erode, structure_erode1);
    cv::imshow("temp_img_erode", img_erode);
    cv::Mat labels, stats, centroids;
    int num_labels = connectedComponentsWithStats(img_erode, labels, stats, centroids, 8, CV_16U);
    std::cout<<"num_labels: " <<num_labels<<std::endl;
    std::vector<int> areas;
    if (num_labels>1)
    {
        for (int i = 1; i < num_labels; i++) // 忽略背景标签0
        {
            areas.push_back(stats.at<int>(i, cv::CC_STAT_AREA));
        }
        int max_area_label = max_element(areas.begin(), areas.end()) - areas.begin() + 1;
        maxArea = (labels == max_area_label);
        std::cout << "getMaxArea()" <<std::endl;

        return true;
    }
    std::cout << "getMaxArea()" <<std::endl;

    return false;
    
}
// 入口圆形筛选函数
bool ModelNode::selectCircle(cv::Vec3f &circle, const std::vector<cv::Vec3f> temp_circles, cv::Mat &mask)
{
    std::cout << "selectCircle()" <<std::endl;
    cv::Mat maxArea;
    bool flag_getMaxArea = getMaxArea(maxArea);
    if (!flag_getMaxArea)
    {
        std::cout << "Can't find max area !" <<std::endl;
        return false;
    }
    // 大尺度闭运算
    cv::morphologyEx(maxArea, mask, cv::MORPH_CLOSE, kernelBigClose);
    std::cout << "selectCircle()2" <<std::endl;

    for (int i = 0;i<temp_circles.size();i++)
    {
        bool flag_found = true;
        for (int j=0;j<8;j++)
        {
            cv::Point pointInCircle((int)(temp_circles[i][0]+(temp_circles[i][2]-inCircleVal)*std::cos(CV_PI/4*j)), (int)(temp_circles[i][1]+(double)(temp_circles[i][2]-inCircleVal)*std::sin(CV_PI/4*j)));
            cv::Point pointOutCircle((int)(temp_circles[i][0]+(temp_circles[i][2]+outCircleVal)*std::cos(CV_PI/4*j)), (int)(temp_circles[i][1]+(double)(temp_circles[i][2]+outCircleVal)*std::sin(CV_PI/4*j)));
            if (pointInCircle.x < 0 || pointInCircle.x > 639 || pointInCircle.y > 479 || pointInCircle.y < 0 
                || pointOutCircle.x < 0 || pointOutCircle.x > 639 || pointOutCircle.y > 479 || pointOutCircle.y < 0)
            {
                flag_found=false;
                break;
            }
            if (mask.at<bool>(pointInCircle)==255||mask.at<bool>(pointOutCircle)!=255)
            {
                flag_found=false;
                break;
            }
        }
        if (!flag_found)
        {
            continue;
        }
        circle=temp_circles[i];
        std::cout << "selectCircle()" <<std::endl;
        return true;
    }
    changeErrorType(Circle_detection_is_not_stable);
    return false;
}
// 直线检测函数
void ModelNode::detectLine(const cv::Mat img_gaussian, std::vector<cv::Vec4i> &tempLines)
{
    std::cout << "detectLine()" <<std::endl;

    cv::Mat img_edge;
    cv::Canny(img_gaussian, img_edge, canny_threshLow, canny_threshUp);
    cv::threshold(img_edge,img_edge, 170, 255, cv::THRESH_BINARY);
    cv::HoughLinesP(img_edge,tempLines, 1,CV_PI/180, lineThresh, minLineLength,maxLineGap);    
}
bool ModelNode::selectLine(const cv::Mat img_erode, std::vector<cv::Vec4i> tempLines, cv::Vec4i &line,const cv::Vec3f circle)
{
    std::cout << "selectLine()" <<std::endl;

    // 对线段列表按照长度从大到小排列
    std::sort(tempLines.begin(), tempLines.end(), GreaterLength());
    // for 循环从列表中筛选直线
    for (int i=0;i<tempLines.size();i++)
    {
        int x1_i = tempLines[i][0],x2_i = tempLines[i][2],y1_i=tempLines[i][1],y2_i=tempLines[i][3];
        cv::Point endPoint1(x1_i, y1_i), endPoint2(x2_i,y2_i), midPoint((x1_i+x2_i)/2, (y1_i+y2_i)/2);
        // 将z轴分量置为零计算三维向量，利用叉乘较方便的计算垂直直线指向二维圆心的方向向量
        Eigen::Vector3d temp_dirLine1((double)(x2_i-x1_i),(double)(y2_i-y1_i),0),temp_dirEndToCentre((double)(circle[0]-x1_i), (double)(circle[1]-y1_i),0);
        Eigen::Vector3d dirNorm=temp_dirLine1.cross(temp_dirEndToCentre).cross(temp_dirLine1).normalized();
        cv::Point flag_point1(x1_i+(int)(pixelNum_translate*dirNorm[0]), y1_i+(int)(pixelNum_translate*dirNorm[1])),flag_point2(x2_i+(int)(pixelNum_translate*dirNorm[0]), y2_i+(int)(pixelNum_translate*dirNorm[1]));
        if (flag_point1.x>639 || flag_point1.x<0 ||flag_point1.y>479 || flag_point1.y<0 
            || flag_point2.x>639 || flag_point2.x<0 ||flag_point2.y>479 || flag_point2.y<0)
        {
            continue;
        }

        // 调试代码3
        cv::Mat img_copy;
        m_img.copyTo(img_copy);
        cv::drawMarker(img_copy, endPoint1, cv::Scalar(0,255,0),2, 10, 2);
        cv::drawMarker(img_copy, endPoint2, cv::Scalar(0,255,0),2, 10, 2);
        cv::drawMarker(img_copy, midPoint, cv::Scalar(0,255,0),2, 10, 2);
        cv::drawMarker(img_copy, flag_point1, cv::Scalar(0,0,255),2, 10, 2);
        cv::drawMarker(img_copy, flag_point2, cv::Scalar(0,0,255),2, 10, 2);

        cv::imshow("img_endPoint", img_copy);
        
        if (img_erode.at<bool>(endPoint1)==255 || img_erode.at<bool>(endPoint2)==255 || img_erode.at<bool>(midPoint)==255)
        {
            continue;
        }
        if (img_erode.at<bool>(flag_point1)!=255 || img_erode.at<bool>(flag_point2)!=255)
        {
            continue;
        }
        // 将直线两端点以及圆心像素坐标投影到三维平面上
        line = tempLines[i];
        return true;
    }
    changeErrorType(Line_mismatch_condition);
    return false;
}

void ModelNode::calModelParam(const cv::Vec3f circle,const cv::Vec4i line,const Eigen::Vector4d param, 
                Eigen::Vector3d &planeNormalVec,Eigen::Vector3d &verticalVec, Eigen::Vector3d &circleCentreInInit)
{
    std::cout << "calModelParam()" <<std::endl;

    Eigen::Vector3d temp_circleCentre,endPoint1,endPoint2;
    from2dTo3dPlane(Eigen::Vector2d((double)circle[0],(double)circle[1]), temp_circleCentre, param);
    from2dTo3dPlane(Eigen::Vector2d((double)line[0],(double)line[1]), endPoint1, param);
    from2dTo3dPlane(Eigen::Vector2d((double)line[2],(double)line[3]), endPoint2, param);
    Eigen::Vector3d lineDirInCur = (endPoint2-endPoint1).normalized();
    Eigen::Vector3d lineDirInInit = m_transform_curToInit.block(0,0,3,3) * lineDirInCur;

    // 调试代码
    std::cout << "lineDirInInit.transpose(): "<<lineDirInInit.transpose() << std::endl;
    
    if (std::abs(lineDirInInit.dot(Eigen::Vector3d(0,-1,0))) < std::cos(45))
    {
        verticalVec = lineDirInInit.cross(Eigen::Vector3d(0,-1,0).cross(lineDirInInit)).normalized();
        std::cout <<"verticalVec: "<<verticalVec.transpose()<<std::endl;
    }
    else
    {
        verticalVec = lineDirInInit.dot(Eigen::Vector3d(0,-1,0)) > 0 ? lineDirInInit.normalized() : (-lineDirInInit.normalized());
        std::cout <<"verticalVec: "<<verticalVec.transpose()<<std::endl;
    }
    planeNormalVec = (m_transform_curToInit.block(0,0,3,3) * param.block(0,0,3,1)).normalized();
    std::cout <<"planeNormalVec: "<<planeNormalVec.transpose()<<std::endl;
    Eigen::Vector4d tempCentreInCur(temp_circleCentre[0],temp_circleCentre[1],temp_circleCentre[2],1);
    circleCentreInInit = (m_transform_curToInit*tempCentreInCur).block(0,0,3,1);
    std::cout <<"circleCentre: "<<circleCentreInInit.transpose()<<std::endl;
    std::cout << "calModelParam()" <<std::endl;
    return;
}

void ModelNode::ransacModelParam()
{
    std::cout << "ransacModelParam()" <<std::endl;

    int iterNum = calRansacIterNum();
    std::cout <<"iterNum: " <<iterNum<<std::endl;
    cv::waitKey(0);
    int maxInlierNum_centre = 0;
    int maxInlierNum_dirVec = 0;
    int maxInlierNum_normVec = 0;

    Eigen::Vector3d final_centrePosition;
    Eigen::Vector3d final_normVec;
    Eigen::Vector3d final_dirVec;

    //debug
    int loopCount=0;
    std::cout << "global_vecs_direction.size(): " <<global_vecs_direction.size() <<std::endl;
    std::cout << "global_vecs_norm.size(): " << global_vecs_norm.size()<<std::endl;
    std::cout << "global_centrePostions.size(): "<<global_centrePostions.size() <<std::endl;
    int debugLoopCount=0;
    for (int i=0;i<iterNum;i++) 
    {
        loopCount++;
        debugLoopCount++;
        int inlierNum_centre = 0;
        int inlierNum_dirVec = 0;
        int inlierNum_normVec = 0;
        std::random_shuffle(global_centrePostions.begin(), global_centrePostions.end());
        std::random_shuffle(global_vecs_direction.begin(), global_vecs_direction.end());
        std::random_shuffle(global_vecs_norm.begin(), global_vecs_norm.end());
        double tempSum_x=0,tempSum_y=0,tempSum_z=0;

        Eigen::Vector3d result_centre;
        ceres::Problem problem_dirVec;
        ceres::Problem problem_normVec;
        double a_dirVec=0,b_dirVec=-1,c_dirVec=0;
        double a_normVec=0,b_normVec=0,c_normVec=1;
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
                new ceres::AutoDiffCostFunction<normVecResidual,1,1,1,1>(
                    new normVecResidual(global_vecs_direction[j].x(), global_vecs_direction[j].y(),global_vecs_direction[j].z())
                ),
                NULL,
                &a_dirVec,&b_dirVec,&c_dirVec
            );
            problem_normVec.AddResidualBlock(
                new ceres::AutoDiffCostFunction<normVecResidual,1,1,1,1>(
                    new normVecResidual(global_vecs_norm[j].x(), global_vecs_norm[j].y(),global_vecs_norm[j].z())
                ),
                NULL,
                &a_normVec,&b_normVec,&c_normVec
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
        // c_dirVec = sqrt(1-pow(a_dirVec,2)-pow(b_dirVec,2));
        // c_normVec = sqrt(1-pow(a_normVec,2)-pow(b_normVec,2));
        //debug
        if (debugLoopCount==5)
        {
            //debug
            std::cout <<"result_centre: " << result_centre.transpose()<<std::endl;
            std::cout <<"maxInlierNum_centre: "<<maxInlierNum_centre<<std::endl;
            std::cout <<"final_centrePosition: "<<final_centrePosition.transpose()<<std::endl;
            std::cout << "dirVec: " << a_dirVec<<" "<<b_dirVec<<" "<<c_dirVec<<std::endl;
            std::cout <<"maxInlierNum_dirVec: "<<maxInlierNum_dirVec<<std::endl;
            std::cout <<"final_dirVec: "<<final_dirVec.transpose()<<std::endl;
            std::cout << "normVec: " << a_normVec<<" "<<b_normVec<<" "<<c_normVec<<std::endl; 
            std::cout <<"maxInlierNum_normVec: "<<maxInlierNum_normVec<<std::endl;
            std::cout <<"final_normVec: "<<final_normVec.transpose()<<std::endl;
            debugLoopCount=0;   
            cv::waitKey(0);        
        }
        Eigen::Vector3d temp_dirVec(a_dirVec,b_dirVec,c_dirVec);
        Eigen::Vector3d temp_normVec(a_normVec,b_normVec,c_normVec);
        temp_dirVec.normalize();
        temp_normVec.normalize();
        // 得到优化的变量以后使用该参数模型利用与样点参数之间的夹角余弦是否在夹角阈值范围内来筛选内点，并统计内点数量
        for(int n=0;n<global_vecs_direction.size();n++) {
            if (temp_dirVec.dot(global_vecs_direction[n]) > inlier_thresh_dirVec) {
                inlierNum_dirVec++;
            }
            if (temp_normVec.dot(global_vecs_norm[n]) > inlier_thresh_normVec) {
                inlierNum_normVec++;
            }
            if (std::sqrt(std::pow(global_centrePostions[n][0]-result_centre[0],2)
                        +std::pow(global_centrePostions[n][1]-result_centre[1],2)
                        +std::pow(global_centrePostions[n][2]-result_centre[2],2))< inlier_thresh_centre)
            {
                inlierNum_centre++;
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
            final_dirVec = temp_dirVec;
        }
        if (inlierNum_normVec > maxInlierNum_normVec) 
        {
            maxInlierNum_normVec = inlierNum_normVec;
            final_normVec = temp_normVec;
        }
        if (loopCount > modelThresh/2 && maxInlierNum_dirVec < modelThresh/10)
        {
            inlier_thresh_dirVec-=0.01;
        }
        if (loopCount > modelThresh/2 && maxInlierNum_normVec < modelThresh/10)
        {
            inlier_thresh_normVec-=0.01;
        }
    }
    final_dirVec = final_normVec.cross(final_dirVec).cross(final_normVec).normalized();
    std::cout << "******************************" << std::endl;
    std::cout << "The final result of centre position: " << final_centrePosition.transpose() <<std::endl;
    std::cout << "The final result of direction vector: " << final_dirVec.transpose() <<std::endl;
    std::cout << "The final result of plane norm vector: " << final_normVec.transpose() <<std::endl;
    
    Yaxis = final_normVec;
    Zaxis = final_dirVec;
    Xaxis = Yaxis.cross(Zaxis);
    centrePosition=final_centrePosition;
    frontLeftUnder = centrePosition-Zaxis*220-Xaxis*220;
    structureLeftUnder1 = frontLeftUnder+Yaxis*185;
    structureLeftUnder2 = frontLeftUnder+Yaxis*410;
    backLeftUnder = frontLeftUnder+Yaxis*735;
    frontRightUnder=centrePosition+220*Xaxis-220*Zaxis;
    frontLeftUp=centrePosition-220*Xaxis+220*Zaxis;

    std::cout << "Xaxis: " <<Xaxis.transpose() << std::endl;
    std::cout << "Yaxis: " <<Yaxis.transpose() << std::endl;
    std::cout << "Zaxis: " <<Zaxis.transpose() << std::endl;
    std::cout << "centrePosition: " <<centrePosition.transpose() << std::endl;
    std::cout << "frontLeftUnder: " <<frontLeftUnder.transpose() << std::endl;
    std::cout << "structureLeftUnder1: " <<structureLeftUnder1.transpose() << std::endl;
    std::cout << "structureLeftUnder2: " <<structureLeftUnder2.transpose() << std::endl;
    std::cout << "backLeftUnder: " <<backLeftUnder.transpose() << std::endl;
    std::cout << "frontRightUnder: " <<frontRightUnder.transpose() << std::endl;
    std::cout << "frontLeftUp: " << frontLeftUp.transpose() << std::endl;
    cv::waitKey(0);
}

void ModelNode::buildFront()
{
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<220;j++)
        {
            Eigen::Vector3d pointPosition = frontLeftUnder+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(frontRightUnder-frontLeftUnder)*((double)j/(double)220);
            if ((pointPosition-centrePosition).norm() < 100)
                continue;
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
}
void ModelNode::buildBack()
{
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<220;j++)
        {
            Eigen::Vector3d pointPosition = backLeftUnder+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(frontRightUnder-frontLeftUnder)*((double)j/(double)220);
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
}
void ModelNode::buildSide()
{
    // 从前方左下角开始逐行建模
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<368;j++)
        {
            Eigen::Vector3d pointPosition = frontLeftUnder+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(backLeftUnder-frontLeftUnder)*((double)j/(double)368);
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
    // 从前方左上角开始逐行建模
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<368;j++)
        {
            Eigen::Vector3d pointPosition = frontLeftUp+(frontRightUnder-frontLeftUnder)*((double)i/(double)220)+(backLeftUnder-frontLeftUnder)*((double)j/(double)368);
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
    // 从前方右下角开始逐行建模
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<368;j++)
        {
            Eigen::Vector3d pointPosition = frontRightUnder+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(backLeftUnder-frontLeftUnder)*((double)j/(double)368);
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
    // 前方左下角开始逐行建模
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<368;j++)
        {
            Eigen::Vector3d pointPosition = frontLeftUnder+(frontRightUnder-frontLeftUnder)*((double)i/(double)220)+(backLeftUnder-frontLeftUnder)*((double)j/(double)368);
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
}
void ModelNode::buildStructure1()
{
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<220;j++)
        {
            Eigen::Vector3d pointPosition = structureLeftUnder1+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(frontRightUnder-frontLeftUnder)*((double)j/(double)220);
            if (i>90 && j<130)
                continue;
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
}
void ModelNode::buildStructure2()
{
    for (int i=1;i<220;i++)
    {  
        for(int j=1;j<220;j++)
        {
            Eigen::Vector3d pointPosition = structureLeftUnder2+(frontLeftUp-frontLeftUnder)*((double)i/(double)220)+(frontRightUnder-frontLeftUnder)*((double)j/(double)220);
            if (i<125 && j>95)
                continue;
            pcl::PointXYZ point((float)pointPosition.x(),(float)pointPosition.y(),(float)pointPosition.z());
            pcl_obstacle.points.push_back(point);
        }
    }
}

// 障碍物建模主函数
bool ModelNode::Model()
{
    std::cout << "Model()" <<std::endl;

    // 入口圓形檢測
    std::vector<cv::Vec3f> temp_circles;
    cv::Mat img_gaussian;
    houghCircleDetect(temp_circles,img_gaussian);
    if (temp_circles.size()==0) {
        changeErrorType(Can_not_find_circle);
        return false;
    }

    // 调试代码3
    cv::Mat img_circle;
    m_img.copyTo(img_circle);
    for (int i =0;i<temp_circles.size();i++)
    {
        cv::circle(img_circle, cv::Point(temp_circles[i][0], temp_circles[i][1]), temp_circles[i][2], cv::Scalar(0,255,0), 2);        
    }
    cv::imshow("init_circle_img", img_circle);

    cv::Vec3f circle;
    cv::Mat mask;                                                     // 这个mask是用来保存最大区域掩码图像
    bool flag_findCircle = selectCircle(circle, temp_circles, mask);
    if (!flag_findCircle)
        return false;

    // 利用稀疏点云拟合平面获得平面距离，并判断当前平面是否足够正对相机
    Eigen::Vector4d param;
    cv::Mat img_erode;
    bool flag_poseCorrect = detectPoseCorrect(param,mask,img_erode);
    if(!flag_poseCorrect) return false;

    // 从拟合得到的平面参数中得到当前相机坐标系下的平面法向量以及距离平面的距离值
    double distance = std::abs(param[3]);

    // 当相机距离检测平面太近时，可能检测不到直线
    if (distance < distance_thresh) {
        changeErrorType(TooClose);
        return false;
    }
    // 大阈值检测直线
    std::vector<cv::Vec4i> tempLines;
    detectLine(img_gaussian, tempLines);
    if (tempLines.size()==0)
    {
        changeErrorType(No_detect_line);
        return false;
    }

    // 选择直线
    cv::Vec4i line;
    bool flag_findLine = selectLine(img_erode,tempLines,line,circle);
    if (!flag_findLine) return false;

    // 计算三维全局参数;
    Eigen::Vector3d planeNormalVec, verticalVec, circleCentre;
    calModelParam(circle, line, param,planeNormalVec, verticalVec, circleCentre);
    global_centrePostions.push_back(circleCentre);
    global_vecs_direction.push_back(verticalVec);
    global_vecs_norm.push_back(planeNormalVec);
    successNum++;
    if (global_centrePostions.size() < modelThresh)
    {
        return false;
    }

    // debug
    for (int i=0;i<global_centrePostions.size();i++)
    {
        std::cout<< "global_centrePostions[" << i<<"]: "<<global_centrePostions[i].transpose()<< std::endl;
        std::cout<< "global_vecs_direction[" << i<<"]: "<<global_vecs_direction[i].transpose()<< std::endl;
        std::cout<< "global_vecs_norm[" << i<<"]: "<<global_vecs_norm[i].transpose()<< std::endl;
        std::cout<< "-----------------------------------------------------" << std::endl;
    }
    cv::waitKey(0);

    // 开始ransac优化平面法向量、圆心位置以及垂直向上方向参数
    ransacModelParam();

    // 开始利用结构关系构建目标障碍物点云地图
    buildFront();
    buildBack();
    buildSide();
    buildStructure1();
    buildStructure2();

    return true;
}

int ModelNode::calRansacIterNum()
{
    std::cout << "calRansacIterNum()"<<std::endl;
    float tempFloat = 1.0;
    for (int i=0;i<sample_length;i++) 
        tempFloat*=(inlier_probability * (float)modelThresh-(float)i)/(float)(modelThresh-i);
    int iterNum = std::ceil((float)1.0/tempFloat);
    std::cout << "***************************" << std::endl;
    std::cout<<"Ransac iteration time is: " <<  iterNum <<std::endl;
    return iterNum;
}

void ModelNode::from2dTo3dPlane(const Eigen::Vector2d inputPoint, Eigen::Vector3d &outputPoint, Eigen::Vector4d paramPlane)
{
    std::cout << "from2dTo3dPlane()"<<std::endl;

    double a=paramPlane[0];
    double b=paramPlane[1];
    double c=paramPlane[2];
    double d=paramPlane[3];
    outputPoint.z() = -d/(a*(inputPoint.x()-cx)/fx+b*(inputPoint.y()-cy)/fy+c);
    outputPoint.x() = outputPoint.z()*(inputPoint.x()-cx)/fx;
    outputPoint.y() = outputPoint.z()*(inputPoint.y()-cy)/fy;    
}


bool ModelNode::detectPoseCorrect(Eigen::Vector4d &param, const cv::Mat mask, cv::Mat &img_erode)
{
    std::cout << "detectPoseCorrect()"<<std::endl;
    
    cv::Mat temp_imgErode;
    cv::erode(mask, img_erode, structure_erode2);
    // cv::threshold(temp_imgErode,img_erode,150,255,cv::THRESH_BINARY);
    // 点云变换坐标系至当前帧
    pcl::PointCloud<pcl::PointXYZ> PointCloud_curFrame;
    pcl::transformPointCloud(m_pointCloud, PointCloud_curFrame, m_transform_initToCur);
    // 逐个点云逆投影至像素平面进行筛选
    pcl::PointCloud<pcl::PointXYZ> finalPointCloud;
    cv::imshow("img_erode", img_erode);
    // 调试代码3
    cv::Mat temp_img;
    m_img.copyTo(temp_img);

    for (auto point:PointCloud_curFrame.points) 
    {
        Eigen::Vector3f tempPixelPoint = m_projectMatrix * point.getVector3fMap();
        cv::Point pixelPoint((int)(tempPixelPoint[0]/tempPixelPoint[2]), (int)(tempPixelPoint[1]/tempPixelPoint[2]));
        // TODO: 
        // 特征点掩码判断    
        if (pixelPoint.x>639 || pixelPoint.y>479 || pixelPoint.x<0||pixelPoint.y<0|| img_erode.at<bool>(pixelPoint) == 0) 
        {
            continue;            
        }

        // 调试代码3
        cv::drawMarker(temp_img,cv::Point(pixelPoint), cv::Scalar(0,255,0),2, 5, 1);

        finalPointCloud.points.push_back(point);
    }

    cv::imshow("select keypoints", temp_img);

    // 用筛选后的点云拟合平面
    if (finalPointCloud.points.size()<mapPointNumThresh) {
        changeErrorType(Point_cloud_is_little);
        return false;
    }
    param = calParam(finalPointCloud);

    // 将拟合平面后的法向量点乘当前帧坐标系z轴向量
    Eigen::Vector3d zAxis(0,0,1);
    double cosValue = param.block(0,0,3,1).dot(zAxis);
    // 两向量余弦值判断
    if (abs(cosValue) < cosValueThresh_planeNormAndCameraZaxis) {
        changeErrorType(The_camera_is_not_straight_on_the_plane);
        RCLCPP_INFO(this->get_logger(), "cosValue: %f °", std::acos(cosValue)/CV_PI * 180);
        return false;    
    }
    return true;
}

void ModelNode::PubModel()
{
    std::cout << "PubModel()"<<std::endl;

    sensor_msgs::msg::PointCloud2 pclMsg_obstacle;
    pcl::toROSMsg(pcl_obstacle, pclMsg_obstacle);
    pclMsg_obstacle.header = m_header_initFrame;
    pcl_pub->publish(pclMsg_obstacle);
    flag_pubModel = true;
}

Eigen::Vector4d ModelNode::calParam(pcl::PointCloud<pcl::PointXYZ> pointCloud) {

    std::cout << "calParam()"<<std::endl;

    Eigen::Vector4d param;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud_Ptr=pointCloud.makeShared();
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 设置分割系数
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(distanceThresh_plandAndPoint); // attention: the parameter fill in here is not supporting variant;
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

// 析构函数 delete function()
ModelNode::~ModelNode() {}