#include "slam.hpp"

using namespace std;
using namespace cv;

// 类初始化函数
slamNode::slamNode(ORB_SLAM3::System* slam):Node("slam")
{   
    // slamInitTime = std::chrono::steady_clock::now();
    // 打印节点初始化信息
    RCLCPP_INFO(this->get_logger(), "slamNode is initiating");

    visualOdom = slam;
    state = -1;
    // 实例化相机图像消息订阅者指针
    imgSub = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10, std::bind(&slamNode::grabImg_pubSlam, this, std::placeholders::_1));
    slamPub = this->create_publisher<interface::msg::Slam>("slam", 10);
    scale_fact = 1.0;
}

// 循环计算相机位姿，并发布话题
void slamNode::grabImg_pubSlam(const sensor_msgs::msg::Image::SharedPtr img)
{
    try
    {
        // img2cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        img2cvPtr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    // sensor_msgs::msg::Image testImg; // .................................
    // distortionImg(img2cvPtr, testImg,"slam系统之前"); // ............................
    Tci = visualOdom->TrackMonocular(img2cvPtr->image, img2cvPtr->header.stamp.sec);
    // 将slam系统跟踪到的相机位姿转化为interface消息中的cam_pose
    Eigen::Matrix3f R_init2cur = Tci.matrix().block(0, 0, 3, 3);
    Eigen::Quaternionf q_init2cur(R_init2cur);
    interface::msg::Slam slamMsg;  // 定义ros2消息接口变量
    // 将slam系统跟踪到的当前帧的地图点转化为interface消息中的point_cloud
    state = visualOdom->GetTrackingState();
    pointCloud = visualOdom->GetTrackedMapPoints();

    // 下面这段代码使用iterator迭代器将orbslam获取的mappoint添加至pcl点云中
    if (state == 2)
    {
        slamMsg.header = img->header;
        slamMsg.transform_init2cur.pose.orientation.set__x(q_init2cur.x());
        slamMsg.transform_init2cur.pose.orientation.set__y(q_init2cur.y());
        slamMsg.transform_init2cur.pose.orientation.set__z(q_init2cur.z());
        slamMsg.transform_init2cur.pose.orientation.set__w(q_init2cur.w());
        slamMsg.transform_init2cur.pose.position.set__x(Tci.matrix()(0, 3));
        slamMsg.transform_init2cur.pose.position.set__y(Tci.matrix()(1, 3));
        slamMsg.transform_init2cur.pose.position.set__z(Tci.matrix()(2, 3));
        slamMsg.transform_init2cur.header.stamp = img->header.stamp;
        slamMsg.transform_init2cur.header.frame_id = "cur_frame";
        slamMsg.point_cloud.header.stamp = img->header.stamp; 
        slamMsg.point_cloud.header.frame_id = "init_frame"; 
        slamMsg.point_cloud.width = pointCloud.size();
        slamMsg.point_cloud.height = 1;
        slamMsg.point_cloud.is_dense = false;
        slamMsg.point_cloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(slamMsg.point_cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        sensor_msgs::PointCloud2Iterator<float> iter_x(slamMsg.point_cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(slamMsg.point_cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(slamMsg.point_cloud, "z");
        int length_pointcloud=0;
        for (auto* mapPoint : pointCloud)
        {
            if (!mapPoint)
                continue;
            *iter_x = mapPoint->GetWorldPos().x();
            *iter_y = mapPoint->GetWorldPos().y();
            *iter_z = mapPoint->GetWorldPos().z();      
            ++iter_x;
            ++iter_y;
            ++iter_z;
            length_pointcloud++;
        };
        // std::cout << "length of pointcloud : " << length_pointcloud << std::endl;
        slamMsg.point_cloud.set__width(length_pointcloud);
        slamMsg.point_cloud.header.set__stamp(img->header.stamp);
        slamPub->publish(slamMsg);
        // cout << "publish the slam msg success" << endl;
    }
}

// void slamNode::distortionImg(cv_bridge::CvImagePtr imgPtr, sensor_msgs::msg::Image& img, std::string name)
// {
//     cv::FileStorage fr("/home/weiwei/Desktop/project/ObsAvoidance/src/slam/data/small.yaml", cv::FileStorage::READ);
//     float fx, fy, cx, cy, k1, k2, p1, p2, k3;
//     fr["Camera.fx"] >> fx;
//     fr["Camera.fy"] >> fy;
//     fr["Camera.cx"] >> cx;
//     fr["Camera.cx"] >> cy;
//     fr["Camera.k1"] >> k1;
//     fr["Camera.k2"] >> k2;
//     fr["Camera.p1"] >> p1;
//     fr["Camera.p2"] >> p2;
//     fr["Camera.k3"] >> k3;
//     cout << fx << fy << cx<< cy<< k1<< k2<< p1<< p2<< k3<<endl;
//     cv::Mat intrinsicMatrix = (cv::Mat_<float>(3, 3) << fx, 0, cx, 0, fy, cy, 0,0,1);
//     cv::Mat distCoeff = (cv::Mat_<float>(1, 5) << k1, k2, p1, p2, k3);
//     cv::Mat tempImg;
//     cv::undistort(imgPtr->image, tempImg, intrinsicMatrix, distCoeff);
//     cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", tempImg).toImageMsg(img);
//     // cv::imshow("矫正前", imgPtr->image); //.....................................
//     cv::imshow(name, tempImg); //................................
// }

slamNode::~slamNode()
{
    cout << "关闭slam系统" << endl;
    visualOdom->Shutdown(); 
    delete visualOdom;
}