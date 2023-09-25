#include "media.hpp"

mediaNode::mediaNode():Node("media")
{
    RCLCPP_INFO(this->get_logger(), "initiating the mediaNode");
    mapPointCallbackGroup = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    slamSub = this->create_subscription<interface::msg::Slam>("slam", 3, std::bind(&mediaNode::slam_callback, this, std::placeholders::_1));
    mapPointSer = this->create_service<interface::srv::MapPoint>("mappoint", std::bind(&mediaNode::mapPoint_callback, this, std::placeholders::_1, std::placeholders::_2),
                                                                 rmw_qos_profile_services_default, mapPointCallbackGroup);
    camPoseSer = this->create_service<interface::srv::CamPose>("campose", std::bind(&mediaNode::camPose_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void mediaNode::mapPoint_callback(const interface::srv::MapPoint::Request::SharedPtr request,
                            const interface::srv::MapPoint::Response::SharedPtr response)
{
    rclcpp::Rate rate(std::chrono::nanoseconds(500000000));
    while (camPose_pointCloud_img == NULL)
    {
        RCLCPP_INFO(this->get_logger(), "waiting for the slam-system online");
        rate.sleep();
    }
    std::unique_lock<std::mutex> lock(mtx);
    response->set__img(camPose_pointCloud_img->img); 
    response->set__cam_pose(camPose_pointCloud_img->cam_pose);
    response->set__world2cam(camPose_pointCloud_img->world2cam);
    response->set__point_cloud(camPose_pointCloud_img->point_cloud);
}

void mediaNode::camPose_callback(const interface::srv::CamPose::Request::SharedPtr request,
                                const interface::srv::CamPose::Response::SharedPtr response)
{
    response->set__cam_pose(camPose_pointCloud_img->cam_pose);
}

void mediaNode::slam_callback(const interface::msg::Slam::SharedPtr slamMsg)
{
    std::unique_lock<std::mutex> lock(mtx);
    camPose_pointCloud_img = slamMsg;
    cv_bridge::CvImagePtr imgPtr = cv_bridge::toCvCopy(camPose_pointCloud_img->img, sensor_msgs::image_encodings::BGR8);
}

// 返回给消息点云和关键点图像坐标信息的消息
mediaNode::~mediaNode() {}