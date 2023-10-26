#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/OcTree.h"
#include "interface/srv/transform.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <Eigen/Eigen>
#include "yaml-cpp/yaml.h"
#include <pcl/point_types.h>
#include "pcl/common/transforms.h"
#include <pcl/filters/extract_indices.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <ceres/ceres.h>
#include <fstream>
#include <algorithm>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"


class cdcr_controller:public rclcpp::Node
{
private:
    
public:
    
};