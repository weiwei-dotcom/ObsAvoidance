#include "planner.hpp"

path_planner::path_planner():Node("path_planner")
{
    Eigen::Matrix3d Eigen_test;
    Eigen_test << 1,1,1,1,1,1,1,1,1;
    std::cout << Eigen_test << std::endl;
    // cv::Mat temp_mat = cv::imread("/home/weiwei/Desktop/project/ObsAvoidance/data/2024-01-12_13-48.png");
    // cv::imshow("temp_mat", temp_mat);
    // cv::waitKey(0);
    Sophus::SO3d temp_sophus(Eigen::Matrix3d::Identity());
    std::cout << temp_sophus.matrix() << std::endl;
    // ceres::Problem temp_problem;
    return;
}