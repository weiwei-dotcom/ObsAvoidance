#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <cmath>
#include "cdcr_joint.hpp"


class cdcr
{
private:
    unsigned int holes_number_;
    unsigned int joint_number_;
    unsigned int union_joint_number_;
    // the element represent the first cable attach point index of every joint.
    std::vector<int> joints_hole_index_;
    // length of each cable at each joint, the index of row and col represent the joint index and attach index.
    Eigen::MatrixXd joints_cable_length;
public:
    
};
