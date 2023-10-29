#include <Eigen/Eigen>
#include <sophus/se3.hpp>


class cdcr_joint
{
public:
    cdcr_joint(double curve_length,
               double rigid1_length,
               double rigid2_length,
               double base_disk_radius,
               double move_disk_radius,
               double base_disk_thick,
               double move_disk_thick,
               int holes_number);
    double calCableLengthAtIndex(int cable_attach_index);
    // Eigen::VectorXd calCablesLengthFromJointVariables();
    Eigen::VectorXd calCablesLength();
    Eigen::Matrix4d calTransform();
    void updateJoint(double alpha, double theta);

    // Method of obtaining the joint param variables from external.
    Eigen::VectorXd getCablesLength();
    Eigen::Matrix4d getTransform();
    double getDiskRadius();
    double getContinuumLength();
    double getCurveLength();
    double getRigid1Length();
    double getRigid2Length();
    double getTheta();
    double getAlpha();
    double getDiskthick();

private:
    double base_disk_radius_,move_disk_radius_,base_disk_thick_,move_disk_thick;
    double continuum_length_;
    double curve_length_, rigid1_length_, rigid2_length_;
    double alpha_,theta_;
    Eigen::VectorXd cables_length_;
    Eigen::Matrix4d transform_;
    
};