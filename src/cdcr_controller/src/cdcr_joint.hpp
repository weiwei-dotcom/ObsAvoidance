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
    // calculate all of the disk's cable length from array formulation and return the a set of cable length value contain in the vector
    Eigen::VectorXd calCablesLength();
    // calculate the transformation from the param of joint
    Eigen::Matrix4d calTransform();
    // calculate and update the param of joint class(alpha, theta, transform, cable length)
    void updateJoint(double alpha, double theta);

    // Method of obtaining the joint param variables from external.
    Eigen::VectorXd getCablesLength();
    Eigen::Matrix4d getTransformMoveToBase();
    Eigen::Matrix4d getTransformBaseToWorld();
    double getDiskRadius();
    double getContinuumLength();
    double getCurveLength();
    double getBaseRigidLength();
    double getMoveRigidLength();
    double getTheta();
    double getAlpha();
    double getDiskthick();

private:
    double base_disk_radius_,move_disk_radius_,base_disk_thick_,move_disk_thick_;
    double continuum_length_;
    double curve_length_, base_rigid_length_, move_rigid_length_;
    double alpha_,theta_;
    Eigen::VectorXd cables_length_;
    Eigen::Matrix4d transform_move_to_base_,transform_base_to_world_;
    
};