#include "cdcr_joint.hpp"


cdcr_joint::cdcr_joint(double curve_length = 30,
                       double base_rigid_length_ = 15,
                       double move_rigid_length = 15,
                       double base_disk_radius = 45,
                       double move_disk_radius = 45,
                       double base_disk_thick = 4,
                       double move_disk_thick = 4,
                       int holes_number = 24)
{
    //TODO:
    
}

double cdcr_joint::calCableLengthAtIndex(int cable_attach_index)
{
    //TODO:

}

// Eigen::VectorXd calCablesLengthFromJointVariables();

Eigen::VectorXd cdcr_joint::calCablesLength()
{
    //TODO:

}

Eigen::Matrix4d cdcr_joint::calTransform()
{
    //TODO:

}

void cdcr_joint::updateJoint(double alpha, double theta)
{
    //TODO:

}

// Method of obtaining the joint param variables from external.
Eigen::VectorXd cdcr_joint::getCablesLength()
{
    //TODO:

}

Eigen::Matrix4d cdcr_joint::getTransformMoveToBase()
{
    //TODO:
}
Eigen::Matrix4d cdcr_joint::getTransformBaseToWorld()
{
    //TODO:
}
double cdcr_joint::getDiskRadius()
{
    //TODO:

}

double cdcr_joint::getContinuumLength()
{
    //TODO:

}

double cdcr_joint::getCurveLength()
{
    //TODO:

}

double cdcr_joint::getBaseRigidLength()
{
    //TODO:

}
double cdcr_joint::getMoveRigidLength()
{
    //TODO:

}
double cdcr_joint::getTheta()
{
    //TODO:

}

double cdcr_joint::getAlpha()
{
    //TODO:

}

double cdcr_joint::getDiskthick()
{
    //TODO:

}