#include "cdcr_joint.hpp"


cdcr_joint::cdcr_joint(double curve_length = 30,
            double rigid1_length = 15,
            double rigid2_length = 15,
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

Eigen::Matrix4d cdcr_joint::getTransform()
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

double cdcr_joint::getRigid1Length()
{
    //TODO:

}

double cdcr_joint::getRigid2Length()
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