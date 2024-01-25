#include "joint.hpp"

Joint::Joint(){}
Joint::Joint(double length_rigid1,double length_rigid2,double length_continuum)
{
    this->alpha = 1e-4;
    this->theta = 1e-4;
    this->length_rigid1 = length_rigid1;
    this->length_rigid2 = length_rigid2;
    this->length_continuum = length_continuum;
    this->length = length_continuum+length_rigid1+length_rigid2;
    this->transform = Eigen::Matrix4d::Identity();
    this->transform(2,3) = this->length_rigid1+this->length_rigid2+this->length_continuum;
    return;
}

Eigen::Matrix4d Joint::getTransform()
{
    Eigen::Matrix4d tempTransform;
    tempTransform << pow(sin(this->alpha),2)+cos(this->theta)*pow(cos(this->alpha),2),
                     (cos(this->theta)-1)*cos(this->alpha)*sin(this->alpha),
                     sin(this->theta)*cos(this->alpha),
                     this->length_continuum*(1-cos(this->theta))*cos(this->alpha)/this->theta+this->length_rigid2*sin(this->theta)*cos(this->alpha),
                     (cos(this->theta)-1)*cos(this->alpha)*sin(this->alpha),
                     pow(cos(this->alpha),2)+pow(sin(this->alpha),2)*cos(this->theta),
                     sin(this->theta)*sin(this->alpha),
                     this->length_continuum*(1-cos(this->theta))*sin(this->alpha)/this->theta+this->length_rigid2*sin(this->theta)*sin(this->alpha),
                     -sin(this->theta)*cos(this->alpha),
                     -sin(this->theta)*sin(this->alpha),
                     cos(this->theta),
                     this->length_continuum/this->theta*sin(this->theta)+length_rigid1+length_rigid2*cos(this->theta),
                     0,0,0,1;

    this->transform = tempTransform;
    return this->transform;
}

