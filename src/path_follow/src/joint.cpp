#include "joint.hpp"

Joint::Joint(){}
Joint::Joint(const double &length_rigid1,const double &length_rigid2,const double &length_continuum, const int &cable_id)
{
    this->cable_id_ = cable_id;
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

Eigen::Vector3d Joint::calCableLength(const int &cable_id)
{
    Eigen::Vector3d cable_length;
    for (int i=0;i<3;i++)
    {
        Eigen::Vector4d point_plat_1 = Eigen::Vector4d::Ones();
        Eigen::Vector4d point_plat_2 = Eigen::Vector4d::Ones();
        
        point_plat_1[0] = cos(((double)cable_id+(double)i*8.0)/24.0*M_PI*2);
        point_plat_1[1] = sin(((double)cable_id+(double)i*8.0)/24.0*M_PI*2);
        point_plat_1[2] = 0.0;

        point_plat_2 = transform * point_plat_1;
        Eigen::Vector3d temp_vec = (point_plat_2-point_plat_1).block(0,0,3,1);
        cable_length[i] = temp_vec.norm();
    }
    return cable_length;
}

