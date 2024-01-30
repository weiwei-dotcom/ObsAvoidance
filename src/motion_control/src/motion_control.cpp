#include "motion_control.hpp"


MotionControl::MotionControl():Node("motion_control")
{
    this->declare_parameter<int16_t>("joints_number", 6);
    this->declare_parameter<int16_t>("cables_number", 24);
    this->declare_parameter<int16_t>("segs_number", 2);
    this->declare_parameter<double_t>("plat_radius", 45.0);

    this->joints_number = this->get_parameter("joints_nubmer").as_int();
    this->cables_number = this->get_parameter("cables_number").as_int();
    this->segs_number = this->get_parameter("segs_number").as_int();
    this->plat_radius = this->get_parameter("plat_radius").as_double();

    std::vector<Eigen::Vector2d> temp_joint_values;
    for (int i=0;i < this->joints_number;i++)
    {
        this->declare_parameter<std::double_t>(std::string("joint") + std::to_string(i)+std::string("_rigid1_length"), 0.0);
        this->declare_parameter<std::double_t>(std::string("joint") + std::to_string(i)+std::string("_rigid2_length"), 0.0);
        this->declare_parameter<std::double_t>(std::string("joint") + std::to_string(i)+std::string("_continuum_length"), 100.0);
        this->declare_parameter<std::int16_t>(std::string("joint") + std::to_string(i)+std::string("_cable_id"), i);
        this->rigid1_lengths[i] = this->get_parameter(std::string("joint") + std::to_string(i)+std::string("_rigid1_length")).as_double();
        this->rigid2_lengths[i] = this->get_parameter(std::string("joint") + std::to_string(i)+std::string("_rigid2_length")).as_double();
        this->continuum_lengths[i] = this->get_parameter(std::string("joint") + std::to_string(i)+std::string("_continuum_length")).as_double();
        this->cable_ids[i][0] = this->get_parameter(std::string("joint") + std::to_string(i)+std::string("_cable_id")).as_int();
        this->cable_ids[i][1] = this->cable_ids[i][0] + 8;
        this->cable_ids[i][2] = this->cable_ids[i][1] + 8;

        temp_joint_values.push_back(Eigen::Vector2d(1e-4,1e-4));
    }
    calRobotCableLength(this->cable_lengths, this->cable_ids, temp_joint_values, this->continuum_lengths
                        ,this->rigid1_lengths,this->rigid2_lengths);
    
    return;
}

Eigen::Vector3d MotionControl::calJointCableLength(const int& src_joint_id,const Eigen::Vector3i& src_cable_ids, const std::vector<Eigen::Matrix4d>& src_transforms)
{
    Eigen::Vector3d dst_cable_lengths = Eigen::Vector3d::Zero();
    Eigen::Vector4d position_0(
        cos((double)src_cable_ids[0]/24.0*M_PI) * this->plat_radius,
        sin((double)src_cable_ids[0]/24.0*M_PI) * this->plat_radius,
        0.0,1.0
    );
    Eigen::Vector4d position_1(
        cos((double)src_cable_ids[1]/24.0*M_PI) * this->plat_radius,
        sin((double)src_cable_ids[1]/24.0*M_PI) * this->plat_radius,
        0.0,1.0
    );
    Eigen::Vector4d position_2(
        cos((double)src_cable_ids[2]/24.0*M_PI) * this->plat_radius,
        sin((double)src_cable_ids[2]/24.0*M_PI) * this->plat_radius,
        0.0,1.0
    );
    for (int i = 0; i<=src_joint_id; i++)
    {
        dst_cable_lengths[0] += (src_transforms[i] * position_0 - position_0).norm();
        dst_cable_lengths[1] += (src_transforms[i] * position_1 - position_1).norm();
        dst_cable_lengths[2] += (src_transforms[i] * position_2 - position_2).norm();
    }
    return dst_cable_lengths;
}

void MotionControl::calRobotCableLength(std::vector<Eigen::Vector3d> &dst_cable_lengths,
                                        const std::vector<Eigen::Vector3i> &src_cable_ids,
                                        const std::vector<Eigen::Vector2d> &src_joint_values,
                                        const std::vector<double> &src_continuum_lengths,
                                        const std::vector<double> &src_rigid1_lengths,
                                        const std::vector<double> &src_rigid2_lengths)
{
    dst_cable_lengths.clear();
    int src_joints_number = src_cable_ids.size();
    std::vector<Eigen::Matrix4d> temp_transforms;
    for (int i=0;i<src_joints_number;i++)
    {
        temp_transforms.push_back(this->getJointTransform(src_joint_values[i],
                                                        src_continuum_lengths[i], 
                                                        src_rigid1_lengths[i],
                                                        src_rigid2_lengths[i]));
        dst_cable_lengths.push_back(calJointCableLength(i, this->cable_ids[i], temp_transforms));
    }
    return;
}

Eigen::Matrix4d MotionControl::getJointTransform(const Eigen::Vector2d &joint_value, const double &continuum_length, const double &rigid1_length, const double &rigid2_length)
{
    Eigen::Matrix4d tempTransform;
    tempTransform << pow(sin(joint_value.x()),2)+cos(joint_value.y())*pow(cos(joint_value.x()),2),
                     (cos(joint_value.y())-1)*cos(joint_value.x())*sin(joint_value.x()),
                     sin(joint_value.y())*cos(joint_value.x()),
                     continuum_length*(1-cos(joint_value.y()))*cos(joint_value.x())/joint_value.y()+rigid2_length*sin(joint_value.y())*cos(joint_value.x()),
                     (cos(joint_value.y())-1)*cos(joint_value.x())*sin(joint_value.x()),
                     pow(cos(joint_value.x()),2)+pow(sin(joint_value.x()),2)*cos(joint_value.y()),
                     sin(joint_value.y())*sin(joint_value.x()),
                     continuum_length*(1-cos(joint_value.y()))*sin(joint_value.x())/joint_value.y()+rigid2_length*sin(joint_value.y())*sin(joint_value.x()),
                     -sin(joint_value.y())*cos(joint_value.x()),
                     -sin(joint_value.y())*sin(joint_value.x()),
                     cos(joint_value.y()),
                     continuum_length/joint_value.y()*sin(joint_value.y())+rigid1_length+rigid2_length*cos(joint_value.y()),
                     0,0,0,1;
    return tempTransform;
}
