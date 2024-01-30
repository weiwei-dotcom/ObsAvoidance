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

        this->cable_lengths.push_back(Eigen::VectorXd(cables_number));
        Eigen::Vector2d temp_joint_value(1e-4,1e-4);
        Eigen::Matrix4d temp_transform = this->getJointTransform(temp_joint_value,this->continuum_lengths[i],
                                                                this->rigid1_lengths[i],this->rigid2_lengths[i]);
        // 对从基座穿绳到当前关节动平台上对应id的绳长进行计算
        for (int j = 0; j<24; j++)
        {
            Eigen::Vector4d static_position(
                cos((double)j/24.0*M_PI) * this->plat_radius,
                sin((double)j/24.0*M_PI) * this->plat_radius,
                0.0,1.0
            );
            Eigen::Vector4d dynamic_position = temp_transform * static_position;
            if (i==0)
                cable_lengths[i][j] = (dynamic_position-static_position).norm();
            else
                cable_lengths[i][j] = (dynamic_position-static_position).norm()+cable_lengths[i-1][j];
        }
    }
    
    return;
}

Eigen::Vector3d MotionControl::calJointCableLength(const int& src_joint_id,const Eigen::Vector3d& src_cable_ids, const std::vector<Eigen::Matrix4d>& src_transforms)
{
    Eigen::Vector3d dst_cable_lengths;
    Eigen::Vector4d 
    for (int i = 0; i<src_joint_id; i++)
    {
        src_transforms * 
    }
    return;
}

void MotionControl::calRobotCableLength(std::vector<Eigen::Vector3d> &dst_cable_lengths,
                                        const int &src_joints_number,const int& src_cables_number,
                                        const std::vector<Eigen::Vector2d> &src_joint_values,
                                        const std::vector<double> &src_continuum_lengths,
                                        const std::vector<double> &src_rigid1_lengths,
                                        const std::vector<double> &src_rigid2_lengths)

void MotionControl::calCableLength( std::vector<Eigen::VectorXd> &temp_cable_lengths,
                                    const int &temp_joints_number,const int& temp_cables_number,
                                    const std::vector<Eigen::Vector2d> &temp_joint_values,
                                    const std::vector<double> &temp_continuum_lengths,
                                    const std::vector<double> &temp_rigid1_lengths,
                                    const std::vector<double> &temp_rigid2_lengths)
{
    temp_cable_lengths.clear();
    temp_cable_lengths.resize(temp_joints_number,Eigen::VectorXd(temp_cables_number));
    for (int i=0; i<temp_joints_number; i++)
    {
        Eigen::Matrix4d temp_transform = this->getJointTransform(temp_joint_values[i],temp_continuum_lengths[i],
                                                                temp_rigid1_lengths[i],temp_rigid2_lengths[i]);
        for (int j=0; j<temp_cables_number; j++)
        {
            Eigen::Vector4d static_position(
                cos((double)j/24.0*M_PI) * this->plat_radius,
                sin((double)j/24.0*M_PI) * this->plat_radius,
                0.0,1.0
            );
            Eigen::Vector4d dynamic_position = temp_transform * static_position;
            if (i==0)
                temp_cable_lengths[i][j] = (dynamic_position-static_position).norm();
            else
                temp_cable_lengths[i][j] = (dynamic_position-static_position).norm()+temp_cable_lengths[i-1][j];
        }
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
