#include <iostream>
#include <Eigen/Eigen>


class Joint
{
public:
Joint();
Joint(double length_rigid1,double length_rigid2,double length_continuum);
Eigen::MatrixXd joint_points,joint_points_tangent_vectors;
double length,length_rigid1,length_rigid2,length_continuum;
Eigen::Matrix4d transform;
Eigen::Matrix4d trans_plat1_ref,trans_plat2_ref;
Eigen::Matrix4d trans_ref_plat1,trans_ref_plat2;
double last_alpha,alpha,last_theta,theta;
Eigen::Matrix4d getTransform();
private:
};
