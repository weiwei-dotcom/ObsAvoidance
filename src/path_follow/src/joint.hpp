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
double last_alpha,alpha,last_theta,theta;
Eigen::Matrix4d getTransform();
private:
};
