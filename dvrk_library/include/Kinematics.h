#include <Eigen/Dense>
#include <ros/ros.h>
#define PI 3.14159265359

class Kinematics{
protected:
  Eigen::MatrixXd dh_params;
  Eigen::Matrix4d world_to_base=Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d base_to_rcm=Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d rcm_to_tip=Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d tip_to_end_frame=Eigen::Matrix4d::Identity(4,4);
  Eigen::MatrixXd joint_values;
public:
  virtual Eigen::MatrixXd inverse_kinematics(Eigen::Matrix4d base_to_end)=0;
  virtual void set_dh_params()=0;
  void read_dh_params();
  Eigen::Matrix4d compute_mod_dh_matrix(double alpha, double a, double theta, double d);
  Eigen::Matrix4d ForwardKinematics();
  void set_joints(Eigen::MatrixXd joints);
  void set_world_to_base(Eigen::Matrix4d mat);
  void set_base_to_rcm(Eigen::Matrix4d mat);
  void set_tip_to_end(Eigen::Matrix4d mat);
};

void Kinematics::set_world_to_base(Eigen::Matrix4d mat)
{
  world_to_base=mat;
}

void Kinematics::set_base_to_rcm(Eigen::Matrix4d mat)
{
  base_to_rcm=mat;
}

void Kinematics::set_tip_to_end(Eigen::Matrix4d mat)
{
  tip_to_end_frame=mat;
}

void Kinematics::set_joints(Eigen::MatrixXd joints)
{
  joint_values=joints;
}

void Kinematics::read_dh_params()
{
  std::cout << rcm_to_tip << '\n';
}

Eigen::Matrix4d Kinematics::compute_mod_dh_matrix(double alpha, double a, double theta, double d)
{
  Eigen::Matrix4d DH_matrix;

  DH_matrix << cos(theta), -sin(theta), 0, a,
               sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -d*sin(alpha),
               sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), d*cos(alpha),
               0, 0, 0, 1;
  return DH_matrix;
}

Eigen::Matrix4d Kinematics::ForwardKinematics()
{
  set_dh_params();

  if (dh_params.cols()!=4)
  {
    std::cout << "DH Parameters specified are wrong. Check again." << '\n';
    return Eigen::Matrix4d::Zero(4,4);
  }
  for (int i=0; i< dh_params.rows();i++)
  {
    rcm_to_tip=rcm_to_tip*compute_mod_dh_matrix(dh_params(i,0),dh_params(i,1),dh_params(i,2),dh_params(i,3));
  }

  return world_to_base*base_to_rcm*rcm_to_tip*tip_to_end_frame;
}
