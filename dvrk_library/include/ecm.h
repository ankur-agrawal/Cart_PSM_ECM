#include <iostream>
#include <Kinematics.h>

class ecm: public Kinematics
{
public:
  ecm(std::string name)
  {
    ecm_name=name;
    joint_values=Eigen::MatrixXd::Zero(4,1);
  }
  std::string ecm_name;
  Eigen::MatrixXd inverse_kinematics(Eigen::Matrix4d base_to_tip);
  void set_dh_params();
};

//This is Inverse Kinematics where the input matrix is given in the base frame of the ecm and not the RCM frame
Eigen::MatrixXd ecm::inverse_kinematics(Eigen::Matrix4d base_to_end)
{
  Eigen::MatrixXd joint_angles(4,1);

  joint_angles(1,0) = atan2(base_to_end(0,2), sqrt(pow(base_to_end(1,2),2)+pow(base_to_end(2,2),2)));
  joint_angles(0,0) = atan2(base_to_end(1,2)/cos(joint_angles(1,0)), base_to_end(2,2)/cos(joint_angles(1,0)));
  joint_angles(3,0) = atan2(-base_to_end(0,0)/cos(joint_angles(1,0)), base_to_end(0,1)/cos(joint_angles(1,0)));
  joint_angles(2,0) = sqrt(pow(base_to_end(0,3)-0.612599,2)+ pow(base_to_end(1,3),2)+pow(base_to_end(2,3)-0.101595,2))-0.0007;
  return joint_angles;
}

void ecm::set_dh_params()
{
  dh_params=Eigen::MatrixXd::Identity(4,4);
  dh_params << PI/2, 0, PI/2-joint_values(0,0), 0,
        -PI/2, 0, -PI/2+joint_values(1,0), 0,
        PI/2, 0, 0, -0.3822+joint_values(2,0),
        0, 0, 0+joint_values(3,0), 0.3829 ;
}
