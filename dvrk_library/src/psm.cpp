#include <iostream>
#include <Kinematics.h>
#include <cisstRobot/robManipulator.h>
#include <ros/package.h>
#include <cisstVector.h>


class psm: public Kinematics
{
public:
  psm(std::string name)
  {
    psm_name=name;
    joint_values=Eigen::MatrixXd::Zero(6,1);
  }
  std::string psm_name;
  Eigen::MatrixXd inverse_kinematics(Eigen::Matrix4d base_to_tip);
  void set_dh_params();
};

Eigen::MatrixXd psm::inverse_kinematics(Eigen::Matrix4d base_to_end)
{
  Eigen::MatrixXd joint_angles(6,1);

  joint_angles(1,0) = atan2(base_to_end(0,2), sqrt(pow(base_to_end(1,2),2)+pow(base_to_end(2,2),2)));
  joint_angles(0,0) = atan2(base_to_end(1,2)/cos(joint_angles(1,0)), base_to_end(2,2)/cos(joint_angles(1,0)));
  joint_angles(3,0) = atan2(-base_to_end(0,0)/cos(joint_angles(1,0)), base_to_end(0,1)/cos(joint_angles(1,0)));
  joint_angles(2,0) = sqrt(pow(base_to_end(0,3),2)+ pow(base_to_end(1,3),2)+pow(base_to_end(2,3),2))+0.0014918;
  return joint_angles;
}

void psm::set_dh_params()
{
  dh_params=Eigen::MatrixXd::Identity(7,4);
  dh_params << PI/2, 0, PI/2+joint_values(0,0), 0,
        -PI/2, 0, -PI/2+joint_values(1,0), 0,
        PI/2, 0, 0, -0.386292+joint_values(2,0),
        0, 0, 0+joint_values(3,0), 0.3668002,
        -PI/2, 0, -PI/2+joint_values(4,0), 0,
        -PI/2, 0.018, -PI/2+joint_values(5,0), 0;
        // -PI/2, 0, -PI/2, 0;

  // dh_params << 1.5708, 0, 1.5708+joint_values(0,0), 0,
  //       -1.5708, 0, -1.5708+joint_values(1,0), 0,
  //       1.5708, 0, 0, -0.3822+joint_values(2,0),
  //       0, 0, 0+joint_values(3,0), 0.3829 ;
}

int main(int argc, char **argv)
{
  psm psm("psm");
  Eigen::MatrixXd joints(6,1);
  Eigen::MatrixXd joints_calculated(6,1);
  joints << 0,0,0,0,0,0;
  Eigen::Matrix4d psm_base_to_rcm;
  // psm_base_to_rcm << 0,1,0,0,
  //                   -1,0,0,0,
  //                    0,0,1,0,
  //                    0,0,0,1;
  // psm.set_base_to_rcm(psm_base_to_rcm);
  //
  // Eigen::Matrix4d psm_world_to_base;
  // psm_world_to_base << 1,0,0,0,
  //                      0,1,0,0,
  //                      0,0,1,0,
  //                      0,0,0,1;
  // psm.set_world_to_base(psm_world_to_base);
  //
  // Eigen::Matrix4d psm_tip_to_end;
  // psm_tip_to_end << 0,-1,0,0,
  //                   -1,0,0,0,
  //                   0,0,-1,0,
  //                   0,0,0,1;
  // psm.set_tip_to_end(psm_tip_to_end);
  //
  psm.set_joints(joints);
  Eigen::Matrix4d m=psm.ForwardKinematics();
  // joints_calculated=psm.inverse_kinematics(m);
  std::cout << m << '\n';

  std::string filename = ros::package::getPath("py_cpp");
  // std::string filename = "/home/ankur/Catkin_ws/src/Cart_PSM_ECM/py_cpp/config/dvpsm.rob";
  filename.append("/config/dvpsm.rob");
  robManipulator psm_manip;
  robManipulator::Errno result;
  result = psm_manip.LoadRobot(filename);
  // if (result == robManipulator::EFAILURE)
  // {
  //   ROS_ERROR("failed to load manipulator config file: %s", filename.c_str());
  // }
  // else
  // {
  //   ROS_INFO("loaded psm manipulator");
  // }
  std::cout << '\n';
  vctDoubleVec psm_joint_current;
  vctFrm4x4 psm_pose_current;
  psm_joint_current.SetSize(6);
  psm_joint_current[0]=0;
  psm_joint_current[1]=0;
  psm_joint_current[2]=0;
  psm_joint_current[3]=0;
  psm_joint_current[4]=0;
  psm_joint_current[5]=0;

  psm_pose_current= psm_manip.ForwardKinematics(psm_joint_current);
  psm_manip.InverseKinematics(psm_joint_current, psm_pose_current);
  std::cout << psm_pose_current << '\n';
  std::cout << psm_joint_current << '\n';
  return 0;
}
