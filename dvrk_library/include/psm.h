#include <iostream>
#include <Kinematics.h>
#include <cisstRobot/robManipulator.h>
#include <ros/package.h>
#include <cisstVector.h>
#include <std_msgs/Float64.h>


class psm: public Kinematics
{
public:
  psm(std::string name, ros::NodeHandle& n)
  {
    psm_name=name;
    joint_values=Eigen::MatrixXd::Zero(6,1);
    std::string filename = ros::package::getPath("dvrk_library");
    filename.append("/config/dvpsm.rob");
    result = psm_manip.LoadRobot(filename);
    psmPub.resize(6);
    psmPub[0]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/yaw_joint/SetPositionTarget",1000);
    psmPub[1]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/pitch_back_joint/SetPositionTarget",1000);
    psmPub[2]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/main_insertion_joint/SetPositionTarget",1000);
    psmPub[3]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/large_needle_driver/tool_roll_joint/SetPositionTarget",1000);
    psmPub[4]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/large_needle_driver/tool_pitch_joint/SetPositionTarget",1000);
    psmPub[5]=n.advertise<std_msgs::Float64>("/dvrk/"+psm_name+"/large_needle_driver/tool_yaw_joint/SetPositionTarget",1000);
    set_joint_limits();

  }
  bool robotLoaded();
  Eigen::MatrixXd inverse_kinematics(Eigen::Matrix4d base_to_tip);
  void set_dh_params();
  Eigen::MatrixXd joints_cur=Eigen::MatrixXd::Zero(6,1);
  Eigen::MatrixXd joints_cmd=Eigen::MatrixXd::Zero(6,1);
  void PublishJoints();
  void set_joint_limits();
  void clip_joints();
  void set_init_joints(double j1,double j2,double j3,double j4,double j5,double j6);
private:
  Eigen::MatrixXd joint_limits;
  std::string psm_name;
  robManipulator psm_manip;
  robManipulator::Errno result;
  std::vector<ros::Publisher> psmPub;
};

Eigen::MatrixXd psm::inverse_kinematics(Eigen::Matrix4d base_to_end)
{
  Eigen::MatrixXd joint_angles(6,1);
  vctDoubleVec psm_joint_calculated;
  vctFrm4x4 psm_pose_current;
  for (int i=0;i<4;i++)
  {
    for (int j=0;j<4;j++)
    {
      psm_pose_current[i][j]=base_to_end(i,j);
    }
  }
  psm_joint_calculated.SetSize(6);
  psm_joint_calculated[0]=0;
  psm_joint_calculated[1]=0;
  psm_joint_calculated[2]=0;
  psm_joint_calculated[3]=0;
  psm_joint_calculated[4]=0;
  psm_joint_calculated[5]=0;

  Eigen::Matrix4d end_to_base=base_to_end.inverse();
  psm_manip.InverseKinematics(psm_joint_calculated, psm_pose_current);
  joint_angles << psm_joint_calculated[0],
                       psm_joint_calculated[1],
                       psm_joint_calculated[2],
                       psm_joint_calculated[3],
                       psm_joint_calculated[4],
                       psm_joint_calculated[5];
  // Eigen::Matrix4d end_to_base= base_to_end.inverse();
  // joint_angles(5,0) = atan2(-end_to_base(0,3), -end_to_base(1,3));
  // joint_angles(4,0) = atan2(end_to_base(2,3), -0.0091-sin(joint_angles(5,0))*end_to_base(0,3)-cos(joint_angles(5,0))*end_to_base(1,3));
  // joint_angles(2,0) = sin(joint_angles(4,0))*end_to_base(2,3)+cos(joint_angles(4,0))*(0.0091+sin(joint_angles(5,0))*end_to_base(0,3)+cos(joint_angles(5,0))*end_to_base(1,3))+0.4318-0.4162;
  return joint_angles;
}

void psm::set_dh_params()
{
  dh_params=Eigen::MatrixXd::Identity(6,4);
  dh_params << PI/2, 0, PI/2+joint_values(0,0), 0,
        -PI/2, 0, -PI/2+joint_values(1,0), 0,
        PI/2, 0, 0, -0.4318+joint_values(2,0),  //-0.386292
        0, 0, 0+joint_values(3,0),0.4162 ,  //0.3668002
        -PI/2, 0, -PI/2+joint_values(4,0), 0,
        -PI/2, 0.0091, -PI/2+joint_values(5,0), 0;  //0.018
        // -PI/2, 0, -PI/2,0.0102;
}

bool psm::robotLoaded()
{
  return result;
}

void psm::PublishJoints()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(6);
  msg[0].data=joints_cmd(0,0);
  msg[1].data=joints_cmd(1,0);
  msg[2].data=joints_cmd(2,0);
  msg[3].data=joints_cmd(3,0);
  msg[4].data=joints_cmd(4,0);
  msg[5].data=joints_cmd(5,0);

  psmPub[0].publish(msg[0]);
  psmPub[1].publish(msg[1]);
  psmPub[2].publish(msg[2]);
  psmPub[3].publish(msg[3]);
  psmPub[4].publish(msg[4]);
  psmPub[5].publish(msg[5]);
}

void psm::set_joint_limits()
{
  joint_limits=Eigen::MatrixXd::Zero(6,2);
  joint_limits << -1.6055, 1.5994,
                 -0.93556, 0.94249,
                   0 , 0.24,
                   -3.0456,3.0456,
                   -3.0414,3.0528,
                   -3.0481,3.0376;
}

void psm::clip_joints()
{
  for (int i=0; i<6; i++)
  {
    if (joints_cmd(i,0)<joint_limits(i,0))
      joints_cmd(i,0)=joint_limits(i,0);
    if (joints_cmd(i,0)>joint_limits(i,1))
      joints_cmd(i,0)=joint_limits(i,1);
  }
}

void psm::set_init_joints(double j1,double j2,double j3,double j4,double j5,double j6)
{
  joints_cmd << j1,
                j2,
                j3,
                j4,
                j5,
                j6;
  this->set_joints(joints_cmd);
  this->PublishJoints();
}
