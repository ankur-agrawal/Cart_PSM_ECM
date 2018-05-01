#include <iostream>
#include <Kinematics.h>
#include <std_msgs/Float64.h>
#include <ros/package.h>

class ecm: public Kinematics
{
public:
  ecm(std::string name, ros::NodeHandle n)
  {
    ecm_name=name;
    joint_values=Eigen::MatrixXd::Zero(4,1);
    ecmPub.resize(4);
    ecmPub[0]=n.advertise<std_msgs::Float64>("/dvrk/"+ecm_name+"/yaw_joint/SetPosition",1000);
    ecmPub[1]=n.advertise<std_msgs::Float64>("/dvrk/"+ecm_name+"/pitch_front_joint/SetPositionTarget",1000);
    ecmPub[2]=n.advertise<std_msgs::Float64>("/dvrk/"+ecm_name+"/main_insertion_joint/SetPosition",1000);
    ecmPub[3]=n.advertise<std_msgs::Float64>("/dvrk/"+ecm_name+"/tool_joint/SetPositionTarget",1000);
  }
  Eigen::MatrixXd inverse_kinematics(Eigen::Matrix4d base_to_tip);
  void set_dh_params();
  void PublishJoints();
  Eigen::MatrixXd joints_cur=Eigen::MatrixXd::Zero(4,1);
  Eigen::MatrixXd joints_cmd=Eigen::MatrixXd::Zero(4,1);
private:
  Eigen::MatrixXd joint_limits;
  std::string ecm_name;
  std::vector<ros::Publisher> ecmPub;
};

//This is Inverse Kinematics where the input matrix is given in the base frame of the ecm and not the RCM frame
Eigen::MatrixXd ecm::inverse_kinematics(Eigen::Matrix4d base_to_end)
{
  Eigen::MatrixXd joint_angles(4,1);

  joint_angles(1,0) = atan2(-base_to_end(0,0), sqrt(pow(base_to_end(1,0),2)+pow(base_to_end(2,0),2)));
  joint_angles(0,0) = atan2(-base_to_end(1,0)/cos(joint_angles(1,0)), -base_to_end(2,0)/cos(joint_angles(1,0)));
  joint_angles(3,0) = atan2(base_to_end(0,1)/cos(joint_angles(1,0)), base_to_end(0,2)/cos(joint_angles(1,0)));
  joint_angles(2,0) = sqrt(pow(base_to_end(0,3)-0.612599,2)+ pow(base_to_end(1,3),2)+pow(base_to_end(2,3)-0.101595,2))-0.0007;
  return joint_angles;
}

void ecm::set_dh_params()
{
  dh_params=Eigen::MatrixXd::Identity(4,4);
  dh_params << PI/2, 0, PI/2+joint_values(0,0), 0,
        -PI/2, 0, -PI/2+joint_values(1,0), 0,
        PI/2, 0, 0, -0.3822+joint_values(2,0),
        0, 0, 0+joint_values(3,0), 0.3829 ;
}

void ecm::PublishJoints()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(4);
  msg[0].data=joints_cmd(0,0);
  msg[1].data=joints_cmd(1,0);
  msg[2].data=joints_cmd(2,0);
  msg[3].data=joints_cmd(3,0);

  ecmPub[0].publish(msg[0]);
  ecmPub[1].publish(msg[1]);
  ecmPub[2].publish(msg[2]);
  ecmPub[3].publish(msg[3]);
}
