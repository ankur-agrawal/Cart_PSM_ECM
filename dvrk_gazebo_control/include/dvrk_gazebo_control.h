#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <boost/bind.hpp>
#include <gazebo_msgs/LinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
class dvrk_gazebo_control{
private:
  std::vector<ros::Publisher> ecmPub, psm1Pub, psm2Pub, psm3Pub;
  ros::Subscriber link_states, joint_states, masterSub;
  ros::Publisher plot_x, plot_y, plot_z;
  std::vector<ros::Publisher> cartPub;
  std::ofstream outdata;
  std::ofstream outdata_rot;
  Eigen::Vector4d lastMaster;
  Eigen::Matrix4d ECM_base;
  Eigen::Matrix4d ECM_base_to_tip;
  Eigen::Matrix4d ECM_world_to_tip;
  double ECM_joints[4];
  double PSM1_joints[6];
  double PSM2_joints[6];
  Eigen::Matrix4d ECM_DH;
  Eigen::Matrix4d psm1_base=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d psm1_tip=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d psm2_base=Eigen::Matrix4d::Identity();
  Eigen::Matrix4d psm2_tip=Eigen::Matrix4d::Identity();
  int init=1;
public:
  dvrk_gazebo_control(ros::NodeHandle n){
    ecmPub.resize(4);
    ecmPub[0]=n.advertise<std_msgs::Float64>("/dvrk/ecm/yaw_joint/SetPositionTarget",1000);
    ecmPub[1]=n.advertise<std_msgs::Float64>("/dvrk/ecm/pitch_front_joint/SetPositionTarget",1000);
    ecmPub[2]=n.advertise<std_msgs::Float64>("/dvrk/ecm/main_insertion_joint/SetPositionTarget",1000);
    ecmPub[3]=n.advertise<std_msgs::Float64>("/dvrk/ecm/tool_joint/SetPosition",1000);


    psm1Pub.resize(5);
    psm1Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_yaw_joint/SetPositionTarget",1000);
    psm1Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_pitch_joint_1/SetPositionTarget",1000);
    psm1Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_insertion_joint/SetPositionTarget",1000);
    psm1Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/outer_roll_joint/SetPositionTarget",1000);
    psm1Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM1/rev_joint/SetPosition",10);

    psm2Pub.resize(5);
    psm2Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_yaw_joint/SetPositionTarget",1000);
    psm2Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_pitch_joint_1/SetPositionTarget",1000);
    psm2Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_insertion_joint/SetPositionTarget",1000);
    psm2Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/outer_roll_joint/SetPositionTarget",1000);
    psm2Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM2/rev_joint/SetPosition",10);

    psm3Pub.resize(5);
    psm3Pub[0]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_yaw_joint/SetPosition",1000);
    psm3Pub[1]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_pitch_joint_1/SetPositionTarget",1000);
    psm3Pub[2]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_insertion_joint/SetPosition",1000);
    psm3Pub[3]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/outer_roll_joint/SetPosition",1000);
    psm3Pub[4]=n.advertise<std_msgs::Float64>("/dvrk/PSM3/rev_joint/SetPosition",1000);
    //
    plot_x=n.advertise<std_msgs::Float64>("/plotx",1000);
    plot_y=n.advertise<std_msgs::Float64>("/ploty",1000);
    plot_z=n.advertise<std_msgs::Float64>("/plotz",1000);

    outdata.open("ecm_yaw_step.csv");
    outdata_rot.open("ecm_yaw_angle.csv");

    cartPub.resize(19);
    char topic[100];
    for (int i=1;i<4;i++)
    {
      for (int j=0;j<5;j++)
      {
        sprintf(topic, "/dvrk/SUJ/SUJ_psm1%d_J%d/SetPosition", i,j);

        cartPub[5*(i-1)+j] = n.advertise<std_msgs::Float64>(topic,1000);
      }
    }
    for (int j=0;j<4;j++)
    {
      sprintf(topic, "/dvrk/SUJ/SUJ_ECM_J%d/SetPosition",j);

      cartPub[15+j] = n.advertise<std_msgs::Float64>(topic,1000);
    }
    link_states=n.subscribe("/gazebo/link_states", 100, &dvrk_gazebo_control::getECMEndEffector, this);
    joint_states=n.subscribe("/dvrk/joint/states", 100, &dvrk_gazebo_control::readJointStates, this);
    masterSub=n.subscribe("/dvrk/MTM/LeftPose", 100, &dvrk_gazebo_control::readMaster, this);
    setECM_base();
    setECM_DH();
    ECM_FK();

    const geometry_msgs::Pose msg;
    readMaster(msg);
    // Eigen::Quaterniond q;
    // std::cout << q.toRotationMatrix() << '\n';
    // std::cout <<Eigen::Quaterniond(1,0,0,0) << '\n';
    // Eigen::MatrixXd pos(3,1);
    // pos(0,0)=ECM_base(0,3);
    // pos(1,0)=ECM_base(1,3);
    // pos(2,0)=ECM_base(2,3);
    // std::cout << pos << '\n';
    // PSM_IK(pos, PSM1_joints);
    // std::cout << PSM1_joints[0] << '\t' << PSM1_joints[1] << '\t'<< PSM1_joints[2] << '\n';
  }
  void setECM_DH();
  void setECM_base();
  Eigen::Matrix4d compute_mod_dh_matrix(double alpha, double a, double theta, double d);
  void ECM_FK();
  void PSM_IK(Eigen::MatrixXd pos, double (&PSM_joints)[6]);
  void showImage(const sensor_msgs::ImageConstPtr& img);
  void getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg);
  void readJointStates(const sensor_msgs::JointStatePtr &msg);
  void readMaster(const geometry_msgs::Pose &msg);
  void PublishCartStates();
  void PublishECMStates();
  void PublishPSM1States(double PSM_joints[6]);
  void PublishPSM2States(double PSM_joints[6]);
  void PublishPSM3States();

};
