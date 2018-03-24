#include "dvrk_gazebo_control.h"
static double pitch_angle=0;
static double yaw_angle=0;
static double direction;
static double iter;
static bool step_pitch=1;
static bool step_yaw=0;

Eigen::Matrix4d dvrk_gazebo_control::compute_mod_dh_matrix(double alpha, double a, double theta, double d)
{
  Eigen::Matrix4d DH_matrix;
  DH_matrix(0,0)=cos(theta);
  DH_matrix(0,1)=-sin(theta);
  DH_matrix(0,2)=0;
  DH_matrix(0,3)=a;
  DH_matrix(1,0)=sin(theta)*cos(alpha);
  DH_matrix(1,1)=cos(theta)*cos(alpha);
  DH_matrix(1,2)=-sin(alpha);
  DH_matrix(1,3)=-d*sin(alpha);
  DH_matrix(2,0)=sin(theta)*sin(alpha);
  DH_matrix(2,1)=cos(theta)*sin(alpha);
  DH_matrix(2,2)=cos(alpha);
  DH_matrix(2,3)=d*cos(alpha);
  DH_matrix(3,0)=0;
  DH_matrix(3,1)=0;
  DH_matrix(3,2)=0;
  DH_matrix(3,3)=1;
  return DH_matrix;
}

void dvrk_gazebo_control::setECM_DH()
{

  ECM_DH(0,0)=1.5708;
  ECM_DH(0,1)=0;
  ECM_DH(0,2)=ECM_joints[0]+1.5708;
  ECM_DH(0,3)=0;
  ECM_DH(1,0)=-1.5708;
  ECM_DH(1,1)=0;
  ECM_DH(1,2)=ECM_joints[1]-1.5708;
  ECM_DH(1,3)=0;
  ECM_DH(2,0)=1.5708;
  ECM_DH(2,1)=0;
  ECM_DH(2,2)=0;
  ECM_DH(2,3)=ECM_joints[2]-0.3822;
  ECM_DH(3,0)=0;
  ECM_DH(3,1)=0;
  ECM_DH(3,2)=ECM_joints[3];
  ECM_DH(3,3)=0.3829;

}

void dvrk_gazebo_control::ECM_FK()
{
  ECM_base_to_tip << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  for (int i=0; i<4;i++)
  {
    ECM_base_to_tip=ECM_base_to_tip * compute_mod_dh_matrix(ECM_DH(i,0),ECM_DH(i,1),ECM_DH(i,2),ECM_DH(i,3));
  }
  ECM_world_to_tip=ECM_base*ECM_base_to_tip;
}

void dvrk_gazebo_control::setECM_base()
{
  ECM_base(0,0)=0;
  ECM_base(0,1)=1;
  ECM_base(0,2)=0;
  ECM_base(0,3)=0.6126;
  ECM_base(1,0)=-1;
  ECM_base(1,1)=0;
  ECM_base(1,2)=0;
  ECM_base(1,3)=0;
  ECM_base(2,0)=0;
  ECM_base(2,1)=0;
  ECM_base(2,2)=1;
  ECM_base(2,3)=1.6016;
  ECM_base(3,0)=0;
  ECM_base(3,1)=0;
  ECM_base(3,2)=0;
  ECM_base(3,3)=1;
}
void dvrk_gazebo_control::PSM_IK(Eigen::MatrixXd pos, double (&PSM_joints)[6])
{
  // double PSM_joints[6];
  PSM_joints[0]=atan2(pos(0,0),-pos(1,0));
  PSM_joints[1]=atan2(-(pos(2,0)+0.369),sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)));
  PSM_joints[2]=sqrt(pow(pos(0,0),2)+pow(pos(1,0),2)+pow((pos(2,0)+0.369),2));
  PSM_joints[3]=0;
  PSM_joints[4]=0;
  PSM_joints[5]=0;
}

void dvrk_gazebo_control::readMaster(const geometry_msgs::Pose &Master)
{
  if (init==1)
  {
    lastMaster(0,0)=Master.position.x;
    lastMaster(1,0)=Master.position.y;
    lastMaster(2,0)=Master.position.z;
    lastMaster(3,0)=1;
    init=0;
    return;
  }

  Eigen::MatrixXd diff(4,1);
  Eigen::MatrixXd psm1_tip_pos(4,1), PSM1_tip(4,1);
  Eigen::Matrix4d PSM1_tip_in_ecm;
  // PSM1_tip_in_ecm=ECM_world_to_tip.inverse()*psm1_tip;
  psm1_tip_pos(0,3)=psm1_tip(0,0);
  psm1_tip_pos(1,3)=psm1_tip(1,0);
  psm1_tip_pos(2,3)=psm1_tip(2,0);
  diff(0,0)=Master.position.x-lastMaster(0,0);
  diff(1,0)=Master.position.y-lastMaster(1,0);
  diff(2,0)=Master.position.z-lastMaster(2,0);
  diff(3,0)=1;
  // PSM1_tip_in_ecm_pos=PSM1_tip_in_ecm_pos+diff;
  PSM1_tip=psm1_tip_pos+ECM_world_to_tip*diff;
  PSM_IK(PSM1_tip, PSM1_joints);
  PublishPSM1States(PSM1_joints);
  std::cout << PSM1_tip << '\n';
}

void dvrk_gazebo_control::readJointStates(const sensor_msgs::JointStatePtr &msg)
{
  std_msgs::Float64 msg2;
  for (int i=0;i<msg->name.size();i++)
  {
    if (step_pitch)
    {
      if (!msg->name[i].compare("dvrk/ecm/pitch_front_joint"))
      {
        msg2.data=msg->position[i];
        plot_x.publish(msg2);
        outdata << msg->header.stamp<< '\t' << msg->position[i] << '\n';
        outdata_rot << msg->header.stamp<< '\t' << pitch_angle << '\n';
      }
    }
    if (step_yaw)
    {
      if (!msg->name[i].compare("dvrk/ecm/yaw_joint"))
      {
        msg2.data=msg->position[i];
        plot_x.publish(msg2);
        outdata << msg->header.stamp<< '\t' << msg->position[i] << '\n';
        outdata_rot << msg->header.stamp<< '\t' << yaw_angle << '\n';
      }
    }
  }
}

void dvrk_gazebo_control::getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg)
{
  gazebo_msgs::LinkState ecm_rcm;

  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk::PSM1::outer_insertion_link"))
    // if (!msg->name[i].compare("dvrk::PSM1::one_tool_wrist_link"))
    {
      ecm_rcm.pose = msg->pose[i];
    }
    if (!msg->name[i].compare("dvrk::PSM1::remote_center_link"))
    // if (!msg->name[i].compare("dvrk::PSM1::one_tool_wrist_link"))
    {
      psm1_base(0,3) = msg->pose[i].position.x;
      psm1_base(1,3) = msg->pose[i].position.y;
      psm1_base(2,3) = msg->pose[i].position.z;
      Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
      // q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      psm1_base(0,0)=R(0,0);
      psm1_base(0,1)=R(0,1);
      psm1_base(0,2)=R(0,2);
      psm1_base(1,0)=R(1,0);
      psm1_base(1,1)=R(1,1);
      psm1_base(1,2)=R(1,2);
      psm1_base(2,0)=R(2,0);
      psm1_base(2,1)=R(2,1);
      psm1_base(2,2)=R(2,2);
    }
    if (!msg->name[i].compare("dvrk::PSM1::tool_main_link"))
    {
      psm1_tip(0,3) = msg->pose[i].position.x;
      psm1_tip(1,3) = msg->pose[i].position.y;
      psm1_tip(2,3) = msg->pose[i].position.z;
      Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
      // q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      psm1_tip(0,0)=R(0,0);
      psm1_tip(0,1)=R(0,1);
      psm1_tip(0,2)=R(0,2);
      psm1_tip(1,0)=R(1,0);
      psm1_tip(1,1)=R(1,1);
      psm1_tip(1,2)=R(1,2);
      psm1_tip(2,0)=R(2,0);
      psm1_tip(2,1)=R(2,1);
      psm1_tip(2,2)=R(2,2);
    }
    if (!msg->name[i].compare("dvrk::PSM2::remote_center_link"))
    // if (!msg->name[i].compare("dvrk::PSM1::one_tool_wrist_link"))
    {
      psm2_base(0,3) = msg->pose[i].position.x;
      psm2_base(1,3) = msg->pose[i].position.y;
      psm2_base(2,3) = msg->pose[i].position.z;
      Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
      // q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      psm2_base(0,0)=R(0,0);
      psm2_base(0,1)=R(0,1);
      psm2_base(0,2)=R(0,2);
      psm2_base(1,0)=R(1,0);
      psm2_base(1,1)=R(1,1);
      psm2_base(1,2)=R(1,2);
      psm2_base(2,0)=R(2,0);
      psm2_base(2,1)=R(2,1);
      psm2_base(2,2)=R(2,2);
    }
    if (!msg->name[i].compare("dvrk::PSM2::tool_main_link"))
    {
      psm2_tip(0,3) = msg->pose[i].position.x;
      psm2_tip(1,3) = msg->pose[i].position.y;
      psm2_tip(2,3) = msg->pose[i].position.z;
      Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
      // q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      psm2_tip(0,0)=R(0,0);
      psm2_tip(0,1)=R(0,1);
      psm2_tip(0,2)=R(0,2);
      psm2_tip(1,0)=R(1,0);
      psm2_tip(1,1)=R(1,1);
      psm2_tip(1,2)=R(1,2);
      psm2_tip(2,0)=R(2,0);
      psm2_tip(2,1)=R(2,1);
      psm2_tip(2,2)=R(2,2);
    }
  }
  // Eigen::Matrix4d psm_base_to_tip = psm_base.inverse()*psm_tip;
  // Eigen::MatrixXd pos(3,1);
  // pos << psm_tip(0,3)-psm_base(0,3), psm_tip(1,3)-psm_base(1,3), psm_tip(2,3)-psm_base(2,3);
  // PSM_IK(pos, PSM1_joints);
  // std::cout << pos(0,0) << '\t' << pos(1,0) << '\t' << pos(2,0)<< '\n';
  // std::cout << PSM1_joints[0] <<'\t' << PSM1_joints[1] <<'\t'<< PSM1_joints[2] <<'\t'<< '\n';

  // outdata << ecm_rcm.pose.position.x << '\t' << ecm_rcm.pose.position.y << '\t' << ecm_rcm.pose.position.z<< '\n';
  // outdata_rot << ecm_rcm.pose.orientation.x << '\t' << ecm_rcm.pose.orientation.y << '\t' << ecm_rcm.pose.orientation.z << '\t' << ecm_rcm.pose.orientation.w << '\n';
  // outdata.close();
  std_msgs::Float64 msg2;
  // msg2.data=ecm_rcm.pose.position.x;
  // plot_x.publish(msg2);
  // msg2.data=ecm_rcm.pose.position.y;
  // plot_y.publish(msg2);
  // msg2.data=ecm_rcm.pose.position.z;
  // plot_z.publish(msg2);
  msg2.data=0;
  // ecmPub[3].publish(msg2);
  // psm1Pub[3].publish(msg2);
  //
  if (iter > 5000)
  {
    if (step_pitch)
      pitch_angle=0.5;
    if (step_yaw)
      yaw_angle=1;
  }
  else
  {
    // if (step_pitch)
      pitch_angle=0;
    // if (step_yaw)
      yaw_angle=0;
  }

  iter=iter+1;


  // if (pitch_angle>0.5)
  // {
  //   direction=1;
  //   yaw_angle=yaw_angle+0.1;
  // }
  // if (yaw_angle > 1)
  // {
  //   yaw_angle=1;
  // }
  // if (pitch_angle<-0.5)
  //   direction=0;
  // if (direction==0)
  //   pitch_angle=pitch_angle+0.001;
  // else
  //   pitch_angle=pitch_angle-0.001;

  // std::cout << pitch_angle << '\n';
  // msg2.data=pitch_angle;
  // ecmPub[1].publish(msg2);
  // psm1Pub[1].publish(msg2);
  //
  // // std::cout << yaw_angle << '\n';
  // // yaw_angle=0;
  // msg2.data=yaw_angle;
  // ecmPub[0].publish(msg2);
  // psm1Pub[0].publish(msg2);
  //
  // msg2.data=0;
  // ecmPub[2].publish(msg2);
  // psm1Pub[2].publish(msg2);
  // msg2.data=0;

  // PublishCartStates();
  // PublishECMStates();
  // PublishPSM1States();
  // PublishPSM2States();
  // PublishPSM3States();

}

void dvrk_gazebo_control::PublishECMStates()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(4);
  msg[0].data=0;
  msg[1].data=0;
  msg[2].data=0;
  msg[3].data=0;

  ecmPub[0].publish(msg[0]);
  ecmPub[1].publish(msg[1]);
  ecmPub[2].publish(msg[2]);
  ecmPub[3].publish(msg[3]);

}

void dvrk_gazebo_control::PublishPSM1States(double PSM_joints[6])
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=PSM_joints[0];
  msg[1].data=PSM_joints[1];
  msg[2].data=PSM_joints[2];
  msg[3].data=PSM_joints[3];
  // msg[4].data=PSM_joints[4];

  psm1Pub[0].publish(msg[0]);
  psm1Pub[1].publish(msg[1]);
  psm1Pub[2].publish(msg[2]);
  psm1Pub[3].publish(msg[3]);
  // psm1Pub[4].publish(msg[4]);

}
void dvrk_gazebo_control::PublishPSM2States(double PSM_joints[6])
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=-0.75;
  msg[1].data=0;
  msg[2].data=0;
  msg[3].data=0;
  msg[4].data=0;

  psm2Pub[0].publish(msg[0]);
  psm2Pub[1].publish(msg[1]);
  psm2Pub[2].publish(msg[2]);
  psm2Pub[3].publish(msg[3]);
  psm2Pub[4].publish(msg[4]);

}

void dvrk_gazebo_control::PublishPSM3States()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(5);
  msg[0].data=0;
  msg[1].data=0;
  msg[2].data=0;
  msg[3].data=0;
  msg[4].data=0;

  psm3Pub[0].publish(msg[0]);
  psm3Pub[1].publish(msg[1]);
  psm3Pub[2].publish(msg[2]);
  psm3Pub[3].publish(msg[3]);
  psm3Pub[4].publish(msg[4]);

}

void dvrk_gazebo_control::PublishCartStates()
{
  std::vector<std_msgs::Float64> msg;
  msg.resize(19);

  msg[0].data=0;
  msg[1].data=-0.57;
  msg[2].data=-0.74;
  msg[3].data=1.16;
  msg[4].data=-1.00;

  msg[5].data=0;
  msg[6].data=0.36;
  msg[7].data=1.14;
  msg[8].data=-0.74;
  msg[9].data=0.60;

  msg[10].data=0;
  msg[11].data=0;
  msg[12].data=0;
  msg[13].data=0;
  msg[14].data=0;

  msg[15].data=0.75;
  msg[16].data=0;
  msg[17].data=0;
  msg[18].data=0;
  for (int i=1;i<4;i++)
  {
    for (int j=0;j<5;j++)
    {
      cartPub[5*(i-1)+j].publish(msg[5*(i-1)+j]);
    }
  }
  for (int j=0;j<4;j++)
  {
    cartPub[15+j].publish(msg[15+j]);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dvrk_gazebo_control_node");
  ros::NodeHandle n;
  dvrk_gazebo_control obj(n);
  // int i, j;

  std_msgs::Float64 msg;
  // char link[100];


  while (ros::ok())
  {
    ros::spin();
  }
}
