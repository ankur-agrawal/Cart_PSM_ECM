#include "dvrk_gazebo_control.h"
// #include <string>

void dvrk_gazebo_control::getECMEndEffector(const gazebo_msgs::LinkStatesPtr &msg)
{
  gazebo_msgs::LinkState ecm_roll;
  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk::ecm::rcm"))
    // if (!msg->name[i].compare("dvrk::PSM1::one_tool_wrist_link"))
    {
      ecm_roll.pose = msg->pose[i];
    }
  }
  // std::cout << ecm_roll.pose.position.x << '\t' << ecm_roll.pose.position.y << '\t' << ecm_roll.pose.position.z<< '\n';
  std_msgs::Float64 msg2;
  msg2.data=ecm_roll.pose.position.x;
  plot_x.publish(msg2);
  msg2.data=ecm_roll.pose.position.y;
  plot_y.publish(msg2);
  msg2.data=ecm_roll.pose.position.z;
  plot_z.publish(msg2);
  msg2.data=0;
  ecmPub[0].publish(msg2);
  ecmPub[3].publish(msg2);

  static double pitch_angle;
  static double direction;
  msg2.data=pitch_angle;
  if (pitch_angle>0.7)
    direction=1;
  if (pitch_angle<-0.2)
    direction=0;
  if (direction==0)
    pitch_angle=pitch_angle+0.001;
  else
    pitch_angle=pitch_angle-0.001;
  ecmPub[1].publish(msg2);
  msg2.data=0;
  ecmPub[2].publish(msg2);
  msg2.data=0;

  PublishCartStates();

}
void dvrk_gazebo_control::PublishCartStates()
{
  std_msgs::Float64 msg;
  msg.data=0;
  for (int i=1;i<4;i++)
  {
    for (int j=0;j<5;j++)
    {
      cartPub[5*(i-1)+j].publish(msg);
    }
  }
  for (int j=0;j<4;j++)
  {
    cartPub[15+j].publish(msg);
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
