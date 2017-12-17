#include "dvrk_gazebo_control_plugin.h"

void trial(const std_msgs::Float64Ptr msg, int i)
{
  std::cout << "i" << '\n';
}

namespace dvrk_plugins
{
void dvrkGazeboControlPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  parent_model=_model;
  sdf=_sdf;
  gazebo::physics::JointPtr joint;

  model_nh_ = ros::NodeHandle(parent_model->GetName());
  num_joints=parent_model->GetJoints().size();
  sub_command.resize(num_joints);
  pub_states.resize(num_joints);
  for (int i=0;i<num_joints; i++)
  {
    joint=parent_model->GetJoints()[i];
    std::string joint_namespace, joint_name;
    getJointStrings(joint, joint_namespace, joint_name);
    // std::cout << parent_model->GetJoints()[i]->GetAngle(0).Radian() << '\n';
    boost::function<void (const std_msgs::Float64Ptr)>f(boost::bind(&dvrkGazeboControlPlugin::MoveJoint,this, _1,joint, joint_namespace));
    sub_command[i] = model_nh_.subscribe<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/command",1,f);
    pub_states[i] = model_nh_.advertise<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/states", 1000);
  }
  this->PublishStates();
}
void dvrkGazeboControlPlugin::print(const std_msgs::BoolConstPtr& msg)
{
  if (msg->data)
    // printf("%d\n",parent_model->GetChildCount());
    std::cout << parent_model->GetJoints()[1]->GetAngleCount()<<'\t'<<parent_model->GetJoints()[1]->GetAngle(0).Radian() << '\t' << parent_model->GetJoints()[1]->GetForce(0)<< '\n';
  else
    ROS_INFO("Hello World!");
}

void dvrkGazeboControlPlugin::MoveJoint(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint, std::string joint_namespace)
{
  if (!joint_namespace.compare("PSM1"))
  {
    joint->SetPosition(0,msg->data);
  }

  else
  {
    // std::cout << "Bye" << '\n';
    gazebo::common::PID pid;
    pid=gazebo::common::PID(600,0,0);
    parent_model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
    parent_model->GetJointController()->SetPositionTarget(joint->GetScopedName(), msg->data);
  }
  PublishStates();
}

void dvrkGazeboControlPlugin::getJointStrings(gazebo::physics::JointPtr joint, std::string &str1, std::string &str2)
{
  std::string joint_name=joint->GetName();
  std::size_t pos=joint_name.find("::");
  str1=joint_name.substr(0,pos);
  str2=joint_name.substr(pos+2);
  // std::cout << str1.c_str() << '\t' << str2.c_str() << '\n';
}

void dvrkGazeboControlPlugin::PublishStates()
{
  std_msgs::Float64 msg;
  // std::cout << "HI" << '\n';
  for (int n=0; n<num_joints; n++)
  {
    msg.data=parent_model->GetJoints()[n]->GetAngle(0).Radian();
    pub_states[n].publish(msg);
  }
}

}
