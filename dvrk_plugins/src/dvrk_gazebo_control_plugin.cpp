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
  sub_position.resize(num_joints);
  sub_positionTarget.resize(num_joints);
  sub_Force.resize(num_joints);
  pub_states.resize(num_joints);
  for (int i=0;i<num_joints; i++)
  {
    joint=parent_model->GetJoints()[i];

    std::string joint_namespace, joint_name;
    getJointStrings(joint,joint_namespace, joint_name);
    pub_states[i] = model_nh_.advertise<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/states", 1000);
    // std::cout << (joint_name.find("outer_pitch_joint")!=std::string::npos) << '\n';
    //if ((joint_name.find("outer_pitch_joint")!=std::string::npos) && joint_name.compare("one_outer_pitch_joint_1")||(joint_name.find("fixed")!=std::string::npos))
    //  continue;
    // std::cout << parent_model->GetJoints()[i]->GetAngle(0).Radian() << '\n';
    boost::function<void (const std_msgs::Float64Ptr)>PositionFunc(boost::bind(&dvrkGazeboControlPlugin::SetPosition,this, _1,joint));
    boost::function<void (const std_msgs::Float64Ptr)>PositionTargetFunc(boost::bind(&dvrkGazeboControlPlugin::SetPositionTarget,this, _1,joint));
    boost::function<void (const std_msgs::Float64Ptr)>ForceFunc(boost::bind(&dvrkGazeboControlPlugin::SetForce,this, _1,joint));
    sub_position[i] = model_nh_.subscribe<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/SetPosition",1,PositionFunc);
    sub_positionTarget[i] = model_nh_.subscribe<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/SetPositionTarget",1,PositionTargetFunc);
    sub_Force[i] = model_nh_.subscribe<std_msgs::Float64>(joint_namespace+"/"+joint_name+"/SetEffort",1,ForceFunc);

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

void dvrkGazeboControlPlugin::SetPosition(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint->SetPosition(0,msg->data);
  PublishStates();
}

void dvrkGazeboControlPlugin::SetPositionTarget(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint_class joint_obj(joint);
  double p, i, d;
  joint_obj.getPID(p, i, d);

  if (p<0|| i<0 || d<0)
  {
    ROS_FATAL_STREAM("PID controller could not be initialized. Check if the parameters have been loaded properly on the ROS parameter server." << '\n');
  }
  gazebo::common::PID pid;
  pid=gazebo::common::PID(p,i,d);
  parent_model->GetJointController()->SetPositionPID(joint->GetScopedName(), pid);
  parent_model->GetJointController()->SetPositionTarget(joint->GetScopedName(), msg->data);

  PublishStates();
}

void dvrkGazeboControlPlugin::SetForce(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint)
{
  joint->SetForce(0,msg->data);
  PublishStates();
}

void dvrkGazeboControlPlugin::PublishStates()
{
  std_msgs::Float64 msg;
  gazebo::physics::JointPtr joint;
  for (int n=0; n<num_joints; n++)
  {
    joint=parent_model->GetJoints()[n];
    std::string joint_namespace, joint_name;
    getJointStrings(joint,joint_namespace, joint_name);
    if ((joint_name.find("fixed")!=std::string::npos))
      continue;
    //msg.data=parent_model->GetJoints()[n]->GetAngle(0).Radian();
    
    //std::map<std::string, double> x; 
    //x = parent_model->GetJointController()->GetForces();
    //msg.data= x[joint_name];

    msg.data=joint->GetForceTorque(0).body2Torque[0];

    pub_states[n].publish(msg);
  }
}

void dvrkGazeboControlPlugin::getJointStrings(gazebo::physics::JointPtr jointPtr, std::string &str1, std::string &str2)
{
  std::string joint_name=jointPtr->GetName();
  std::size_t pos=joint_name.find("::");
  str1=joint_name.substr(0,pos);
  str2=joint_name.substr(pos+2);
}

void joint_class::setJointStrings()
{
  std::string joint=jointPtr->GetName();
  std::size_t pos=joint.find("::");
  joint_namespace=joint.substr(0,pos);
  joint_name=joint.substr(pos+2);
}

void joint_class::setController()
{
  double p_def=-1, i_def=-1, d_def=-1;
  model_nh_.param(std::string("/"+joint_namespace+"/"+joint_name+"_controller/p"), p, p_def);
  model_nh_.param(std::string("/"+joint_namespace+"/"+joint_name+"_controller/i"), i, i_def);
  model_nh_.param(std::string("/"+joint_namespace+"/"+joint_name+"_controller/d"), d, d_def);
}

void joint_class::getPID(double& K_p, double& K_i, double& K_d)
{
  K_p=p;
  K_d=d;
  K_i=i;
}

}
