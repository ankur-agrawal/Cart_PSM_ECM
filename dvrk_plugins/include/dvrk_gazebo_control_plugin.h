#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <string.h>
#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace dvrk_plugins
{
class dvrkGazeboControlPlugin : public gazebo::ModelPlugin
{
public:
  dvrkGazeboControlPlugin(){}

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void print(const std_msgs::BoolConstPtr& msg);

  void getJointStrings(gazebo::physics::JointPtr jointPtr, std::string &str1, std::string &str2);

  void SetPosition(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint);

  void SetPositionTarget(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint);

  void SetForce(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint);

  void PublishStates();

private:
  gazebo::physics::ModelPtr parent_model;
  sdf::ElementPtr sdf;
  gazebo::transport::NodePtr node;
  std::vector<ros::Subscriber> sub_position, sub_positionTarget, sub_Force;
  std::vector<ros::Publisher> pub_states;
  int num_joints;
protected:
  ros::NodeHandle model_nh_;

};

class joint_class : public dvrkGazeboControlPlugin
{
public:
  joint_class(gazebo::physics::JointPtr joint)
  {
    jointPtr=joint;
    setJointStrings();
    setController();
  }
  void setJointStrings();
  void setController();
  void getPID(double& K_p, double& K_i, double& K_d);

private:
  gazebo::physics::JointPtr jointPtr;
  std::string joint_namespace;
  std::string joint_name;
  double p, i, d;

};

GZ_REGISTER_MODEL_PLUGIN(dvrkGazeboControlPlugin)
}
