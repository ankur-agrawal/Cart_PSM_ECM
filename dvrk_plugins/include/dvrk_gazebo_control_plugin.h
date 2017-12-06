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

  void MoveJoint(const std_msgs::Float64Ptr& msg, gazebo::physics::JointPtr joint, std::string joint_namespace);

  void getJointStrings(gazebo::physics::JointPtr joint, std::string &str1, std::string &str2);

  void PublishStates();

private:
  gazebo::physics::ModelPtr parent_model;
  sdf::ElementPtr sdf;
  gazebo::transport::NodePtr node;
  std::vector<ros::Subscriber> sub_command;
  std::vector<ros::Publisher> pub_states;
  ros::NodeHandle model_nh_;
  int num_joints;

};
GZ_REGISTER_MODEL_PLUGIN(dvrkGazeboControlPlugin)
}
