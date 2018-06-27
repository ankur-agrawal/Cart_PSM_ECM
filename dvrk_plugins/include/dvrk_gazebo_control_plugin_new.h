// Boost
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

// ROS
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <std_msgs/Bool.h>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ros_control
#include <controller_manager/controller_manager.h>

#include "simulation_interface.h"

namespace dvrk_plugins
{
class dvrkGazeboControlPlugin : public gazebo::ModelPlugin
{
public:
  virtual ~dvrkGazeboControlPlugin();

  // Overloaded Gazebo entry point
  virtual void Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf);

  // Called by the world update start event
  void Update();

  // Called on World reset
  virtual void Reset();

protected:

  // Node Handles
  ros::NodeHandle model_nh_;

  // Pointer to the model
  gazebo::physics::ModelPtr parent_model_;
  sdf::ElementPtr sdf_;

  // Strings
  std::string robot_namespace_;

  // deferred load in case ros is blocking
  boost::thread deferred_load_thread_;

  // Pointer to the update event connection
  gazebo::event::ConnectionPtr update_connection_;

  boost::shared_ptr<pluginlib::ClassLoader<dvrk_plugins::SimInterface> > sim_loader_;
  boost::shared_ptr<dvrk_plugins::SimInterface> sim_;

  // dvrk_plugins::SimInterface *sim_;

  // Controller manager
  boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

  // Timing
  ros::Duration control_period_;
  ros::Time last_update_sim_time_ros_;
  ros::Time last_write_sim_time_ros_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  ros::Subscriber e_stop_sub_;  // Emergency stop subscriber
  void eStopCB(const std_msgs::BoolConstPtr& e_stop_active);

};

}
