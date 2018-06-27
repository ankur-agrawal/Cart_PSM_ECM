// Boost
#include <boost/bind.hpp>

#include "dvrk_gazebo_control_plugin_new.h"

namespace dvrk_plugins
{

dvrkGazeboControlPlugin::~dvrkGazeboControlPlugin()
{
  // Disconnect from gazebo events
  update_connection_.reset();
}

void dvrkGazeboControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{

  ROS_INFO_STREAM_NAMED("dvrk_plugins","Loading dvrk plugin");

  // Save pointers to the model
  parent_model_ = parent;
  sdf_ = sdf;

  if (!parent_model_)
  {
    ROS_ERROR_STREAM_NAMED("loadThread","parent model is NULL");
    return;
  }

  if(!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("dvrk_plugins","A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // Get the Gazebo simulation period
  #if GAZEBO_MAJOR_VERSION >= 8
  ros::Duration gazebo_period(parent_model_->GetWorld()->Physics()->GetMaxStepSize());
  #else
  ros::Duration gazebo_period(parent_model_->GetWorld()->GetPhysicsEngine()->GetMaxStepSize());
  #endif

  if(sdf_->HasElement("controlPeriod"))
  {
    control_period_ = ros::Duration(sdf_->Get<double>("controlPeriod"));

    // Check the period against the simulation period
    if( control_period_ < gazebo_period )
    {
      ROS_ERROR_STREAM_NAMED("dvrk_plugins","Desired controller update period ("<<control_period_
        <<" s) is faster than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else if( control_period_ > gazebo_period )
    {
      ROS_WARN_STREAM_NAMED("dvrk_plugins","Desired controller update period ("<<control_period_
        <<" s) is slower than the gazebo simulation period ("<<gazebo_period<<" s).");
    }
    else
    {
      control_period_= gazebo_period;
    }
  }
  else
  {
    control_period_ = gazebo_period;
    ROS_DEBUG_STREAM_NAMED("dvrk_plugins","Control period not found in URDF/SDF, defaulting to Gazebo period of "
      << control_period_);
  }

  // Get namespace for nodehandle
  if(sdf_->HasElement("robotNamespace"))
  {
    robot_namespace_ = sdf_->GetElement("robotNamespace")->Get<std::string>();
  }
  else
  {
    robot_namespace_ = parent_model_->GetName(); // default
  }

  model_nh_ = ros::NodeHandle(robot_namespace_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;
  if (sdf_->HasElement("eStopTopic"))
  {
    const std::string e_stop_topic = sdf_->GetElement("eStopTopic")->Get<std::string>();
    e_stop_sub_ = model_nh_.subscribe(e_stop_topic, 1, &dvrkGazeboControlPlugin::eStopCB, this);
  }
  // try
  // {
    sim_loader_.reset(new pluginlib::ClassLoader<dvrk_plugins::SimInterface>("dvrk_plugins","dvrk_plugins::SimInterface"));

    sim_ = sim_loader_->createInstance("dvrk_plugins/SimInterface");

    // sim_= new dvrk_plugins::SimInterface;
    // if(!sim_->initSim(robot_namespace_, model_nh_, parent_model_))
    // {
    //   ROS_FATAL_NAMED("dvrk_plugins","Could not initialize simulation interface");
    //   return;
    // }

    // ROS_DEBUG_STREAM_NAMED("ros_control_plugin","Loading controller_manager");
    // controller_manager_.reset(new controller_manager::ControllerManager(sim_.get(), model_nh_));

    update_connection_= gazebo::event::Events::ConnectWorldUpdateBegin(boost::bind(&dvrkGazeboControlPlugin::Update, this));
  // }
  // catch(pluginlib::LibraryLoadException &ex)
  // {
  //   ROS_FATAL_STREAM_NAMED("gazebo_ros_control","Failed to create robot simulation interface loader: "<<ex.what());
  // }
}

void dvrkGazeboControlPlugin::Update()
{
  // Get the simulation time and period
  #if GAZEBO_MAJOR_VERSION >= 8
    gazebo::common::Time gz_time_now = parent_model_->GetWorld()->SimTime();
  #else
    gazebo::common::Time gz_time_now = parent_model_->GetWorld()->GetSimTime();
  #endif
  ros::Time sim_time_ros(gz_time_now.sec, gz_time_now.nsec);
  ros::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  // sim_->eStopActive(e_stop_active_);

  // Check if we should update the controllers
  if(sim_period >= control_period_)
  {
    // Store this simulation time
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // std::cout << sim_period.toSec() << '\n';
}

void dvrkGazeboControlPlugin::Reset()
{
  // Reset timing variables to not pass negative update periods to controllers on world reset
  last_update_sim_time_ros_ = ros::Time();
  last_write_sim_time_ros_ = ros::Time();
}

void dvrkGazeboControlPlugin::eStopCB(const std_msgs::BoolConstPtr& e_stop_active)
{
  e_stop_active_ = e_stop_active->data;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(dvrkGazeboControlPlugin)
}
