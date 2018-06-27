#include "simulation_interface.h"

namespace dvrk_plugins
{

bool SimInterface::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model)
{
  std::cout << "initialised" << '\n';
}

bool SimInterface::readSim(ros::Time time, ros::Duration period)
{
  std::cout << "read" << '\n';
}

void SimInterface::writeSim(ros::Time time, ros::Duration period)
{
  std::cout << "written" << '\n';
}
}

PLUGINLIB_EXPORT_CLASS(dvrk_plugins::SimInterface, hardware_interface::RobotHW)
