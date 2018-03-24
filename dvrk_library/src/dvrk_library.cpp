#include "dvrk_library.h"

void dvrk_library::update(double v)
{
  m_thread_mutex->lock();
  t=v;
  m_thread_mutex->unlock();
}

int dvrk_library::read()
{
  return t;
}

void dvrk_library::exit_thread()
{
  // m_thread->interrupt();
  run_thread=0;
  std::cout << "Thread for publishing data has been interrupted" << '\n';
}

void dvrk_library::publish_thread()
{
  std::cout << "Thread for publishing data has been started" << '\n';
  std_msgs::Float64 m;
  while (ros::ok() && run_thread)
  {
    boost::this_thread::sleep_for(boost::chrono::microseconds(100));
    ros::spinOnce();
    m_thread_mutex->lock();
    m.data=t;
    m_thread_mutex->unlock();
    pub.publish(m);
  }
  std::cout << "Thread for publishing data has finished executing" << '\n';
  return;
}

void dvrk_library::subscribe(const gazebo_msgs::LinkStatesPtr &msg)
{
  Eigen::Matrix4d ecm_base=Eigen::Matrix4d::Identity(4,4);
  Eigen::Matrix4d ecm_tip=Eigen::Matrix4d::Identity(4,4);
  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk::ecm::base_link"))
    {
      ecm_base(0,3) = msg->pose[i].position.x;
      ecm_base(1,3) = msg->pose[i].position.y;
      ecm_base(2,3) = msg->pose[i].position.z;
      Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
      // q.normalize();
      Eigen::Matrix3d R = q.toRotationMatrix();
      ecm_base(0,0)=R(0,0);
      ecm_base(0,1)=R(0,1);
      ecm_base(0,2)=R(0,2);
      ecm_base(1,0)=R(1,0);
      ecm_base(1,1)=R(1,1);
      ecm_base(1,2)=R(1,2);
      ecm_base(2,0)=R(2,0);
      ecm_base(2,1)=R(2,1);
      ecm_base(2,2)=R(2,2);
    }
    for (int i=0;i<msg->pose.size();i++)
    {
      if (!msg->name[i].compare("dvrk::ecm::camera_link"))
      {
        ecm_tip(0,3) = msg->pose[i].position.x;
        ecm_tip(1,3) = msg->pose[i].position.y;
        ecm_tip(2,3) = msg->pose[i].position.z;
        Eigen::Quaterniond q(msg->pose[i].orientation.w,msg->pose[i].orientation.x,msg->pose[i].orientation.y,msg->pose[i].orientation.z);
        // q.normalize();
        Eigen::Matrix3d R = q.toRotationMatrix();
        ecm_tip(0,0)=R(0,0);
        ecm_tip(0,1)=R(0,1);
        ecm_tip(0,2)=R(0,2);
        ecm_tip(1,0)=R(1,0);
        ecm_tip(1,1)=R(1,1);
        ecm_tip(1,2)=R(1,2);
        ecm_tip(2,0)=R(2,0);
        ecm_tip(2,1)=R(2,1);
        ecm_tip(2,2)=R(2,2);
      }
    }
  }
  ecm ECM("ecm");
  Eigen::MatrixXd joints(4,1);
  joints=ECM.inverse_kinematics(ecm_base.inverse()*ecm_tip);
  std::cout << joints << '\n';
}


// int main(int argc, char **argv)
// {
//   dvrk_library b;
//   b.init();
//   // ros::init(argc, argv, "dvrk_gazebo_control_node");
//   // ros::NodeHandle n;
//   std::cout << ros::ok() << '\n';
//   // while (ros::ok())
//   // {
//   //
//   // }
//   return 0;
// }
