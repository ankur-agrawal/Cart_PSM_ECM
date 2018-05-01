#include "dvrk_library.h"

void dvrk_library::update()
{
  // m_thread_mutex->lock();
  // t=v;
  // m_thread_mutex->unlock();
  // psm1_manip->set_init_joints(-0.5,0,0.1,0,0,0);
  ros::spinOnce();
  psm1_manip->PublishJoints();
}

int dvrk_library::read()
{
  return t;
}

bool dvrk_library::exit_python()
{
  // std::cout << ros::ok() << '\n';
  // return exit;
  return ros::ok();
}

void dvrk_library::exit_thread()
{
  run_thread=0;
  std::cout << "Thread for publishing data has been interrupted" << '\n';
}

// void dvrk_library::publish_thread()
// {
//   std::cout << "Thread for publishing data has been started" << '\n';
//   std_msgs::Float64 m;
//   while (ros::ok() && run_thread)
//   {
//     boost::this_thread::sleep_for(boost::chrono::microseconds(100));
//     ros::spinOnce();
//     // m_thread_mutex->lock();
//     // m.data=t;
//     // m_thread_mutex->unlock();
//     // teleop();
//     psm1_manip->PublishJoints();
//     // ecm_manip->PublishJoints();
//   }
//   std::cout << "Thread for publishing data has finished executing" << '\n';
//   exit=true;
//   return;
// }

void dvrk_library::subscribe(const gazebo_msgs::LinkStatesPtr &msg)
{

  for (int i=0;i<msg->pose.size();i++)
  {
    if (!msg->name[i].compare("dvrk::ecm::base_link"))
    {
      LinkStateToEigen(ecm_base, msg->pose[i]);
    }
    if (!msg->name[i].compare("dvrk::ecm::camera_link"))
    {
      LinkStateToEigen(ecm_tip, msg->pose[i]);
    }
    if (!msg->name[i].compare("dvrk::PSM1::remote_center_link"))
    {
      LinkStateToEigen(psm1_base, msg->pose[i]);
    }
    if (!msg->name[i].compare("dvrk::PSM1::large_needle_driver::tool_tip_link"))
    {
      LinkStateToEigen(psm1_tip, msg->pose[i]);
    }
    if (!msg->name[i].compare("dvrk::PSM2::remote_center_link"))
    {
      LinkStateToEigen(psm2_base, msg->pose[i]);
    }
    if (!msg->name[i].compare("dvrk::PSM2::large_needle_driver::tool_tip_link"))
    {
      LinkStateToEigen(psm2_tip, msg->pose[i]);
    }
  }
  // ecm_manip->joints_cmd=ecm_manip->inverse_kinematics(ecm_base.inverse()*ecm_tip);
  // std::cout << ecm_manip->joints_cmd.transpose() << '\n';
  // ecm_manip->PublishJoints();
  // Eigen::MatrixXd ecm_joints(4,1);
  // ecm_joints=ecm_manip->inverse_kinematics(ecm_base.inverse()*ecm_tip);

  // psm1_manip->joints_cur=psm1_manip->inverse_kinematics(psm1_base.inverse()*psm1_tip);
  // psm2_manip->joints_cur=psm2_manip->inverse_kinematics(psm2_base.inverse()*psm2_tip);
}

void dvrk_library::subscribeMaster(const geometry_msgs::PoseStampedPtr &msg, int arm)
{
  if (arm==0)
  {
    iter=iter+1;
    LinkStateToEigen(leftMasterPose, msg->pose);
  }
  if (arm==1)
  {
    iter=iter+1;
    LinkStateToEigen(rightMasterPose, msg->pose);
  }
}

void dvrk_library::subscribeOculus(const geometry_msgs::PoseStampedPtr &msg)
{
  Eigen::Matrix4d oculus_pose=Eigen::Matrix4d::Identity(4,4);
  LinkStateToEigen(oculus_pose, msg->pose);
  Eigen::Matrix4d oculus_to_camera;
  Eigen::Matrix4d ecm_end_oculus;
  // oculus_to_camera << 0,0,1,0,
  //                     0,1,0,0,
  //                    -1,0,0,0,
  //                     0,0,0,1;
  oculus_to_camera << 0,1,0,0,
                     -1,0,0,0,
                      0,0,-1,0,
                      0,0,0,1;
  ecm_end_oculus << 0,0,1,0,
                   -1,0,0,0,
                    0,1,0,0,
                    0,0,0,1;
  // Eigen::Matrix4d ecm_tip_to_end;
  // ecm_tip_to_end << 0,0,-1,0,
  //                   0,1,0,0,
  //                   1,0,0,0,
  //                   0,0,0,1;
  ecm_manip->joints_cmd=ecm_manip->inverse_kinematics(oculus_to_camera*oculus_pose*ecm_end_oculus.inverse());
  ecm_manip->joints_cmd(2,0)=0.0;
  std::cout << ecm_manip->joints_cmd.transpose() << '\n';
  ecm_manip->PublishJoints();
  // std::cout << oculus_pose << '\n';
}


void dvrk_library::teleop()
{
  // if (iter<1)
  // {
  //   last_leftMasterPose=leftMasterPose;
  //   last_rightMasterPose=rightMasterPose;
  //   return;
  // }
  psm1_tip=psm1_manip->ForwardKinematics();

  // ecm_tip=Eigen::Matrix4d::Identity();
  // Eigen::MatrixXd diff_translation(4,1);
  // Eigen::Matrix4d psm1_tip_command=psm1_tip;
  // Eigen::MatrixXd cur_psm1_tip_position(4,1);
  // diff_translation(0,0) = leftMasterPose(0,3)-last_leftMasterPose(0,3);
  // diff_translation(1,0) = leftMasterPose(1,3)-last_leftMasterPose(1,3);
  // diff_translation(2,0) = leftMasterPose(2,3)-last_leftMasterPose(2,3);
  // diff_translation(3,0) = 0;


  // cur_psm1_tip_position=psm1_tip.block<4,1>(0,3);
  // psm1_tip_command.block<4,1>(0,3)=cur_psm1_tip_position+ecm_tip*diff_translation;
  // std::cout << psm1_tip_command.block<4,1>(0,3).transpose() << '\n';

  // psm1_manip->joints_cmd=psm1_manip->inverse_kinematics(psm1_base.inverse()*psm1_tip_command);
  psm1_manip->joints_cmd=psm1_manip->inverse_kinematics(psm1_base.inverse()*psm1_tip);
  // std::cout << psm1_manip->joints_cmd.transpose() << '\n';
  // psm1_manip->clip_joints();
  psm1_manip->set_joints(psm1_manip->joints_cmd);

  std::cout << psm1_tip.block<4,1>(0,3).transpose() << '\n';
  // psm1_manip->joints_cmd(0,0)=psm1_manip->joints_cmd(0,0)+0.0001;
  // psm1_manip->PublishJoints();
  // std::cout << "out of teleop" << '\n';
  last_leftMasterPose=leftMasterPose;
  // ros::spinOnce();
}

void dvrk_library::LinkStateToEigen(Eigen::Matrix4d &mat, const geometry_msgs::Pose msg)
{
  // std::cout << "in link state" << '\n';
    mat(0,3) = msg.position.x;
    mat(1,3) = msg.position.y;
    mat(2,3) = msg.position.z;

    Eigen::Quaterniond q(msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z);
    Eigen::Matrix3d R = q.toRotationMatrix();
    mat(0,0)=R(0,0);
    mat(0,1)=R(0,1);
    mat(0,2)=R(0,2);
    mat(1,0)=R(1,0);
    mat(1,1)=R(1,1);
    mat(1,2)=R(1,2);
    mat(2,0)=R(2,0);
    mat(2,1)=R(2,1);
    mat(2,2)=R(2,2);
    // std::cout << "out of link state" << '\n';
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
