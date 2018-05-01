#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include "ecm.h"
#include "psm.h"

// #include <boost/python/module.hpp>
// #include <boost/python/def.hpp>
// #include <boost/chrono/include.hpp >

class dvrk_library{
 private:
   double t;
   ros::Publisher pub;
   ros::Subscriber sub, sub_laprotek_left, sub_laprotek_right, sub_oculus;
   boost::thread* m_thread;
   boost::mutex *m_thread_mutex = new boost::mutex;
   bool run_thread;
   bool exit;
   int iter=0;

   ecm *ecm_manip;
   psm *psm1_manip;
   psm *psm2_manip;

   Eigen::Matrix4d ecm_base;
   Eigen::Matrix4d ecm_tip;
   Eigen::Matrix4d psm1_base;
   Eigen::Matrix4d psm1_tip;
   Eigen::Matrix4d psm2_base;
   Eigen::Matrix4d psm2_tip;
   Eigen::Matrix4d leftMasterPose;
   Eigen::Matrix4d rightMasterPose;
   Eigen::Matrix4d last_leftMasterPose;
   Eigen::Matrix4d last_rightMasterPose;
   Eigen::MatrixXd psm1_joints;
   Eigen::MatrixXd psm2_joints;
   Eigen::MatrixXd ecm_joints;
 public:
   dvrk_library()
   {
     t=0;
     int argc=0;
     char** argv;
     exit=0;
     ros::init(argc, argv, "dvrk_library_node");
     ros::NodeHandle n;

     ecm_manip= new ecm("ecm", n);
     psm1_manip = new psm("PSM1", n);
     // psm2_manip = new psm("PSM2", n);
     // sleep(1);

     sub=n.subscribe("/gazebo/link_states", 1000, &dvrk_library::subscribe, this);

     boost::function<void (const geometry_msgs::PoseStampedPtr)>LeftLaprotek(boost::bind(&dvrk_library::subscribeMaster,this, _1,0));
     boost::function<void (const geometry_msgs::PoseStampedPtr)>RightLaprotek(boost::bind(&dvrk_library::subscribeMaster,this, _1,1));
     sub_laprotek_left=n.subscribe<geometry_msgs::PoseStamped>("/Laprotek/Lefthandle/Pose", 1000, LeftLaprotek);
     sub_laprotek_right=n.subscribe<geometry_msgs::PoseStamped>("/Laprotek/Righthandle/Pose", 1000, RightLaprotek);

     sub_oculus=n.subscribe("/openhmd/pose", 1000, &dvrk_library::subscribeOculus, this);
     // m_thread=new boost::thread(&dvrk_library::publish_thread, this);

     ecm_base=Eigen::Matrix4d::Identity(4,4);
     ecm_tip=Eigen::Matrix4d::Identity(4,4);
     psm1_base=Eigen::Matrix4d::Identity(4,4);
     psm1_tip=Eigen::Matrix4d::Identity(4,4);
     psm2_base=Eigen::Matrix4d::Identity(4,4);
     psm2_tip=Eigen::Matrix4d::Identity(4,4);
     leftMasterPose=Eigen::Matrix4d::Identity(4,4);
     rightMasterPose=Eigen::Matrix4d::Identity(4,4);
     last_leftMasterPose=Eigen::Matrix4d::Identity(4,4);
     last_rightMasterPose=Eigen::Matrix4d::Identity(4,4);
     run_thread=1;

     psm1_manip->set_init_joints(-0.5,0,0.1,0,0,0);
     
   };

   void update();
   int read();
   void exit_thread();
   // void publish_thread();
   void subscribe(const gazebo_msgs::LinkStatesPtr &msg);
   void subscribeMaster(const geometry_msgs::PoseStampedPtr &msg, int arm);
   void subscribeOculus(const geometry_msgs::PoseStampedPtr &msg);
   void LinkStateToEigen(Eigen::Matrix4d &mat, const geometry_msgs::Pose msg);
   void teleop();
   bool exit_python();

};

BOOST_PYTHON_MODULE(dvrk_library)
{
    using namespace boost::python;
    class_<dvrk_library>("init")
      .def("teleop", &dvrk_library::teleop)
      .def("update", &dvrk_library::update)
      .def("read", &dvrk_library::read)
      .def("exit_thread", &dvrk_library::exit_thread)
      .def("quit", &dvrk_library::exit_python)
    ;
}
