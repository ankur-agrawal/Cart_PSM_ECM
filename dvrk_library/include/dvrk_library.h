#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <boost/python.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <ecm.h>

// #include <boost/python/module.hpp>
// #include <boost/python/def.hpp>
// #include <boost/chrono/include.hpp >

class dvrk_library{
 private:
   double t;
   ros::Publisher pub;
   ros::Subscriber sub;
   boost::thread* m_thread;
   boost::mutex *m_thread_mutex = new boost::mutex;
   bool run_thread;
   int iter=0;
 public:
   dvrk_library(){
     t=0;
     int argc=0;
     char** argv;
     ros::init(argc, argv, "py_cpp_node");
     ros::NodeHandle n;
     pub=n.advertise<std_msgs::Float64>("/publish_trial",1000);
     sub=n.subscribe("/gazebo/link_states", 1000, &dvrk_library::subscribe, this);
     run_thread=1;
     m_thread=new boost::thread(&dvrk_library::publish_thread, this);
     };

   void update(double v);
   int read();
   void exit_thread();
   void publish_thread();
   void subscribe(const gazebo_msgs::LinkStatesPtr &msg);
};

BOOST_PYTHON_MODULE(dvrk_library)
{
    using namespace boost::python;
    class_<dvrk_library>("init")
      .def("update", &dvrk_library::update)
      .def("read", &dvrk_library::read)
      .def("exit_thread", &dvrk_library::exit_thread)
    ;
}
