// #include "trial.h"
#include "server.h"


template<typename C>
class RosBridge{
  typedef void (C::*my_func_ptr)( geometry_msgs::PoseStamped&,  geometry_msgs::PoseStamped&);
  typedef void (C::*my_func_ptr2)( sensor_msgs::JointState&);
  typedef void (C::*my_func_ptr3)( geometry_msgs::Wrench&,  geometry_msgs::Wrench&);

private:
   geometry_msgs::PoseStamped leftPose;
   geometry_msgs::PoseStamped rightPose;
   geometry_msgs::Wrench leftWrench;
   geometry_msgs::Wrench rightWrench;
   sensor_msgs::JointState Joints;
   my_func_ptr func;
   my_func_ptr2 func2;
   my_func_ptr3 func3;
   C *obj;

  ros::Publisher leftHandlePosePub;
  ros::Publisher rightHandlePosePub;
  ros::Publisher HandleJointsPub;
  ros::Subscriber leftWrenchSub;
  ros::Subscriber rightWrenchSub;

public:
  RosBridge(C *class_obj){
  obj=class_obj;
  leftWrench.force.x=0;
  leftWrench.force.y=0;
  leftWrench.force.z=0;
  leftWrench.torque.x=0;
  leftWrench.torque.y=0;
  leftWrench.torque.z=0;
  rightWrench.force.x=0;
  rightWrench.force.y=0;
  rightWrench.force.z=0;
  rightWrench.torque.x=0;
  rightWrench.torque.y=0;
  rightWrench.torque.z=0;
  }
  void setFuncPtr(my_func_ptr function);
  void setFuncPtr2(my_func_ptr2 function);
  void setFuncPtr3(my_func_ptr3 function);
  // void setObject(C obj);
  void init();
  void setPoses();
  void setJoints();
  void setWrenches();
  void print();
  void publishPoses();
  void publishJoints();
  void subscribeWrench(const geometry_msgs::WrenchStampedPtr &msg, int arm);
};

template<typename C>void RosBridge<C>::setFuncPtr(my_func_ptr function)
{
  func=function;
}

template<typename C>void RosBridge<C>::setFuncPtr2(my_func_ptr2 function)
{
  func2=function;
}

template<typename C>void RosBridge<C>::setFuncPtr3(my_func_ptr3 function)
{
  func3=function;
}

template<typename C>void RosBridge<C>::setPoses()
{
  (obj->*func)(leftPose, rightPose);
}

template<typename C>void RosBridge<C>::setJoints()
{
  (obj->*func2)(Joints);
}

template<typename C>void RosBridge<C>::setWrenches()
{
  (obj->*func3)(leftWrench, rightWrench);
}

template<typename C>void RosBridge<C>::print()
{
  std::cout << Joints.position[0] << '\t' << Joints.position[1] << '\t' << Joints.position[2] << '\t' << Joints.position[3] << '\t' << Joints.position[4] << '\t' << Joints.position[5] << '\t' << Joints.position[6] << '\t' << '\n';
  std::cout << Joints.position[7] << '\t' << Joints.position[8] << '\t' << Joints.position[9] << '\t' << Joints.position[10] << '\t' << Joints.position[11] << '\t' << Joints.position[12] << '\t' << Joints.position[13] << '\t' << '\n';
}

template<typename C>void RosBridge<C>::init()
{
  int argc;
  char** argv;
  // ros::M_string s;
  ros::init(argc, argv, "laprotek_comm_node");
  ros::NodeHandle n;
  leftHandlePosePub = n.advertise<geometry_msgs::PoseStamped>("/Laprotek/LeftHandle/Pose",1000);
  rightHandlePosePub = n.advertise<geometry_msgs::PoseStamped>("/Laprotek/RightHandle/Pose",1000);
  HandleJointsPub = n.advertise<sensor_msgs::JointState>("/joint_states",1000);
  boost::function<void (const geometry_msgs::WrenchStampedPtr)>leftWrenchFunc(boost::bind(&RosBridge<C>::subscribeWrench,this, _1,0));
  boost::function<void (const geometry_msgs::WrenchStampedPtr)>rightWrenchFunc(boost::bind(&RosBridge<C>::subscribeWrench,this, _1,1));
  leftWrenchSub = n.subscribe<geometry_msgs::WrenchStamped>("/Laprotek/LeftHandle/Wrench", 1000, leftWrenchFunc);
  rightWrenchSub = n.subscribe<geometry_msgs::WrenchStamped>("/Laprotek/RightHandle/Wrench", 1000, rightWrenchFunc);
}

template<typename C>void RosBridge<C>::publishPoses()
{
  leftHandlePosePub.publish(leftPose);
  rightHandlePosePub.publish(rightPose);
}

template<typename C>void RosBridge<C>::publishJoints()
{
  HandleJointsPub.publish(Joints);

}

template<typename C>void RosBridge<C>::subscribeWrench(const geometry_msgs::WrenchStampedPtr &msg, int arm)
{
  if (arm==0)
  {
    leftWrench=msg->wrench;
  }
  if (arm==1)
  {
    rightWrench=msg->wrench;
  }
}
