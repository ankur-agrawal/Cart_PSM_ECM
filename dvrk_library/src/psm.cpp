#include <psm.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "psm_node");
  ros::NodeHandle n;
  psm psm("psm", n);
  Eigen::MatrixXd joints(6,1);
  Eigen::MatrixXd joints_calculated(6,1);
  joints << 0,0,0.2,0,0.1,0.1;
  // // Eigen::Matrix4d psm_base_to_rcm;
  // // psm_base_to_rcm << 0,1,0,0,
  // //                   -1,0,0,0,
  // //                    0,0,1,0,
  // //                    0,0,0,1;
  // // psm.set_base_to_rcm(psm_base_to_rcm);
  // //
  // // Eigen::Matrix4d psm_world_to_base;
  // // psm_world_to_base << 1,0,0,0,
  // //                      0,1,0,0,
  // //                      0,0,1,0,
  // //                      0,0,0,1;
  // // psm.set_world_to_base(psm_world_to_base);
  // //
  // // Eigen::Matrix4d psm_tip_to_end;
  // // psm_tip_to_end << 0,-1,0,0,
  // //                   -1,0,0,0,
  // //                   0,0,-1,0,
  // //                   0,0,0,1;
  // // psm.set_tip_to_end(psm_tip_to_end);
  // //
  psm.set_joints(joints);
  Eigen::Matrix4d m=psm.ForwardKinematics();
  joints_calculated=psm.inverse_kinematics(m);
  std::cout << joints_calculated << '\n';

  return 0;
}
