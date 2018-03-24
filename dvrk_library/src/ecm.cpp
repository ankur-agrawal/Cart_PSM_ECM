#include <ecm.h>

int main(int argc, char **argv)
{
  ecm ECM("ecm");
  Eigen::MatrixXd joints(4,1);
  Eigen::MatrixXd joints_calculated(4,1);
  joints << 0.1, 0.2, 0.3,0.4;
  Eigen::Matrix4d ecm_base_to_rcm;
  ecm_base_to_rcm << 0,1,0,0.612599,
                     -1,0,0,0,
                      0,0,1,0.101595,
                      0,0,0,1;
  ECM.set_base_to_rcm(ecm_base_to_rcm);
  Eigen::Matrix4d ecm_world_to_base;
  ecm_world_to_base << 1,0,0,0,
                       0,1,0,0,
                       0,0,1,0,
                       0,0,0,1;
  ECM.set_world_to_base(ecm_world_to_base);

  Eigen::Matrix4d ecm_tip_to_end;
  ecm_tip_to_end << 0,-1,0,0,
                    -1,0,0,0,
                    0,0,-1,0,
                    0,0,0,1;
  ECM.set_tip_to_end(ecm_tip_to_end);

  ECM.set_joints(joints);
  Eigen::Matrix4d m=ECM.ForwardKinematics();
  joints_calculated=ECM.inverse_kinematics(m);
  std::cout << joints_calculated << '\n';
  return 0;
}
