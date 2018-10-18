#include <Kin/roboticsCourse.h>

void testDynamics(){
  Simulator S("pin_balancing.ors");
  
  arr q,qdot,qddot;
  arr M,Minv,F,u;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot.setZero();
  u=qdot;
  
  double tau = .01; //duration of one time step = .01sec
  
  for(uint i=0;i<1000;i++){
    u = -0.1 * qdot; //a little friction
    S.stepDynamics(u, tau);
    S.watch(false);
    S.getJointAnglesAndVels(q, qdot);
  }
  
  S.watch();
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);
  testDynamics();
  return 0;
}
