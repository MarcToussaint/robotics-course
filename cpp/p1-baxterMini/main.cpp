#include <Kin/kin.h>
#include <RosCom/baxter.h>
#include <Operate/robotOperation.h>

void minimal_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  arr q0 = K.getJointState();

  BaxterInterface B(true);
  B.send_q(q0);

  for(uint i=0;i<10;i++){
    rai::wait(.1);
    cout <<B.get_q() <<endl;
    cout <<B.get_qdot() <<endl;
    cout <<B.get_u() <<endl;
  }
  K.watch(true);

  arr q = q0;
  q = 0.;
  K.setJointState(q);
  B.send_q(q);
  K.watch(true);
}


void spline_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  K.addObject("object", rai::ST_capsule, {.2, .05}, {1., 1., 0.}, -1., 0, {.8, .0, 1.});
  arr q_home = K.getJointState();

  arr q_zero = 0.*q_home;

  RobotOperation B(K);
  cout <<"joint names: " <<B.getJointNames() <<endl;
  B.move({q_zero,q_home}, {5.,10.});
  B.move({q_zero}, {15.}); //appends
  B.wait();
  rai::wait();

  q_home(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  B.move({q_home}, {4.});
  B.wait();

  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  minimal_use();

  spline_use();

  return 0;
}
