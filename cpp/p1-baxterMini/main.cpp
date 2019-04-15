#include <Kin/kin.h>
#include <RosCom/baxter.h>
#include <Operate/robotInterface.h>

void minimal_use(){
  rai::KinematicWorld K;
  K.addFile("../../rai-robotModels/baxter/baxter.g");
  arr q0 = K.getJointState();

  BaxterInterface B;
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
  arr q0 = K.getJointState();

  arr q = q0;
  q = 0.;

  RobotInterface B(K);
  cout <<"joint names: " <<B.getJointNames() <<endl;
  B.move({q,q0}, {4.,8.});
  B.move({q}, {12.}); //appends
  for(;;){
    rai::wait(.1);
    cout <<"q:" <<B.getJointPositions() <<endl;
    if(!B.timeToGo()) break;
  }
  rai::wait();

  q = q0;
  q(-1) = .1; //last joint set to .1: left gripper opens 10cm (or 20cm?)
  q(-2) = .05; //last joint set to .1: right gripper opens 10cm (or 20cm?)
  B.move({q}, {4.});
  B.wait();

  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  minimal_use();

  spline_use();

  return 0;
}
