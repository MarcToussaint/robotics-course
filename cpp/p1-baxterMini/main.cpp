#include <Kin/kin.h>
#include <RosCom/baxter.h>

void reach(){
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
  for(uint i=0;i<50;i++){
    B.send_q(q);
    rai::wait(.1);
  }
  K.watch(true);

}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  reach();

  return 0;
}
