#include <Kin/kin.h>
#include <RosCom/baxter.h>
#include <Operate/robotOperation.h>

void minimal_use(){
  //load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter.g");

  //define a home and zero pose
  arr q_home = C.getJointState();
  arr q_zero = 0.*q_home;

  //launch the interface
  BaxterInterface B(true);

  for(uint i=0;i<40;i++){
    rai::wait(.1);
    B.send_q(q_home); //repeatedly send q_home as reference -> moves
    cout <<B.get_q() <<endl;
    cout <<B.get_qdot() <<endl;
    cout <<B.get_u() <<endl;
  }

  //just once send q_zero as reference -> will hardly move
  B.send_q(q_zero);

  C.setJointState(q_zero);
  C.watch(true);
}


void spline_use(){
  //load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter.g");

  //define a home and zero pose
  arr q_home = C.getJointState();
  arr q_zero = 0.*q_home;

  //launch the interface
  RobotOperation B(C);
  cout <<"joint names: " <<B.getJointNames() <<endl;
  B.sendToReal(true);

  //spline motion of the reference
  B.move({q_zero}, {10.});
  B.wait();

  //output states
  for(;;){
    cout <<" q:" <<B.getJointPositions()
        <<" gripper right:" <<B.getGripperOpened("right") <<' ' <<B.getGripperGrabbed("right")
       <<" gripper left:" <<B.getGripperOpened("left") <<' ' <<B.getGripperGrabbed("left")
      <<endl;
    if(!B.timeToGo()) break;
    rai::wait(.1);
  }
  cout <<"motion done!" <<endl;
  rai::wait();

  //instantaneous move of the reference (baxter does interpolation)
  B.moveHard(q_home);
  rai::wait();

  //close right gripper
  q_home(-2) = 1.;
  B.moveHard(q_home);
  rai::wait();

  //open right gripper
  q_home(-2) = 0.;
  B.moveHard(q_home);
  rai::wait();

  //close left gripper
  q_home(-1) = 1.;
  B.moveHard(q_home);
  rai::wait();

  //open left gripper
  q_home(-1) = 0.;
  B.moveHard(q_home);
  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  minimal_use();

  spline_use();

  return 0;
}
