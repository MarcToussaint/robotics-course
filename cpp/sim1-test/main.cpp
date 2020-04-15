#include <Kin/kin.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>

//===========================================================================

void test(){
  rai::Configuration C;
  C.addFile("model.g");
  C.watch(true);

//  rai::Simulation S(C, S._bullet, true);
  rai::Simulation S(C, S._physx, true);

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  arr Xstart = C.getFrameState();

  for(uint k=0;k<100;k++){

    //restart from the same state multiple times
    S.setState(Xstart);

    for(uint t=0;t<300;t++){
      S.step({}, tau, S._none);

      if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow
      tic.waitForTic();

      //some good old fashioned IK
      arr q = C.getJointState();
      Value diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
      diff.y *= .005/length(diff.y);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * diff.y;
      C.setJointState(q);

      //a crazy disturbance, lifting the box suddenly
      if(!(t%100)){
        arr p = C["box"]->getPosition();
        p(0) += .05;
        p(2) += .2;
        C["box"]->setPosition(p);

        S.setState(C.getFrameState());
      }
    }
  }
}

//===========================================================================

void testGrasp(){
  rai::Configuration C;
  C.addFile("model.g");

  C.selectJointsByName({"finger1", "finger2"}, true);

//  rai::Simulation S(C, S._bullet, true);
  rai::Simulation S(C, S._physx, true);

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;t<900;t++){
    C.stepSwift();
//    C.reportProxies();

    S.step({}, tau, S._none);
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow
    tic.waitForTic();

    //some good old fashioned IK
    if(t<=300){
      arr q = C.getJointState();
      Value diff = C.feature(FS_oppose, {"finger1", "finger2", "ring4"})->eval(C);
      diff.y *= rai::MIN(.008/length(diff.y), 1.);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * diff.y;
      C.setJointState(q);
    }

    if(t==300){
      S.closeGripper("gripper");
    }

    if(t>300){
      arr q = C.getJointState();
      Value diff = C.feature(FS_position, {"gripper"})->eval(C);
      q -= pseudoInverse(diff.J, NoArr, 1e-2) * ARR(0.,0.,-2e-4);
      C.setJointState(q);
    }

    if(t==600){
      S.openGripper("gripper");
    }
  }
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  test();
  testGrasp();

  return 0;
}
