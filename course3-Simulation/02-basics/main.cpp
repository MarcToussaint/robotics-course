#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

//===========================================================================

void basics(){
  //-- REAL WORLD configuration, which is attached to the physics engine
  // accessing this directly would be cheating!
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._physx, true);
  S.cameraview().addSensor("camera");

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  // what you know about the world and compute things (controls, contacts, etc)
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- the following is the simulation loop
  arr q;
  byteA rgb;
  floatA depth;
  double tau = .01; //time step

  for(uint t=0;t<300;t++){
    rai::wait(tau); //remove to go faster

    //grab sensor readings from the simulation
    q = S.get_q();
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    //some good old fashioned IK
    C.setJointState(q); //set your robot model to match the real q
    C.watch();
    Value diff = C.feature(FS_positionDiff, {"L_gripperCenter", "object"})->eval(C);
    arr vel = pseudoInverse(diff.J, NoArr, 1e-2) * (-diff.y);

    //send velocity controls to the simulation
    S.step(vel, tau, S._velocity);
  }
  rai::wait();
}


//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  basics();

  return 0;
}
