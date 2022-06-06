#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

#include <KOMO/komo.h>

#include <iomanip>

//===========================================================================

void testPushes(){
  rai::Configuration C;
  C.addFile("model.g");

  rai::Simulation S(C, S._bullet);

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  arr Xstart = C.getFrameState();

  for(uint k=0;k<5;k++){

    //restart from the same state multiple times
    S.setState(Xstart);

    for(uint t=0;t<300;t++){
      tic.waitForTic();
      if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

      //some good old fashioned IK
      arr q = C.getJointState();
      arr diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
      diff *= .008/length(diff);
      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * diff;

      S.step(q, tau, S._position);

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

  rai::Simulation S(C, S._bullet);

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;;t++){
    tic.waitForTic();

//    C.stepSwift();
//    C.reportProxies();

    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    arr q = C.getJointState();

    //some good old fashioned IK
    if(t>40 && t<=300){
      arr diff = C.feature(FS_oppose, {"finger1", "finger2", "ring4"})->eval(C);
      diff *= rai::MIN(.008/length(diff), 1.);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * diff;
    }

    if(t==300){
      S.closeGripper("gripper");
    }

    if(S.getGripperIsGrasping("gripper")){
      arr diff = C.feature(FS_position, {"gripper"})->eval(C);
      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * ARR(0.,0.,-2e-3);
    }

    if(t==600){
      S.openGripper("gripper");
    }

    if(t>1000 && S.getGripperIsOpen("gripper")){
      break;
    }

    S.step(q, tau, S._position);
  }
}

//===========================================================================

void testGrasp2(){
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  C.addFrame("box")->setShape(rai::ST_ssBox, {.8, .05, .05, .01})\
      .setPosition({0,0,1.}).setMass(1).setContact(1);

  rai::Simulation S(C, S._bullet);

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;;t++){
    tic.waitForTic();

    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    arr q = C.getJointState();

    //some good old fashioned IK
    if(t>40 && t<=300){
      arr diff = C.feature(FS_positionRel, {"R_gripperCenter", "box"})->eval(C);
      diff(2) -= 0.01;
      diff *= rai::MIN(.01/length(diff), 1.);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * diff;
    }

    if(t==300){
      S.closeGripper("R_gripper");
    }

    if(S.getGripperIsGrasping("R_gripper")){
      arr diff = C.feature(FS_position, {"R_gripper"})->eval(C);
      q -= pseudoInverse(diff.J(), NoArr, 1e-2) * ARR(0.,0.,-2e-3);
    }

    if(t==600){
      S.openGripper("R_gripper");
    }

    if(t>1000 && S.getGripperIsOpen("R_gripper")){
      break;
    }

    S.step(q, tau, S._position);
  }
}
//===========================================================================

void testOpenClose(){
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._bullet);

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  C.watch(true);

  double tau = .01;

  S.closeGripper("R_gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.watch();

    S.step({}, tau, S._none);
    cout <<S.getGripperWidth("R_gripper") <<endl;
    if(S.getGripperIsClose("R_gripper")) break;
  }

  S.openGripper("R_gripper");
  for(uint t=0;;t++){
    rai::wait(tau);

    arr q = S.get_q();
    C.setJointState(q);
    C.watch();

    S.step({}, tau, S._none);
    cout <<S.getGripperWidth("R_gripper") <<endl;
    if(S.getGripperIsOpen("R_gripper")) break;
  }
}

//===========================================================================

void makeRndScene(){
  rai::Configuration C;

  for(uint i=0;i<30;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {rnd.uni(.2,.8), rnd.uni(.1,.4), rnd.uni(.05,.2), .01};
    obj->setShape(rai::ST_ssBox, size);
    rai::Transformation pose;
    pose.setRandom();
    pose.pos.y *= .3;
    pose.pos.y += .5;
    pose.pos.z += 2.;
    obj->setPose(pose);
    obj->setMass(.2);
  }

  FILE("z.rndObjects.g") <<C; //write configuration into a file

  C.addFile("../../scenarios/pandasTable.g");

  rai::Simulation S(C, S._bullet);
//  rai::Simulation S(C, S._physx);
  S.cameraview().addSensor("camera");

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;t<300;t++){
    tic.waitForTic();
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    S.step({}, tau, S._none);

    cout <<"depth in range: " <<depth.min() <<' ' <<depth.max() <<endl;
  }

  C.sortFrames();
  FILE("z.g") <<C; //write configuration into a file

  rai::wait();
}

//===========================================================================

void testFriction(){
  rai::Configuration C;

  for(int i=0;i<10;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {.1,.1,.1, .01};
    obj->setShape(rai::ST_ssBox, size);
    obj->setPosition({(i-5)*.2,0.,1.});
    obj->setMass(.2);
    obj->addAttribute("friction", .02*i);
  }

  for(int i=0;i<10;i++){
    rai::Frame *obj = C.addFrame(STRING("ball" <<i));
    arr size = {.05};
    obj->setShape(rai::ST_sphere, size);
    obj->setPosition({(i-5)*.2,.5,2.});
    obj->setMass(.2);
    obj->addAttribute("restitution", .1*i);
  }

  C.addFile("../../scenarios/pandasTable.g");

  C["table"]->setQuaternion({1.,-.1,0.,0.}); //tilt the table!!
  C["table"]->addAttribute("restitution", .5);

  rai::Simulation S(C, S._bullet);
  S.cameraview().addSensor("camera");

  double tau=.01;
  Metronome tic(tau);

  int ppmCount=0;
  rai::system("mkdir -p z.vid/; rm -f z.vid/*.ppm");

  for(uint t=0;t<300;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
    write_ppm(S.getScreenshot(), STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<(ppmCount++)<<".ppm"));
  }

  rai::wait();
}

//===========================================================================

void testStackOfBlocks(){
  rai::Configuration C;

  for(int i=0;i<7;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {.2,.2,.2, .02};
    obj->setShape(rai::ST_ssBox, size);
    obj->setPosition({0.,0.,1.+i*.25});
    obj->setMass(1.); //does not seem to have much effect?
//    obj->addAttribute("friction", 1.);
//    obj->addAttribute("restitution", .01);
  }

  C.addFile("../../scenarios/pandasTable.g");

  rai::Simulation S(C, S._bullet);
//  rai::Simulation S(C, S._physx);

  double tau=.01;  //jumps a bit for tau=.01
  Metronome tic(tau);

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
  }

  rai::wait();
}

//===========================================================================

void testBlockOnMoving(){
  rai::Configuration C;

  rai::Frame *obj= C.addFrame("block");
  obj->setShape(rai::ST_ssBox, {.2,.2,.2, .02});
  obj->setPosition({0.,0.,1.});
  obj->setMass(1.);
  obj->setColor({.7,.3,.3});

  C.addFrame("world");
  rai::Frame *table = C.addFrame("table", "world");
  table->setShape(rai::ST_ssBox, {2.,2.,.1, .02});
  table->setPosition({0.,0.,.5});
  table->setJoint(rai::JT_rigid); //COMMENT THIS LINE, AND YOU'LL SEE THE ISSUE

  rai::Simulation S(C, S._bullet, 4);

  double tau=.01;
  Metronome tic(tau);

  for(uint t=0;t<4./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
    arr pos = table->getPosition();
    pos(0) += .01;
    table->setPosition(pos);

  }

  rai::wait();
}

void testNoPenetrationImp(){
  rai::Configuration C0;
  C0.addFile("../../scenarios/pandasTable.g");
  rai::Frame* stick = C0.addFrame("stick");
  stick->setShape(rai::ST_capsule, {0.5, 0.025});
  stick->setPosition({0,0,.68}).set_X()->addRelativeRotationDeg(90.,0,1,0);
  stick->setContact(1);
  stick->setMass(0.1);

  rai::Frame* tip = C0.addFrame("stickTip", "stick");
  tip->setShape(rai::ST_marker, {0.1});
  tip->setRelativePosition({0,0,-0.25});

  KOMO komo;                     //create a solver
  komo.setModel(C0);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 1, 1., 1);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)
  komo.addObjective({}, FS_positionRel, {"R_gripperCenter", "stick"}, OT_eq, {1e2}, {0, 0, 0.2});
  komo.addObjective({}, FS_scalarProductXZ, {"R_gripperCenter", "stick"}, OT_eq, {1e2});
  komo.addObjective({}, FS_vectorZ, {"R_gripperCenter"}, OT_eq, {1e2}, {0,0,1});

  komo.optimize();
  komo.view(true);

  C0.setJointState(komo.getConfiguration_qAll(0));

  for(bool activate: {false, true}){
    rai::Configuration C;
    C.copy(C0);
    rai::Simulation S(C, S._bullet);
    if(activate) S.addImp(S._noPenetrations, {}, {});
    double tau=.01;
    Metronome tic(tau);

    S.closeGripper("R_gripper");
    while(!S.getGripperIsGrasping("R_gripper")){
      tic.waitForTic();
      S.step({}, tau, S._none);
    }

    for(uint t=0;t<1./tau;t++){
      tic.waitForTic();
      arr y = C.feature(FS_position, {"R_gripper"})->eval(C);

      arr Jt = ~y.J();
      arr vel = inverse(Jt*y.J()+eye(C.getJointStateDimension())) * Jt * ARR(0,0,1.);
      S.step(vel, tau, S._velocity);
    }


    for(uint t=0;t<2./tau;t++){
      tic.waitForTic();
      arr y = C.feature(FS_position, {"stickTip"})->eval(C);

      arr Jt = ~y.J();
      arr vel = inverse(Jt*y.J()+eye(C.getJointStateDimension())) * Jt * ARR(0,0,-1.);
      S.step(vel, tau, S._velocity);
    }
  }
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

//  testStackOfBlocks();
//  testPushes();
//  testGrasp();
//  testGrasp2();
//  testOpenClose();
//  makeRndScene();
//  testFriction();
//  testBlockOnMoving();
  testNoPenetrationImp();

  return 0;
}
