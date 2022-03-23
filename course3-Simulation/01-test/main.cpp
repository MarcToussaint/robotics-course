#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

#include <iomanip>

//===========================================================================

void testPushes(){
  rai::Configuration C;
  C.addFile("model.g");

  rai::Simulation S(C, S._bullet);
  S.cameraview().addSensor("camera");

  double tau=.01;
  Metronome tic(tau);
  byteA rgb;
  floatA depth;

  for(uint t=0;t<300;t++){
    tic.waitForTic();
    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    //some good old fashioned IK
    arr q = C.getJointState();
    arr diff = C.feature(FS_positionDiff, {"gripper", "box"})->eval(C);
    arr J = *diff.jac;
//    q -= 1e-1*inverse(~J*J+1e-2*eye(q.N))*~J*diff;
    diff *= .01/length(diff);
    q -= pseudoInverse(J, NoArr, 1e-2) * diff;
    S.step(q, tau, S._position);
//    S.step({}, tau, S._none);
  }
}

//===========================================================================

void testGrasp(){
  rai::Configuration C;
  C.addFile("model.g");

  arr q0 = C.getJointState();

  rai::Simulation S(C, S._bullet);
  S.cameraview().addSensor("camera");

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);
  for(uint t=0;;t++){
    tic.waitForTic();

    C.stepSwift();
//    C.reportProxies();

    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    arr q = C.getJointState();

    //some good old fashioned IK
    if(t>40 && t<=200){
      arr diff = C.feature(FS_positionDiff, {"gripperCenter", "ring4"})->eval(C);
      diff(1) -= 0.2;
      diff(2) -= 0.2;
      diff *= rai::MIN(.01/length(diff), 1.);

      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * diff;
    }
    if(t>200 && t<=400){
      arr diff = C.feature(FS_oppose, {"finger1", "finger2", "ring4"})->eval(C);
//      arr diff = C.feature(FS_positionDiff, {"gripperCenter", "ring4"})->eval(C);
      diff *= rai::MIN(.01/length(diff), 1.);

      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * diff;
    }

    if(t==400){
      S.closeGripper("gripper");
    }

    if(S.getGripperIsGrasping("gripper")){
      arr diff = C.feature(FS_position, {"gripper"})->eval(C);
      q += pseudoInverse(*diff.jac, NoArr, 1e-2) * ARR(0.01,0.01,.005);
    }

    if(t==700){
      S.openGripper("gripper");
    }

    if(t>700){
      q += 0.01*(q0-q);
    }

    if(t>1000 && S.getGripperIsOpen("gripper")){
      break;
    }

    S.step(q, tau, S._position);
  }
}

//===========================================================================
void testSliding(){
  rai::Configuration C;
  C.addFile("model.g");

  arr q0 = C.getJointState();

  rai::Simulation S(C, S._bullet);
  S.cameraview().addSensor("camera");

  byteA rgb;
  floatA depth;
  double tau=.01;
  Metronome tic(tau);
  for(uint t=0;;t++){
    tic.waitForTic();


    if(!(t%10)) S.getImageAndDepth(rgb, depth); //we don't need images with 100Hz, rendering is slow

    arr q = C.getJointState();

    //some good old fashioned IK
    if(t<=500){
      arr y = C.feature(FS_position, {"gripperCenter"})->eval(C);
      if(t<=300) q += pseudoInverse(*y.jac, NoArr, 1e-2) * ARR(0,0,-.01);
      else q += pseudoInverse(*y.jac, NoArr, 1e-2) * ARR(0.0,0.01,-.01);
    } else if(t<800){
      arr diff = C.feature(FS_positionDiff, {"gripperCenter", "stick"})->eval(C);
      if(t<700) diff(2) -= 0.3;
      diff *= rai::MIN(.02/length(diff), 1.);
      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * diff;
    } else if(t==800){
      S.closeGripper("gripper");
    }

    if(S.getGripperIsGrasping("gripper")){
      arr diff = C.feature(FS_positionDiff, {"stickTip", "target"})->eval(C);
      diff *= rai::MIN(.01/length(diff), 1.);
      q -= pseudoInverse(*diff.jac, NoArr, 1e-2) * diff;
    }

    if(t>1500){
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

  rai::wait();
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

  rai::wait();
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
  rai::wait();
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

  for(uint t=0;t<500;t++){
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
    obj->setMass(.1);
    obj->addAttribute("friction", .02*i);
  }

  for(int i=0;i<10;i++){
    rai::Frame *obj = C.addFrame(STRING("ball" <<i));
    arr size = {.05};
    obj->setShape(rai::ST_sphere, size);
    obj->setPosition({(i-5)*.2,-.1,2.});
    obj->setMass(.1);
    obj->addAttribute("restitution", .1*i);
  }

  rai::Frame *table = C.addFrame("table");
  table->setShape(rai::ST_ssBox, {3.,2.,.1, .02}).setColor({.3,.3,.3});
  table->setPosition({0,0,.6}).setQuaternion({1.,-.1,0.,0.}); //tilt the table!!
  table->addAttribute("friction", .1);


  rai::Simulation S(C, S._bullet);

  double tau=.01;
  Metronome tic(tau);

  int ppmCount=0;
  rai::system("mkdir -p z.vid/; rm -f z.vid/*.ppm");

  for(uint t=0;t<500;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
    write_ppm(S.getScreenshot(), STRING("z.vid/"<<std::setw(4)<<std::setfill('0')<<(ppmCount++)<<".ppm"));
  }

  rai::wait();
}

////===========================================================================

void testStackOfBlocks(){
  rai::Configuration C;

  for(int i=0;i<15;i++){
    rai::Frame *obj = C.addFrame(STRING("obj" <<i));
    arr size = {.2,.2,.2, .02};
    obj->setShape(rai::ST_ssBox, size);
    obj->setPosition({0.,0.,1.+i*.25});
    obj->setMass(1.); //does not seem to have much effect?
    obj->addAttribute("friction", 1.);
    obj->addAttribute("restitution", .0);
  }

  C.addFile("../../scenarios/pandasTable.g");

  rai::Simulation S(C, S._bullet);

  double tau=.01;  //jumps a bit for tau=.01
  Metronome tic(tau);

  for(uint t=0;t<5./tau;t++){
    tic.waitForTic();

    S.step({}, tau, S._none);
  }

  rai::wait();
}

////===========================================================================

void testBallOnMovingTable(){
  rai::Configuration C;

  rai::Frame *obj= C.addFrame("ball");
  obj->setShape(rai::ST_sphere, {.1});
  obj->setPosition({0.,0.1,1.});
  obj->setMass(.1);
  obj->setColor({.6,.4,.4});

  C.addFrame("table_base")->setPosition({0.,0.0,.5});
  rai::Frame *table = C.addFrame("table", "table_base");
  table->setShape(rai::ST_ssBox, {2.,2.,.1, .02});
  table->setJoint(rai::JT_hingeX);
  table->joint->limits = {-.5, .5};

  rai::Simulation S(C, S._bullet, 4);

  double tau=.01;
  Metronome tic(tau);
  for(uint t=0;t<5./tau;t++){
    tic.waitForTic();
    S.step({}, tau, S._none);
  }

  rai::wait();
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  testStackOfBlocks();
  testBallOnMovingTable();
  makeRndScene();
  testFriction();
  testPushes();

  testOpenClose();
  testGrasp();
  testSliding();


  return 0;
}
