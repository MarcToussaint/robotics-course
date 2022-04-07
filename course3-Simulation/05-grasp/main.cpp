#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Geo/depth2PointCloud.h>

#include <Kin/feature.h>
#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

//===========================================================================

void grasp_the_hopping_ball(){
  //-- setup your RealWorld
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  //delete frames with certain names
  for(uint o=1;o<30;o++){
    rai::Frame *f = RealWorld[STRING("obj"<<o)];
    if(f) delete f;
  }

  rai::Frame *realObj = RealWorld["obj0"];
  realObj->setColor({1.,0,0}); //set the color of one objet to red!
  realObj->setShape(rai::ST_sphere, {.03});
//  realObj->setShape(rai::ST_ssBox, {.05, .05, .2, .01});
  realObj->setPosition({0., .2, 2.});
  realObj->setContact(1);

  rai::Simulation S(RealWorld, S._bullet, 1);
  S.cameraview().addSensor("camera");

  //add an imp!!
  S.addImp(S._objectImpulses, {"obj0"}, {});

  //-- setup your model world
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame *obj = C.addFrame("object");
  obj->setColor({1.,1.,0}); //set the color of one objet to red!
  obj->setShape(rai::ST_sphere, {.03});

  C.watch(false, "model world start state");

  //set the intrinsic camera parameters
  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};

  //get a reference to the camera frame
  rai::Frame *cameraFrame = C["camera"];

  //-- the following is the simulation loop
  arr q;
  byteA _rgb;
  floatA _depth;
  double tau = .01; //time step
  Metronome tic(tau);

  bool gripping = false;
  bool grasped = false;

  for(uint t=0;t<1000;t++){
    tic.waitForTic();

    //grab sensor readings from the simulation
    q = S.get_q();
    if(!(t%10)){
      S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow

      //--<< perception pipeline
    }

    //TOTAL CHEAT: grab the true position from the RealWorld
    arr objectPosition = realObj->getPosition();

    //set the model object to percept
    obj->setPosition(objectPosition);

    C.setJointState(q); //set your robot model to match the real q
    C.watch();

    //some good old fashioned IK
    auto diff = C.feature(FS_positionDiff, {"R_gripperCenter", "object"})->eval(C);
    auto vecX = C.feature(FS_scalarProductXZ, {"R_gripperCenter", "world"})->eval(C);

    //stack them
    arr y, J;

    y.append(1e0*diff); //multiply, to make faster
    J.append(1e0*diff.J());

    y.append(vecX-arr{0.}); //subtract target, here scalarProduct=0
    J.append(vecX.J());

    arr vel = 2.* pseudoInverse(J, NoArr, 1e-2) * (-y);

    C.watch();

    if(!gripping && length(diff) < .02){
      S.closeGripper("R_gripper", .05, .3);
      gripping = true;
    }

    if(gripping && S.getGripperIsGrasping("R_gripper")){
      cout <<"GRASPED!" <<endl;
      break;
    }

    //send no controls to the simulation
    S.step(vel, tau, S._velocity);
  }
  rai::wait();
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  grasp_the_hopping_ball();

  return 0;
}
