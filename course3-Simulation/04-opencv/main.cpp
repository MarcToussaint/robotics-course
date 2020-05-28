#include <Perception/opencv.h> //always include this first! OpenCV headers define stupid macros
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

#include <Kin/frame.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>

//===========================================================================

void minimal_use_with_webcam(){
  Var<byteA> _rgb;       // (beyond this course: a 'Var<type>' is mutexed data, into which threads (like a cam) can write)
  Var<floatA> _depth;

#if 0 //using ros
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
  //  RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);
#else //using a webcam
  OpencvCamera cam(_rgb);
#endif

  //looping images through opencv
  for(uint i=0;i<1000;i++){
    cout <<i <<endl;
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0)   cv::imshow("OPENCV - rgb", rgb);
      if(depth.total()>0) cv::imshow("OPENCV - depth", 0.5*depth); //white=2meters
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
  }
}

//===========================================================================

void use_within_simulation(){
  //-- basic setup
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  RealWorld.getFrameByName("obj1")->setColor({1.,0,0}); //set the color of one objet to red!

  rai::Simulation S(RealWorld, S._physx, true);
  S.cameraview().addSensor("camera");

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state");


  // NEW: set the intrinsic camera parameters
  double f = 0.895;
  f *= 360.; //focal length is needed in pixels (height!), not meters!
  arr Fxypxy = {f, f, 320., 180.};

  //get a reference to the camera frame
  rai::Frame *cameraFrame = C["camera"];
//  cameraFrame->setPose(d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");

  //-- the following is the simulation loop
  arr q;
  byteA _rgb;
  floatA _depth;
  arr points;
  double tau = .01; //time step

  //move the robot to become visible
  q = S.get_q();;
  q(0) = -1.;
  q(1) = .5;
  S.step(q, tau, S._position);

  for(uint t=0;t<300;t++){
    rai::wait(tau); //remove to go faster

    //grab sensor readings from the simulation
    q = S.get_q();
    if(!(t%10)){
      S.getImageAndDepth(_rgb, _depth); //we don't need images with 100Hz, rendering is slow

      // NEW
      depthData2pointCloud(points, _depth, Fxypxy);
      cameraFrame->setPointCloud(points, _rgb);
      V.recopyMeshes(C); //update the model display!
      V.setConfiguration(C);

      //start working in CV, display the images there
      {
        cv::Mat rgb = CV(_rgb);
        cv::Mat depth = CV(_depth);

        if(rgb.total()>0)   cv::imshow("OPENCV - rgb", rgb);
        if(depth.total()>0) cv::imshow("OPENCV - depth", 0.5*depth); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }

    }

    //send no controls to the simulation
    S.step({}, tau, S._none);
  }
  rai::wait();
}

//===========================================================================

void multipleCameras(){
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");

  //change the position of the central sensor
  rai::Frame* f = RealWorld.getFrameByName("camera");
  f->setPosition(f->getPosition() + arr{0.,0.,.5});

  //add a frame for the additional camera
  f = RealWorld.addFrame("R_gripperCamera", "R_gripper");
  f->setRelativePosition({.0, .1, 0.});
  f->setShape(rai::ST_marker, {.5});

  rai::Simulation S(RealWorld, S._physx, true);
  S.cameraview().addSensor("camera"); //camera is a pre-existing frame that specifies the intrinsic camera parameter
  S.cameraview().addSensor("Rcamera", "R_gripperCamera", 640, 360, 1.); //R_gripperCamera is a fresh frame - we have to specify intrinsic parameters explicitly

  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::ConfigurationViewer V;
  V.setConfiguration(C, "model world start state");

  byteA _rgb;
  floatA _depth;

  for(uint k=0;k<5;k++){
    //get images from the wrist
    S.cameraview().selectSensor("Rcamera");
    S.getImageAndDepth(_rgb, _depth);
    rai::wait();

    //get images from the main sensor
    S.cameraview().selectSensor("camera");
    S.getImageAndDepth(_rgb, _depth);
    rai::wait();
  }

}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  minimal_use_with_webcam();
//  use_within_simulation();
  multipleCameras();

  return 0;
}
