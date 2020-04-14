#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>
#include <Kin/cameraview.h>

//===========================================================================

void minimal_use(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

#if 1 //using ros
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
  //  RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);
#else //using a webcam
  OpencvCamera cam(_rgb);
#endif

  //looping images through opencv
  for(uint i=0;i<1000;i++){
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }
  }
}

//===========================================================================

void wrist_camera(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

 RosCamera kin(_rgb, _depth, "cameraRosNodeMarc", "/cameras/right_hand_camera/image", "", true);

  //looping images through opencv
  for(uint i=0;i<1000;i++){
    _rgb.waitForNextRevision();
    {
      byteA RGB = _rgb.get();
      cv::Mat rgb = cv::Mat(RGB.d0, RGB.d1, CV_8UC4, RGB.p);
      cv::cvtColor(rgb, rgb, cv::COLOR_BGRA2RGBA);

      if(rgb.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }
  }
}

//===========================================================================

void get_objects_into_configuration(){
  //subscribe to image and depth
  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  //set the intrinsic camera parameters
  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.};

  //convert to point cloud (in camera frame, used only for display)
  Depth2PointCloud d2p(_depth, Fxypxy);

  //interface to baxter to query it's real pose
  BaxterInterface B(true);

  //load a configuration
  rai::Configuration C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");

  //add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->set_Q()->setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");

  for(uint i=0;i<10000;i++){
    //wait for the next rgb image
    _rgb.waitForNextRevision();

    //sync C with the real baxter pose
    arr q_real = B.get_q();
    if(q_real.N==C.getJointStateDimension())
      C.setJointState(q_real);

    //display the point cloud by setting it as cameraFrame shape
    if(d2p.points.get()->N>0){
      cameraFrame->setPointCloud(d2p.points.get(), _rgb.get());
      int key = C.watch(false);
      if(key=='q') break;
    }

    //display the images
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }

    //example on how to convert image to 3D coordinates:
    double x_pixel_coordinate=0., y_pixel_coordinate=0., depth_from_depthcam=1.2;
    arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
    depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
    cameraFrame->ensure_X().applyOnPoint(pt); //transforms into world coordinates
  }

//  rai::wait();
}

//===========================================================================

void usingCameraSimulation(){
  //load a configuration
  rai::Configuration C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");

  //add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->set_Q()->setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");

  //associate an opengl renderer with the camera frame
  rai::CameraView camSim(C);
  camSim.addSensor("myCam", "camera", 640, 480, 1.);
  camSim.selectSensor("myCam");

  //add an object into the rendered scene
  rai::Frame *realWorldObj = camSim.C.addFrame("obj");
  realWorldObj->setPosition({1., 0., 1.});
  realWorldObj->setShape(rai::ST_ssBox, {.1, .2, .3, .01});
  realWorldObj->setColor({.1, .7, .1});

  //compute things
  byteA rgb;
  floatA depth;
  arr pcl;
  camSim.computeImageAndDepth(rgb, depth);
  camSim.computePointCloud(pcl, depth, false);

  //display the images
  {
    cv::Mat _rgb = CV(rgb);
    cv::Mat _depth = CV(depth);
    cv::cvtColor(_rgb, _rgb, cv::COLOR_BGR2RGB);

    if(_rgb.total()>0 && _depth.total()>0){
      cv::imshow("rgb", _rgb.clone()); //white=2meters
      cv::imshow("depth", 0.5*_depth.clone()); //white=2meters
      int key = cv::waitKey(100);
//      if((key&0xff)=='q') break;
    }
  }

  //add the rendered point cloud as shape of the cameraFrame in configuration C
  cameraFrame->setPointCloud(pcl, rgb);
  //...and display it
  C.watch(true);
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  minimal_use();
//  wrist_camera();
//  get_objects_into_configuration();
//  usingCameraSimulation();

  return 0;
}
