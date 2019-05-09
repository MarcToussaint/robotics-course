#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

void minimal_use(){

  Var<byteA> _rgb;
  Var<floatA> _depth;

#if 1 //using ros
  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");
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


void get_objects_into_configuration(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.}; //intrinsic camera parameters

  Depth2PointCloud d2p(_depth, Fxypxy);

  BaxterInterface B(true);

  rai::KinematicWorld C;
  C.addFile("model.g");

  rai::Frame *pcl = C.addFrame("pcl", "head");
  pcl->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
  pcl->calc_X_from_parent();

  for(uint i=0;i<10000;i++){
    _rgb.waitForNextRevision();

    arr q_real = B.get_q();
    if(q_real.N==C.getJointStateDimension())
      C.setJointState(q_real);

    if(d2p.points.get()->N>0){
      C.gl().dataLock.lock(RAI_HERE);
      pcl->setPointCloud(d2p.points.get(), _rgb.get());
      C.gl().dataLock.unlock();
      int key = C.watch(false);
      if(key=='q') break;
    }

    { //display
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }

    //how to convert image to 3D coordinates:
    double x_pixel_coordinate=0., y_pixel_coordinate=0., depth_from_depthcam=1.2;
    arr pt = { x_pixel_coordinate, y_pixel_coordinate, depth_from_depthcam };
    depthData2point(pt, Fxypxy); //transforms the point to camera xyz coordinates
    pcl->X.applyOnPoint(pt); //transforms into world coordinates
  }

//  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  minimal_use();
  get_objects_into_configuration();

  return 0;
}
