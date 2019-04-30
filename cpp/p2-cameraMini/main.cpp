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
  for(uint i=0;i<100;i++){
    _rgb.waitForNextRevision();
    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }
  }
}

void get_objects_into_configuration(){
  Var<byteA> _rgb;
  Var<floatA> _depth;

  RosCamera cam(_rgb, _depth, "cameraRosNode", "/camera/rgb/image_rect_color", "/camera/depth_registered/image_raw");

  Depth2PointCloud d2p(_depth, 600.f, 600.f, 320.f, 240.f);

  BaxterInterface B(true);

  rai::KinematicWorld C;
  C.addFile("model.g");
  C.setJointState(B.get_q());

  rai::Frame *pcl = C.addFrame("pcl", "camera");
  for(uint i=0;i<10000;i++){
    _rgb.waitForNextRevision();

    if(d2p.points.get()->N>0){
      C.gl().dataLock.lock(RAI_HERE);
      pcl->setPointCloud(d2p.points.get());
      C.gl().dataLock.unlock();
      C.watch(false);
    }

    {
      cv::Mat rgb = CV(_rgb.get());
      cv::Mat depth = CV(_depth.get());

      if(rgb.total()>0 && depth.total()>0){
        cv::imshow("rgb", rgb); //white=2meters
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::waitKey(1);
      }
    }
  }

  rai::wait();
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  minimal_use();
//    get_objects_into_configuration();

  return 0;
}
