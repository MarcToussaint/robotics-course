#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>
#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>

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
    cv::Mat rgb = CV(_rgb.get());
    cv::Mat depth = CV(_depth.get());

    if(rgb.total()>0 && depth.total()>0){
      cv::imshow("rgb", rgb); //white=2meters
      cv::imshow("depth", 0.5*depth); //white=2meters
      cv::waitKey(1);
    }
  }
}

//void get_objects_into_configuration(){
//  RosCom ROS;

//  Var<byteA> rgb;
//  Var<floatA> depth;

//  SubscriberConv<sensor_msgs::Image, byteA, &conv_image2byteA> subRgb(rgb, "/camera/rgb/image_rect_color");
//  SubscriberConv<sensor_msgs::Image, floatA, &conv_imageu162floatA> subDepth(depth, "/camera/depth_registered/image_raw");


//  Depth2PointCloud d2p(depth, 1.);

//  rai::KinematicWorld C;
//  C.addFile("model.g");
//  rai::Frame *pcl = C.addFrame("pcl", "camera", "shape:pointCloud");
//  for(uint i=0;i<100;i++){
////    cout <<d2p.points.get()->N <<endl;
//    pcl->shape->mesh().V = d2p.points.get();
//    pcl->shape->mesh().V.reshape(640*480,3);
//    C.watch(false);
//    rai::wait(.1);
//  }

//  rai::wait();
//}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  minimal_use();

  return 0;
}
