#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

#include <Operate/robotOperation.h>

void tracking(){
  //load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");
  arr q_home = C.getJointState();
  arr Wmetric = diag(1., C.getJointStateDimension());

  //add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
  cameraFrame->calc_X_from_parent();

  //add a frame for the object
  rai::Frame *objectFrame = C.addFrame("obj");
  objectFrame->setShape(rai::ST_ssBox, {.1, .1, .1, .02});
  objectFrame->setColor({.8, .8, .1});

  RobotOperation B(C);
  B.sync(C);

  Var<byteA> _rgb;
  Var<floatA> _depth;

#if 1 //using ros
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
#else //using a webcam
  OpencvCamera cam(_rgb);
#endif

  //set the intrinsic camera parameters
  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.};

  arr hsvFilter = rai::getParameter<arr>("hsvFilter");
  hsvFilter.reshape(2,3);

  //looping images through opencv
  for(uint i=0;i<1000;i++){
    _depth.waitForNextRevision();

    byteA myRgb = _rgb.get();
    floatA myDepth = _depth.get();
    int H = myRgb.d0, W=myRgb.d1;

    if(myRgb.d0 != myDepth.d0 || myRgb.d1 != myDepth.d1) continue;

    {
      cv::Mat rgb = CV(myRgb);
      cv::Mat depth = CV(myDepth);
      cv::blur(rgb, rgb, cv::Size(5,5));

      cv::Mat hsv, mask;
      cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
      cv::inRange(hsv,
                  cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
                  cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

      //-- compute contours
      std::vector<std::vector<cv::Point> > contours;
      cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

      //-- find largest
      int largest = -1;
      double maxSize;
      for(uint i=0; i<contours.size(); i++){
        double size = cv::contourArea(cv::Mat(contours[i]));
        if(largest<0 || size>maxSize){
          largest = i;
          maxSize = size;
        }
      }

      //-- assign pixelLabels by drawing the contour polygon or hull directly into the image with the right labels!
      mask = cv::Scalar(0);
      cv::drawContours(mask, contours, largest, cv::Scalar(128), CV_FILLED);

      //-- grab the depth values
      floatA depthValues;
      double objX=0.,objY=0.;
      for(int x=0;x<W;x++) for(int y=0;y<H;y++){
        if(mask.at<byte>(y,x)){
          float d = myDepth(y,x);
          if(d>.1 && d<2.){
            depthValues.append(d);
            objX += x;
            objY += y;
          }
        }
      }

      if(depthValues.N){
//        depthValues.sort();
//        objDepth = depthValues.elem(depthValues.N/2);
        double objDepth = sum(depthValues)/double(depthValues.N);

        objX /= double(depthValues.N);
        objY /= double(depthValues.N);

        arr objCoords = {objX, objY, objDepth};

        cout <<"object coordinates: " <<objCoords <<"  ";

        //example on how to convert image to 3D coordinates:
        depthData2point(objCoords, Fxypxy); //transforms the point to camera xyz coordinates
        cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates

        cout <<"object coordinates: " <<objCoords <<endl;

        objectFrame->setPosition(objCoords);

//        C.setJointState(q_home);
//        B.sync(C);
        C.watch();
      }

      //IK
      {
        arr y, J, Phi, PhiJ;

        //1st task: track circle with right hand
        arr target = objectFrame->getPosition();
        C.evalFeature(y, J, FS_position, {"baxterR"});  //"handR" is the name of the right hand ("handL" for the left hand)
        Phi.append( (y-target) / 1e-2);
        PhiJ.append( J / 1e-2 );

//      //2nd task: joint should stay close to zero
//      K.evalFeature(y, J, FS_qItself, {});
//      Phi .append( y / 1e0);
//      PhiJ.append( J / 1e0 );

//      //3rd task: left hand should point upwards
//      K.evalFeature(y, J, FS_vectorZ, {"handL"});  //"handR" is the name of the right hand ("handL" for the left hand)
//      target = {0.,0.,1.};
//      Phi.append( (y-target) / 1e-2);
//      PhiJ.append( J / 1e-2 );


        //IK compute joint updates
        arr q = C.getJointState();
        q -= 0.1*inverse(~PhiJ*PhiJ + Wmetric) * ~PhiJ * Phi;
        C.setJointState(q);
        B.moveHard(q);
        C.watch();
      }


      if(rgb.total()>0 && depth.total()>0){
        cv::drawContours( rgb, contours, largest, cv::Scalar(0,0,0), 2, 8);
        cv::imshow("rgb", rgb);
        cv::drawContours( depth, contours, largest, cv::Scalar(0), 2, 8);
        cv::imshow("depth", 0.5*depth); //white=2meters
        cv::imshow("mask", mask);
        int key = cv::waitKey(1);
        if((key&0xff)=='q') break;
      }
    }
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  tracking();

  return 0;
}
