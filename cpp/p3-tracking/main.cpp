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
  // load a configuration
  rai::KinematicWorld C;
  C.addFile("../../rai-robotModels/baxter/baxter_new.g");
  arr q_home = C.getJointState();
  arr Wmetric = diag(1., C.getJointStateDimension());

  // add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->Q.setText("d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)");
  cameraFrame->calc_X_from_parent();

  // add a frame for the object
  rai::Frame *objectFrame = C.addFrame("obj");
  objectFrame->setShape(rai::ST_ssBox, {.1, .1, .1, .02});
  objectFrame->setColor({.8, .8, .1});

  // launch robot interface
  RobotOperation B(C);
  B.sync(C);

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");

  // set the intrinsic camera parameters
  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.};

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  // looping
  for(uint i=0;i<1000;i++){
    _depth.waitForNextRevision();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();

    if(rgb.rows != depth.rows) continue;

    // blur
    cv::blur(rgb, rgb, cv::Size(5,5));

    // hsv filter
    cv::Mat hsv, mask;
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
                cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

    // compute contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    // find largest
    arr sizes(contours.size());
    for(uint i=0; i<contours.size(); i++) sizes(i) = cv::contourArea(cv::Mat(contours[i]));
    uint largest = sizes.maxIndex();

    // draw the contour interior into the mask
    mask = cv::Scalar(0);
    cv::drawContours(mask, contours, largest, cv::Scalar(128), CV_FILLED);

    // grab the depth values and mean x,y coordinates
    floatA depthValues;
    double objX=0.,objY=0.;
    for(int y=0;y<mask.rows;y++) for(int x=0;x<mask.cols;x++){
      if(mask.at<byte>(y,x)){
        float d = depth.at<float>(y,x); //myDepth(y,x);
        if(d>.1 && d<2.){
          depthValues.append(d);
          objX += x;
          objY += y;
        }
      }
    }

    if(depthValues.N){
      objX /= double(depthValues.N);
      objY /= double(depthValues.N);

      // median
      double objDepth = depthValues.median_nonConst();
      // mean
      //double objDepth = sum(depthValues)/double(depthValues.N);

      // image coordinates
      arr objCoords = {objX, objY, objDepth};

      // camera coordinates
      depthData2point(objCoords, Fxypxy); //transforms the point to camera xyz coordinates

      // world coordinates
      cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates

      //cout <<"object coordinates: " <<objCoords <<endl;

      objectFrame->setPosition(objCoords);
    }

    // tracking IK
    {
      arr y, J, Phi, PhiJ;

      //1st task: track circle with right hand
      arr target = objectFrame->getPosition();
      C.evalFeature(y, J, FS_position, {"baxterR"});  //"handR" is the name of the right hand ("handL" for the left hand)
      Phi.append( (y-target) / 1e-2);
      PhiJ.append( J / 1e-2 );

      //2nd task: joint should stay close to zero
      C.evalFeature(y, J, FS_qItself, {});
      Phi .append( (y-q_home) / 1e0 );
      PhiJ.append( J / 1e0 );

      //      //3rd task: left hand should point upwards
      //      K.evalFeature(y, J, FS_vectorZ, {"handL"});  //"handR" is the name of the right hand ("handL" for the left hand)
      //      target = {0.,0.,1.};
      //      Phi.append( (y-target) / 1e-2);
      //      PhiJ.append( J / 1e-2 );

      // IK compute joint updates
      arr q = C.getJointState();
      q -= 0.1*inverse(~PhiJ*PhiJ + Wmetric) * ~PhiJ * Phi;

      C.setJointState(q);
      B.moveHard(q);
      C.watch();
    }

    if(rgb.total()>0 && depth.total()>0){
      if(contours.size()){
        cv::drawContours( rgb, contours, largest, cv::Scalar(255,0,0), 2, 8);
        cv::drawContours( depth, contours, largest, cv::Scalar(0), 2, 8);
      }
      cv::imshow("rgb", rgb);
      cv::imshow("depth", 0.5*depth); //white=2meters
      cv::imshow("mask", mask);
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  tracking();

  return 0;
}
