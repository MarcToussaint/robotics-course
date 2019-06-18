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
  arr Wmetric = diag(2., C.getJointStateDimension());

  // add a frame for the camera
  rai::Frame *cameraFrame = C.addFrame("camera", "head");
  cameraFrame->setPosition({-0.0499056, 0.231561, 1.7645});
  cameraFrame->setQuaternion({0.971032, 0.237993, -0.00607315, 0.0204557});
  arr Fxypxy = {539.637, 540.941, 317.533, 260.024};
  arr Pinv = arr(3,4,{
  0.00185141, 6.4155e-05, -0.59182, -0.0499056,
   6.82597e-05, -0.00163821, 0.85137, 0.231561,
   3.98992e-05, -0.000854289, -0.665166, 1.7645});

  // add a frame for the object
  rai::Frame *objectFrame = C.addFrame("obj");
  objectFrame->setShape(rai::ST_ssBox, {.1, .1, .1, .02});
  objectFrame->setColor({.8, .8, .1});

  // add a frame for the endeff reference
  rai::Frame *pointerFrame = C.addFrame("pointer", "baxterL");
  pointerFrame->setShape(rai::ST_ssBox, {.05, .05, .05, .01});
  pointerFrame->setColor({.8, .1, .1});
  pointerFrame->setRelativePosition({0.,0.,-.0});

  // launch robot interface
  RobotOperation B(C);
  B.sync(C);

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  B.moveHard(q_home);
  rai::wait();
  B.sync(C);

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

#if 0
      // camera coordinates
      depthData2point(objCoords, Fxypxy); //transforms the point to camera xyz coordinates

      // world coordinates
      cameraFrame->X.applyOnPoint(objCoords); //transforms into world coordinates
#else
      //direct affine projective transformation
      objCoords(0) *= objCoords(2);
      objCoords(1) *= objCoords(2);
      objCoords.append(1.);
      objCoords = Pinv * objCoords;
#endif

      //cout <<"object coordinates: " <<objCoords <<endl;

      objectFrame->setPosition(objCoords);
      C.watch();
    }

    // tracking IK
    if(true){
      arr y, J, Phi, PhiJ;

      //1st task: track circle with right hand
      arr target = objectFrame->getPosition();
      C.evalFeature(y, J, FS_position, {"pointer"});  //"handR" is the name of the right hand ("handL" for the left hand)
      Phi.append( (y-target) * 1e2);
      PhiJ.append( J * 1e2 );

      //2nd task: joint should stay close to zero
      C.evalFeature(y, J, FS_qItself, {});
      Phi .append( (y-q_home) * 1e0 );
      PhiJ.append( J * 1e0 );

      //      //3rd task: left hand should point upwards
      //      K.evalFeature(y, J, FS_vectorZ, {"handL"});  //"handR" is the name of the right hand ("handL" for the left hand)
      //      target = {0.,0.,1.};
      //      Phi.append( (y-target) / 1e-2);
      //      PhiJ.append( J / 1e-2 );

      // IK compute joint updates
      arr q = C.getJointState();
      q -= 0.05*inverse(~PhiJ*PhiJ + Wmetric) * ~PhiJ * Phi;

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
