#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

#include <RosCom/roscom.h>
#include <RosCom/rosCamera.h>
#include <Kin/frame.h>
#include <Gui/opengl.h>
#include <RosCom/baxter.h>

#include <Operate/robotOperation.h>

#include <Kin/cameraview.h>

#include "help.h"

/*
  attached green balls to both wrists

move along grid

record

q worldL worldR imageL = (x,y,d) imageR=(x,y,d)
*/


void collectData(){
  // load a configuration
  rai::Configuration C;
  C.addFile("model.g");
  arr q_home = C.getJointState();
  arr Wmetric = diag(1., C.getJointStateDimension());

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
//  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  // create a 3D grid of target points
  arr grid = ::grid({-.2,-.2,-.2}, {.2,.2,.2}, {3,3,3});
  int gridCount = 0;
  rai::Transformation centerR = C["volumeR"]->ensure_X();
  rai::Transformation centerL = C["volumeL"]->ensure_X();

  // add a frame for the object
  rai::Frame *targetRFrame = C.addFrame("target");
  targetRFrame->setShape(rai::ST_ssBox, {.05, .05, .05, .02});
  targetRFrame->setColor({.8, .8, .1});

  rai::Frame *targetLFrame = C.addFrame("target");
  targetRFrame->setShape(rai::ST_ssBox, {.05, .05, .05, .02});
  targetRFrame->setColor({.8, .8, .1});

  ofstream fil("z.data");
  Graph data;
  Node_typed<arr> *data_q = data.newNode<arr>({"q"}, {});
  Node_typed<arr> *data_XR = data.newNode<arr>({"XR"}, {});
  Node_typed<arr> *data_xR = data.newNode<arr>({"xR"}, {});

  // launch robot interface
#if 1
  RobotOperation B(C);
  B.sync(C);
  B.move(q_home, {5.});
  rai::wait();
  B.sync(C);
#endif

  uint countStable=0;

  // looping
  for(;;){
    _depth.waitForNextRevision();

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();

    if(rgb.rows != depth.rows) continue;

    GetLargestObjects OBJ(rgb, depth, hsvFilter, 2, false);

    if(OBJ.cameraCoords(0,0)<OBJ.cameraCoords(1,0)){
      arr tmp;
      tmp = OBJ.cameraCoords[0];
      OBJ.cameraCoords[0] = OBJ.cameraCoords[1];
      OBJ.cameraCoords[1] = tmp;
    }

    cout <<"markers:\n" <<OBJ.cameraCoords <<endl;

    // tracking IK
    {
      arr y, J, Phi, PhiJ;

      targetRFrame->setPosition((centerR * rai::Vector(grid[gridCount])).getArr());
      targetLFrame->setPosition((centerL * rai::Vector(grid[gridCount])).getArr());

      //1st task: track circle with right hand
      arr target = targetRFrame->getPosition();
      C.evalFeature(y, J, FS_position, {"baxterR"});  //"handR" is the name of the right hand ("handL" for the left hand)
      Phi.append( (y-target) * 1e2);
      PhiJ.append( J * 1e2 );

      if(sumOfSqr(y-target)<1e-5){
        countStable++;
        if(countStable>30){
          //save a data point
          if(OBJ.cameraCoords(0,0)<rgb.cols-10 && OBJ.cameraCoords(0,1)<rgb.rows-10
             && OBJ.cameraCoords(0,0)>10 && OBJ.cameraCoords(0,1)>10
             && OBJ.cameraCoords(1,0)>10 && OBJ.cameraCoords(1,1)>10){
            data_q->value = C.getJointState();
            data_XR->value = C["calibR"]->getPosition();
            data_xR->value = OBJ.cameraCoords[0];
            fil <<data <<endl;
            //          rai::wait();
          }else{
            cout <<"SKIPPED" <<endl;
          }

          gridCount++;
          if(gridCount>=(int)grid.d0){ gridCount=0; break; }
        }
      }else{
        countStable = 0;
      }

      target = targetLFrame->getPosition();
      C.evalFeature(y, J, FS_position, {"baxterL"});  //"handR" is the name of the right hand ("handL" for the left hand)
      Phi.append( (y-target) * 1e2);
      PhiJ.append( J * 1e2 );

      //2nd task: joint should stay close to zero
      C.evalFeature(y, J, FS_qItself, {});
      Phi .append( (y-q_home) * 1e0 );
      PhiJ.append( J * 1e0 );

//      //3rd task: left hand should point upwards
//      C.evalFeature(y, J, FS_vectorZ, {"baxterR"});  //"handR" is the name of the right hand ("handL" for the left hand)
//      target = {0.,0.,1.};
//      Phi.append( (y-target) * 1e-0);
//      PhiJ.append( J * 1e-0 );

      // IK compute joint updates
      arr q = C.getJointState();
      q -= .05*inverse(~PhiJ*PhiJ + Wmetric) * ~PhiJ * Phi;
      C.setJointState(q);

      B.moveHard(q);
      C.watch();
    }

    if(rgb.total()>0 && depth.total()>0){
      cv::imshow("rgb", rgb);
      cv::imshow("depth", 0.5*depth); //white=2meters
//      cv::imshow("mask", OBJ.mask);
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
  }
}

//===========================================================================

void optimize(){
//  Graph data("realCalib3.data");
  Graph data("z.data");

  //-- load data
  uint n = data.N;
  arr X(n,3), x(n,4);
  for(uint i=0;i<n;i++){
    arr Z = data.elem(i)->get<arr>("XR");
    X[i] = Z;

    arr z = data.elem(i)->get<arr>("xR");
    //-- undo projection
    z(0) *= z(2);
    z(1) *= z(2);
    z.append(1.);
    x[i] = z;

    //-- filter data based on depth
    double d = z(2);
    if(d>2.){
      X[i]=0.; x[i]=0.;
    }
  }

  //-- multiple iterations
  arr Pinv, K, R, t;
  for(uint k=0;k<10;k++){
    Pinv = ~X * x * inverse_SymPosDef(~x*x);
    decomposeInvProjectionMatrix(K, R, t, Pinv);
    for(uint i=0;i<n;i++){
      double ei = sqrt(sumOfSqr(X[i] - Pinv*x[i]));
      cout <<"   error on data " <<i <<": " <<ei;
      if(ei>.05){ X[i]=0.; x[i]=0.; cout <<" -- removed"; }
      cout <<endl;
    }
    double err = sqrt(sumOfSqr(x*~Pinv - X)/double(n));
    cout <<"total ERROR = " <<err <<endl;
    if(err<.01) break;
  }

  //-- correct for radius
  double radius = .02;
  for(uint i=0;i<n;i++){
    if(sumOfSqr(X[i])>1e-8){
      arr rel = X[i] - t;
      X[i] -= rel*(radius/length(rel)); //pull ``closer'', to the ball front
    }
  }
  Pinv = ~X * x * inverse_SymPosDef(~x*x);
  decomposeInvProjectionMatrix(K, R, t, Pinv);
  cout <<"total ERROR after radius correction = " <<sqrt(sumOfSqr(x*~Pinv - X)/double(n)) <<endl;


  //-- output
  {
    //my convention...
    arr flip = diag(arr({1.,-1.,-1.}));
    K = K*flip;
    R = flip*R;
  }
  rai::Quaternion rot;
  rot.setMatrix(~R);
  cout <<"*** total Pinv:\n" <<Pinv <<endl;
  cout <<"*** camera intrinsics K:\n" <<K <<endl;
  cout <<"*** camera world pos: " <<t <<endl;
  cout <<"*** camera world rot: " <<rot.getArr4d() <<endl;

  //-- test/example
  for(uint i=0;i<0;i++){
    cout <<"X=\n" <<X[i] <<endl;
    cout <<"x=\n" <<x[i] <<endl;
    cout <<"PX=\n" <<Pinv*X[i] <<endl;
//    cout <<"TX=\n" <<R*X(i,{0,2})-t <<endl;
    cout <<"KTX=\n" <<K*R*(X(i,{0,2})-t) <<endl;
  }
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

//  collectData();

  optimize();

  return 0;
}

/* results

for camera1:

total ERROR = 0.00395552
total ERROR after radius correction = 0.00396736
*** total Pinv:
[0.00187839, 7.8914e-05, -0.614197, -0.048283,
 8.75529e-05, -0.00162767, 0.864538, 0.228542,
 5.11853e-05, -0.000906396, -0.676576, 1.78686]
*** camera intrinsics K:
[531.597, -6.16406, -313.788,
 0, -536.315, -243.297,
 0, 0, -0.989076]
*** camera world pos: [-0.048283, 0.228542, 1.78686]
*** camera world rot: [0.967548, 0.251242, -0.00732202, 0.0259533]



for camera2:

total ERROR = 0.00241405
total ERROR after radius correction = 0.00221046
*** total Pinv:
[0.00180045, 5.51994e-06, -0.569533, -0.0330757,
 -1.82321e-06, -0.00133149, 1.00136, 0.125005,
 5.08217e-05, -0.00117336, -0.439092, 1.55487]
*** camera intrinsics K:
[555.197, -8.21031, -334.467,
 0, -563.526, -271.392,
 0, 0, -1.02162]
*** camera world pos: [-0.0330757, 0.125005, 1.55487]
*** camera world rot: [0.935411, 0.35328, -0.0133783, 0.00451155]

*/
