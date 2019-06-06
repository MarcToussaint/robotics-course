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
  rai::KinematicWorld C;
  C.addFile("model.g");
  arr q_home = C.getJointState();
  arr Wmetric = diag(1., C.getJointStateDimension());

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
#if 1
  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");
//  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/kinect/rgb/image_rect_color", "/kinect/depth_registered/sw_registered/image_rect_raw", true);
#else
  //associate an opengl renderer with the camera frame
  Var<rai::KinematicWorld> C_visual;
  C_visual.set() = C;
  rai::Sim_CameraView camSim(C_visual, _rgb, _depth, .1);
  camSim.C.addSensor("myCam", "camera");
  camSim.C.selectSensor("myCam");

  // output ground truth
  cout <<"groud truth camera:\n";
  camSim.C.currentSensor->cam.report();
#endif

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  // create a 3D grid of target points
  arr grid = ::grid({-.2,-.2,-.2},{.2,.2,.2}, {2,2,2});
  int gridCount = 0;
  rai::Transformation centerR = C["volumeR"]->X;
  rai::Transformation centerL = C["volumeL"]->X;

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
#endif

  uint countStable=0;

  // looping
  for(;;){
    _depth.waitForNextRevision();

//    C_visual.set()->setJointState(C.getJointState());

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();

    if(rgb.rows != depth.rows) continue;

    GetLargestObjects OBJ(rgb, depth, hsvFilter, 2, true);

    if(OBJ.objCoords(0,0)<OBJ.objCoords(1,0)){
      arr tmp;
      tmp = OBJ.objCoords[0];
      OBJ.objCoords[0] = OBJ.objCoords[1];
      OBJ.objCoords[1] = tmp;
    }

    cout <<"markers:\n" <<OBJ.objCoords <<endl;

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
          if(OBJ.objCoords(0,0)<rgb.cols-10 && OBJ.objCoords(0,1)<rgb.rows-10
             && OBJ.objCoords(0,0)>10 && OBJ.objCoords(0,1)>10
             && OBJ.objCoords(1,0)>10 && OBJ.objCoords(1,1)>10){
            data_q->value = C.getJointState();
            data_XR->value = C["calibR"]->getPosition();
            data_xR->value = OBJ.objCoords[0];
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
  }

  //-- first iteration
  arr Pinv, K, R, t;
  Pinv = ~X * x * inverse_SymPosDef(~x*x);
  decomposeInvProjectionMatrix(K, R, t, Pinv);
  cout <<"1st iter ERROR = " <<sqrt(sumOfSqr(x*~Pinv - X)/double(n)) <<endl;
  cout <<"*** camera origin in world: " <<t <<endl;
//  cout <<"P = " <<P <<endl;

  //-- correct for radius
  double radius = .02;
  for(uint i=0;i<n;i++){
    arr rel = X[i] - t;
    X[i] -= rel*(radius/length(rel)); //pull ``closer'', to the ball front
  }

  //-- second iteration
  Pinv = ~X * x * inverse_SymPosDef(~x*x);
  decomposeInvProjectionMatrix(K, R, t, Pinv);
  cout <<"2nd iter ERROR = " <<sqrt(sumOfSqr(x*~Pinv - X)/double(n)) <<endl;
//  cout <<"P = " <<P <<endl;

  //-- output
  {
    //my convention...
    arr flip = diag(arr({1.,-1.,-1.}));
    K = K*flip;
    R = flip*R;
  }
  rai::Quaternion rot;
  rot.setMatrix(~R);
  cout <<"*** total Pinv:\n";
  cout <<" Pinv:\n" <<Pinv <<endl;
  cout <<"*** camera intrinsics:\n" <<K <<endl;
  cout <<"*** camera origin in world: " <<t <<endl;
  cout <<"*** camera rotation in world: " <<rot.getArr4d() <<endl;

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
