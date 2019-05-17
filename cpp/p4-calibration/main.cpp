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


void calibration(){
  // load a configuration
  rai::KinematicWorld C;
  C.addFile("model.g");
  arr q_home = C.getJointState();
  arr Wmetric = diag(1., C.getJointStateDimension());

  // launch camera
  Var<byteA> _rgb;
  Var<floatA> _depth;
//  RosCamera cam(_rgb, _depth, "cameraRosNodeMarc", "/camera/rgb/image_raw", "/camera/depth/image_rect");

  //associate an opengl renderer with the camera frame
  Var<rai::KinematicWorld> C_visual;
  C_visual.set() = C;
  rai::Sim_CameraView camSim(C_visual, _rgb, _depth, .1);
  camSim.C.addSensor("myCam", "camera", 640, 480, 1.);
  camSim.C.selectSensor("myCam");

  // set the intrinsic camera parameters
  double f = 1./tan(0.5*60.8*RAI_PI/180.);
  f *= 320.;
  arr Fxypxy = {f, f, 320., 240.};

  // set hsv filter parameters
  arr hsvFilter = rai::getParameter<arr>("hsvFilter").reshape(2,3);

  // create a 3D grid of target points
  arr grid = ::grid({-.2,-.25,-.25},{.2,.25,.25}, {4,4,4});
  int gridCount = 0;
  arr centerR = C["volumeR"]->getPosition();
  arr centerL = C["volumeL"]->getPosition();

  // add a frame for the object
  rai::Frame *targetFrame = C.addFrame("target");
  targetFrame->setShape(rai::ST_ssBox, {.05, .05, .05, .02});
  targetFrame->setColor({.8, .8, .1});

  ofstream fil("z.data");
  Graph data;
  Node_typed<arr> *data_q = data.newNode<arr>({"q"}, {});
  Node_typed<arr> *data_XR = data.newNode<arr>({"XR"}, {});
  Node_typed<arr> *data_xR = data.newNode<arr>({"xR"}, {});

  // looping
  for(;;){
    _depth.waitForNextRevision();

    C_visual.set()->setJointState(C.getJointState());

    // grap copies of rgb and depth
    cv::Mat rgb = CV(_rgb.get()).clone();
    cv::Mat depth = CV(_depth.get()).clone();

    if(rgb.rows != depth.rows) continue;

    GetLargestObjects OBJ(rgb, depth, hsvFilter, 2);

    if(OBJ.objCoords(0,0)<OBJ.objCoords(1,0)){
      arr tmp;
      tmp = OBJ.objCoords[0];
      OBJ.objCoords[0] = OBJ.objCoords[1];
      OBJ.objCoords[1] = tmp;
    }

//    // camera coordinates
//    for(uint i=0;i<OBJ.objCoords.d0;i++){
//      depthData2point(OBJ.objCoords[i](), Fxypxy); //transforms the point to camera xyz coordinates
//      C["camera"]->X.applyOnPoint(OBJ.objCoords[i]()); //transforms into world coordinates
//    }

    cout <<"markers: " <<OBJ.objCoords <<endl;

    // tracking IK
    {
      arr y, J, Phi, PhiJ;

      targetFrame->setPosition(centerR + grid[gridCount]);

      //1st task: track circle with right hand
      arr target = targetFrame->getPosition();
      C.evalFeature(y, J, FS_position, {"baxterR"});  //"handR" is the name of the right hand ("handL" for the left hand)
      Phi.append( (y-target) / 1e-2);
      PhiJ.append( J / 1e-2 );

      if(sumOfSqr(y-target)<1e-5){
        data_q->value = C.getJointState();
        data_XR->value = C["calibR"]->getPosition();
        data_xR->value = OBJ.objCoords[0];
        fil <<data <<endl;
        gridCount++;
        if(gridCount>=grid.d0){ gridCount=0; break; }
      }

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
      q -= .5*inverse(~PhiJ*PhiJ + Wmetric) * ~PhiJ * Phi;
      C.setJointState(q);

//      B.moveHard(q);
      C.watch();

    }

    if(rgb.total()>0 && depth.total()>0){
      cv::imshow("rgb", rgb);
//      cv::imshow("depth", 0.5*depth); //white=2meters
//      cv::imshow("mask", OBJ.mask);
      int key = cv::waitKey(1);
      if((key&0xff)=='q') break;
    }
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  calibration();

  return 0;
}
