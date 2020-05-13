#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

//===========================================================================
void using_KOMO_for_PathPlanning(){
    //we don't need to instantiate the real world/simluation here
    //-- MODEL WORLD configuration, this is the data structure on which you represent
    rai::Configuration C;
    C.addFile("../../scenarios/pandasTable.g");
    rai::Simulation S(C, S._physx, true);
    S.cameraview().addSensor("camera");
    //-- using the viewer, you can view configurations or paths
    rai::ConfigurationViewer V;
    V.setConfiguration(C, "model world start state", true);

    //-- optimize a single configuration using KOMO

    KOMO komo;                     //create a solver
    komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
    komo.setTiming(1., 30, 5., 3);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

    //now add objectives!

    //the default control objective:
    komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

    //task objectives:
    komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "L_gripperCenter"}, OT_sos, {1e1});
    komo.addObjective({1.}, FS_scalarProductXY, {"R_gripperCenter", "L_gripperCenter"}, OT_eq, {1e1}, {-1});
    komo.addObjective({1.}, FS_scalarProductZZ, {"R_gripperCenter", "L_gripperCenter"}, OT_eq, {1e1}, {-1});
    komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
    komo.addObjective({}, FS_accumulatedCollisions, {}, OT_ineq, {1});

    //initialize the solver
    komo.optimize();

    //get the joint vector from the optimized configuration
    arr q = komo.getJointState(1.);

    C.setJointState(q); //set your working config into the optimized state
    V.setConfiguration(C, "optimized configuration", true); //display it

    V.setPath(komo.getPath_frames(), "optimized path", true);
    V.playVideo(true, 5.0);

    rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}
//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);
  using_KOMO_for_PathPlanning();

  return 0;
}
