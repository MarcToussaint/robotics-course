#include <Kin/kin.h>
#include <Kin/frame.h>
#include <Kin/feature.h>
#include <Kin/simulation.h>
#include <Kin/viewer.h>
#include <KOMO/komo.h>

//===========================================================================

void using_KOMO_for_IK(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");
  arr q0 = C.getJointState();

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 1, 1., 1);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 1, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_eq, {1e2});

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(0);

  C.setJointState(q); //set your working config into the optimized state
  C.watch(true, "optimized configuration"); //display it


  //-- redoing the optimization with the same KOMO object!
  //   (Warning: In doubt, rather create a new KOMO instance for each optimization.)

  //let's change an objective:
  std::shared_ptr<Objective> ob = komo.objectives(1); //which is the positionDiff added above
  ob->feat->setTarget({0., 0., .1}); //new target in that feature space: 10cm height difference
  //optimize
  komo.optimize(0.); //don't add noise or reinitialize

  C.setJointState(komo.getConfiguration_qOrg(0)); //set your working config into the optimized state
  C.watch(true, "optimized configuration"); //display it


  //-- execute this in simulation
  rai::Configuration RealWorld;
  RealWorld.addFile("../../scenarios/challenge.g");
  rai::Simulation S(RealWorld, S._physx, true);

  S.step();
  S.setMoveTo(q, 2.); //2 seconds to goal
  S.setMoveTo(q0, 1.); //1 second back home
  for(uint t=0;;t++){
    cout <<"time to move: " <<S.getTimeToMove() <<endl;
    double tau=.001; //can set anything here time...
    S.step({}, tau);
    rai::wait(tau);
    if(S.getTimeToMove()<0.) break;
  }
  rai::wait();
}

//===========================================================================

void using_KOMO_for_PathPlanning(){
  //we don't need to instantiate the real world/simluation here

  //-- MODEL WORLD configuration, this is the data structure on which you represent
  rai::Configuration C;
  C.addFile("../../scenarios/pandasTable.g");

  rai::Frame* obj = C.addFrame("object");
  obj->setPosition({1., 0., 1.5});
  obj->setQuaternion({1., 0., 1., 0});
  obj->setShape(rai::ST_capsule, {.2, .02});
  obj->setColor({1., .0, 1.});

  //-- using the viewer, you can view configurations or paths
  C.watch(true, "model world start state");

  //-- optimize a single configuration using KOMO

  KOMO komo;                     //create a solver
  komo.setModel(C, true);        //tell it use C as the basic configuration (internally, it will create copies of C on which the actual optimization runs)
  komo.setTiming(1., 40, 5., 2);  //we want to optimize a single step (1 phase, 1 step/phase, duration=1, k_order=1)

  //now add objectives!

  //the default control objective:
  komo.add_qControlObjective({}, 2, 1.); //sos-penalize (with weight=1.) the finite difference joint velocity (k_order=1) between x[-1] (current configuration) and x[0] (to be optimized)

  //task objectives:
  komo.addObjective({1.}, FS_positionDiff, {"R_gripperCenter", "object"}, OT_sos, {1e2});

  komo.addObjective({1.}, FS_qItself, {}, OT_eq, {1e2}, {}, 1);

  //initialize the solver
  komo.optimize();

  //get the joint vector from the optimized configuration
  arr q = komo.getConfiguration_qOrg(komo.T-1);

  C.setJointState(q); //set your working config into the optimized state

  komo.view(true, "optimized configuration"); //display it
  komo.view_play();

  rai::wait(.1); //TODO: otherwise the opengl gets hung up?
}

//===========================================================================

int main(int argc,char **argv){
  rai::initCmdLine(argc, argv);

  using_KOMO_for_IK();
//  using_KOMO_for_PathPlanning();

  return 0;
}
