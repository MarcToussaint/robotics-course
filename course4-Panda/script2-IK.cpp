#include <Kin/kin.h>
#include <KOMO/komo.h>
#include <Optim/NLP_Solver.h>

int main(int argc, char** argv){
  rai::setRaiPath(STRING(std::getenv("HOME") <<"/git/rai-python/rai-robotModels"));

  rai::Configuration C;
  C.addFile(rai::raiPath("../rai-robotModels/scenarios/pandaSingle.g"));
  C.view();

  C.addFrame("box","table")
    ->setRelativePosition({-.25,.1,.1})
    .setShape(rai::ST_ssBox, {.1,.1,.1,.02})
    .setColor({1,.5,0});
  C.view();

  KOMO komo;
  komo.setModel(C, true);
  komo.setTiming(1., 1, 5., 0);
  komo.add_qControlObjective({}, 0, 1e-0);
  komo.addObjective({}, FS_accumulatedCollisions, {}, OT_eq);
  komo.addObjective({}, FS_jointLimits, {}, OT_ineq);
  komo.addObjective({}, FS_positionDiff, {"l_gripper", "box"}, OT_eq, {1e1});

  auto ret = NLP_Solver()
    .setProblem(komo.nlp())
    .setOptions( rai::OptOptions().set_stopTolerance(1e-2).set_verbose(4) )
    .solve();
  cout <<ret <<endl;

  komo.view(false, "IK solution");

  arr q = komo.getPath_qOrg();
  cout <<q <<endl;

  C.setJointState(q[0]);
  C.view();

  return 0;
}



