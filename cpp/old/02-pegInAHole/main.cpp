#include <Kin/roboticsCourse.h>

void peg_in_a_hole(){
  Simulator S("pegInAHole.g");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  arr q,y_target,y,J,W,Phi,PhiJ;
  uint n = S.getJointDimension();

  S.getJointAngles(q);
  W.setDiag(1.,n);

  arr y_hole,y0;
  S.kinematicsPos(y_hole,"hole");
  S.kinematicsPos(y0,"peg");
  double y_deviation;
  for(uint i=0;i<1000;i++){

    Phi.clear();
    PhiJ.clear();

    //1st task: peg positioned within hole
    S.kinematicsPos(y,"peg");
    S.jacobianPos  (J,"peg");
    y_target = y + .01*(y_hole-y); //exponentially approach the hole
    y_deviation = 1e-1; //(deviation is 1/precision^2)
    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //2nd task: collisions
    S.kinematicsContacts(y);
    S.jacobianContacts(J);
    y_target = ARR(0.); //target is zero collision costs
    y_deviation = 1e-2;
    Phi .append((y - y_target) / y_deviation);
    PhiJ.append(J / y_deviation);

    //compute joint updates
    q -= inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    S.setJointAngles(q);
  }
}

int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  peg_in_a_hole();

  return 0;
}
