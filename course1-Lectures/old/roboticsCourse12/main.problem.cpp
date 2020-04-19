#include <Kin/roboticsCourse.h>


void testSimulator(){
  Simulator S("human.g");
  cout <<"joint dimensions = " <<S.getJointDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr q,qdot;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot = 0.; //1.;

  for(uint i=0;i<1000;i++){
    //q += .001;
    //S.setJointAngles(q);
    S.stepOde(qdot);
  }
}

void walkPhase(arr& Phi,arr& PhiJ,Simulator &S,
	       const char* stanceFoot,const char* moveFoot,
	       uint phase){
  arr y_target,y,J;
  arr stance,move;

  S.kinematicsPos(stance,stanceFoot);
  S.kinematicsPos(move  ,moveFoot);

  //1st task: move the move-foot
  S.kinematicsPos(y,moveFoot);
  S.jacobianPos  (J,moveFoot);
  switch(phase){
  case 0:  y_target = y;  break; //hold (only move COM)
  case 1:  y_target = y+ARR(0,0,.1);  break; //up
  case 2:  y_target = y+ARR(0,-.1,0)+.1*(move-stance);  break; //fwd
  case 3: //down
    y_target = y+ARR(0,0,-.15);
    if(y(2)<.03) y_target = y;
    break;
  }

  Phi  = (y - y_target)/1e-1;
  PhiJ = J / 1e-1;

  //2nd task: keep move-foot straight
  S.kinematicsVec(y,moveFoot);
  S.jacobianVec  (J,moveFoot);
  y_target = ARR(0,0,1);

  Phi .append((y - y_target)/1e-2);
  PhiJ.append( J / 1e-2 );

  //2nd task: move COM over stance foot
  y_target = ARR(stance(0), stance(1) -.05);
  S.kinematicsCOM(y);
  S.jacobianCOM  (J);

  if(!phase){
    Phi .append((y - y_target)/1e-2 );
    PhiJ.append( J / 1e-2 );
  }else{
    Phi .append((y - y_target)/1e-4 );
    PhiJ.append( J / 1e-4 );
  }

  cout <<"COM= " <<y <<" stance= " <<stance <<" move= " <<move <<endl;
}


void staticWalk(){
  Simulator S("human.g");
  cout <<"joint dimensions = " <<S.getJointDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  arr q,qdot,W,Phi,PhiJ;
  S.getJointAngles(q);
  W.setDiag(1.,q.N);
  qdot.resizeAs(q);
  qdot = 0.; //1.;
  uint LR=1;

  for(uint t=0;t<5000;t++){
    cout <<"t= " <<t <<' ';
    
    uint phase=(t/200)%4;
    if(!(t%800)){
      LR ^= 1;
      if(!LR) S.anchorKinematicChainIn("rfoot");
      else    S.anchorKinematicChainIn("lfoot");
    }
    if(!LR) walkPhase(Phi,PhiJ,S,"rfoot","lfoot",phase);
    else    walkPhase(Phi,PhiJ,S,"lfoot","rfoot",phase);


    //compute desired joint velocities using inverse kinematics
    qdot = -inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;

    //q += .02*qdot;  S.setJointAngles(q);
    S.stepOde(qdot,!(t%10));
  }
}


int main(int argc,char **argv){
  //testSimulator();
  staticWalk();  
  return 0;
}
