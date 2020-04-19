#include <Kin/roboticsCourse.h>


void testDynamicSimulation(){
  Simulator S("arm3.g");
  arr q,qdot,qddot;
  arr M,Minv,F,tau;
  S.getJointAngles(q);
  qdot.resizeAs(q);
  qdot = 0.;
  tau=qdot;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();        //pause and watch initial posture

  double Delta = .01; //duration of one time step = .01sec
  bool gravity = mlr::getParameter<bool>("gravity",true);
  bool control = mlr::getParameter<bool>("control",true);

  for(uint i=0;i<1000;i++){
    
    //** CONTROLLER part

    //get system equation
    S.getDynamics(M,F,qdot,gravity);

    //controller: compute some torques
    
    //pure friction
    tau = -.1 * qdot;

    //separate PDs in each joint separately:
    //somehow compute Kp and Kd
    //tau(0) = Kp * (0. - q(0)) + Kd * (0. - qdot(0));
    //tau(1) = ...

    //compute coordinated tau so that each joint behaves like a PD:
    //...
    
    //** SIMULATOR part (you could replace this with the real robot)
    //integrate the system equation (that's all you need as a dynamics engine!!)
    inverse(Minv,M);
    qddot = Minv * (tau - F);
    //Euler integration (Runge-Kutte4 would be much more precise...)
    q    += Delta * qdot;
    qdot += Delta * qddot;
    S.setJointAnglesAndVels(q,qdot);

    //output
    cout <<" t = " <<.01*i
	 <<" E = " <<S.getEnergy()
	 <<" q = " <<q <<endl;
    
  }
}


int main(int argc,char **argv){
  mlr::initCmdLine(argc,argv);

  testDynamicSimulation();

  return 0;
}
