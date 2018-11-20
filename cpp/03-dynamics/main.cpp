#include <Kin/kin.h>

void execTrajectory(){
  rai::KinematicWorld K("pegArm.ors");
  K.sortFrames();
  double noise = .01;
  bool gravity = true;
  uint n=K.getJointStateDimension();

  double tau=.01;
  uint N = 500;
  double T = N*tau;

  arr q,qdot, qddot_desired;
  arr M,F,u(n);
  arr integral = zeros(n);
  arr q0, q_goal;

  K.getJointState(q, qdot);
  q = q+0.4;
  K.setJointState(q,qdot);

  q_goal = zeros(n);
  q0 = q;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.watch();
  
  ofstream dat("z.q");

  for(uint i=0;i<=N;i++){
    double t = i * tau;
    u = 0.;

    // b) compute desired position, velocity and acceleration with sine motion profile


    
    // c) PD controller


    // d) PID controller


    // e) Inverse dynamics feedforward control
    K.equationOfMotion(M,F);



    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    K.stepDynamics(u, tau, noise, gravity);
    K.watch(false);
    K.getJointState(q, qdot);

    cout  <<" t=" <<tau*t  <<"sec E=" <<K.getEnergy()  <<"  q = " <<q <<endl;
    dat <<q <<endl;
  }

  gnuplot("plot 'z.q' us 0:1, '' us 0:2, '' us 0:3");
  K.watch(true);
}


int main(int argc,char **argv){

  execTrajectory();
  
  return 0;
}
