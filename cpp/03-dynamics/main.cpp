#include <Kin/kin.h>

void holdSteady(){
  mlr::KinematicWorld K("pegArm.ors");
  //set noise to zero for easier inspection of the results
  double noise = 0.; //1.;
  bool gravity = true;
  uint n=K.getJointStateDimension();

  double tau=.01;
  uint T=200;
  
  arr q,qdot;
  arr M,F,u(n);
  
  K.getJointState(q, qdot);
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;

  K.watch();
  

  ofstream dat("z.q");
#if 0 //this is a first solution found by hand: I had to tune K myself
  double Kp=1000.;
  double Kd=.2*sqrt(4.*Kp); //but Kd can be set to ensure exact critical damping (exact exponential decay of perturbations)
#else
  //this is MUCH more intuitive:
  double decayTime=50.*tau; //exponential decay in 50 steps
  double dampingRatio = .8; //slight overshooting -- you can almost always choose this within [.8, 1.]
  //convert:
  double lambda = -decayTime*dampingRatio/log(.1);
  double Kp=mlr::sqr(1./lambda);
  double Kd=2.*dampingRatio/lambda;
#endif

  /* In the generated plot, check that the decay is really within exactly  50 steps.
   * Note that the initialization is not in rest but also perturbed.
   * Note that at T=100 all joints are perturbed equally -- and they decay exactly equally, although
   * they are very non-linearly coupled.
   */

  for(uint t=0;t<=T;t++){
    K.equationOfMotion(M,F);

    //no controller torques
    arr qddot = -Kp*q - Kd*qdot;
    u = M*qddot + F;
//    u = 0.; //use M, F, and some desired qddot to compute u

    //dynamic simulation (simple Euler integration of the system dynamics, look into the code)
    K.stepDynamics(u, tau, noise, gravity);
    K.watch(false);
    K.getJointState(q, qdot);

    //some impuls
    if(t==100){
      qdot+=1.;
      K.setJointState(q, qdot);
    }

//    cout  <<" t=" <<tau*t  <<"sec E=" <<K.getEnergy()  <<"  q = " <<q <<endl;
    dat <<q <<endl;
  }

  gnuplot("plot 'z.q' us 0:1, '' us 0:2, '' us 0:3");
  K.watch(true);
}


int main(int argc,char **argv){

  holdSteady();
  
  return 0;
}
