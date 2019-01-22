#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>
//#include <MT/gauss.h>
#include <Plot/plot.h>

arr getControlJacobian(CarSimulator & S, const arr & u,const arr & x){
  double L = S.L;
  arr B(3,2);
  // ...
  return B;
}


void KalmanSLAM(){
  CarSimulator Sim;
  Sim.gl->watch();

  arr u(2),y_meassured;

  arr s(3),S(3,3); //kalman estimates
  arr shat,Shat;

  s(0) = Sim.x; s(1) = Sim.y; s(2) = Sim.theta; //initial at true state
  S.setDiag(0.1); //with noisy constant

  arr W(4,4);  W.setDiag(Sim.observationNoise);
  arr Q(3,3);  Q.setDiag(Sim.dynamicsNoise);

  arr A(3,3),a;  A.setDiag(1);

  arr C, c, K;
  arr y_mean;

  for(uint t=0;t<10000;t++){
    u = ARR(.1, .2); //control signal
    Sim.step(u);
    Sim.getRealNoisyObservation(y_meassured);

    //get the linear observation model
    Sim.getObservationJacobianAtState(C, s);
    Sim.getMeanObservationAtState(y_mean, s);

    //get the linear control model
    arr B;
    B = getControlJacobian(Sim, u, s);
    a = B*u;

    //Kalman filter
    //...
    // Tipp: use variables shat, Shat, c, C and K
    // to compute the new Kalman estimates s and S

    //code to display a covariance
    Sim.gaussiansToDraw.resize(1);
    Sim.gaussiansToDraw(0).A=S.sub(0,1,0,1);
    Sim.gaussiansToDraw(0).a=s.sub(0,1);

    //tracking error
    cout << "estim error " <<maxDiff(s, ARR(Sim.x, Sim.y, Sim.theta)) << endl;
  }
}



int main(int argc,char **argv){
  KalmanSLAM();
  return 0;
}
