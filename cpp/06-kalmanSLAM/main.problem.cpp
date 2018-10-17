#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>
//#include <MT/gauss.h>
#include <Plot/plot.h>


void getControlJacobian(arr& B, CarSimulator & S, const arr & u,const arr & x){
  double carLength = S.L;
  B.resize(3,2);
  //insert correct Jacobian here
  B *= S.tau;
}

//ok, works
void Kalman(){
  CarSimulator Sim;
  Sim.gl->watch();
  
  arr u(2),y_meassured;
  
  arr s(3),S(3,3); //kalman estimates
  arr shat,Shat,dW;
  
  s(0) = Sim.x; s(1) = Sim.y; s(2) = Sim.theta; //initial at true state
  S.setDiag(0.1); //which nosy constant
  
  arr W(4,4);  W.setDiag(Sim.observationNoise);
  arr Q(3,3);  Q.setDiag(Sim.dynamicsNoise);
  
  arr A(3,3),a;  A.setDiag(1);
  
  for(uint t=0;t<10000;t++){
    u = ARR(.1, .2); //control signal
    Sim.step(u);
    Sim.getRealNoisyObservation(y_meassured);
    
    //get the linear observation model
    arr C,y_mean;
    Sim.getObservationJacobianAtState(C, s);
    Sim.getMeanObservationAtState(y_mean, s);

    //get the linear control model
    arr B;
    getControlJacobian(B, Sim, u, s);
    a = B*u;
    
    //Kalman filter

    //code to display a covariance
    Sim.gaussiansToDraw.resize(1);
    Sim.gaussiansToDraw(0).A=S.sub(0,1,0,1);
    Sim.gaussiansToDraw(0).a=s.sub(0,1);
    
    //tracking error
    cout << "estim error " <<maxDiff(s, ARR(Sim.x, Sim.y, Sim.theta)) << endl;
  }
}



int main(int argc,char **argv){
  Kalman();
  return 0;
}
