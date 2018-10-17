#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>

//resample a set of particles to become a set of unit-weight particles
void resample(arr& X, arr& W){
  uintA s=sampleMultinomial_SUS(W,X.d0);          // resample indices   // Stochastic Universal Sampling
  arr Xnew(X.d0,3); // memorize old particles
  for(uint i=0;i<X.d0;i++) Xnew[i] = X[s(i)];
  X = Xnew;
  W = 1./X.d0;
}

int main(int argc,char **argv){
  CarSimulator S;
  arr u(2),y_meassured;
  
  //you have access to:
  //S.observationNoise (use when evaluating a particle likelihood)
  //S.dynamicsNoise (use when propagating a particle)
  
  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
    S.step(u);
    S.getRealNoisyObservation(y_meassured);

    //1) resample weighted particles
    
    //2) ``propagate'' each particle using the system dynamics (see internals of step function of CarSimulator)
    //   add noise to an array is done using   rndGauss(X, S.dynamicsNoise, true);

    //3) compute the likelihood weights for each particle
    //   to evaluate the likelihood of the i-th particle use this:
    //   S.getTrueLandmarksInState(y_at_particle, X(i,0), X(i,1), X(i,2));
    //   and then compare to y_meassured (using a Gauss function with sdv 0.5) to compute the likelihood
    //   don't for get to normalize weights

    //to draw some particles use this (X is a n-times-3 array storing the particles)
    //S.particlesToDraw=X;
    
    cout <<u <<endl <<y_meassured <<endl;
  }

  return 0;
}
