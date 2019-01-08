#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>


/// Resample a set of particles to become a set of unit-weight particles
void resample(arr& X, arr& W){
  uintA s;           // resample indices
  s = sampleMultinomial_SUS(W, X.d0);
  arr Xnew(X.d0, 3); // memorize old particles

  for(uint i=0; i<X.d0; i++) {
    Xnew[i] = X[s(i)];
  }
  X = Xnew;
  W = 1. / X.d0;
}

int main(int argc,char **argv){
  CarSimulator S;
  arr u(2), y_meassured;

  // Step 1) initialize particles
  uint N = 200;
  arr X = zeros(N, 3);
  rndGauss(X, .5, true);
  arr W = ones(N) / double(N);

  // you have access to:
  // S.observationNoise (use when evaluating a particle likelihood)
  // S.dynamicsNoise (use when propagating a particle)

  S.gl->watch();
  for(uint t=0;t<1000;t++){
    u = ARR(.1, .2); //control signal
    S.step(u);
    S.getRealNoisyObservation(y_meassured);

    // Step 2) resample weighted particles
    resample(X,W);

    // Step 3) ``propagate'' each particle using the system dynamics (see internals of step function of CarSimulator)
    //   you can add noise to an array X with   rndGauss(X, S.dynamicsNoise, true);

    // Step 4) compute the likelihood weights for each particle
    //   to get the ideal observation for a state X[i] use this:
    //   S.getMeanObservationAtState(y_ideal, X[i]);

    S.particlesToDraw=X;

    cout <<u <<endl <<y_meassured <<endl;
  }

  return 0;
}
