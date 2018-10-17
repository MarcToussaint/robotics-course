#include <iostream>
#include <Core/array.h>
#include <Plot/plot.h>
#include <Core/util.h>

using namespace std;

////////////////////////////////////////////////////////////////////////////////
// this part simulates the robot and its observations
// in real world applications, this part would be replaced by the real robot

arr robot;
double steering;
byteA maze;

void init(){
  robot=ARR(.5,.5); //initial position
  steering = 0.;
  read_ppm(maze,"maze.ppm");
  maze += (byte)'#'; //wall symbol
  cout <<maze <<endl;
}

void getObservation(arr& y,const arr& x,bool noisy){
  y.resize(5);
  y=1.;

  if(x.min()*maze.d0-1<0 || x.max()*maze.d0+1>maze.d0-1) return;

  byte s[5];
  //center top bottom left right
  s[0] = maze((1.-x(1))*maze.d0  , x(0)*maze.d1);
  s[1] = maze((1.-x(1))*maze.d0-1, x(0)*maze.d1  );
  s[2] = maze((1.-x(1))*maze.d0+1, x(0)*maze.d1  );
  s[3] = maze((1.-x(1))*maze.d0  , x(0)*maze.d1-1);
  s[4] = maze((1.-x(1))*maze.d0  , x(0)*maze.d1+1);

  for(uint k=0;k<5;k++) y(k) = (s[k]=='#'?1.:0.);

  if(noisy){
    for(uint k=0;k<5;k++) if(rnd.uni()<.01) y(k)=1.-y(k);
  }
}

void simulateOneStep(){
  arr old=robot;
  steering += .5*rnd.gauss();
  robot(0) += .01*cos(steering);
  robot(1) += .01*sin(steering);
  arr y;
  getObservation(y,robot,false);
  if(y(0)) robot=old;
}

////////////////////////////////////////////////////////////////////////////////

double likelihood(const arr& y_true,const arr& x){
  arr y;
  getObservation(y,x,false);
  double l=1.;
  for(uint k=0;k<y.N;k++) l*= (y_true(k)==y(k)?.9:.2);
  return l;
}


int main(int argc, char** argv){
  uint N=1000;
  arr X(N,2),W(N);           // N 2-dim particles and N weights
  rndUniform(X,0.,1.,false); // initialize random in [0,1]^2
  W = 1./N;                  // weights uniform
  plotOpengl();
  init();
  arr y;

  for(uint t=0;t<10000;t++){
    // display robot and particles
    plotClear();
    plotPoint(2.*robot-1.);
    plotPoints(2.*X-1.);
    plot(false); //set 'true' to pause and watch...

    // step the simulator
    simulateOneStep();
    getObservation(y,robot,false);
    cout <<"\robservation y=" <<y <<flush;

    // TODO: particle filtering!
  }

  return 0;
}
