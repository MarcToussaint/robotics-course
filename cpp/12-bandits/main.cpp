#include <stdlib.h>
#include <Core/array.h>

arr Bandits = {.7, .3};  //2 bandits

double getReturn(uint decision){
  return rnd.uni() < Bandits(decision) ? 1.:0.;
//  return Bandits(decision) + .1*rnd.gauss();
}


int main(int argc,char **argv){

  rnd.clockSeed(); //random seed rng

  uint K=1000; //trials
  uint T=1000;  //episode lengths
  uint B=Bandits.N;
  arr avg_regret = zeros(T); //regret as a function of time

  for(uint k=0;k<K;k++){ //trials
    arr y = zeros(B);  //store the total return from each bandid
    arr ni = zeros(B); //store the # each has been chosen
    for(uint t=0;t<T;t++){ //time
      uint decision = rnd(B);
      if(t<B) decision = t; //first: play each bandit at least once
      if(t>=B){ //do something cleverer
//        arr Q = zeros(B);
//        for(uint b=0;b<B;b++) Q(b) = ???
//        decision = argmax(Q);
      }
      double r = getReturn(decision);
      y(decision) += r;
      ni(decision) += 1.;
      avg_regret(t) += r;
    }
  }
  avg_regret /= (double)K;
  FILE("z.R") <<avg_regret.reshape(T,1);
  gnuplot("plot 'z.R' us 1 t 'regret'", true);
  return 0;
}
