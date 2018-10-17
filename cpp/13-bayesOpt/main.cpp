#include <stdlib.h>
#include <Core/array.h>
#include <Core/util.h>
#include <Plot/plot.h>
#include <Algo/MLcourse.h>

double f(const arr& x){
  CHECK(x.N==1, "");
  return -.3*cos(7.*x(0)) + sumOfSqr(x);
}

int main(int argc,char **argv){
  rnd.clockSeed(); //random seed rng

  //display the function: evaluate it over a grid
  arr X = grid(1, -2., 2., 100);
  arr f_y(X.d0);
  for(uint i=0;i<X.d0;i++) f_y(i) = f(X[i]);

  plotClear();
  plotFunction(X, f_y);
  plot(false);
  mlr::wait();

  //iterate taking samplings and updating a GP
  uint T=20;
  arr data_X, data_y;
  for(uint t=0;t<T;t++){
    //a random decision
    arr x = -2. + 4.*rand(1);

    /* Tip to implement a better strategy:
     * -- generate a grid X (maybe use the one we already have)
     * -- compute some acquisition function alpha(i) over the grid
     * -- pick x = X[argmax(alpha)]  (the column in X with index argmax_i \alpha(x_i) )
     */

    //query the function
    double y = f(x);

    //augment the data
    data_X.append(~x);
    data_y.append(y);

    //compute a GP
    double fmean = sum(data_y)/data_y.N;
    KernelRidgeRegression GP(data_X, data_y, defaultKernelFunction, -1., fmean);

    //Plotting: evaluate the GP over the grid X:
    arr gp_sigma;
    arr gp_y = GP.evaluate(X, gp_sigma);

    plotClear();
    plotFunctionPrecision(X, gp_y, gp_y+gp_sigma, gp_y-gp_sigma);
    plotFunction(X, f_y);
    plotPoints(data_X, data_y);
    plot(false);
    mlr::wait();
  }

  return 0;
}
