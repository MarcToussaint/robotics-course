#include <Core/array.h>
#include <Plot/plot.h>
#include <Core/util.h>

void generateData(){
  arr X;
  arr x;
  arr A,a;
  double tau=0.02;
  A=ARR(1.,tau,0.,1.); A.reshape(2,2);
  a=ARR(0,-tau);
  x=ARR(0.,1.);
  
  for(uint t=0;t<100;t++){
    x = A*x + a;
    rndGauss(x,.01,true); //transition noise
    X.append(x);
  }
  X.reshape(X.N/2,2);
  
  ofstream fil1("dataReal");
  X.write(fil1," ","\n"," \n");
  fil1.close();

  rndGauss(X,0.1,true); //observation noise
  
  ofstream fil("data");
  X.write(fil," ","\n"," \n");
  fil.close();
  gnuplot("plot 'data' us 0:1");
  mlr::wait();
}

int main(int argc, char** argv){
  generateData();

  return 0;
}
