#include <Kin/kin.h>
#include <Kin/frame.h>

void simpleArrayOperations(){

  /********* see also https://github.com/MarcToussaint/rai-maintenance/blob/master/help/arr.md  ***********/
  
  arr x = {.1, .2, .3};         //directly setting the array
  cout <<"x = " <<x <<endl;

  x += arr({2., 2., 2.});        //adding to an array
  cout <<"x = " <<x <<endl;

  x *= 1.;
  cout <<"x = " <<x <<endl;

  arr y = {-.3,-.2,-.1};
  y.append(x);                 //appending a vector to a vector
  cout <<"y = " <<y <<endl;

  arr M({4,3},{ 1, 0, 0,                //some 4x3 matrix
                0, 1, 0,
                0, 0, 1,
                1, 0, 0 });

  cout <<"M =\n" <<M <<endl;

  cout <<"transpose M =\n" <<~M <<endl;  //matrix transpose

  cout <<"M*x = " <<M*x <<endl;          //matrix-vector product

  cout <<"M^-1 =\n" <<inverse(M) <<endl; //matrix inverse

  M.append(M);                 //appending a matrix to a matrix
  cout <<"M =\n" <<M <<endl;
}


void watchAConfig(){
  rai::Configuration C("human.g");
  cout <<"joint dimensions = " <<C.getJointStateDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  C.watch(true);        //pause and watch initial posture

  arr q = C.getJointState();

  q(0) += 0.1;                 //change the first entry of q-vector
  C.setJointState(q);
  C.watch(true);
  
  q = 0.;                      //set q-vector equal zero
  C.setJointState(q);
  C.watch(true);
}


void reach(){
  rai::Configuration C("human.g");
  arr q,W;
  uint n = C.getJointStateDimension();
  q = C.getJointState();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w, n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  C.watch(true);        //pause and watch initial posture

  arr y_target,y,J;
  for(uint i=0;i<10;i++){
    //1st task:
    y_target = C["rightTarget"]->getPosition(); //{-0.2, -0.4, 1.1};
    C.evalFeature(y, J, FS_position, {"handR"});  //"handR" is the name of the right hand ("handL" for the left hand)

    //compute joint updates
    q += inverse_SymPosDef(~J*J + W)*~J*(y_target - y);
    //NOTATION: ~J is the transpose of J
    
    //sets joint angles AND computes all frames AND updates display
    C.setJointState(q);

    //optional: pause and watch OpenGL
    C.watch(true);
  }
}


int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  switch(rai::getParameter<int>("mode", 2)){ // ./x.exe -mode 1   allows you to select mode from cmd line
  case 0:  simpleArrayOperations();  break;
  case 1:  watchAConfig();  break;
  case 2:  reach();  break;
  }

  return 0;
}

