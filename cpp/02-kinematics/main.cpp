#include <Kin/kin.h>
#include <Kin/frame.h>

void simpleArrayOperations(){

  /********* SEE PAGE 2 OF THE /doc/doc.pdf  ***********/
  
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
  rai::Configuration K("human.g");
  cout <<"joint dimensions = " <<K.getJointStateDimension() <<endl;

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.watch(true);        //pause and watch initial posture

  arr q;
  q = K.getJointState();

  q(0) += 0.1;                 //change the first entry of q-vector
  q = K.getJointState();
  K.watch(true);
  
  q = 0.;                      //set q-vector equal zero
  q = K.getJointState();
  K.watch(true);
}

void reach(){
  rai::Configuration K("human.g");
  arr q,W;
  uint n = K.getJointStateDimension();
  q = K.getJointState();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w,n);  //W is equal the Id_n matrix times scalar w

  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.watch(true);        //pause and watch initial posture

  arr y_target,y,J;
  for(uint i=0;i<10;i++){
    //1st task:
    y_target = {-0.2, -0.4, 1.1}; 
    K.evalFeature(y, J, FS_position, {"handR"});  //"handR" is the name of the right hand ("handL" for the left hand)

    //compute joint updates
    q += inverse_SymPosDef(~J*J + W)*~J*(y_target - y);
    //NOTATION: ~J is the transpose of J
    
    //sets joint angles AND computes all frames AND updates display
    K.setJointState(q);

    //optional: pause and watch OpenGL
    K.watch(true);
  }
}

void circle(){
  rai::Configuration C("human.g");
  arr q,W;
  uint n = C.getJointStateDimension();
  q = C.getJointState();
  double w = rai::getParameter("w",1e-4);
  W.setDiag(w,n);  //W is equal the Id_n matrix times scalar w

  C.watch(true);        //pause and watch initial posture


  arr y_target,y,J;
  for(uint i=0;i<1000;i++){
    y_target = C["rightTarget"]->getPosition(); //{-0.2, -0.4, 1.1};
    y_target += .2 * arr({cos((double)i/20), 0, sin((double)i/20)});
    
    C.evalFeature(y, J, FS_position, {"handR"});  //"handR" is the name of the right hand ("handL" for the left hand)

    cout <<i <<" current eff pos = " <<y <<"  current error = " <<length(y_target-y) <<endl;;
    q += inverse_SymPosDef(~J*J + W)*~J*(y_target - y);
    C.setJointState(q);
    C.watch(false);
  }
}



void multiTask(){
  rai::Configuration C("human.g");
  arr q,y_target,yVec_target,y,J,yVec,JVec,W,Phi,PhiJ;
  uint n = C.getJointStateDimension();
  q = C.getJointState();

  W.setDiag(1.,n);

  C.watch(true);        //pause and watch initial posture


  for(uint i=0;i<10000;i++){
    q = C.getJointState();

    Phi.clear();PhiJ.clear();

    y_target = C["rightTarget"]->getPosition(); //{-0.2, -0.4, 1.1};
    y_target += .2 * arr({cos((double)i/20), 0, sin((double)i/20)});

    //track circle with right hand
    C.evalFeature(y, J, FS_position, {"handR"});  //"handR" is the name of the right hand ("handL" for the left hand)
    Phi.append((y - y_target)/1e-2);
    PhiJ.append( J / 1e-2 );

    // 1st task: joint should stay close to zero



    // 2nd task: left hand should point upwards



    // 3rd task: robot should look at right hand



    //compute joint updates
    q -= 0.1*inverse(~PhiJ*PhiJ + W)*~PhiJ* Phi;
    C.setJointState(q);
    C.watch();
    rai::wait(.1);

  }
}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  switch(rai::getParameter<int>("mode", 3)){
  case 0:  simpleArrayOperations();  break;
  case 1:  watchAConfig();  break;
  case 2:  reach();  break;
  case 3:  circle();  break;
  case 4:  multiTask();  break;
  }

  return 0;
}
