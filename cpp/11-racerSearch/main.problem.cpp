#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Gui/opengl.h>
#include <Algo/algos.h>

void drawEnv(void*);
void glDrawRacer(void *classP);

struct RacerState : VectorFunction{
  //state
  arr q, q_dot;

  //controls
  double u, tau;

  //constant parameters
  double r, l, mA, mB, IA, IB, g, dynamicsNoise; //, c1,c2,Mp,Mc,l ;

  OpenGL gl;
  RacerState(){
    q.resize(2).setZero();
    q_dot.resize(2).setZero();
    q(1) = .01; //1MT_PI/2; //slighly non-upright //MLR_PI; //haning down

    //init constants
    tau = 0.01;
    r=.05;
    l=.5;
    mA=.05;
    mB=.5;
    IA=mA*mlr::sqr(.5*r);
    IB=mB*mlr::sqr(.2*l);
    g = 9.8;

    dynamicsNoise = 0;

    gl.add(drawEnv, this);
    gl.add(glDrawRacer, this);
    gl.camera.setPosition(3., -20., 5.);
    gl.camera.focus(0, 0, .2);
    gl.camera.upright();
    gl.update();
  }

  void getDynamics(arr&M, arr& F, arr& B){
    B = ARR(1./r, -1.); //control matrix

    arr M_A, M_B;
    M_A.setDiag(ARR(mA,mA,IA));
    M_B.setDiag(ARR(mB,mB,IB));

    arr J_A, J_B, J_B_dash;
    J_A.resize(3,2).setZero();  J_A(0,0)=1.;  J_A(2,0)=1./r;
    J_B.resize(3,2).setZero();
    J_B(0,0) = J_B(2,1) = 1.;
    J_B(0,1) =  l*::cos(q(1));
    J_B(1,1) = -l*::sin(q(1));
    J_B_dash.resize(3,2).setZero();
    J_B_dash(0,1) = -l*::sin(q(1));
    J_B_dash(1,1) = -l*::cos(q(1));

    M = ~J_A * M_A * J_A + ~J_B * M_B * J_B;

    arr M_dot = 2. * ~J_B * M_B * J_B_dash * q_dot(1);

    F = M_dot*q_dot
        - ( (~q_dot*~J_B*M_B*J_B_dash*q_dot).scalar() + g*mB*l*::sin(q(1)) )*ARR(0., 1.);
  }

  void fv(arr& y, arr& J, const arr& x){ //returns the acceleration given the state -> used by rk4
    q = x[0];
    q_dot = x[1];

    arr M, Minv, F, B;
    getDynamics(M, F, B);
    inverse_SymPosDef(Minv, M);

    y = Minv * (B*u - F);

//    double T = 0.5 * (~q_dot * M * q_dot).scalar(); //kinetic energy
//    double U = g * mB * l * ::cos(q(1)); //potential energy
//    cout <<"energy = " <<T+U <<endl;

    if(&J) HALT("");
  }

  void stepDynamics(double _u){
    u=_u;
    arr x;
    rk4_2ndOrder(x, cat(q,q_dot).reshape(2,2), *this, tau);
    q=x[0];
    q_dot=x[1];

    if(dynamicsNoise) rndGauss(q_dot, ::sqrt(tau)*dynamicsNoise, true);
  }

};

void glDrawRacer(void *classP){
  RacerState *s=(RacerState*)classP;
  double GLmatrix[16];
  mlr::Transformation f;
  f.setZero();
  //wheels
  f.addRelativeTranslation(s->q(0), 0, s->r);
  f.addRelativeRotationRad(s->q(0)/s->r, 0, 1, 0);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.8,.2,.2);
  glDrawBox(2.*s->r, .01, 2.*s->r);
  //pole
  f.setZero();
  f.addRelativeTranslation(s->q(0), 0, s->r);
  f.addRelativeRotationRad(s->q(1), 0., 1., 0.);
  f.addRelativeTranslation(0., 0., s->l);
  f.getAffineMatrixGL(GLmatrix);
  glLoadMatrixd(GLmatrix);
  glColor(.2,.2,.2);
  glDrawBox(.01, .05, 2.*s->l);
  glLoadIdentity();
}

void testDraw(){
  RacerState s;
  s.gl.watch();
}

void TestMove(){
  RacerState s;
  for (uint t=0; t<400000; t++){
    s.gl.text.clear() <<t <<" ; " <<s.q(0) << " ; " <<s.q(1);
    s.stepDynamics(0.0);
    s.gl.update();
  }
}

int main(int argc,char **argv){
//  testDraw();
  TestMove();

  return 0;
}
