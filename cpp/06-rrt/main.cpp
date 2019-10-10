#include <stdlib.h>
#include <Algo/ann.h>
#include <Kin/kin.h>
#include <Kin/proxy.h>
#include <Gui/opengl.h>
#include <Kin/frame.h>


struct RRT{
private:
  ANN ann;      //ann stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node
  double stepsize;
  uint nearest;

public:
  rai::Mesh lines;

public:
  RRT(const arr& q0, double _stepsize){
    ann.append(q0);      //append q as the root of the tree
    parent.append(0);    //q has itself as parent
    stepsize = _stepsize;
  }

  double getProposalTowards(arr& q){
    //find NN
    nearest=ann.getNN(q);

    //compute little step
    arr d = q - ann.X[nearest]; //difference vector between q and nearest neighbor
    double dist = length(d);
    q = ann.X[nearest] + stepsize/dist * d;
    return dist;
  }

  void add(const arr& q){
    ann.append(q);
    parent.append(nearest);
  }

  void addLineDraw(const arr& q, rai::Configuration& K){
    //We can't draw the edge in the 7-dim joint space!
    //But we can draw a projected edge in 3D endeffector position space:
    arr y_from,y_to;
    arr line;
    K.setJointState(ann.X[nearest]);  K.kinematicsPos(y_from, NoArr, K.getFrameByName("peg"));
    K.setJointState(q             );  K.kinematicsPos(y_to  , NoArr, K.getFrameByName("peg"));
    lines.V.append(y_from);
    lines.V.append(y_to);
    lines.V.reshape(lines.V.N/3, 3);
    lines.T.append({lines.V.d0-2, lines.V.d0-1});
    lines.T.reshape(lines.T.N/2, 2);
  }

  //some access routines
  uint getNearest(){ return nearest; }
  uint getParent(uint i){ return parent(i); }
  uint getNumberNodes(){ return ann.X.d0; }
  arr getNode(uint i){ return ann.X[i]; }
  void getRandomNode(arr& q){ q = ann.X[rnd(ann.X.d0)]; }
};


void RTTplan(){
  rai::Configuration K("pegInAHole.g");

  arr qT = {0.945499, 0.431195, -1.97155, 0.623969, 2.22355, -0.665206, -1.48356};
  arr q0, y_col, q;
  q0 = K.getJointState();
  q=q0;

  cout <<"final posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.setJointState(qT);
  K.watch(true);
  
  cout <<"initial posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  K.setJointState(q0);
  K.watch(true);
  
  K.watch(false);

  double stepsize = .1;
  RRT rrt0(q0, stepsize);

  rai::Frame *f = K.addObject("lines0", NULL, rai::ST_mesh);  // Add a mesh for line drawing to the world
  f->setContact(0);
  
  uint i;
  for(i=0;i<10000;i++){
    //let rrt0 grow in random direction
    rndUniform(q,-RAI_2PI,RAI_2PI,false);

    // compute q_new
    rrt0.getProposalTowards(q);
    K.setJointState(q);
    
    // check if q is collision free
    K.stepSwift();
    K.kinematicsProxyCost(y_col, NoArr);
    if(y_col(0)<=1e-10){
      rrt0.add(q);
      rrt0.addLineDraw(q,K);
    }

    
    //some output
    if(!(i%100)){
      K["lines0"]->shape->mesh() = rrt0.lines;  // updates mesh lines0 with lines from rrt
      K.setJointState(q);
      K.watch(true);
      cout <<"\rRRT samples=" <<i <<" tree sizes = " <<rrt0.getNumberNodes() << std::flush;
    }
    
  }

}

int main(int argc,char **argv){
  rai::initCmdLine(argc,argv);

  RTTplan();

  return 0;
}
