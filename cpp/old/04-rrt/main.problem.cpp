#include <stdlib.h>
#include <Kin/roboticsCourse.h>
#include <Algo/ann.h>
#include <Plot/plot.h>

struct RRT{
private:
  ANN ann;      //ann stores all points added to the tree in ann.X
  uintA parent; //for each point we also store the index of the parent node
  double stepsize;
  uint nearest;

public:
  RRT(const arr& q0, double _stepsize){
    ann   .append(q0); //append q as the root of the tree
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
  
  void addLineDraw(const arr& q, Simulator& S){
    //I can't draw the edge in the 7-dim joint space!
    //But I can draw a projected edge in 3D endeffector position space:
    arr y_from,y_to;
    arr line;
    S.setJointAngles(ann.X[nearest], false);  S.kinematicsPos(y_from,"peg");
    S.setJointAngles(q             , false);  S.kinematicsPos(y_to  ,"peg");
    line.append(y_from); line.reshape(1,line.N);
    line.append(y_to);
    plotLine(line); //add a line to the plot

  }
  
  //some access routines
  uint getNearest(){ return nearest; }
  uint getParent(uint i){ return parent(i); }
  uint getNumberNodes(){ return ann.X.d0; }
  arr getNode(uint i){ return ann.X[i]; }
  void getRandomNode(arr& q){ q = ann.X[rnd(ann.X.d0)]; }
};

void RTTplan(){
  Simulator S("../02-pegInAHole/pegInAHole.ors");
  S.setContactMargin(.02); //this is 2 cm (all units are in meter)
  
  arr qT = {0.945499, 0.431195, -1.97155, 0.623969, 2.22355, -0.665206, -1.48356};
  arr q0, y_col, q;
  S.getJointAngles(q0);
  q=q0;

  S.setJointAngles(qT);
  cout <<"final posture (hit ENTER in the OpenGL window to continue!!)" <<endl;
  S.watch();

  double stepsize = .1;
  RRT rrt(q0, stepsize);
  
  plotModule.colors=false;
  
  for(uint i=0;i<10000;i++){
    rndUniform(q,-MLR_2PI,MLR_2PI,false);
    rrt.getProposalTowards(q);
    S.setJointAngles(q,false);

    S.kinematicsContacts(y_col);
    if(y_col(0)<.5){
      rrt.add(q);
      rrt.addLineDraw(q,S);
    }

    //some output
    if(!(i%1000)) S.setJointAngles(q); //updates diplay (makes it slow)
    cout <<"\rRRT sizes = " <<rrt.getNumberNodes() <<std::flush;
  }
  S.watch();
}

int main(int argc,char **argv){
  RTTplan();

  return 0;
}
