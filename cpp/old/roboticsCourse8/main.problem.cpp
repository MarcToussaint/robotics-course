#include <Kin/roboticsCourse.h>

void inferCameraProjectionMatrix(){
  uint N = 100;
  double pixelNoise = .5;
  VisionSimulator S;
  arr  X;
  S.getRandomWorldPoints(X,N);

  for(uint k=0;k<100;k++){
    cout <<"\n\n*** iteration " <<k <<endl;
    //the real world projects these points onto the camera (here: simulated)
    arr  x;
    S.projectWorldPointsToImagePoints(x,X,pixelNoise);
    // the i-th world point is X[i] (4D because in homogeneous coordinates)
    // the i-th image point is x[i] (3D because in homogeneous coordinates)
  

    //** TODO: now you have to infer the project matrix P  (see slide 24 lecture 8)
    //construct the matrix A:
    arr A,Arow;
    //  each x[i]=``(x,y,w)'' and X[i] contributes 2 rows of lenght 12
    //  use A.append(row); to append these lines rows
    //  use A.reshape(2*N,12); in the end to ensure A is of right size
    //use SVD to decompose A^T A (e.g.,call svd(U,w,V,~A*A); )
    //get the 12 parameters of P from V (the last column is (~V)[11] )
    //reshape this to the 3x4 projection matrix P (e.g., P.reshape(3,4); )


    //** What follows is code to extract human-readable information from P (see slide 32!)
    //We decompose P to get K,R,t (calibration, rotation, translation)
    //get the square part
    /*
    arr KR(3,3),K,R,KRt(3,1);
    P.getMatrixBlock(KR ,0,0);
    P.getMatrixBlock(KRt,0,3);
    arr t = -inverse(KR)*KRt;  t.reshape(3);
    lapack_RQ(K,R,KR);
    if(determinant(R)<1.){ //account it to the calibration matrix, that z-axis is flipped! (OpenGL: glFrustrum flips z-axis)
      R[2]()*=-1.; //flip z-axis
      for(uint i=0;i<3;i++) K(i,2) *=-1.; //flip z-axis
      K /= -K(2,2); //the scaling of K is arbitrary, convention: $K_{3,3}=-1$
    }else{
      K /= K(2,2); //the scaling of K is arbitrary, convention: $K_{3,3}=1$
    }
    R = ~R; //in my convention this R is the inverse rotation!
    mlr::Quaternion r;  r.setMatrix(R.p);

    cout <<"\nProjection Matrix computed from noise points:"
	 <<"\nP=" <<P
	 <<"\nK=" <<K
	 <<"\nR=" <<R
	 <<"\nr=" <<r
	 <<"\nt=" <<t
	 <<"\nposition error = " <<length(t-S.getCameraTranslation()) <<endl;
    */
    S.watch();
  }
  
}

int main(int argc, char **argv){
  inferCameraProjectionMatrix();
}
