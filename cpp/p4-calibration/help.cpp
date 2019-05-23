#include "help.h"

GetLargestObjects::GetLargestObjects(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter, uint num, bool bgr){
  // blur
  cv::blur(rgb, rgb, cv::Size(5,5));

  // hsv filter
  if(bgr) cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
  else cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

  cv::inRange(hsv,
              cv::Scalar(hsvFilter(0,0), hsvFilter(0,1), hsvFilter(0,2)),
              cv::Scalar(hsvFilter(1,0), hsvFilter(1,1), hsvFilter(1,2)), mask);

  // compute contours
  cv::findContours(mask, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

  // find largest
  sizes.resize(contours.size());
  for(uint i=0; i<contours.size(); i++) sizes(i) = cv::contourArea(cv::Mat(contours[i]));

  objCoords.resize(num, 3).setZero();

  if(sizes.N<num) num=sizes.N;

  for(uint i=0;i<num;i++){
    uint largest = sizes.maxIndex();
    double size = sizes(largest);
    sizes(largest)=0.; //done



    // draw the contour interior into the mask
    mask = cv::Scalar(0);
    if(size>300.){
      cv::drawContours(mask, contours, largest, cv::Scalar(128), CV_FILLED);

      // grab the depth values and mean x,y coordinates
      floatA depthValues;
      double objX=0.,objY=0.;
      for(int y=0;y<mask.rows;y++) for(int x=0;x<mask.cols;x++){
        if(mask.at<byte>(y,x)){
          float d = depth.at<float>(y,x); //myDepth(y,x);
          if(d>.1 && d<2.){
            depthValues.append(d);
            objX += x;
            objY += y;
          }
        }
      }

      if(depthValues.N){
        objX /= double(depthValues.N);
        objY /= double(depthValues.N);

        // median
        double objDepth = depthValues.median_nonConst();
        // mean
        //double objDepth = sum(depthValues)/double(depthValues.N);

        objCoords[i] = {objX, objY, objDepth};

        cv::drawContours( rgb, contours, largest, cv::Scalar(255,0,0), 2, 8);
        cv::drawContours( depth, contours, largest, cv::Scalar(0), 2, 8);
      }
    }
  }
}

//===========================================================================

void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P, bool verbose){
  //We decompose P to get K, R, t (calibration, rotation, translation)
  //get the square part
  arr KR(3, 3), KRt(3, 1);
  P.getMatrixBlock(KR , 0, 0);
  P.getMatrixBlock(KRt, 0, 3);
  lapack_RQ(K, R, KR);
  t = -inverse(KR)*KRt;
  t.reshape(3);
//  K /= K(2, 2);   //the scaling of K is arbitrary, convention: $K_{3, 3}=1$
  //R[2]() *= -1.; //OpenGL is flipping the z-axis...
  //rai::Quaternion r;  r.setMatrix(R.p);

//  arr PP=~(K*R);  PP.append(-K*R*t);  PP=~PP;  /*PP/=PP.elem(0);*/  cout <<PP <<endl <<P <<endl;

  if(verbose){
    cout <<"\nProjection Matrix:"
         <<"\nP=" <<P
         <<"\nK=" <<K
         <<"\nR=" <<R
         // <<"\nr=" <<r
         <<"\nt=" <<t
         // <<"\nposition error = " <<length(t-S.getCameraTranslation())
         <<endl;
  }
}

void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P){
  arr KR(3, 3);
  P.getMatrixBlock(KR , 0, 0);
  t = ~P.col(3);
  KR = inverse(KR);
  lapack_RQ(K, R, KR);
}

double projectionError(const arr& P, const arr& x, const arr& X){
  uint N=x.d0;
  arr x2 = X*~P;
  for(uint i=0; i<N; i++) x2[i]() /= x2(i, 2);
  //cout <<x.sub(0, 10, 0, -1) <<x2.sub(0, 10, 0, -1) <<endl;
  return sqrt(sumOfSqr(x-x2)/N);
}

void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X){
  //construct the matrix A:
  uint N=x.d0;
  arr A, zero(4);
  zero.setZero();
  for(uint i=0; i<N; i++){
    //COMPARE slide 24 lecture 8!
    A.append(zero);         A.append(-x(i, 2)*X[i]); A.append(x(i, 1)*X[i]);
    A.append(x(i, 2)*X[i]);  A.append(zero);         A.append(-x(i, 0)*X[i]);
  }
  A.reshape(2*N, 12); //each point contributes two equations (two rows to A)

  //now we need to find p that minimizes |Ap| (subject to |p|=1 because the scaling doesn't matter)
  arr U, w, V;
  svd(U, w, V, ~A*A); //Singular Value Decomposition of A^T A
  cout <<"Algebraic projection error = " <<w(11) <<endl;
  P = (~V)[11];
  P.reshape(3, 4);
  P /= P(0, 0);
  cout <<"image error=" <<projectionError(P, x, X) <<endl;
}

void stereoTriangulation(arr& X, const arr& xL, const arr& xR, const arr& PL, const arr& PR){
  arr B;
  B = skew(xL)*PL;
  B.append(skew(xR)*PR);
  B.reshape(6, 4);
  B.delRows(2);  B.delRows(4);  B.reshape(4, 4);
  //now we need to find p that minimizes |Ap| (subject to |p|=1 because the scaling doesn't matter)
  arr U, w, V;
  svd(U, w, V, ~B*B); //Singular Value Decomposition of A^T A
  //cout <<"Algebraic triangulation error = " <<w(3) <<endl;
  X = (~V)[3];
  X.reshape(4);
  X /= X(3);
}

void stereoTriangulation_nonhom(arr& X, const arr& x, const arr& PL, const arr& PR){
  arr Xhom;
  stereoTriangulation(Xhom, ARR(x(0), x(1), 1.), ARR(x(2), x(3), 1.), PL, PR);
  X.resize(3);
  memmove(X.p, Xhom.p, 3*X.sizeT);
}
