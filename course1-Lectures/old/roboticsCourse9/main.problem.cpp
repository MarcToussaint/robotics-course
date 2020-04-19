#include <iostream>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main(int argc, char** argv){
  VideoCapture capture;
  capture.open(0);

  if(!capture.isOpened()){
    cerr <<"Could not initialize video capture" <<endl;
    return -1;
  }

  Mat img,last;
  uint t;
  for(t=0;;t++){
    capture >>img;
    imshow("original image", img);

    if((waitKey(2)&0xff)==27)  break;
  }

  return 0;
}
