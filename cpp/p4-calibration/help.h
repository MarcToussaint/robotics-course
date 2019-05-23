#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

struct GetLargestObjects {
  //outputs
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat hsv, mask;
  arr sizes;

  arr objCoords;

  GetLargestObjects(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter, uint num=1, bool bgr=false);
};

void decomposeCameraProjectionMatrix(arr& K, arr& R, arr& t, const arr& P, bool verbose);
void decomposeInvProjectionMatrix(arr& K, arr& R, arr& t, const arr& P);
double projectionError(const arr& P, const arr& x, const arr& X);
void estimateCameraProjectionMatrix(arr& P, const arr& x, const arr& X);
void stereoTriangulation(arr& X, const arr& xL, const arr& xR, const arr& PL, const arr& PR);


void stereoTriangulation_nonhom(arr& X_3d, const arr& x_4d, const arr&PL, const arr& PR);
