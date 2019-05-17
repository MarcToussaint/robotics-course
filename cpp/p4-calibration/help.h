#include <Perception/opencv.h> //always include this first!
#include <Perception/opencvCamera.h>
#include <Perception/depth2PointCloud.h>

struct GetLargestObjects {
  //outputs
  std::vector<std::vector<cv::Point> > contours;
  cv::Mat hsv, mask;
  arr sizes;

  arr objCoords;

  GetLargestObjects(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter, uint num=1);
};
