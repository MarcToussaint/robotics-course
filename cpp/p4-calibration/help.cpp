#include "help.h"

GetLargestObjects::GetLargestObjects(cv::Mat& rgb, cv::Mat& depth, const arr& hsvFilter, uint num){
  // blur
  cv::blur(rgb, rgb, cv::Size(5,5));

  // hsv filter
  cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
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
    sizes(largest)=0.; //done

    // draw the contour interior into the mask
    mask = cv::Scalar(0);
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
