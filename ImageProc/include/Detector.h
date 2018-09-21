/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: Detector.h
*
*          Created On: Sat 23 Dec 2017 12:07:10 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_VISION_IMAGE_PROC_DETECTOR_H_
#define IKID_VISION_IMAGE_PROC_DETECTOR_H_

#include <iostream>

#include <opencv2/opencv.hpp>

namespace IKid
{

namespace Vision
{

typedef struct
{
  int x;
  int y;
  int width;
  int height;

  int frame_id;
  int label;
  float score;
} Object;

class Detector
{
public:
  Detector() {}
  virtual ~Detector(){}

  virtual bool Detect(const cv::Mat& image, std::vector<Object>& objects) = 0;
};

} // namespace Vision
} // namespace IKid

#endif  // IKID_VISION_IMAGE_PROC_DETECTOR_H_
