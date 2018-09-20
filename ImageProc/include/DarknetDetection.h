/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: DarknetDetection.h
*
*          Created On: Tue 19 Dec 2017 01:02:29 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef VISION_IMAGE_PROC_DARKNET_DETECTION_H_
#define VISION_IMAGE_PROC_DARKNET_DETECTION_H_

#include <iostream>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "darknet.h"

#include "Detector.h"

using namespace std;

namespace ikid
{

namespace Vision
{

class DarknetDetection: public Detector
{
public:
  DarknetDetection();
  ~DarknetDetection();

  void LoadModel(char* cfg_file, char* weights_file);

  void SetNetParams(float object_thresh = 0.24, float nms_thresh = 0.3, float hier_thresh = 0.5)
  {
    object_thresh_ = object_thresh;
    nms_thresh_ = nms_thresh;
    hier_thresh_ = hier_thresh;
  }

  bool Detect(const cv::Mat& image, vector<Object>& objects);

private:
  float* Mat2Float(const cv::Mat& image);
  void RescaleBoxes(const cv::Mat& image, int num, box *boxes);

private:
  network *net_;

  float object_thresh_;
  float nms_thresh_;
  float hier_thresh_;
};

} // namespace Vision
} // namespace ikid

#endif
