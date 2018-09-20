/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProc.cc
*
*          Created On: Fri 01 Dec 2017 11:58:12 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "ImageProc.h"

using namespace ikid;
using namespace Vision;

ImageProc::ImageProc(const VisionOptions& options)
  : options_(options)
{
}

ImageProc::~ImageProc()
{
  if (detector_)
  {
    delete detector_;
    detector_ = nullptr;
  }
}

bool ImageProc::Init()
{
  DarknetDetection* darknet_detector = new DarknetDetection();
  if (darknet_detector)
  {
    darknet_detector->SetNetParams(options_.object_thresh
                         , options_.nms_thresh
                         , options_.hier_thresh);

    darknet_detector->LoadModel(const_cast<char*>(options_.net_prototxt.c_str())
                              , const_cast<char*>(options_.model_file.c_str()));

    detector_ = darknet_detector;
    return true;
  }

  return false;
}

void ImageProc::Detect(const cv::Mat& image, std::vector<Object>& objects)
{
  detector_->Detect(image, objects);
}
