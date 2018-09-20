/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProc.h
*
*          Created On: Sat 02 Dec 2017 12:07:38 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef VISION_IMAGE_PROC_IMAGE_PROC_H_
#define VISION_IMAGE_PROC_IMAGE_PROC_H_

#include <stdio.h>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <glog/logging.h>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"

#include "VisionOptions.h"
#include "DarknetDetection.h"

namespace ikid {

namespace Vision {

class ImageProc
{

public:
  ImageProc(const VisionOptions& options);
  ~ImageProc();
  void Detect(const cv::Mat& image, std::vector<Object>& objects);

  bool Init();

private:
  const VisionOptions options_;

  Detector* detector_;
};

} // namespace Vision
} // namespace ikid

#endif
