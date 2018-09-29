/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProc.h
*
*          Created On: Fri 15 Dec 2017 09:12:29 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_PERCEPTION_IMAGE_PROC_H_
#define IKID_PERCEPTION_IMAGE_PROC_H_

#include <stdio.h>
#include <iostream>

#include <chrono>
#include <gflags/gflags.h>

#include "PerceptionOptions.h"
#include "DarknetDetector.h"

namespace IKid
{

namespace Perception
{

class ImageProc
{
public:
  ImageProc(const ImageProcOptions& options);

  ~ImageProc();

  void Init();

  //void ListToRosMsg(const std::vector<Object>& in, ikid_msgs::ObjectList* out);

  void Detect(cv::Mat& image, std::vector<Object>& objs);

public:
  //::ros::NodeHandle node_handle_;
  //::ros::Publisher object_list_publisher_;
  //::ros::Rate loop_rate_(10);

  //cv::VideoCapture cap_;

  ImageProcOptions options_;

  //image_transport::ImageTransport it_;

private:
  std::unique_ptr<Detector> detector_;
};

} // namespace Perception
} // namespace IKid

#endif  // IKID_VISION_IMAGE_PROC_NODE_H_
