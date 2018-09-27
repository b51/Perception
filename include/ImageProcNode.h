/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProcNode.h
*
*          Created On: Fri 15 Dec 2017 09:12:29 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_VISION_IMAGE_PROC_NODE_H_
#define IKID_VISION_IMAGE_PROC_NODE_H_

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include <chrono>
#include <gflags/gflags.h>

#include <image_transport/image_transport.h>

#include "shm_util.h"
#include "VisionOptions.h"
#include "DarknetDetection.h"

namespace IKid
{

namespace Perception
{

class ImageProcNode
{
public:
  ImageProcNode(const VisionOptions& options);

  ~ImageProcNode();

  bool Init();

  bool CameraInit(int index, int w, int h);

  //void ListToRosMsg(const std::vector<Object>& in, ikid_msgs::ObjectList* out);

  void Run();

private:
  void UpdateShm(const std::vector<Object>& objs);

public:
  ::ros::NodeHandle node_handle_;
  ::ros::Publisher object_list_publisher_;
  //::ros::Rate loop_rate_(10);

  cv::VideoCapture cap_;

  VisionOptions options_;

  image_transport::ImageTransport it_;

private:
  std::unique_ptr<Detector> detector_;
};

} // namespace Perception
} // namespace IKid

#endif  // IKID_VISION_IMAGE_PROC_NODE_H_
