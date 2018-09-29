/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: PerceptionNode.h
*
*          Created On: Thu 27 Sep 2018 04:49:37 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_PERCEPTION_PERCEPTION_NODE_H_
#define IKID_PERCEPTION_PERCEPTION_NODE_H_

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/transform_listener.h>
#include <glog/logging.h>

#include "shm_util.h"
#include "ImageProc.h"
#include "Observation.h"
#include "PerceptionOptions.h"

namespace IKid
{

namespace Perception
{

class PerceptionNode
{
public:
  PerceptionNode(const PerceptionOptions& options);
  ~PerceptionNode();

  void Init();
  void Run();

private:
  bool CameraInit(int index, int w, int h);
  void UpdateShm();

public:
  ::ros::NodeHandle node_handle_;
  image_transport::ImageTransport it_;

private:
  PerceptionOptions options_;
  cv::VideoCapture cap_;
  std::unique_ptr<ImageProc> image_proc_;
  std::unique_ptr<Observation> observation_;
};

} // namespace Perception
} // namespace IKid

#endif // IKID_PERCEPTION_PERCEPTION_NODE_H_
