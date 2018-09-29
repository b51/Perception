/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: PerceptionNode.cc
*
*          Created On: Thu 27 Sep 2018 04:31:59 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "PerceptionNode.h"

namespace IKid
{
namespace Perception
{

PerceptionNode::PerceptionNode(const PerceptionOptions& options)
    : options_(options)
    , it_(node_handle_)
{
  LOG(INFO) << "debug_mode       : " << options_.debug_mode;
  LOG(INFO) << "log_level        : " << options_.log_level;
  LOG(INFO) << "camera_index     : " << options_.camera_index;
  LOG(INFO) << "image_width      : " << options_.image_width;
  LOG(INFO) << "image_height     : " << options_.image_height;
}

PerceptionNode::~PerceptionNode()
{
}

void PerceptionNode::Init()
{
  /* Open shms */
  int ret;
  ret = vision_shm_open();
  if (ret < 0)
    LOG(ERROR) << "Vision shm open falied.";
  else
    LOG(INFO) << "Vision shm opend.";

  ret = world_shm_open();
  if (ret < 0)
    LOG(ERROR) << "World shm open falied.";
  else
    LOG(INFO) << "World shm opend.";

  /* Init Camera */
  if (!CameraInit(options_.camera_index
                , options_.image_width
                , options_.image_height))
    LOG(FATAL) << "Cannot open Camera " << options_.camera_index
               << " with width: " << options_.image_width
               << " height: " << options_.image_height;
  /* Init ImageProc */
  image_proc_ = cartographer::common::make_unique<ImageProc>(options_.image_proc_options);

  /* Init Observation */
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);

  observation_ = cartographer::common::make_unique<Observation>(options_.observation_options, &tf_buffer);
}

bool PerceptionNode::CameraInit(int index, int w, int h)
{
  cap_.open(index);

  if (!cap_.isOpened())
    return false;

  cap_.set(CV_CAP_PROP_FRAME_WIDTH, w);
  cap_.set(CV_CAP_PROP_FRAME_HEIGHT, h);
  LOG(INFO) << "Camera "<< index <<" inited";
  LOG(INFO) << "width: " << w << " height: " << h;

  return true;
}


void PerceptionNode::UpdateShm()
{
}

void PerceptionNode::Run()
{

  cv::Mat image;
  std::vector<Object> objs;
  //ikid_msgs::ObjectList list2pub;

  while (node_handle_.ok())
  {
    cap_ >> image;
    objs.clear();
    image_proc_->Detect(image, objs);
    observation_->Observe(objs);

    /*
    list2pub.header.stamp = ::ros::Time::now();
    list2pub.objects.clear();
    ListToRosMsg(objs, &list2pub);
    object_list_publisher_.publish(list2pub);
    */
  }
}

} // namespace Perception
} // namespace IKid
