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
  LOG(INFO) << "camera_index     : " << options_.camera_index;
  LOG(INFO) << "image_width      : " << options_.image_width;
  LOG(INFO) << "image_height     : " << options_.image_height;
  LOG(INFO) << "net_input_width  : " << options_.net_input_width;
  LOG(INFO) << "net_input_height : " << options_.net_input_height;
  LOG(INFO) << "topic            : " << options_.object_list_topic;
  LOG(INFO) << "queue            : " << options_.object_list_publisher_queue_size;

  LOG(INFO) << "net_prototxt     : " << options_.net_prototxt;
  LOG(INFO) << "model_file       : " << options_.model_file;
  LOG(INFO) << "object_thresh    : " << options_.object_thresh;
  LOG(INFO) << "nms_thresh       : " << options_.nms_thresh;
  LOG(INFO) << "hier_thresh      : " << options_.hier_thresh;
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
  image_proc_ = cartographer::common::make_unique<ImageProc>(options_);
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
