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
  image_proc_->Init();
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
  // set object detected false
  for (const auto& vision_key: vision_keys)
  {
    double* pr = vision_shm_set_ptr(vision_key.first, vision_key.second.size);
    *pr = 0.;
  }

  for (int i = 0; i < objs_.size(); i++)
  {
    int size = vision_keys[ball_detection_key].size;
    double v[size] = {1 // detected
                     // 3d position
                     , positions_[i].x()
                     , positions_[i].y()
                     , positions_[i].z()
                     // bounding box
                     , double(objs_[i].x)
                     , double(objs_[i].y)
                     , double(objs_[i].width)
                     , double(objs_[i].height)};
    double *pr;
    switch (objs_[i].label)
    {
      case BALL:
      {
        pr = vision_shm_set_ptr(ball_detection_key, size);
        break;
      }
      case GOAL_POST:
        pr = vision_shm_set_ptr(goal_detection_key, size);
        break;
      case PENALTY_SPOT:
        pr = vision_shm_set_ptr(spot_detection_key, size);
        break;
      case TEAMMATE:
        pr = vision_shm_set_ptr(teammate_detection_key, size);
        break;
      case OPPONENT_ROBOT:
        pr = vision_shm_set_ptr(opponent_detection_key, size);
        break;
      default:
        break;
    }
    if (objs_[i].label >= BALL && objs_[i].label < OBJECT_END)
    {
      for (int j = 0; j < size; j++)
        *(pr+j) = v[j];
    }
  }
}

void PerceptionNode::Run()
{
  /* Init Observation */
  // Move tf_buffer from Init to here for tf_buffer will be used in observation_,
  // and if construct tf_buffer in Init(), its lifetime will end when Init() done and
  // the pointer &tf_buffer will have no object that lead to error.
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
  tf2_ros::TransformListener tf(tf_buffer);
  observation_ = cartographer::common::make_unique<Observation>(options_.observation_options, &tf_buffer);

  cv::Mat image;
  //ikid_msgs::ObjectList list2pub;

  while (node_handle_.ok())
  {
    objs_.clear();
    positions_.clear();

    cap_ >> image;
    image_proc_->Detect(image, objs_);
    observation_->Observe(objs_, positions_);
    UpdateShm();
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
