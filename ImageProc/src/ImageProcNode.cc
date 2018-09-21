/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProcNode.cc
*
*          Created On: Sat 02 Dec 2017 12:35:26 AM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "ImageProcNode.h"

namespace IKid
{
namespace Vision
{

ImageProcNode::ImageProcNode(const VisionOptions& options):
  options_(options)
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

#if 0
  if (options_.debug_mode)
  {
    //TODO Add image publish for debug
  }
#endif
  //object_list_publisher_ = node_handle_.advertise<ikid_msgs::ObjectList>(options_.object_list_topic, options_.object_list_publisher_queue_size);
}

ImageProcNode::~ImageProcNode()
{
  if (detector_)
    detector_.reset(nullptr);
}

bool ImageProcNode::Init()
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
    return false;

  /* Init Detector */
  std::unique_ptr<DarknetDetection>darknet_detector =
      cartographer::common::make_unique<DarknetDetection>();
  if (darknet_detector)
  {
    darknet_detector->SetNetParams(options_.object_thresh
                                 , options_.nms_thresh
                                 , options_.hier_thresh);

    darknet_detector->LoadModel(const_cast<char*>(options_.net_prototxt.c_str())
                              , const_cast<char*>(options_.model_file.c_str()));
    detector_ = std::move(darknet_detector);
  }

  return true;
}

bool ImageProcNode::CameraInit(int index, int w, int h)
{
  cap_.open(index);

  if (!cap_.isOpened())
  {
    LOG(ERROR) << "Can not open camera " << index;
    return false;
  }

  cap_.set(CV_CAP_PROP_FRAME_WIDTH, w);
  cap_.set(CV_CAP_PROP_FRAME_HEIGHT, h);
  LOG(INFO) << "Camera "<< index <<" inited";
  LOG(INFO) << "width: " << w << " height: " << h;

  return true;
}

/*
void ImageProcNode::ListToRosMsg(const std::vector<Object>& in, ikid_msgs::ObjectList* out)
{
  ikid_msgs::Object obj;
  for (auto i: in)
  {
    obj.x      = i.x;
    obj.y      = i.y;
    obj.width  = i.width;
    obj.height = i.height;
    obj.label  = i.label;
    obj.score  = i.score;

    out->objects.emplace_back(obj);
  }
}
*/

void ImageProcNode::UpdateShm(const std::vector<Object>& objs)
{
  for (const auto& obj : objs)
  {
    double v[4] = {obj.x*1., obj.y*1., obj.width*1., obj.height*1.};
    switch (obj.label)
    {
      case 0:       //TODO for ball
      {
        double* pr = vision_shm_set_ptr(ball_bbox_key, vision_keys[ball_bbox_key].size);
        for (int i = 0; i < vision_keys[ball_bbox_key].size; i++)
          *(pr+i) = v[i];
        break;
      }
      case 1:       //TODO for goal
      {
        double* pr = vision_shm_set_ptr(goal_bbox_key, vision_keys[goal_bbox_key].size);
        for (int i = 0; i < vision_keys[goal_bbox_key].size; i++)
          *(pr+i) = v[i];
        break;
      }
    }
  }
}

void ImageProcNode::Run()
{
  cv::Mat image;
  std::vector<Object> objs;
  //ikid_msgs::ObjectList list2pub;

#ifdef DEBUG
  int count = 0;
  auto start = std::chrono::high_resolution_clock::now();
#endif

  while (node_handle_.ok())
  {
    cap_ >> image;
    //image = cv::imread("/home/compiler/Pictures/Lenna.png");

#ifdef DEBUG
    count++;
    auto finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    if (elapsed.count() > options_.log_interval)
    {
      LOG(INFO) << "fps: " << count/elapsed.count();
      start = finish;
      count = 0;
    }

    for (auto obj: objs)
    {
      int label2color = obj.label % 255;
      cv::Scalar color(label2color, 0, label2color);
      cv::rectangle(image, cv::Rect(obj.x, obj.y, obj.width, obj.height), color, 2);
      cv::putText(image, std::to_string(obj.label), cv::Point(obj.x, obj.y)
                , CV_FONT_HERSHEY_PLAIN, 1
                , color);
      cv::putText(image, std::to_string(obj.score).substr(0, 3), cv::Point(obj.x+obj.width, obj.y)
                , CV_FONT_HERSHEY_PLAIN, 1
                , color);
    }

    cv::imshow("predict", image);
    cv::waitKey(5);
#endif

    objs.clear();
    detector_->Detect(image, objs);

    /*
    list2pub.header.stamp = ::ros::Time::now();
    list2pub.objects.clear();
    ListToRosMsg(objs, &list2pub);
    object_list_publisher_.publish(list2pub);
    */
  }
}


} // namespace Vision
} // namespace IKid
