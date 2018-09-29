/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: Observation.cc
*
*          Created On: Thu 27 Sep 2018 05:21:55 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "Observation.h"

namespace IKid
{
namespace Perception
{

Observation::Observation(const ObservationOptions& options
                       , tf2_ros::Buffer* const tf_buffer)
    : options_(options)
    , tf_bridge_(options_.tracking_frame
               , options_.lookup_transform_timeout_sec
               , tf_buffer)
{
  LOG(INFO) << "ball_diameter               : " << options.ball_diameter;
  LOG(INFO) << "camera_focus                : " << options.camera_focus;
  LOG(INFO) << "tracking_frame              : " << options.tracking_frame;
  LOG(INFO) << "lookup_transform_timeout_sec: " << options.lookup_transform_timeout_sec;
  /*
  const auto sensor_to_tracking =
      tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash(frame_id));

  trajectory_builder_->AddSensorData(
      sensor_id, carto::sensor::TimedPointCloudData{
                     time, sensor_to_tracking->translation().cast<float>(),
                     carto::sensor::TransformTimedPointCloud(
                         ranges, sensor_to_tracking->cast<float>())});
  */
}

Observation::~Observation()
{
}

void Observation::Observe(std::vector<Object>& objs)
{
}

} // namespace Perception
} // namespace IKid
