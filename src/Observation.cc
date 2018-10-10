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

using Rigid3d = cartographer::transform::Rigid3d;

const std::string& CheckNoLeadingSlash(const std::string& frame_id)
{
  if (frame_id.size() > 0)
  {
    CHECK_NE(frame_id[0], '/') << "The frame_id " << frame_id
                               << " should not start with a /. See 1.7 in "
                                  "http://wiki.ros.org/tf2/Migration.";
  }
  return frame_id;
}

Observation::Observation(const ObservationOptions& options
                       , tf2_ros::Buffer* const tf_buffer)
    : options_(options)
    , tf_bridge_(options.tracking_frame
               , options.lookup_transform_timeout_sec
               , tf_buffer)
{
  LOG(INFO) << "ball_diameter               : " << options.ball_diameter;
  LOG(INFO) << "camera_focus                : " << options.camera_focus;
  LOG(INFO) << "camera_center_x             : " << options.camera_center_x;
  LOG(INFO) << "camera_center_y             : " << options.camera_center_y;
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

void Observation::Observe(std::vector<Object>& objs, std::vector<Eigen::Vector3d>& positions)
{
  cartographer::common::Time time = FromRos(ros::Time::now());
  auto camera_to_tracking = tf_bridge_.LookupToTracking(time, CheckNoLeadingSlash("camera_link"));
  Eigen::Vector3d camera_matrix;
  int x0 = options_.camera_center_x;
  int y0 = options_.camera_center_y;
  for (auto& obj : objs)
  {
    double scale;

    camera_matrix << options_.camera_focus, -(obj.x - x0), -(obj.y - y0);
    Eigen::Vector3d position = camera_to_tracking->cast<double>() * camera_matrix;
    switch (obj.label)
    {
      case BALL:
      {
        scale = std::max(obj.width, obj.height) / options_.ball_diameter;
        break;
      }
      case GOAL_POST:
        break;
      case PENALTY_SPOT:
        break;
      case TEAMMATE:
        break;
      case OPPONENT_ROBOT:
        break;
      default:
        break;
    }
    position = position/scale;
    positions.emplace_back(position);
  }
}

} // namespace Perception
} // namespace IKid
