/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: Observation.h
*
*          Created On: Thu 27 Sep 2018 07:32:09 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#ifndef IKID_PERCEPTION_OBSERVATION_H_
#define IKID_PERCEPTION_OBSERVATION_H_

#include <memory>

#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"

#include "PerceptionOptions.h"
#include "ImageProc.h"
#include "tf_bridge.h"

namespace IKid
{
namespace Perception
{

class Observation
{
public:
  Observation(const ObservationOptions& options, tf2_ros::Buffer* const tf_buffer);
  ~Observation();
  void Observe(std::vector<Object>& objs, std::vector<Eigen::Vector3d>& positions);

private:
  const TfBridge tf_bridge_;
  ObservationOptions options_;
};

} // namespace Perception
} // namespace IKid

#endif
