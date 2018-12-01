/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef IKID_PERCEPTION_PERCEPTION_OPTIONS_H_
#define IKID_PERCEPTION_PERCEPTION_OPTIONS_H_

#include <string>
#include <vector>
#include <tuple>

#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/port.h"

namespace IKid
{
namespace Perception
{

struct ObservationOptions
{
  double ball_diameter;
  int camera_focus;
  int camera_center_x;
  int camera_center_y;

  std::string tracking_frame;
  double lookup_transform_timeout_sec;
};

inline ObservationOptions CreateObservationOptions(
    cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
  ObservationOptions options;
  options.ball_diameter =
      lua_parameter_dictionary->GetDouble("ball_diameter");
  options.camera_focus =
      lua_parameter_dictionary->GetInt("camera_focus");
  options.camera_center_x =
      lua_parameter_dictionary->GetInt("camera_center_x");
  options.camera_center_y =
      lua_parameter_dictionary->GetInt("camera_center_y");

  options.tracking_frame =
      lua_parameter_dictionary->GetString("tracking_frame");
  options.lookup_transform_timeout_sec =
      lua_parameter_dictionary->GetDouble("lookup_transform_timeout_sec");
  return options;
}

struct ImageProcOptions
{
  std::string object_list_topic;
  int object_list_publisher_queue_size;

  int net_input_width;
  int net_input_height;

  // net options
  std::string net_prototxt;
  std::string model_file;
  float object_thresh;
  float nms_thresh;
  float hier_thresh;

  int log_interval;
};

inline ImageProcOptions CreateImageProcOptions(
    cartographer::common::LuaParameterDictionary* const lua_parameter_dictionary)
{
  ImageProcOptions options;

  options.object_list_publisher_queue_size = lua_parameter_dictionary->GetInt("publisher_queue_size");

  options.net_input_width = lua_parameter_dictionary->GetInt("net_input_width");
  options.net_input_height = lua_parameter_dictionary->GetInt("net_input_height");

  options.net_prototxt = lua_parameter_dictionary->GetString("net_prototxt");
  options.model_file = lua_parameter_dictionary->GetString("model_file");
  options.object_thresh = lua_parameter_dictionary->GetDouble("object_thresh");
  options.nms_thresh = lua_parameter_dictionary->GetDouble("nms_thresh");
  options.hier_thresh = lua_parameter_dictionary->GetDouble("hier_thresh");
  options.log_interval = lua_parameter_dictionary->GetInt("log_interval");

  return options;
}

struct PerceptionOptions
{
  bool debug_mode;
  int log_level;

  int camera_index;
  int image_width;
  int image_height;

  ImageProcOptions image_proc_options;
  ObservationOptions observation_options;
};

namespace carto = cartographer;

inline PerceptionOptions CreatePerceptionOptions(carto::common::LuaParameterDictionary* const
                                  lua_parameter_dictionary)
{
  PerceptionOptions options;

  options.debug_mode = lua_parameter_dictionary->GetBool("debug_mode");
  options.log_level = lua_parameter_dictionary->GetInt("log_level");

  options.camera_index = lua_parameter_dictionary->GetInt("camera_index");
  options.image_width = lua_parameter_dictionary->GetInt("image_width");
  options.image_height = lua_parameter_dictionary->GetInt("image_height");

  options.image_proc_options =
      CreateImageProcOptions(lua_parameter_dictionary->GetDictionary("image_proc").get());

  options.observation_options =
      CreateObservationOptions(lua_parameter_dictionary->GetDictionary("observation").get());
  return options;
}

inline PerceptionOptions LoadOptions(const std::string& configuration_directory
                                   , const std::string& configuration_basename)
{
  auto file_resolver = carto::common::make_unique<
      carto::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreatePerceptionOptions(&lua_parameter_dictionary);
}

} // namespace Perception
} // namespace IKid

#endif // IKID_PERCEPTION_PERCEPTION_OPTIONS_H_
