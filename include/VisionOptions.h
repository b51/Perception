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

#ifndef IKID_VISION_IMAGE_PROC_VISION_OPTIONS_H_
#define IKID_VISION_IMAGE_PROC_VISION_OPTIONS_H_

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

struct VisionOptions
{
  bool debug_mode;
  int log_level;

  int camera_index;
  int image_width;
  int image_height;
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

namespace carto = cartographer;

inline VisionOptions CreateVisionOptions(carto::common::LuaParameterDictionary* const
                                  lua_parameter_dictionary)
{
  VisionOptions options;

  options.debug_mode = lua_parameter_dictionary->GetBool("debug_mode");
  options.log_level = lua_parameter_dictionary->GetInt("log_level");
  options.camera_index = lua_parameter_dictionary->GetInt("camera_index");
  options.image_width = lua_parameter_dictionary->GetInt("image_width");
  options.image_height = lua_parameter_dictionary->GetInt("image_height");

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

inline VisionOptions LoadOptions(const std::string& configuration_directory
                        , const std::string& configuration_basename)
{
  auto file_resolver = carto::common::make_unique<
      carto::common::ConfigurationFileResolver>(
          std::vector<std::string>{configuration_directory});
  const std::string code =
      file_resolver->GetFileContentOrDie(configuration_basename);
  carto::common::LuaParameterDictionary lua_parameter_dictionary(
      code, std::move(file_resolver));

  return CreateVisionOptions(&lua_parameter_dictionary);
}

} // namespace Perception
} // namespace IKid

#endif // IKID_VISION_IMAGE_PROC_VISION_OPTIONS_H_
