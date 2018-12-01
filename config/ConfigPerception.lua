--########################################################################
--
--              Author: b51
--                Mail: b51live@gmail.com
--            FileName: ConfigPerception.lua
--
--          Created On: Mon 11 Dec 2017 12:03:40 AM CST
--     Licensed under The MIT License [see LICENSE for details]
--
--########################################################################

local cmd = io.popen("rospack find PerceptionNode");
local result = cmd:read("*a");
cmd.close();
local s = string.gsub(result, '%s+$', '')
local data_path = s.."/data/";

OBSERVATION =
{
  ball_diameter = 0.15,
  camera_focus = 456,
  camera_center_x = 322,
  camera_center_y = 244,
  tracking_frame = "base_link",
  lookup_transform_timeout_sec = 1.0,
}

IMAGE_PROC =
{
  publisher_queue_size = 0,
  net_input_width = 128,
  net_input_height = 128,

  net_prototxt = data_path.."tiny-yolo.cfg",
  model_file = data_path.."tiny-yolo.weights",
  object_thresh = 0.24,
  nms_thresh = 0.2,
  hier_thresh = 0.5,
  log_interval = 120, -- second
}

perception_options =
{
  debug_mode = false,
  log_level = 0,

  camera_index = 0,
  image_width = 640,
  image_height = 480,

  image_proc = IMAGE_PROC,
  observation = OBSERVATION,
}

return perception_options;
