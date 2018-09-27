--########################################################################
--
--              Author: b51
--                Mail: b51live@gmail.com
--            FileName: ConfigVision.lua
--
--          Created On: Mon 11 Dec 2017 12:03:40 AM CST
--     Licensed under The MIT License [see LICENSE for details]
--
--########################################################################

local cmd = io.popen("rospack find ImageProcNode");
local result = cmd:read("*a");
cmd.close();
local s = string.gsub(result, '%s+$', '')
local data_path = s.."/data/";

vision_options =
{
  camera_index = 0,
  image_width = 640,
  image_height = 480,
  net_input_width = 128,
  net_input_height = 128,

  net_prototxt = data_path.."tiny-yolo.cfg",
  model_file = data_path.."tiny-yolo.weights",
  object_thresh = 0.24,
  nms_thresh = 0.2,
  hier_thresh = 0.5,
  log_interval = 120, -- second

  debug_mode = false,
  log_level = 0,
}

return vision_options;
