/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: ImageProcNodeMain.cc
*
*          Created On: Fri 15 Dec 2017 09:11:18 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "ImageProcNode.h"

using namespace ikid;
using namespace Vision;

DEFINE_string(configuration_directory, "",
                  "First directory in which configuration files are searched, "
                  "second is always the Cartographer installation to allow "
                  "including files from there.");
DEFINE_string(configuration_basename, "",
                  "Basename, i.e. not containing any directory prefix, of the "
                  "configuration file.");


int main(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  VisionOptions vision_options =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  FLAGS_stderrthreshold = vision_options.log_level;

  ::ros::init(argc, argv, "ImageProcNode");
  ::ros::start();

  //::ScopedRosLogSink ros_log_sink;
  ImageProcNode image_proc_node(vision_options);
  image_proc_node.Init();
  image_proc_node.Run();

  ::ros::shutdown();
}
