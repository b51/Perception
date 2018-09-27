/*************************************************************************
*
*              Author: b51
*                Mail: b51live@gmail.com
*            FileName: PerceptionNodeMain.cc
*
*          Created On: Thu 27 Sep 2018 03:56:46 PM CST
*     Licensed under The MIT License [see LICENSE for details]
*
************************************************************************/

#include "PerceptionNode.h"

using namespace IKid;
using namespace Perception;

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

  PerceptionOptions options =
        LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  FLAGS_minloglevel = options.log_level;
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  ::ros::init(argc, argv, "PerceptionNode");
  ::ros::start();

  //::ScopedRosLogSink ros_log_sink;
  PerceptionNode perception_node(options);
  perception_node.Init();
  perception_node.Run();

  ::ros::shutdown();
}
