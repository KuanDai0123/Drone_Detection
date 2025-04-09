#include <ros/ros.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include "panorama.h"

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  ros::init(argc, argv, "panorama");

  ros::NodeHandle nh;

  panorama::Panorama pano(nh);

  ros::spin();

  return 0;
}
