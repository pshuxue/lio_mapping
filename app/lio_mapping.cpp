#include <iostream>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include "imu/imu_tracker.h"
#include "util/velodyne.h"

int main(int argc, char **argv)
{
  FLAGS_colorlogtostderr = true;
  gflags::ParseCommandLineFlags(&argc, &argv, true); // 这一句如果不加,则在终端输入的时候不会解析命令行

  ImuTracker imu_tracker("/home/psx/data/imu.csv");
  std::cout
      << "Hello, world!\n";
}
