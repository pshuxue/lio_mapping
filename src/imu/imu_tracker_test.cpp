#include "imu/imu_tracker.h"
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <gflags/gflags.h>
#include <Eigen/Geometry>
#include <iostream>

using namespace std;

TEST(GLog, glog)
{
  ImuTracker imu_tracker("/home/psx/data/imu.csv");

  cout << imu_tracker.RelativeRotation(1627002823.327344957, 1627002823.427344957).coeffs() << endl;
}

int main(int argc, char *argv[])
{
  gflags::ParseCommandLineFlags(&argc, &argv, true); // 这一句如果不加,则在终端输入的时候不会解析命令行
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}