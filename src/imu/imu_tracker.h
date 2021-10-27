#include <iostream>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include "util/common.h"

struct ImuData
{
  double timestamp;
  Eigen::Quaterniond pos;
  Eigen::Vector3d ang_vel;
  Eigen::Vector3d lin_acc;
};

class ImuTracker
{
private:
  std::string imu_path;
  std::vector<ImuData> imus;

public:
  DEFINE_POINTER_TYPE(ImuTracker);
  ImuTracker(const std::string &path);
  ~ImuTracker();

  //return q12
  Eigen::Quaterniond RelativeRotation(double t1, double t2);

private:
  Eigen::Quaterniond RotationByInterpolation(double t);

  //返回imus中t的前一个时间戳的索引
  int binarySearch(double t);

  void LoadImu();
};
