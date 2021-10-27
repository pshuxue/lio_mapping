#include "imu/imu_tracker.h"
#include <fstream>
#include <sstream>
#include <glog/logging.h>
using namespace std;

ImuTracker::ImuTracker(const std::string &path)
{
  imu_path = path;
  LoadImu();
}

ImuTracker::~ImuTracker()
{
}

void ImuTracker::LoadImu()
{
  imus.clear();
  std::ifstream fin(imu_path);
  while (!fin.eof())
  {
    string str;
    getline(fin, str);
    if (str.length() == 0 || str[0] == '#')
      continue;
    for (char &c : str)
    {
      if (c == ',')
        c = ' ';
    }
    stringstream ss(str);
    ImuData imu;
    double t;
    ss >> t >> imu.lin_acc.x() >> imu.lin_acc.y() >> imu.lin_acc.z() >>
        imu.pos.w() >> imu.pos.x() >> imu.pos.y() >> imu.pos.z() >>
        imu.ang_vel.x() >> imu.ang_vel.y() >> imu.ang_vel.z();

    imu.timestamp = t / 1000000000.0;
    imus.push_back(imu);
  }
}

Eigen::Quaterniond ImuTracker::RelativeRotation(double t1, double t2)
{
  CHECK_LT(t1, t2);
  Eigen::Quaterniond qwc1;
  Eigen::Quaterniond qwc2;
  if (t1 < imus[0].timestamp)
    qwc1 = imus[0].pos;
  else
    qwc1 = RotationByInterpolation(t1);

  if (t2 > imus.back().timestamp)
    qwc2 = imus.back().pos;
  else
    qwc2 = RotationByInterpolation(t2);

  return qwc1.inverse() * qwc2;
}

Eigen::Quaterniond ImuTracker::RotationByInterpolation(double t)
{
  int left = binarySearch(t);
  int right = left + 1;
  double left_t = t - imus[left].timestamp;
  double right_t = imus[right].timestamp - t;

  Eigen::Quaterniond q_l = imus[left].pos;
  Eigen::Quaterniond q_r = imus[right].pos;
  Eigen::Quaterniond dq = q_l.inverse() * q_r;
  Eigen::AngleAxisd vec(dq);
  double d_angle = vec.angle() * left_t / (left_t + right_t);
  Eigen::AngleAxisd v(d_angle, vec.axis());
  return q_l * Eigen::Quaterniond(v);
}

int ImuTracker::binarySearch(double t)
{
  int left = 0;
  int right = imus.size() - 1;
  if (t > imus[right].timestamp)
  {
    return right;
  }
  if (t < imus[left].timestamp)
  {
    return left;
  }

  while (left < right - 1)
  {
    int mid = (left + right) / 2;
    if (t >= imus[mid].timestamp)
    {
      left = mid;
    }
    else
    {
      right = mid;
    }

    cout << "left " << left << " right " << right << endl;
  }
  return left;
}
