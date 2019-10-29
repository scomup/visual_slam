#ifndef VISUAL_SLAM_SENSOR_ODOMETRY_DATA_H_
#define VISUAL_SLAM_SENSOR_ODOMETRY_DATA_H_

#include "src/transform/rigid_transform.h"

namespace visual_slam {
namespace sensor {

struct OdometryData {
  double time;
  transform::Rigid3d pose;

bool operator<(const double rhs) const{
  return this->time < rhs;
}
};

}  // namespace sensor
}  // namespace visual_slam

#endif  // VISUAL_SLAM_SENSOR_ODOMETRY_DATA_H_

