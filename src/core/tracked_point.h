#ifndef VISUAL_SLAM_CORE_TRACKED_POINTS_H
#define VISUAL_SLAM_CORE_TRACKED_POINTS_H

#include "Eigen/Core"

#include <deque>

namespace visual_slam
{
namespace core
{

class Frame;
struct TrackedPoint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3f Pw;
    std::list<std::pair<Frame*, int>> frames;
};

} //namespace core
} //namespace visual_slam

#endif // VISUAL_SLAM_CORE_TRACKED_POINTS_H
