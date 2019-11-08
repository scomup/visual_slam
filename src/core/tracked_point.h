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
    TrackedPoint(Eigen::Vector3f p)
        : Pw(p), frame_cont(1){id = next_id++;};
    void update(Eigen::Vector3f p)
    {
        Pw = (frame_cont * Pw + p) / (frame_cont + 1);
        frame_cont++;
    }
    Eigen::Vector3f Pw;
    int frame_cont;
    int id;
    static long unsigned int next_id;
    cv::Mat desc;

};


} //namespace core
} //namespace visual_slam

#endif // VISUAL_SLAM_CORE_TRACKED_POINTS_H
