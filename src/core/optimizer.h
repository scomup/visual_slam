
#ifndef VISUAL_SLAM_CORE_OPTIMIZER_H_
#define VISUAL_SLAM_CORE_OPTIMIZER_H_

#include "yaml-cpp/yaml.h"
#include "src/core/reprojection_error.h"
#include "src/core/frame.h"
#include "src/core/ceres_pose.h"
namespace visual_slam
{

namespace core
{

struct PoseData
{
    std::array<double, 3> translation;
    // Rotation quaternion as (w, x, y, z).
    std::array<double, 4> rotation;
};

class Optimizer
{
public:
    Optimizer(){};

    void addPose(const transform::Rigid3f pose);
    void addPoints(const std::vector<Eigen::Vector3f>& points);
    void addKeys(const std::vector<cv::Point>& points);
    void solve();
    void addReprojectionEdges(ceres::Problem &problem, Eigen::Matrix3f K, std::list<std::vector<int>>& tracks);
    transform::Rigid3f getNewPose() const;
private:
    PoseData FromPose(const transform::Rigid3f &pose);


    std::vector<PoseData> poses_;
    YAML::Node *config_;
    std::vector<std::array<double, 3>> points_;
    std::vector<std::vector<std::array<double, 2>>> keys_;
};


} // namespace core
} // namespace visual_slam

#endif
