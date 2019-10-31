#include <cmath>
//#include "ceres/ceres.h"
#include <iostream>
#include <fstream>

#include "src/core/optimizer.h"
#include "src/common/make_unique.h"

namespace visual_slam
{

namespace core
{



void Optimizer::addPose(const transform::Rigid3f pose)
{
    poses_.emplace_back(FromPose(pose));
}

void Optimizer::addPoints(const std::vector<TrackedPoint *>& tps)
{
    std::vector< int > t;
    for (int i = 0; i< (int)tps.size(); i++)
    {
        auto& p = tps[i];
        if(p== nullptr)
            continue;
        points_.push_back({p->Pw.x(), p->Pw.y(), p->Pw.z()});
        t.push_back(i);
    }
    tracks_.push_back(t);
}

void Optimizer::addKeys(const std::vector<cv::Point> &points)
{
    std::vector< std::array<double, 2> > keys;
    for (auto p : points)
        keys.push_back({(double)p.x, (double)p.y});
    keys_.push_back(keys);
}

PoseData Optimizer::FromPose(const transform::Rigid3f &pose)
{
    
    return PoseData{{{pose.translation().x(), pose.translation().y(),
                                 pose.translation().z()}},
                               {{pose.rotation().w(), pose.rotation().x(),
                                 pose.rotation().y(), pose.rotation().z()}}};
                                 
}

void Optimizer::solve(){


}

transform::Rigid3f Optimizer::getNewPose() const
{
    auto pose = poses_.back();

    return transform::Rigid3f(Eigen::Vector3f(pose.translation[0],pose.translation[1],pose.translation[2]),
                  Eigen::Quaternionf(pose.rotation[0], pose.rotation[1], pose.rotation[2], pose.rotation[3]));

}
void Optimizer::addReprojectionEdges(ceres::Problem &problem, Eigen::Matrix3f K)
{

    Eigen::Matrix2f information;
    information << 1, 0, 0, 1;

    ceres::CostFunction *cost_function;
    cost_function = ReprojectionError::Create(information, K);

    auto& pose = poses_.back();
    auto& keys = keys_.back();
    auto& track = tracks_.back();

    auto param = common::make_unique<ceres::QuaternionParameterization>();
    for (size_t i = 0; i < points_.size() - 1; i++)
    {
        auto &point3d = points_[i];

        int cur_idx = track[i];
        if(cur_idx == -1)
            continue;
        std::array<double, 2>& uv = keys[cur_idx];

        problem.AddParameterBlock(pose.translation.data(), 3);
        problem.AddParameterBlock(pose.rotation.data(), 4, param.release());

        problem.AddResidualBlock(cost_function, nullptr,
                                 point3d.data(),
                                 uv.data(),
                                 pose.translation.data(),
                                 pose.rotation.data());
        problem.SetParameterBlockConstant(reinterpret_cast<double *>(uv.data()));
        problem.SetParameterBlockConstant(reinterpret_cast<double *>(point3d.data()));                     
    }
}


} // namespace core
} // namespace visual_slam
