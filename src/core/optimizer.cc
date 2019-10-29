#include <cmath>
//#include "ceres/ceres.h"
#include <iostream>
#include <fstream>

#include "src/core/optimizer.h"

namespace visual_slam
{

namespace core
{



void Optimizer::addPose(transform::Rigid3f pose)
{
    poses_.emplace_back(FromPose(pose));
}

void Optimizer::addPoints(std::vector<Eigen::Vector3f> &points)
{
    for (auto p : points)
        points_.emplace_back(p.x(), p.y(), p.z());
}

PoseData Optimizer::FromPose(const transform::Rigid3f &pose)
{
    
    return PoseData{{{pose.translation().x(), pose.translation().y(),
                                 pose.translation().z()}},
                               {{pose.rotation().w(), pose.rotation().x(),
                                 pose.rotation().y(), pose.rotation().z()}}};
                                 
}


void Optimizer::addReprojectionEdges(ceres::Problem &problem, Eigen::Matrix3f K, std::list<std::vector<int>> &tracks, std::list<Frame *> frames)
{
    
    if (tracks.size() <= 2)
        return;
    Eigen::Matrix2f information;
    information << 1, 0, 0, 1;

    ceres::CostFunction *cost_function;
    cost_function = ReprojectionError::Create(information, K);

    //const frames.back()->pose();
    //auto pose = FromPose( frames.back()->pose());
     
    //float* p  = a.data();
    //float * bMat = b.data();

    //float* p = pose.translation().data();

    auto pose = poses_.back();
    
    
    for (size_t i = 0; i < points_.size() - 1; i++)
    {
        auto &point3d = points_[i];

        auto curr_track = *(tracks.rbegin());
        auto keys = frames.back()->keys0();
        //auto last_track = *(++tracks.rbegin());
        int cur_idx = curr_track[i];
        auto kp = keys[cur_idx];
        std::array<double, 3> translation;
        problem.AddResidualBlock(cost_function, nullptr,  point3d.data(), pose.translation.data(), pose.rotation.data());
/*
        problem.AddResidualBlock(cost_function, nullptr,
                                 point3d.data(),
                                 &kp.x,
                                 &kp.y,
                                 pose.translation.data(),
                                 pose.rotation.data());
             */                    
    }
}


} // namespace core
} // namespace visual_slam
