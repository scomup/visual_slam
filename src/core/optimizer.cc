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

void Optimizer::setFrameNum(const int n)
{
    frames_num_ = n;
}

void Optimizer::addPose(const int frame_id, const transform::Rigid3f pose)
{
    poses_.emplace_back(FromPose(pose));
}

void Optimizer::addPoints(const int frame_id, const std::vector<TrackedPoint *> &tps)
{
    std::vector<int> t;

    for (int i = 0; i < (int)tps.size(); i++)
    {
        auto &p = tps[i];
        if (p == nullptr)
            continue;

        if (check_points_.find(p) != check_points_.end())
        {
            int tp_id = check_points_[p];
            tracks_[tp_id][frame_id] = i;
            //std::cout<<keys_[0][tracks_[tp_id][0]][0]<<","<<keys_[0][tracks_[tp_id][0]][1]<<std::endl;
            //std::cout<<keys_[1][tracks_[tp_id][1]][0]<<","<<keys_[0][tracks_[tp_id][0]][1]<<std::endl;
        }
        else
        {
            points_.push_back({p->Pw.x(), p->Pw.y(), p->Pw.z()});
            auto t = std::vector<int>(frames_num_);
            std::fill(t.begin(), t.end(), -1);
            t[frame_id] = i;
            check_points_[p] = points_.size()-1;
            tracks_.push_back(t);
        }
    }
}

void Optimizer::addKeys(const int frame_id, const std::vector<cv::Point> &points)
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


    if(points_.size()==0)
        return;

    auto param = common::make_unique<ceres::QuaternionParameterization>();

    ceres::CostFunction *cost_pose;
    cost_pose = PoseError::Create(100, 100);

    for (size_t i = 0; i < poses_.size() - 1; i++)
    {
        /*
        problem.AddParameterBlock(poses_[i].translation.data(), 3);
        problem.AddParameterBlock(poses_[i].rotation.data(), 4, param.release());
        problem.AddParameterBlock(poses_[i+1].translation.data(), 3);
        problem.AddParameterBlock(poses_[i+1].rotation.data(), 4, param.release());

        problem.AddResidualBlock(cost_pose, nullptr,
                                 poses_[i].translation.data(),
                                 poses_[i].rotation.data(),
                                 poses_[i+1].translation.data(),
                                 poses_[i+1].rotation.data());
                                 */
    }

    ceres::CostFunction *cost_function;
    cost_function = ReprojectionError::Create(information, K);

    for (size_t i = 0; i < tracks_.size(); i++)//for each tracked point
    {//i 
        for (size_t j = 0; j < frames_num_; j++)//for each frame 
        {
            auto &point3d = points_[i];
            int key_id = tracks_[i][j];
            if (key_id == -1)
                continue;
            std::array<double, 2> &uv = keys_[j][key_id];
            //std::cout<<uv[0]<<","<<uv[1]<<std::endl;

            problem.AddParameterBlock(poses_[j].translation.data(), 3);
            problem.AddParameterBlock(poses_[j].rotation.data(), 4, param.release());

            problem.AddResidualBlock(cost_function, nullptr,
                                     point3d.data(),
                                     uv.data(),
                                     poses_[j].translation.data(),
                                     poses_[j].rotation.data());
            problem.SetParameterBlockConstant(reinterpret_cast<double *>(uv.data()));
            //problem.SetParameterBlockConstant(reinterpret_cast<double *>(point3d.data()));
        }
    }
}


} // namespace core
} // namespace visual_slam
