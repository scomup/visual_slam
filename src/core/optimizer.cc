#include <cmath>
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

void Optimizer::addPoints(const std::vector<TrackedPoint *> &points)
{
    //double avg = 0;
    for (auto p : points)
    {
        points_.push_back({p->Pw.x(), p->Pw.y(), p->Pw.z()});
        //avg += p->frame_cont;
    }
    //std::cout << "frame_cont:" << avg / points.size() << std::endl;
}

void Optimizer::addKeys(const std::vector<cv::Point> &points)
{
    std::vector<std::array<double, 2>> keys;
    for (auto p : points)
        keys.push_back({(double)p.x, (double)p.y});
    keys_.push_back(keys);
}

PoseData Optimizer::FromPose(const transform::Rigid3f &pose)
{
    auto rpy = pose.rotation().toRotationMatrix().eulerAngles(0, 1, 2);
    return PoseData{{{pose.translation().x(), pose.translation().y(),
                      pose.translation().z()}},
                    {{rpy.x(),
                      rpy.y(), rpy.z()}}};
}

void Optimizer::solve()
{
}

transform::Rigid3f Optimizer::getPose(const int id) const
{
    auto pose = poses_[id];
    const Eigen::AngleAxisf roll_angle(pose.rotation[0],  Eigen::Vector3f::UnitX());
    const Eigen::AngleAxisf pitch_angle(pose.rotation[1], Eigen::Vector3f::UnitY());
    const Eigen::AngleAxisf yaw_angle(pose.rotation[2], Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q =  yaw_angle * pitch_angle * roll_angle;
    return transform::Rigid3f(Eigen::Vector3f(pose.translation[0], pose.translation[1], pose.translation[2]), q);
}
void Optimizer::addReprojectionEdges(ceres::Problem &problem, Eigen::Matrix3f K, std::list<std::vector<int>> &tracks, bool fix_camera_or_point)
{
    if (tracks.size() < 2)
        return;
    Eigen::Matrix2f information;
    information << 1, 0, 0, 1;
    ceres::CostFunction *cost_function;
    cost_function = ReprojectionError::Create(information, K);
    auto param = common::make_unique<ceres::QuaternionParameterization>();
    auto loss = common::make_unique<ceres::HuberLoss>(100);

    ceres::CostFunction *cost_pose;
    cost_pose = PoseError::Create(100, 100);

    /*
    problem.AddParameterBlock(poses_[0].translation.data(), 3);
    problem.AddParameterBlock(poses_[0].rotation.data(), 3);

    for (size_t i = 1; i < poses_.size(); i++)
    {
        problem.AddParameterBlock(poses_[i].translation.data(), 3);
        problem.AddParameterBlock(poses_[i].rotation.data(), 3);

        problem.AddResidualBlock(cost_pose, nullptr,
                                 poses_[i-1].translation.data(),
                                 poses_[i-1].rotation.data(),
                                 poses_[i].translation.data(),
                                 poses_[i].rotation.data());
    }
    */
    /*
    for (auto &pose : poses_)
    {
        problem.AddParameterBlock(pose.translation.data(), 3);
        problem.AddParameterBlock(pose.rotation.data(), 3);
        if (!fix_camera_or_point)
        {
            problem.SetParameterBlockConstant(reinterpret_cast<double *>(pose.translation.data()));
            problem.SetParameterBlockConstant(reinterpret_cast<double *>(pose.rotation.data()));
        }
    }*/

    for (size_t i = 0; i < points_.size() - 1; i++)
    {
        auto &point3d = points_[i];

        int frame_id = 0;
        //std::cout<<point3d[0]<<","<<point3d[1]<<","<<point3d[2]<<std::endl;
        int trecked_frame = 0;
        int frame_num = tracks.size();
        //for (auto &track : tracks)
        //{
        //    int cur_idx = track[i];
        //    if (cur_idx != -1)
        //    {
        //        trecked_frame++;
        //    }
        //}

        for (auto &track : tracks)
        {
            int cur_idx = track[i];
            //if (frame_num != trecked_frame)
            //    continue;
            if (cur_idx != -1)
            {
                auto &pose = poses_[frame_id];
                std::array<double, 2> &uv = keys_[frame_id][cur_idx];
                problem.AddResidualBlock(cost_function, loss.release(),
                                         point3d.data(),
                                         uv.data(),
                                         pose.translation.data(),
                                         pose.rotation.data());
                problem.SetParameterBlockConstant(reinterpret_cast<double *>(uv.data()));
                if (fix_camera_or_point)
                {
                    problem.SetParameterBlockConstant(reinterpret_cast<double *>(point3d.data()));
                }

                //std::cout << "frame_" << frame_id << ":" << uv[0] << "," << uv[1] << std::endl;
            }

            frame_id++;
        }
        //std::cout<<"------------"<<std::endl;
    }

    /*
    if (tracks.size() < 2)
        return;
    Eigen::Matrix2f information;
    information << 1, 0, 0, 1;

    ceres::CostFunction *cost_function;
    cost_function = ReprojectionError::Create(information, K);

    auto& pose = poses_.back();
    
    auto &keys = keys_.back();
    auto param = common::make_unique<ceres::QuaternionParameterization>();
    for (size_t i = 0; i < points_.size() - 1; i++)
    {
        auto &point3d = points_[i];

        if(point3d[0] == 0 && point3d[1]==0 && point3d[2]==0)
            continue;

        auto curr_track = *(tracks.rbegin());
        //auto prev_track = *(++tracks.rbegin());
        int cur_idx = curr_track[i];
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
    }*/
}

 std::vector<std::array<double, 3>>& Optimizer::getPoints() {
    return points_;

}

} // namespace core
} // namespace visual_slam
