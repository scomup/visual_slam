#include "src/core/tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include<iostream>

#include<mutex>

#include "src/core/frame.h"
#include <boost/format.hpp>

using namespace std;

namespace visual_slam
{
namespace core
{

static int find(std::vector<int> &v, int target)
{
    auto result = std::find(v.begin(), v.end(), target);

    if (result == v.end())
    {
        return -1;
    }
    else
    {
        return std::distance(v.begin(), result);
    }
}

Tracking::Tracking(const YAML::Node *config)
    : config_(config),
      max_tracks_((*config_)["tracking"]["max_tracks"].as<int>())
{
    kp_frontend_ = new SuperpointFrontend(config);
    Frame::setFrameParam(config);
}

void Tracking::HandleOdometry(std::unique_ptr<sensor::OdometryData> odom)
{
}

void Tracking::updatePoses()
{
    if (frames_.size() == 1)
    {
        transform::Rigid3f pose = transform::Rigid3f();
        frames_.back()->setPose(pose);
        return;
    }
    auto &current_frame = *(frames_.rbegin());
    auto &curr_keys = current_frame->keys0();

    auto &last_frame = *(++frames_.rbegin());
    auto &last_keys = last_frame->keys0();

    auto curr_track = *(tracks_.rbegin());
    auto last_track = *(++tracks_.rbegin());
}
void Tracking::updateTracks()
{
    if (frames_.size() < 2)
        return;

    auto &current_frame = *(frames_.rbegin());
    auto &curr_keys = current_frame->keys0();
    auto &curr_desc = current_frame->desc0();

    auto &last_frame = *(++frames_.rbegin());
    auto &last_keys = last_frame->keys0();
    auto &last_desc = last_frame->desc0();

    auto matches = kp_frontend_->twoWayMatching(last_keys, last_desc, curr_keys, curr_desc);

    if (tracks_.size() == 0)
    {
        tracks_.push_back(std::vector<int>());
    }
    std::vector<int> &last_tracks = tracks_.back();
    std::cout<<tracked_points_.size()<<"   "<< last_tracks.size()<<std::endl;
    tracks_.push_back(std::vector<int>(last_tracks.size(), -1));
    std::vector<int> &curr_tracks = tracks_.back();

    for (int i = 0; i < (int)matches.size(); i++)
    {
        int last_id = std::get<0>(matches[i]);
        int curr_id = std::get<1>(matches[i]);

        int loc = find(last_tracks, last_id);

        if (loc == -1)
        {
            //new point
            for (auto &t : tracks_)
            {
                t.push_back(-1);
            }
            last_tracks.back() = last_id;
            curr_tracks.back() = curr_id;
            Eigen::Vector3f point3d(0,0,0);
            last_frame->computePoint3d(last_id, point3d);
            tracked_points_.push_back(point3d);
            
        }
        else
        {
            curr_tracks[loc] = curr_id;
        }
    }

    for (int i = 0; i < (int)last_tracks.size(); i++)
    {
        bool is_tracking = false;
        for (auto &t : tracks_)
        {
            if(t[i] != -1)
                is_tracking = true;
        }
        if(is_tracking == false)
        {
            for (auto &t : tracks_)
            {
                t.erase(t.begin() + i);
            }
            tracked_points_.erase(tracked_points_.begin() + i);
            i--;
        }
    }
}

void Tracking::HandleImage(std::unique_ptr<sensor::MultiImageData> image)
{
    Frame *current_frame = new Frame(image->image0, image->image1, image->time, kp_frontend_);
    frames_.push_back(current_frame);
    updateTracks();
    updatePoses();
    

    if(frames_.size() >= 2){
    cv::Mat result_color;
    cv::cvtColor(image->image0, result_color, CV_GRAY2BGR);

    auto &curr_tracks = tracks_.back();
    auto& curr_keys = current_frame->keys0();
    for (int i= 0;i<(int)curr_tracks.size();i++)
    {
        if(curr_tracks[i] == -1)
            continue;
        auto point = curr_keys[curr_tracks[i]];
        auto point3d = tracked_points_[i];
        std::string s = (boost::format("%3.1f") % point3d.z()).str();
        cv::circle(result_color, point, 2, cv::Scalar(0, 255, 0));
        cv::putText(result_color, s, point, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);
    }

    auto frame0 = frames_.begin();
    auto frame1 = std::next(frame0);
    auto track0 = tracks_.begin();
    auto track1 = std::next(track0);
    while (true)
    {
        if (track1 == tracks_.end())
            break;
        for (int i = 0; i < (int)track0->size(); i++)
        {
            int a = (*track0)[i];
            int b = (*track1)[i];

            if (a == -1 || b == -1)
                continue;

            auto &keys0 = (*frame0)->keys0();
            auto &keys1 = (*frame1)->keys0();

            auto point_a = keys0[a];
            auto point_b = keys1[b];
            //cv::circle(result_color, point_a, 2, cv::Scalar(0, 255, 0));
            //cv::circle(result_color, point_b, 2, cv::Scalar(0, 255, 0));
            cv::line(result_color, point_a, point_b, cv::Scalar(0, 255, 0));
        }
        frame0 = std::next(frame0);
        frame1 = std::next(frame1);
        track0 = std::next(track0);
        track1 = std::next(track1);
    }


    cv::imshow("win", result_color);
    cv::waitKey(1);
    }

    if ((int)frames_.size() > max_tracks_)
    {
        auto old_frame = frames_.front();
        delete old_frame;
        frames_.pop_front();
    }
    if ((int)tracks_.size() > max_tracks_)
    {
        tracks_.pop_front();
    }
}



} // namespace core
} // namespace visual_slam
