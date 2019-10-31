#include "src/core/tracking.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include<iostream>

#include<mutex>

#include "src/core/frame.h"
#include <boost/format.hpp>

#include <boost/format.hpp>
#include "src/core/optimizer.h"


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
    float fx = (*config_)["camera"]["fx"].as<float>();
    float fy = (*config_)["camera"]["fy"].as<float>();
    float cx = (*config_)["camera"]["cx"].as<float>();
    float cy = (*config_)["camera"]["cy"].as<float>();
    K_ = Eigen::Matrix3f::Identity();
    K_(0, 0) = fx;
    K_(1, 1) = fy;
    K_(0, 2) = cx;
    K_(1, 2) = cy;
    Tcw_ = transform::Rigid3f();

}

void Tracking::HandleOdometry(std::unique_ptr<sensor::OdometryData> odom)
{
}

void Tracking::updatePoses()
{
    if (frames_.size() == 1)
    {
        //Set first frame as origin
        transform::Rigid3f Tcw = transform::Rigid3f();
        frames_.back()->setTcw(Tcw);
        return;
    }
    if (frames_.size() >= 2)
    {
        Optimizer opt;
        //Set new frame to the same pose as the previous frame
        (*frames_.rbegin())->setTcw((*(++frames_.rbegin()))->Tcw());


        auto frame_it = frames_.begin();
        auto track_it = tracks_.begin();
        while (frame_it != frames_.end())
        {
            opt.addPose((*frame_it)->Tcw());

            opt.addKeys((*frame_it)->keys0());
            frame_it++;
            track_it++;
        }
        opt.addPoints(tracked_points_);

        ceres::Problem problem;

        opt.addReprojectionEdges(problem, K_, tracks_);

        ceres::Solver::Options options;
        options.max_num_iterations = 100;
        options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

        ceres::Problem::EvaluateOptions evaluate_options;
        double total_cost = 0.0;
        std::vector<double> residuals;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
        std::cout << "Finial total cost:" << total_cost << "\n";

        Tcw_ = opt.getNewPose();
        frames_.back()->setTcw(Tcw_);
        //showReporjection("after_");
    }
}
void Tracking::updateTracks()
{
    if (frames_.size() < 2)
        return;

    auto &current_frame = *(frames_.rbegin());
    auto &curr_keys = current_frame->keys0();
    auto &curr_desc = current_frame->desc0();

    auto &prev_frame = *(++frames_.rbegin());
    auto &prev_keys = prev_frame->keys0();
    auto &prev_desc = prev_frame->desc0();

    auto matches = kp_frontend_->twoWayMatching(prev_keys, prev_desc, curr_keys, curr_desc);

    if (tracks_.size() == 0)
    {
        tracks_.push_back(std::vector<int>());
    }
    std::vector<int> &prev_tracks = tracks_.back();
    //std::cout<<tracked_points_.size()<<"   "<< prev_tracks.size()<<std::endl;
    tracks_.push_back(std::vector<int>(prev_tracks.size(), -1));
    std::vector<int> &curr_tracks = tracks_.back();

    for (int i = 0; i < (int)matches.size(); i++)
    {
        int prev_id = std::get<0>(matches[i]);
        int curr_id = std::get<1>(matches[i]);

        int loc = find(prev_tracks, prev_id);

        if (loc == -1 )
        {

//if(frames_.size()==2){
            //new point
            for (auto &t : tracks_)
            {
                t.push_back(-1);
            }
            prev_tracks.back() = prev_id;
            curr_tracks.back() = curr_id;
            Eigen::Vector3f point3d(0,0,0);
            bool good =prev_frame->computePoint3d(prev_id, point3d);
            auto& Tcw = prev_frame->Tcw();
            //auto pose_inv 
            tracked_points_.push_back(point3d);
            
//}
        }
        else
        {
            curr_tracks[loc] = curr_id;
        }
    }

    for (int i = 0; i < (int)prev_tracks.size(); i++)
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
void Tracking::showReporjection(std::string mark){
    auto frame_it = frames_.rbegin();
    auto track_it = tracks_.rbegin();
    int frame_id = 0;
    while (frame_it != frames_.rend())
    {
        float error = 0;
        int c = 0;
        auto &Tcw = (*frame_it)->Tcw();
        auto &key = (*frame_it)->keys0();
        cv::Mat result_color;
        cv::cvtColor((*frame_it)->image_, result_color, CV_GRAY2BGR);



        for (int i = 0; i < (int)tracked_points_.size(); i++)
        {
            auto &Pw = tracked_points_[i];

            if (Pw(0) == 0 && Pw(1) == 0 && Pw(2) == 0)
                continue;
            int key_id = (*track_it)[i];
            if (key_id == -1)
                continue;
            Eigen::Vector3f local_point = Tcw * Pw;
            Eigen::Vector3f reprojection_point = K_ * local_point;
            reprojection_point = reprojection_point / reprojection_point.z();
            cv::Point loc(reprojection_point.x(), reprojection_point.y());
            
                
            cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, key[key_id], 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc, key[key_id], cv::Scalar(0, 255, 0));
            
            double d = sqrt((key[key_id].x - loc.x) * (key[key_id].x - loc.x) + (key[key_id].y - loc.y) * (key[key_id].y - loc.y));
            error += d;
            c++;
            if(d>20)
                (*track_it)[i] = -1;
        }
        for(auto k :key){
            cv::circle(result_color, k, 1, cv::Scalar(0, 255, 0));
        }
        
        printf("%sframe_%d: Total Reporjection error %f\n",mark.c_str(),(*frame_it)->id(),error/c);
        std::cout<<Tcw<<std::endl;
        char s[200];
        sprintf(s,"frame%d.png",(*frame_it)->id());
        auto ss = std::string(s);
        //cv::imwrite(std::string("/home/liu/workspace/visual_slam/build/")+mark+ss,result_color);
        cv::imshow("win",result_color);
        cv::waitKey(1);
        frame_id++;

        frame_it++;
        track_it++;
        break;

    }


}

const transform::Rigid3f& Tracking::Tcw() const{
    return Tcw_;
}
void Tracking::HandleImage(std::unique_ptr<sensor::MultiImageData> image)
{
    Frame *current_frame = new Frame(image->image0, image->image1, image->time, kp_frontend_);
    if(frames_.size() == 2){
        //frames_.pop_back();
        //tracks_.pop_back();
    }
    frames_.push_back(current_frame);
    updateTracks();
    //showReporjection("before_");
    updatePoses();
    updatePoses();
    showReporjection("after");


    /*
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
    */
    

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
