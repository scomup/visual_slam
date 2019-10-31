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
    //Frame::setFrameParam(config);

    fx_ = (*config)["camera"]["fx"].as<float>();
    fy_ = (*config)["camera"]["fy"].as<float>();
    cx_ = (*config)["camera"]["cx"].as<float>();
    cy_ = (*config)["camera"]["cy"].as<float>();
    bf_ = (*config)["camera"]["baseline"].as<float>() * fx_;
    nn_thresh_ = (*config)["superpoint"]["nn_thresh"].as<float>();
    K_ = Eigen::Matrix3f::Identity();
    K_(0, 0) = fx_;
    K_(1, 1) = fy_;
    K_(0, 2) = cx_;
    K_(1, 2) = cy_;
    inv_K_ = K_.inverse().eval();
    Tcw_ = transform::Rigid3f();

}

void Tracking::HandleOdometry(std::unique_ptr<sensor::OdometryData> odom)
{
}

void Tracking::updatePoseByReferenceFrame(Frame *frame)
{
    Frame *ref_frame = frames_.back();
    Optimizer opt;
    //Set new frame to the same pose as the reference frame.
    frame->setTcw(ref_frame->Tcw());

    auto frame_it = frames_.begin();
    auto track_it = tracks_.begin();

    opt.addPose(frame->Tcw());
    opt.addKeys(frame->keys0());
    opt.addPoints(frame->tps());

    ceres::Problem problem;
    opt.addReprojectionEdges(problem, K_);
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
    //frame->setTcw(Tcw_);
}

void Tracking::trackingByReferenceFrame(Frame* frame)
{
    
    auto &cur_keys = frame->keys0();
    auto &cur_desc = frame->desc0();
    auto &cur_tps = frame->tps();

    auto &ref_frame = *(frames_.rbegin());
    auto &ref_keys = ref_frame->keys0();
    auto &ref_desc = ref_frame->desc0();
    auto &ref_tps = ref_frame->tps();

    auto matches = kp_frontend_->twoWayMatching(ref_keys, ref_desc, cur_keys, cur_desc);

    /*
    if (tracks_.size() == 0)
    {
        tracks_.push_back(std::vector<int>());
    }
    std::vector<int> &prev_tracks = tracks_.back();
    tracks_.push_back(std::vector<int>(prev_tracks.size(), -1));
    std::vector<int> &curr_tracks = tracks_.back();
    */
    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);

        auto tp = ref_tps[ref_id];
        if(tp != nullptr)
        {
            tp->frames.emplace_back(frame, cur_id);
            cur_tps[cur_id] = tp;
        }
    }
        

/*

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
            //bool good =prev_frame->computePoint3d(prev_id, point3d);
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
               // t.erase(t.begin() + i);
            }
            //tracked_points_.erase(tracked_points_.begin() + i);
            i--;
        }
    }
    */
    
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
        (*frame_it)->image_.copyTo(result_color);



        for (int i = 0; i < (int)tracked_points_.size(); i++)
        {
            auto &Pw = tracked_points_[i];

            if (Pw(0) == 0 && Pw(1) == 0 && Pw(2) == 0)
                continue;
            int key_id = (*track_it)[i];
            if (key_id == -1)
                continue;
            Eigen::Vector3f Pc = Tcw * Pw;
            Eigen::Vector3f image_point(Pc.x()/Pc.y(),-Pc.z()/Pc.y(),1);
            Eigen::Vector3f reprojection_point = K_ * image_point;
            reprojection_point = reprojection_point / reprojection_point.z();
            cv::Point loc(reprojection_point.x(), reprojection_point.y());
            
                
            cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, key[key_id], 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc, key[key_id], cv::Scalar(0, 255, 0));
            
            double d = sqrt((key[key_id].x - loc.x) * (key[key_id].x - loc.x) + (key[key_id].y - loc.y) * (key[key_id].y - loc.y));
            error += d;
            c++;
            //if(d>20)
            //    (*track_it)[i] = -1;
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

    //deal the first frame.
    if (current_frame->id() == 0)
    {
        //Set first frame as origin
        current_frame->setTcw(transform::Rigid3f());
        //Create first Tracked points by Stereo matching
        createTrackedPointsbyStereo(current_frame);
        frames_.push_back(current_frame);
        return;
    }
    //find same tracked point in current_frame.
    trackingByReferenceFrame(current_frame);

    //printf("%d %d\n", tps_.size(),tp_num);
    {
        cv::Mat result_color;
        current_frame->image_.copyTo(result_color);
        auto &keys0 = current_frame->keys0();

        for (auto &tp : current_frame->tps())
        {
            if (tp == nullptr)
                continue;
            auto &Pw = tp->Pw;

            int key_id = tp->frames.back().second;
            Eigen::Vector3f Pc = current_frame->Tcw() * Pw;
            Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
            Eigen::Vector3f reprojection_point = inv_K_.inverse() * image_point;
            reprojection_point = reprojection_point / reprojection_point.z();
            cv::Point loc(reprojection_point.x(), reprojection_point.y());

            cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, keys0[key_id], 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc, keys0[key_id], cv::Scalar(0, 255, 0));
            cv::putText(result_color, (boost::format("%3.1f") % Pc.y()).str(), loc, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);
        }
        cv::imshow("before", result_color);
    }
    updatePoseByReferenceFrame(current_frame);
      //printf("%d %d\n", tps_.size(),tp_num);
    {
        cv::Mat result_color;
        current_frame->image_.copyTo(result_color);
        auto &keys0 = current_frame->keys0();

        for (auto &tp : current_frame->tps())
        {
            if (tp == nullptr)
                continue;
            auto &Pw = tp->Pw;

            int key_id = tp->frames.back().second;
            Eigen::Vector3f Pc = Tcw_ * Pw;
            Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
            Eigen::Vector3f reprojection_point = inv_K_.inverse() * image_point;
            reprojection_point = reprojection_point / reprojection_point.z();
            cv::Point loc(reprojection_point.x(), reprojection_point.y());

            cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, keys0[key_id], 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc, keys0[key_id], cv::Scalar(0, 255, 0));
            cv::putText(result_color, (boost::format("%3.1f") % Pc.y()).str(), loc, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);
        }
        cv::imshow("after", result_color);
    }
  
    cv::waitKey(1);

    /*
    if(frames_.size() == 2){
        frames_.pop_back();
        tracks_.pop_back();
    }
    frames_.push_back(current_frame);
    updateTracks();
    //showReporjection("before_");
    updatePoses();
    //updatePoses();
    showReporjection("after");

    if(frames_.size() >= 2){
        cv::Mat result_color;
        (*frame_it)->image_.copyTo(result_color);

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
    */
}


void Tracking::createTrackedPointsbyStereo(Frame* frame){
    auto &keys0 = frame->keys0();
    auto &desc0 = frame->desc0();
    auto &keys1 = frame->keys1();
    auto &desc1 = frame->desc1();
    auto &frame_tps = frame->tps();
    auto matches = kp_frontend_->getStereoMatching(keys0, desc0, keys1, desc1);

    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);

        auto &left_point = keys0[ref_id];
        auto &right_point = keys1[cur_id];

        //check in same line
        if(std::abs(left_point.y - right_point.y)>20)
            continue;

        float disparity = left_point.x - right_point.x;

        //check disparity
        if (disparity <= 1)
        {
            continue;
        }

        float depth = bf_ / disparity;

        Eigen::Vector3f image_point(left_point.x, left_point.y, 1);
        image_point = inv_K_ * image_point;
        Eigen::Vector3f Pc(image_point.x() / image_point.z(), 1, -image_point.y() / image_point.z());
        Pc = Pc * depth;
        Eigen::Vector3f  Pw = Tcw_.inverse() * Pc;


        TrackedPoint* tp = new TrackedPoint;
        tp->Pw = Pw;
        tp->frames.emplace_back(frame, i);

        frame_tps[ref_id] = tp;
        tps_.push_back(tp);
    }
    /*

    cv::Mat result_color;
    frame->image_.copyTo(result_color);
    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);

        auto &left_point = keys0[ref_id];
        auto &right_point = keys1[cur_id];
        cv::circle(result_color, left_point, 3, cv::Scalar(0, 0, 255));
        cv::circle(result_color, right_point, 3, cv::Scalar(0, 255, 0));
        cv::line(result_color, left_point, right_point, cv::Scalar(0, 255, 0));
    }
    */

    /*
    for (auto &tp : frame->tps())
    {
        if (tp == nullptr)
            continue;
        auto &Pw = tp->Pw;

        int key_id = tp->frames.back().second;
        Eigen::Vector3f Pc = Tcw_ * Pw;
        Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
        Eigen::Vector3f reprojection_point = inv_K_.inverse() * image_point;
        reprojection_point = reprojection_point / reprojection_point.z();
        cv::Point loc(reprojection_point.x(), reprojection_point.y());

        cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
        cv::circle(result_color, keys0[key_id], 3, cv::Scalar(0, 255, 0));
        cv::line(result_color, loc, keys0[key_id], cv::Scalar(0, 255, 0));
        cv::putText(result_color, (boost::format("%3.1f") % Pc.y()).str(), loc, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);
    }
    */
    //cv::imshow("win", result_color);
    //cv::waitKey(1);

    //int tp_num = 0;
    /*
    for (int i = 0; i < (int)keys0.size(); i++)
    {
        int best_score_loc = -1;
        double best_score = 999;

        auto &left_point = keys0[i];
        cv::Mat left_desc = desc0.row(i);
        

        for (int j = 0; j < (int)keys1.size(); j++)
        {
            auto &right_point = keys1[j];
            if (std::abs(left_point.y - right_point.y) > 20)
                continue;
            if (left_point.x < right_point.x)
                continue;
            cv::Mat right_desc = desc1.row(j);
            float score = left_desc.dot(right_desc);
            score = (score < -1) ? -1 : (1 < score) ? 1 : score;
            score = sqrt(2 - 2 * score);
            if (best_score > score)
            {
                best_score = score;
                best_score_loc = j;
            }
        }

        //cant find matched point.
        if (best_score_loc == -1)
        {
            continue;
        }
        //matching score is not good.
        if (best_score > nn_thresh_)
        {
            continue;
        }
        auto &right_point = keys1[best_score_loc];
        float disparity = left_point.x - right_point.x;
        //disparity is too small.
        if (disparity <= 1)
        {
            continue;
        }
        float depth = bf_ / disparity;

        Eigen::Vector3f image_point(left_point.x, left_point.y, 1);
        image_point = inv_K_ * image_point;
        Eigen::Vector3f Pc(image_point.x() / image_point.z(), 1, -image_point.y() / image_point.z());
        Pc = Pc * depth;
        Eigen::Vector3f  Pw = Tcw_.inverse() * Pc;

        TrackedPoint* tp = new TrackedPoint;
        tp->Pw = Pw;
        tp->frames.emplace_back(frame, i);
        frame_tps[i] = tp;
        //tp_num++;
        tps_.push_back(tp);

    }*/
}


} // namespace core
} // namespace visual_slam
