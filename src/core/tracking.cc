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

int Tracking::updatePoseByReferenceFrame(Frame *frame)
{
    Frame *ref_frame = frames_.back();
    Optimizer opt;
    //Set new frame to the same pose as the reference frame.
    frame->setTcw(frames_.back()->Tcw());

    auto frame_it = frames_.begin();
    auto track_it = tracks_.begin();

    int frame_id = 0;

    /*
    opt.setFrameNum(frames_.size()+1);
    for (auto &ref_frame : frames_)
    {
        opt.addPose(frame_id, ref_frame->Tcw());
        opt.addKeys(frame_id, ref_frame->keys0());
        opt.addPoints(frame_id, ref_frame->tps());
        frame_id++;
    }
    opt.addPose(frame_id, frame->Tcw());
    opt.addKeys(frame_id, frame->keys0());
    opt.addPoints(frame_id, frame->tps());
    */
    opt.setFrameNum(2);
    opt.addPose(0, ref_frame->Tcw());
    opt.addKeys(0, ref_frame->keys0());
    opt.addPoints(0, ref_frame->tps());

    opt.addPose(1, frame->Tcw());
    opt.addKeys(1, frame->keys0());
    opt.addPoints(1, frame->tps());


    ceres::Problem problem;
    opt.addReprojectionEdges(problem, K_);
    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Problem::EvaluateOptions evaluate_options;
    double total_cost = 0.0;
    std::vector<double> residuals;
    ceres::Solver::Summary summary;
    //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    //std::cout << "Begin total cost:" << total_cost << "\n";


    ceres::Solve(options, &problem, &summary);
    //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    //std::cout << "Finial total cost:" << total_cost << "\n";
    Tcw_ = opt.getNewPose();
    opt.resetTps();

    auto&frame_tps = frame->tps();
    int num = 0;
    for (int i = 0; i < (int)frame_tps.size(); i++)
    {
        auto& p = frame_tps[i];
        if(p== nullptr)
            continue;

        auto &Pw = p->Pw;

        int key_id = i;
        auto loc_o = frame->keys0()[i];
        Eigen::Vector3f Pc = Tcw_ * Pw;
        Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
        Eigen::Vector3f reprojection_point = K_ * image_point;
        reprojection_point = reprojection_point / reprojection_point.z();
        cv::Point loc_r(reprojection_point.x(), reprojection_point.y());

        double d = sqrt((loc_o.x - loc_r.x) * (loc_o.x - loc_r.x) + (loc_o.y - loc_r.y) * (loc_o.y - loc_r.y));
        if(d>20)
        {
            //auto it = std::find(p->frames.begin(), p->frames.end(), std::make_pair(frame,i));
            //p->frames.erase(it);
            frame_tps[i] = nullptr;
        }
        num++;
            
    }
    if(num < 10){
        //std::cout<<"tracking failed!"<<num<<std::endl;
        return num;
    }
    frame->setTcw(Tcw_);
    return num;
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

    //find same tracked_points with reference frame.
    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);

        auto tp = ref_tps[ref_id];
        if(tp != nullptr)
        {
            //tp->frames.emplace_back(frame, cur_id);
            cur_tps[cur_id] = tp;
        }
    }
    int tracked_num;
    //update pose.
    tracked_num = updatePoseByReferenceFrame(frame);
    //remove outliers and update again.
    tracked_num = updatePoseByReferenceFrame(frame);


    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);

        auto tp = ref_tps[ref_id];
        if(tp == nullptr)
        {
            //continue;
            //tp->frames.emplace_back(frame, cur_id);
            //cur_tps[cur_id] = tp;

            auto& cur_key = cur_keys[cur_id];
            auto& ref_key = ref_keys[ref_id];

            // Check parallax between rays
            const float invfx = 1.0f / fx_;
            const float invfy = 1.0f / fy_;
            
            auto& Tcw1 = frame->Tcw();
            auto& Tcw2 = ref_frame->Tcw();
            Eigen::Vector3f xn1((cur_key.x - cx_) * invfx, 1.0, -(cur_key.y - cy_) * invfy);
            //Eigen::Vector3f xn1((cur_key.x - cx_) * invfx, (cur_key.y - cy_) * invfy, 1);
            Eigen::Vector3f ray1 = Tcw1.rotation().inverse() * xn1;

            Eigen::Vector3f xn2((ref_key.x - cx_) * invfx, 1.0, -(ref_key.y - cy_) * invfy);
            //Eigen::Vector3f xn2((ref_key.x - cx_) * invfx, (ref_key.y - cy_) * invfy, 1);
            Eigen::Vector3f ray2 = Tcw2.rotation().inverse() * xn2;

            const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

            if (cosParallaxRays < 0 || cosParallaxRays > 0.999)
                continue;

            // Linear Triangulation Method
            Eigen::Matrix4f A;
            

            Eigen::Matrix4f M1 = Eigen::Matrix4f::Identity();
            M1.block(0,0,3,3) = Tcw1.rotation().normalized().toRotationMatrix();
            M1.block(0,3,3,1) = Tcw1.translation();
            Eigen::Matrix4f M2 = Eigen::Matrix4f::Identity();
            M2.block(0,0,3,3) = Tcw2.rotation().normalized().toRotationMatrix();
            M2.block(0,3,3,1) = Tcw2.translation();

            A.row(0) = xn1(0) * M1.row(1) - M1.row(0);
            A.row(1) = xn1(2) * M1.row(1) - M1.row(2);
            A.row(2) = xn2(0) * M2.row(1) - M2.row(0);
            A.row(3) = xn2(2) * M2.row(1) - M2.row(2);

            Eigen::JacobiSVD<Eigen::Matrix4f> svd( A, Eigen::ComputeFullV | Eigen::ComputeFullU );
            Eigen::Vector4f x3D = svd.matrixV().col(3);

             if (x3D(3) == 0)
                continue;

            x3D = x3D / x3D(3);
            Eigen::Vector3f Pw(x3D(0),x3D(1),x3D(2));
            Eigen::Vector3f Pc1 = Tcw1 * Pw;
            if(Pc1.y() <= 0.2)
                continue;
            Eigen::Vector3f Pc2 = Tcw2 * Pw;
            if(Pc2.y() <= 0.2)
                continue;
            TrackedPoint* tp = new TrackedPoint;
            tp->Pw = Pw;
            tp->frames.emplace_back(ref_frame, ref_id);

            cur_tps[cur_id] = tp;
            ref_frame->tps()[ref_id] = tp;
            //tps_.push_back(tp);
            tracked_num ++;

            /*
            {
                Eigen::Vector3f Pc = Tcw1 * Pw;
                Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
                Eigen::Vector3f reprojection_point = K_ * image_point;
                reprojection_point = reprojection_point / reprojection_point.z();
                cv::Point loc(reprojection_point.x(), reprojection_point.y());
                std::cout << loc << std::endl;
                std::cout << cur_key << std::endl;
            }
            {
                Eigen::Vector3f Pc = Tcw2 * Pw;
                Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
                Eigen::Vector3f reprojection_point = K_ * image_point;
                reprojection_point = reprojection_point / reprojection_point.z();
                cv::Point loc(reprojection_point.x(), reprojection_point.y());
                std::cout << loc << std::endl;
                std::cout << ref_key << std::endl;
            }
            */
            



        }
        //std::cout<<"tracked num: "<<tracked_num<<std::endl;
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
        //(*frame_it)->image_.copyTo(result_color);
        cv::cvtColor((*frame_it)->image_, result_color, cv::COLOR_GRAY2BGR);




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


    {
        auto &ref_frame = *(frames_.rbegin());

        auto &Tcw1 = current_frame->Tcw();
        auto &Tcw2 = ref_frame->Tcw();
        
        float diff_rotation = Eigen::AngleAxisf(Tcw1.rotation() * Tcw2.rotation()).angle();
        float diff_translation = (Tcw1.translation() - Tcw2.translation()).norm();
        //std::cout<<"diff_rotation:"<<diff_rotation<<std::endl;
        //std::cout<<"diff_translation:"<<diff_translation<<std::endl;
        if (diff_translation > 0.05 || diff_rotation > 180. / M_PI * 5.)
        {
            std::cout<<"new key frame("<<current_frame->id()<<").\n";
            //createTrackedPointsbyStereo(current_frame);
            //trackingByReferenceFrame(current_frame);
            //frames_.push_back(current_frame);
        }
    }
    

    /*
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
    */
    
      //printf("%d %d\n", tps_.size(),tp_num);
    {
        //auto &ref_frame = *(frames_.rbegin());
        cv::Mat result_color;
        //current_frame->image_.copyTo(result_color);
        cv::cvtColor(current_frame->image_, result_color, cv::COLOR_GRAY2BGR);
        auto &keys0 = current_frame->keys0();
        auto& frame_tps = current_frame->tps();
        for (int i = 0; i < (int)frame_tps.size(); i++)
        {
            if (frame_tps[i] == nullptr)
                continue;
            auto &Pw = frame_tps[i]->Pw;

            Eigen::Vector3f Pc = current_frame->Tcw() * Pw;
            Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
            Eigen::Vector3f reprojection_point = inv_K_.inverse() * image_point;
            reprojection_point = reprojection_point / reprojection_point.z();
            cv::Point loc(reprojection_point.x(), reprojection_point.y());

            cv::circle(result_color, loc, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, keys0[i], 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc, keys0[i], cv::Scalar(0, 255, 0));
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
    
    //auto matches = kp_frontend_->getStereoMatching(keys0, desc0, keys1, desc1);
    auto matches = kp_frontend_->twoWayMatching(keys0, desc0, keys1, desc1);


    for (int i = 0; i < (int)matches.size(); i++)
    {
        int ref_id = std::get<0>(matches[i]);
        int cur_id = std::get<1>(matches[i]);


        if (frame_tps[ref_id] == nullptr)
        {

            auto &left_point = keys0[ref_id];
            auto &right_point = keys1[cur_id];

            //check in same line
            if (std::abs(left_point.y - right_point.y) > 20)
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
            Eigen::Vector3f Pw = Tcw_.inverse() * Pc;

            TrackedPoint *tp = new TrackedPoint;
            tp->Pw = Pw;
            tp->frames.emplace_back(frame, ref_id);

            frame_tps[ref_id] = tp;
            //tps_.push_back(tp);
        }
        else{
            auto& tp = frame_tps[ref_id];
            {
                //Eigen::Vector3f Pc = frame->Tcw() * tp->Pw;
                //Eigen::Vector3f image_point(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
                //Eigen::Vector3f reprojection_point = K_ * image_point;
                //reprojection_point = reprojection_point / reprojection_point.z();
                //cv::Point loc(reprojection_point.x(), reprojection_point.y());
                //std::cout << loc << std::endl;
                //std::cout << keys0[ref_id] << std::endl;
                tp->frames.emplace_back(frame, ref_id);
            }

    
        }
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
