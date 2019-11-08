#include "src/core/tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

#include <mutex>

#include "src/core/frame.h"
#include <boost/format.hpp>

#include <boost/format.hpp>
#include "src/core/optimizer.h"

using namespace std;

namespace visual_slam
{
namespace core
{
long unsigned int TrackedPoint::next_id = 0;

static const float score(cv::Mat &d1, cv::Mat &d2)
{
    float score = d1.dot(d2);
    score = (score < -1) ? -1 : (1 < score) ? 1 : score;
    score = sqrt(2 - 2 * score);
    return score;
}
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
    Tcw_ = transform::Rigid3f();}

void Tracking::HandleOdometry(std::unique_ptr<sensor::OdometryData> odom)
{
}

cv::Point Tracking::reprojection(const TrackedPoint *tp, const transform::Rigid3f &Tcw) const
{
    auto &Pw = tp->Pw;
    Eigen::Vector3f Pc = Tcw * Pw;
    Eigen::Vector3f local_point = Eigen::Vector3f(Pc.x() / Pc.y(), -Pc.z() / Pc.y(), 1);
    Eigen::Vector3f reprojection_point = K_ * local_point;
    reprojection_point = reprojection_point / reprojection_point.z();
    return cv::Point(reprojection_point.x(), reprojection_point.y());
}
void Tracking::solveCamera()
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

    opt.addReprojectionEdges(problem, K_, tracks_, true);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Problem::EvaluateOptions evaluate_options;
    double total_cost = 0.0;
    std::vector<double> residuals;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    //std::cout << "Finial total cost:" << total_cost << "\n";
    int id = 0;
    for (auto &frame : frames_)
    {
        frame->setTcw(opt.getPose(id));
        id++;
    }
    Tcw_ = frames_.back()->Tcw();
}

void Tracking::solvePoints()
{
    if(frames_.size()<5)
        return;
    Optimizer opt;
    //Set new frame to the same pose as the previous frame
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

    opt.addReprojectionEdges(problem, K_, tracks_, false);

    ceres::Solver::Options options;
    options.max_num_iterations = 100;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Problem::EvaluateOptions evaluate_options;
    double total_cost = 0.0;
    std::vector<double> residuals;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    //problem.Evaluate(evaluate_options, &total_cost, &residuals, nullptr, nullptr);
    //std::cout << "Finial total cost:" << total_cost << "\n";
    int id = 0;
    auto &points = opt.getPoints();
    for (int i = 0; i < (int)tracked_points_.size(); i++)
    {
        tracked_points_[i]->Pw.x() = points[i][0];
        tracked_points_[i]->Pw.y() = points[i][1];
        tracked_points_[i]->Pw.z() = points[i][2];
    }
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
        solveCamera();
    }
}

void Tracking::getPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) const
{
    pc = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < (int)tracked_points_.size(); i++)
    {
        pc->push_back(pcl::PointXYZ(
            tracked_points_[i]->Pw.x(),
            tracked_points_[i]->Pw.y(),
            tracked_points_[i]->Pw.z()));
    }
}

void Tracking::updateTracks()
{
    
    searchProjection(30);

    auto &current_frame = *(frames_.rbegin());
    auto &curr_keys = current_frame->keys0();
    auto &curr_desc = current_frame->desc0();

    auto &prev_frame = *(++frames_.rbegin());
    auto &prev_keys = prev_frame->keys0();
    auto &prev_desc = prev_frame->desc0();
/*
    for()
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
        if(current_frame->tps[curr_id]!= nullptr)
            continue;

        int loc = find(prev_tracks, prev_id);

        if (loc == -1)
        {

            //if(frames_.size()==2){
            //new point
            Eigen::Vector3f point3d(0, 0, 0);
            bool good = prev_frame->computePoint3d(prev_id, point3d);
            if(!good)
                continue;

            for (auto &t : tracks_)
            {
                t.push_back(-1);
            }
            prev_tracks.back() = prev_id;
            curr_tracks.back() = curr_id;

            //auto pose_inv
            auto tp = new TrackedPoint(point3d);
            tp->desc = prev_desc.row(prev_id);
            tracked_points_.push_back(tp);
            current_frame->tps()[curr_id] = tp;
            prev_frame->tps()[prev_id] = tp;
            
        }
        else
        {
            curr_tracks[loc] = curr_id;
            current_frame->tps()[curr_id] = tracked_points_[loc];


            Eigen::Vector3f point3d(0, 0, 0);
            bool good = prev_frame->computePoint3d(prev_id, point3d);
            if (!good)
                continue;
            tracked_points_[loc]->update(point3d);
            tracked_points_[loc]->desc = prev_desc.row(prev_id);
        }
    }

    
    for (int i = 0; i < (int)prev_tracks.size(); i++)
    {
        bool is_tracking = false;
        for (auto &t : tracks_)
        {
            if (t[i] != -1)
                is_tracking = true;
        }
        if (is_tracking == false)
        {
            for (auto &t : tracks_)
            {
                t.erase(t.begin() + i);
            }
            delete tracked_points_[i];
            tracked_points_.erase(tracked_points_.begin() + i);
            i--;
        }
    }*/
}
void Tracking::showReprojection(std::string mark)
{
    auto frame_it = frames_.rbegin();
    auto track_it = tracks_.rbegin();
    int frame_id = 0;
    //bool record = ((*frame_it)->id() == 128);
    while (frame_it != frames_.rend())
    {
        float error = 0;
        int c = 0;
        auto &Tcw = (*frame_it)->Tcw();
        auto &key = (*frame_it)->keys0();
        cv::Mat result_color;
        cv::cvtColor((*frame_it)->image_, result_color, CV_GRAY2BGR);
        cv::resize(result_color,result_color,cv::Size(),2,2);

        for (int i = 0; i < (int)tracked_points_.size(); i++)
        {
            int key_id = (*track_it)[i];
            if (key_id == -1)
                continue;
            cv::Point loc = reprojection(tracked_points_[i], Tcw);
            cv::circle(result_color, loc*2, 3, cv::Scalar(0, 0, 255));
            cv::circle(result_color, key[key_id]*2, 3, cv::Scalar(0, 255, 0));
            cv::line(result_color, loc*2, key[key_id]*2, cv::Scalar(0, 255, 0));
            Eigen::Vector3f Pc = Tcw * tracked_points_[i]->Pw;
            //std::string s = (boost::format("%d,%3.1f") % tracked_points_[i]->id%tracked_points_[i]->Pw.y()).str();
            //std::string s = (boost::format("%3.1f") % Pc.y()).str();
            std::string s = (boost::format("%d") %tracked_points_[i]->frame_cont).str();
            cv::putText(result_color, s, loc*2, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 4);

        }
        for (auto k : key)
        {
            cv::circle(result_color, k*2, 1, cv::Scalar(0, 255, 0));
        }

        //printf("%sframe_%d: Total Reporjection error %f\n", mark.c_str(), (*frame_it)->id(), error / c);
        {
            char s[200];
            sprintf(s, "frame%d.png", (*frame_it)->id());
            auto ss = std::string(s);
            cv::imwrite(std::string("/home/liu/workspace/visual_slam/build/") + mark + ss, result_color);
            cv::imshow("win", result_color);
            cv::waitKey(1);
        }
        frame_id++;

        frame_it++;
        track_it++;
        break;
    }
}

const transform::Rigid3f &Tracking::Tcw() const
{
    return Tcw_;
}
void Tracking::removeOutlier()
{
    int cont = 0;
    auto frame_it = frames_.rbegin();
    auto track_it = tracks_.rbegin();

    while (frame_it != frames_.rend())
    {
        auto &Tcw = (*frame_it)->Tcw();
        auto &key = (*frame_it)->keys0();

        for (int i = 0; i < (int)tracked_points_.size(); i++)
        {
            int key_id = (*track_it)[i];
            if (key_id == -1)
                continue;
            cv::Point proj = key[key_id];
            cv::Point reproj = reprojection(tracked_points_[i], Tcw);

            double dist = sqrt((proj.x - reproj.x) * (proj.x - reproj.x) +
                               (proj.y - reproj.y) * (proj.y - reproj.y));
            if (dist > 20)
            {
                (*track_it)[i] = -1;
                cont++;
            }
        }
        frame_it++;
        track_it++;
    }
    //std::cout<<"Outliers:"<<cont<<std::endl;
}
void Tracking::updateTrackedPoints()
{
    auto &Tcw1 = frames_.front()->Tcw();
    auto &tracks1 = tracks_.front();
    auto &keys1 = frames_.front()->keys0();
    auto &Tcw2 = frames_.back()->Tcw();
    auto &tracks2 = tracks_.back();
    auto &keys2 = frames_.back()->keys0();
    int cont = 0;
    int all = 0;
    Eigen::Matrix4f M1 = Eigen::Matrix4f::Identity();
    M1.block(0, 0, 3, 3) = Tcw1.rotation().normalized().toRotationMatrix();
    M1.block(0, 3, 3, 1) = Tcw1.translation();
    Eigen::Matrix4f M2 = Eigen::Matrix4f::Identity();
    M2.block(0, 0, 3, 3) = Tcw2.rotation().normalized().toRotationMatrix();
    M2.block(0, 3, 3, 1) = Tcw2.translation();

    for (int i = 0; i < (int)tracked_points_.size(); i++)
    {
        int id1 = tracks1[i];
        int id2 = tracks2[i];
        if (id1 == -1 || id2 == -1)
            continue;
        all++;
        auto &key1 = keys1[id1];
        auto &key2 = keys2[id2];

        // Check parallax between rays
        const float invfx = 1.0f / fx_;
        const float invfy = 1.0f / fy_;

        Eigen::Vector3f xn1((key1.x - cx_) * invfx, 1.0, -(key1.y - cy_) * invfy);
        Eigen::Vector3f ray1 = Tcw1.rotation().inverse() * xn1;

        Eigen::Vector3f xn2((key2.x - cx_) * invfx, 1.0, -(key2.y - cy_) * invfy);
        Eigen::Vector3f ray2 = Tcw2.rotation().inverse() * xn2;

        const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

        if (cosParallaxRays < 0 || cosParallaxRays > 0.9998)
            continue;

        // Linear Triangulation Method
        Eigen::Matrix4f A;

        A.row(0) = xn1(0) * M1.row(1) - M1.row(0);
        A.row(1) = xn1(2) * M1.row(1) - M1.row(2);
        A.row(2) = xn2(0) * M2.row(1) - M2.row(0);
        A.row(3) = xn2(2) * M2.row(1) - M2.row(2);

        Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV | Eigen::ComputeFullU);
        Eigen::Vector4f x3D = svd.matrixV().col(3);

        if (x3D(3) == 0)
            continue;

        x3D = x3D / x3D(3);
        Eigen::Vector3f Pw(x3D(0), x3D(1), x3D(2));
        Eigen::Vector3f Pc1 = Tcw1 * Pw;
        if (Pc1.y() <= 0.2)
            continue;
        Eigen::Vector3f Pc2 = Tcw2 * Pw;
        if (Pc2.y() <= 0.2)
            continue;
        //tracked_points_[i]->frame_cont = 0;
        //tracked_points_[i]->update(Pw);
        tracked_points_[i]->Pw = (Pw);
        cont++;
    }
    //std::cout<<"all: "<<tracked_points_.size()<<std::endl;
    //std::cout<<"update: "<<cont<<std::endl;
    //std::cout<<"--------------"<<std::endl;

}

void Tracking::searchProjection(const float radius)
{
    auto &Tcw = frames_.back()->Tcw();
    auto &key = frames_.back()->keys0();
    auto &tps = frames_.back()->tps();
    auto &desc = frames_.back()->desc0();
    tracks_.push_back(std::vector<int>(tracked_points_.size(),-1));
    auto &tracks = tracks_.back();

    for (int i = 0; i < (int)tracked_points_.size(); i++)
    {
        //if(tracks[i] != -1)
        //    continue;
        cv::Point reproj = reprojection(tracked_points_[i], Tcw);
        if(reproj.x > 2*cx_ || reproj.x <0 )
            continue;
        if(reproj.y > 2*cy_ || reproj.y <0 )
            continue;
        float best_score = 999;
        float best_loc = -1;
        for(int j = 0; j < (int)tps.size(); j++)
        {
            if(tps[j]!=nullptr)
                continue;
            auto& proj = key[j];
            double dist = sqrt((proj.x - reproj.x) * (proj.x - reproj.x) +
                               (proj.y - reproj.y) * (proj.y - reproj.y));
            if(dist > radius)
                continue;
            cv::Mat d = desc.row(j);
            float s = score(tracked_points_[i]->desc, d);
            if (best_score > s)
            {
                best_score = s;
                best_loc = j;
            }
        }
        if (best_loc != -1 && best_score < 0.8)
        {
            tracks[i] = best_loc;
            tps[best_loc] = tracked_points_[i];
        }
    }
}

void Tracking::HandleImage(std::unique_ptr<sensor::MultiImageData> image)
{
    Frame *current_frame = new Frame(image->image0, image->image1, image->time, kp_frontend_);
    if (current_frame->id() == 0)
    {
        current_frame->setTcw(transform::Rigid3f());
        auto& keys = current_frame->keys0();
        std::vector<int> t;

        for (int i = 0; i < keys.size(); i++)
        {
            auto &key = keys[i];

            Eigen::Vector3f point3d(0, 0, 0);
            bool good = current_frame->computePoint3d(i, point3d);
            if (!good)
                continue;
            auto tp = new TrackedPoint(point3d);
            tp->desc = current_frame->desc0().row(i);
            current_frame->tps()[i] = tp;
            tracked_points_.push_back(tp);
            t.push_back(i);
        }
        tracks_.push_back(t);
        frames_.push_back(current_frame);
        return;
    }
    else
    {
        current_frame->setTcw(frames_.back()->Tcw());
        frames_.push_back(current_frame);
    }
    searchProjection(30);

    //updateTracks();
    updatePoses();
    removeOutlier();
    updatePoses();
    
    //updateTrackedPoints();

    //solvePoints();
    showReprojection("after");

    /*
    if (frames_.size() >= 2)
    {
        cv::Mat result_color;
        cv::cvtColor(image->image0, result_color, CV_GRAY2BGR);

        auto &curr_tracks = tracks_.back();
        auto &curr_keys = current_frame->keys0();
        for (int i = 0; i < (int)curr_tracks.size(); i++)
        {
            if (curr_tracks[i] == -1)
                continue;
            auto point = curr_keys[curr_tracks[i]];
            auto point3d = tracked_points_[i];
            std::string s = (boost::format("%3.1f") % point3d.z()).str();
            cv::circle(result_color, point, 2, cv::Scalar(0, 255, 0));
            //cv::putText(result_color, s, point, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 1, cv::Scalar(255, 0, 0), 1, 8);
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
    }*/
    

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
