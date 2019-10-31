#ifndef VISUAL_SLAM_CORE_FRAME_H
#define VISUAL_SLAM_CORE_FRAME_H

#include<vector>

#include <opencv2/opencv.hpp>
#include"src/core/superpoint_frontend.h"
#include"src/transform/rigid_transform.h"
#include"src/core/tracked_point.h"

namespace visual_slam{
namespace core{

class MapPoint;
class KeyFrame;


class Frame
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Frame();
    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for mutli cameras.
    Frame(const cv::Mat &im0,
          const cv::Mat &im1,
          const double &timeStamp,
          SuperpointFrontend *kp_frondend_);

    static void setFrameParam(const YAML::Node *config);

    void setTcw(const transform::Rigid3f& Tcw);


    int id()const {return frame_id_;};
    
    const transform::Rigid3f& Tcw() const;
    const std::vector<cv::Point>& keys0() const;
    const cv::Mat& desc0() const;
    const std::vector<cv::Point>& keys1() const;
    const cv::Mat& desc1() const;
    std::vector<TrackedPoint*>& tps();

    cv::Mat image_;
    
private:

    // Frame timestamp.
    double stamp_;

    //static float fx_;
    //static float fy_;
    //static float cx_;
    //static float cy_;
    //static float bf_;
    //static float nn_thresh_;
    //static Eigen::Matrix3f inv_K_;

    static long unsigned int next_id_;
    long unsigned int frame_id_;
    transform::Rigid3f Tcw_;

    std::vector<cv::Point> keys0_, keys1_;
    std::vector<float> depth_;
    std::vector<TrackedPoint*> tps_;
    cv::Mat desc0_; 
    cv::Mat desc1_;
    SuperpointFrontend *kp_frondend_;


};


}// namespace core
}// namespace visual_slam

#endif // VISUAL_SLAM_CORE_FRAME_H
