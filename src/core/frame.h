#ifndef VISUAL_SLAM_CORE_FRAME_H
#define VISUAL_SLAM_CORE_FRAME_H

#include<vector>

#include <opencv2/opencv.hpp>
#include"src/core/superpoint_frontend.h"
#include"src/transform/rigid_transform.h"

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

    void setPose(const transform::Rigid3f& pose);

    bool computePoint3d(const int left_id, Eigen::Vector3f& point3d) const;
    
    const transform::Rigid3f& pose() const;
    const std::vector<cv::Point>& keys0() const;
    const cv::Mat& desc0() const;

private:

    // Frame timestamp.
    double stamp_;

    static float fx_;
    static float fy_;
    static float cx_;
    static float cy_;
    static float bf_;
    static Eigen::Matrix3f inv_K_;

    static long unsigned int next_id_;
    long unsigned int frame_id_;
    transform::Rigid3f pose_;
    // Number of KeyPoints.
    int N_;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::Point> keys0_, keys1_;
    std::vector<float> depth_;


    // ORB descriptor, each row associated to a keypoint.
    cv::Mat desc0_; 
    cv::Mat desc1_;

    SuperpointFrontend *kp_frondend_;


};


}// namespace core
}// namespace visual_slam

#endif // VISUAL_SLAM_CORE_FRAME_H