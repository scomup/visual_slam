#include "src/core/frame.h"

namespace visual_slam
{
namespace core
{
long unsigned int Frame::next_id_ = 0;
float Frame::fx_;
float Frame::fy_;
float Frame::cx_;
float Frame::cy_;
float Frame::bf_;
Eigen::Matrix3f Frame::inv_K_;


// Constructor for stereo cameras.
Frame::Frame(const cv::Mat &im0,
             const cv::Mat &im1,
             const double &timeStamp,
             SuperpointFrontend *kp_frondend)
    :  stamp_(timeStamp), kp_frondend_(kp_frondend)

{
    kp_frondend->getKeyPoints(im0, keys0_, desc0_);
    kp_frondend->getKeyPoints(im1, keys1_, desc1_);


    // Frame ID
    frame_id_ = next_id_++;

}

void Frame::setPose(const transform::Rigid3f &pose)
{
    pose_ = pose;
}

const transform::Rigid3f& Frame::pose() const
{
    return pose_;
}

bool Frame::computePoint3d(const int left_id, Eigen::Vector3f& point3d) const
{
    int minloc = -1;
    double min = 999;

    auto &left_point = keys0_[left_id];
    cv::Mat left_desc = desc0_.row(left_id);

    for (int j = 0; j < (int)keys1_.size(); j++)
    {
        auto &right_point = keys1_[j];
        if (std::abs(left_point.y - right_point.y) > 20)
            continue;
        if (left_point.x < right_point.x)
            continue;
        cv::Mat right_desc = desc1_.row(j);
        float score = left_desc.dot(right_desc);
        score = (score < -1) ? -1 : (1 < score) ? 1 : score;
        score = sqrt(2 - 2 * score);
        if(min > score){
            min = score;
            minloc = j;
        }
    }

    if(minloc == -1)
        return false;
    auto &right_point = keys1_[minloc];


    float disparity = left_point.x - right_point.x;
    if (disparity <= 1)
        return false;
    float depth = bf_/disparity;
    
    Eigen::Vector3f image_point(left_point.x, left_point.y, 1);
    point3d = inv_K_ * image_point;
    point3d /= point3d.z();
    point3d = point3d * depth;
    point3d = pose_ * point3d;
    return true;
}



void Frame::setFrameParam(const YAML::Node *config)
{
    fx_ = (*config)["camera"]["fx"].as<float>();
    fy_ = (*config)["camera"]["fy"].as<float>();
    cx_ = (*config)["camera"]["cx"].as<float>();
    cy_ = (*config)["camera"]["cy"].as<float>();
    bf_ = (*config)["camera"]["baseline"].as<float>() * fx_;
    inv_K_ = Eigen::Matrix3f::Identity();
    inv_K_(0, 0) = fx_;
    inv_K_(1, 1) = fy_;
    inv_K_(0, 2) = cx_;
    inv_K_(1, 2) = cy_;
    inv_K_ = inv_K_.inverse().eval();

}

const std::vector<cv::Point> &Frame::keys0() const
{
    return keys0_;
}

const cv::Mat &Frame::desc0() const
{
    return desc0_;
}

} // namespace core
} // namespace visual_slam
