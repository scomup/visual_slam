#ifndef VISUAL_SLAM_CORE_SUPERPOINT_FRONTEND
#define VISUAL_SLAM_CORE_SUPERPOINT_FRONTEND

#include <torch/script.h>
#include <torch/torch.h>

#include <opencv2/opencv.hpp>

//#include <opencv2/dnn.hpp>
#include <yaml-cpp/yaml.h>
//#include <eigen3/Eigen/Dense>
//#include <eigen3/Eigen/Geometry>
//#include <math.h>

namespace visual_slam
{
namespace core
{

class SuperpointFrontend
{
    typedef std::vector<std::tuple<int, int, float>> matching_info;

public:
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SuperpointFrontend(const YAML::Node *config);
    void getKeyPoints(const cv::Mat &img, std::vector<cv::Point> &points, cv::Mat &desc);
    matching_info getStereoMatching(const std::vector<cv::Point> &left_points,
                                    const cv::Mat &left_descs,
                                    const std::vector<cv::Point> &right_points,
                                    const cv::Mat &right_descs);
    matching_info twoWayMatching(const std::vector<cv::Point> &keys0,
                                 const cv::Mat &desc0,
                                 const std::vector<cv::Point> &keys1,
                                 const cv::Mat &desc1);
    //const float getScore(const cv::Mat &desc0,const cv::Mat &desc1);
private:
    const YAML::Node *config_;
    torch::jit::script::Module module_;
    float conf_thresh_;
    float nn_thresh_;
    int nmx_box_size_;

    //float fx_;
    //float fy_;
    //float cx_;
    //float cy_;
    //float bf_;
    //Eigen::Matrix3f inv_K_;
};

} //namespace core
} //namespace visual_slam

#endif