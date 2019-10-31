#include <opencv2/dnn.hpp>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <math.h>

#include "superpoint_frontend.h"

namespace visual_slam
{
namespace core
{

SuperpointFrontend::SuperpointFrontend(const YAML::Node *config)
    : config_(config)
{
    std::shared_ptr<torch::jit::script::Module> module;
    try
    {
        std::string module_path = (*config_)["superpoint"]["module_path"].as<std::string>();
        module_ = torch::jit::script::Module(torch::jit::load(module_path));
    }
    catch (const c10::Error &e)
    {
        std::cerr << "error loading the model\n";
        exit(-1);
    }
    conf_thresh_ = (*config_)["superpoint"]["conf_thresh"].as<float>();
    nn_thresh_ = (*config_)["superpoint"]["nn_thresh"].as<float>();
    nmx_box_size_ = (*config_)["superpoint"]["nmx_box_size"].as<int>();
    //fx_ = (*config_)["camera"]["fx"].as<float>();
    //fy_ = (*config_)["camera"]["fy"].as<float>();
    //cx_ = (*config_)["camera"]["cx"].as<float>();
    //cy_ = (*config_)["camera"]["cy"].as<float>();
    //bf_ = (*config_)["camera"]["baseline"].as<float>() * fx_;

    //inv_K_ = Eigen::Matrix3f::Identity();
    //inv_K_(0, 0) = fx_;
    //inv_K_(1, 1) = fy_;
    //inv_K_(0, 2) = cx_;
    //inv_K_(1, 2) = cy_;
    //inv_K_ = inv_K_.inverse().eval();;
}

void SuperpointFrontend::getKeyPoints(const cv::Mat &img, std::vector<cv::Point> &points, cv::Mat &desc)
{
    cv::Mat input;
    cv::cvtColor(img, input, CV_BGR2GRAY);
    torch::Tensor tensor_img = torch::from_blob(input.data, at::IntList({1, 1, img.rows, img.cols}), at::ScalarType::Byte);
    tensor_img = tensor_img.to(at::ScalarType::Float) / 255.;

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(tensor_img.to(torch::kCUDA));

    auto output = module_.forward(inputs).toTuple();
    auto _prob = output->elements()[0].toTensor().to(torch::kCPU);
    auto _coarse_desc = output->elements()[1].toTensor().to(torch::kCPU);

    cv::Mat prob(input.rows, input.cols, CV_32FC1, _prob.data<float>());

    std::vector<cv::Rect> boxs;
    std::vector<float> scores;

    for (int i = 0; i < (int)prob.rows; i++)
    {
        for (int j = 0; j < (int)prob.cols; j++)
        {
            if (prob.at<float>(i, j) < conf_thresh_)
                continue;
            boxs.push_back(cv::Rect(j , i, nmx_box_size_, nmx_box_size_));
            scores.push_back(prob.at<float>(i, j));
        }
    }

    std::vector<int> indexes;
    cv::dnn::NMSBoxes(boxs, scores, conf_thresh_, 0.4f, indexes);

    //cv::Mat draw_img;
    //cv::cvtColor(img, draw_img, cv::COLOR_GRAY2BGR);

    torch::Tensor samp_pts = torch::zeros({1, 1, (int)indexes.size(), 2});
    int samp_idx = 0;

    for (auto idx : indexes)
    {
        float x = (float)(boxs[idx].x ) / prob.cols * 2 - 1.;
        float y = (float)(boxs[idx].y ) / prob.rows * 2 - 1.;

        samp_pts.data<float>()[samp_idx * 2] = x;
        samp_pts.data<float>()[samp_idx * 2 + 1] = y;
        points.push_back(cv::Point(boxs[idx].x, boxs[idx].y));
        samp_idx++;
    }

    auto _desc = torch::grid_sampler_2d(_coarse_desc, samp_pts, 0, 0);

    cv::Mat local_desc((int)indexes.size(), 256, CV_32FC1);

    auto _desc_a = _desc.accessor<float, 4>();
    for (int l = 0; l < _desc_a.size(3); l++)
    {
        for (int j = 0; j < _desc_a.size(1); j++)
        {
            local_desc.at<float>(l, j) = _desc_a[0][j][0][l];
        }
    }
    /*
    cv::Mat temp, rp;
    for (int i = 0; i <= local_desc.rows - 1; i++)
    {
        cv::normalize(local_desc.row(i), temp);
        rp = local_desc.rowRange(i, i + 1);
        temp.copyTo(rp);
    }
    */

    local_desc.copyTo(desc);
}

SuperpointFrontend::matching_info
SuperpointFrontend::getStereoMatching(const std::vector<cv::Point> &left_points,
                                      const cv::Mat &left_descs,
                                      const std::vector<cv::Point> &right_points,
                                      const cv::Mat &right_descs)
{
    SuperpointFrontend::matching_info matched;
    cv::Mat dmat(left_points.size(), right_points.size(), CV_32FC1, cv::Scalar::all(1));

    for (int i = 0; i < (int)left_points.size(); i++)
    {
        auto &left_point = left_points[i];
        cv::Mat left_desc = left_descs.row(i);
        for (int j = 0; j < (int)right_points.size(); j++)
        {
            auto &right_point = right_points[j];
            if (std::abs(left_point.y - right_point.y) > 20)
                continue;
            if (left_point.x < right_point.x)
                continue;
            cv::Mat right_desc = right_descs.row(j);
            float score = left_desc.dot(right_desc);
            score = (score < -1) ? -1 : (1 < score) ? 1 : score;
            score = sqrt(2 - 2 * score);
            dmat.at<float>(i, j) = score;
        }
    }

    std::vector<int> idx0, idx1;
    for (int i = 0; i < (int)left_points.size(); i++)
    {
        cv::Point minloc;
        double min;
        cv::minMaxLoc(dmat.row(i), &min, NULL, &minloc, NULL);
        idx0.push_back(minloc.x);
    }
    for (int i = 0; i < (int)right_points.size(); i++)
    {
        cv::Point minloc;
        double min;
        cv::minMaxLoc(dmat.col(i), &min, NULL, &minloc, NULL);
        idx1.push_back(minloc.y);
    }

    for (int i = 0; i < (int)left_points.size(); i++)
    {
        int j = idx0[i];
        if (j != -1 && idx1[j] == i && dmat.at<float>(i, j) < nn_thresh_)
            matched.emplace_back(i, j, dmat.at<float>(i, j));
    }
    return matched;
}

SuperpointFrontend::matching_info
SuperpointFrontend::twoWayMatching(const std::vector<cv::Point> &keys0,
                                   const cv::Mat &desc0,
                                   const std::vector<cv::Point> &keys1,
                                   const cv::Mat &desc1)
{

    SuperpointFrontend::matching_info matched;
    cv::Mat dmat(keys0.size(), keys1.size(), CV_32FC1, cv::Scalar::all(1));

    for (int i = 0; i < (int)keys0.size(); i++)
    {
        //auto &curr_point = keys0[i];
        cv::Mat curr_desc = desc0.row(i);
        for (int j = 0; j < (int)keys1.size(); j++)
        {
            //auto &next_point = keys1[j];
            cv::Mat right_desc = desc1.row(j);
            float score = curr_desc.dot(right_desc);
            score = (score < -1) ? -1 : (1 < score) ? 1 : score;
            score = sqrt(2 - 2 * score);
            dmat.at<float>(i, j) = score;
        }
    }

    std::vector<int> idx0, idx1;
    for (int i = 0; i < (int)keys0.size(); i++)
    {
        cv::Point minloc;
        double min;
        cv::minMaxLoc(dmat.row(i), &min, NULL, &minloc, NULL);
        idx0.push_back(minloc.x);
    }
    for (int i = 0; i < (int)keys1.size(); i++)
    {
        cv::Point minloc;
        double min;
        cv::minMaxLoc(dmat.col(i), &min, NULL, &minloc, NULL);
        idx1.push_back(minloc.y);
    }

    for (int i = 0; i < (int)keys0.size(); i++)
    {
        int j = idx0[i];
        if (j != -1 && idx1[j] == i && dmat.at<float>(i, j) < nn_thresh_)
            matched.emplace_back(i, j, dmat.at<float>(i, j));
    }
    return matched;
}

} // namespace core
} // namespace visual_slam
