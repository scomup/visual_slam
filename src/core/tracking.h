#ifndef VISUAL_SLAM_CORE_TRACKING_H
#define VISUAL_SLAM_CORE_TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
#include<memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "src/sensor/odometry_data.h"
#include "src/sensor/image_data.h"
#include "src/core/superpoint_frontend.h"
#include "src/core/frame.h"
#include"src/core/tracked_point.h"

#include <deque>

namespace visual_slam
{
namespace core
{

class Tracking
{  

public:
    Tracking(const YAML::Node* config);

    void HandleOdometry(const std::unique_ptr<sensor::OdometryData> odom);
    void HandleImage(const std::unique_ptr<sensor::MultiImageData> image);
    const transform::Rigid3f& Tcw() const;
    void getPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& pc) const;

private:
    void updateTracks();
    void updatePoses();
    void showReprojection(std::string mark);
    cv::Point reprojection(const TrackedPoint *tp, const transform::Rigid3f &Tcw) const;
    void removeOutlier();
    void updateTrackedPoints();
    void solveCamera();
    void solvePoints();
    void searchProjection(const float radius);



    std::list<Frame*> frames_;
    SuperpointFrontend *kp_frontend_;
    const YAML::Node* config_;
    const int max_tracks_;
    Eigen::Matrix3f K_;
    Eigen::Matrix3f inv_K_;

    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float bf_;
    float nn_thresh_;
    
    std::list<std::vector<int>> tracks_;
    std::vector<TrackedPoint*> tracked_points_;
    transform::Rigid3f Tcw_;

};

} //namespace core
} //namespace visual_slam

#endif // VISUAL_SLAM_CORE_TRACKING_H
