#ifndef VISUAL_SLAM_CORE_SLAM_H_
#define VISUAL_SLAM_CORE_SLAM_H_

#include<string>
#include<thread>
#include<opencv2/core/core.hpp>
#include"src/sensor/image_data.h"
#include"src/core/tracking.h"
#include"src/common/make_unique.h"
#include "yaml-cpp/yaml.h"


namespace visual_slam
{

namespace core
{
class Slam
{
public:

    Slam(const YAML::Node* config);
    
    int TrackStereoCamera(std::unique_ptr<sensor::MultiImageData> image);

private:
    std::unique_ptr<Tracking> tracking_;

/*
    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    Map* mpMap;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
    Tracking* mpTracker;

    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;


    // The viewer draws the map and the current camera pose. It uses Pangolin.
    Viewer* mpViewer;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;
    std::thread* mptLoopClosing;
    std::thread* mptViewer;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;
    */
};

}// namespace core
}// namespace visual_slam

#endif // VISUAL_SLAM_CORE_SLAM_H_
