
#include <chrono>
#include <thread>
#include <memory>

#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "src/top/playable_bag.h"
#include "src/common/make_unique.h"
#include "sensor_bridge.h"



#if 1
using namespace visual_slam;

top::SensorBridge *sensor_bridge;

void ImageLCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    sensor_bridge->HandleImageMessage("/stereo/left/image_rect", msg);
}

void ImageRCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    sensor_bridge->HandleImageMessage("/stereo/right/image_rect", msg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_slam");
    ros::start();
    ros::NodeHandle nh;

    YAML::Node config = YAML::LoadFile("/home/liu/workspace/visual_slam/config/config.yaml");
    std::unique_ptr<core::Slam> slam_core =  common::make_unique<core::Slam>(&config);
    sensor_bridge = new top::SensorBridge(std::move(slam_core));

    ros::Subscriber l = nh.subscribe("/stereo/left/image_rect", 1, ImageLCallback);
    ros::Subscriber r = nh.subscribe("/stereo/right/image_rect", 1, ImageRCallback);

    ros::spin();
}



#else
using namespace visual_slam;

int main(int argc, char **argv)
{
    ::ros::init(argc, argv, "visual_slam");
    ::ros::start();

    std::string filename_in;

    if (argc >= 2)
    {
        filename_in = argv[1];
        std::cout << "1:" << filename_in << std::endl;
    }
    else
    {
        filename_in = std::string("/home/liu/2019-10-28-11-46-44.bag");
    }

    top::PlayableBagMultiplexer playable_bag_multiplexer;
    std::cout << filename_in << std::endl;
    playable_bag_multiplexer.AddPlayableBag(top::PlayableBag(
        filename_in, 0, ros::TIME_MIN, ros::TIME_MAX, ::ros::Duration(1.0),
        [](const rosbag::MessageInstance &msg) { return true; }));

    YAML::Node config = YAML::LoadFile("/home/liu/workspace/visual_slam/config/config.yaml");

    std::unique_ptr<core::Slam> slam_core =
        common::make_unique<core::Slam>(&config);
    top::SensorBridge bridge(std::move(slam_core));

    while (playable_bag_multiplexer.IsMessageAvailable())
    {
        if (!::ros::ok())
        {
            return 0;
        }

        const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
        const rosbag::MessageInstance &msg = std::get<0>(next_msg_tuple);

        if (msg.isType<sensor_msgs::Image>())
        {
            std::string topic = std::string(msg.getTopic());
            bridge.HandleImageMessage(topic, msg.instantiate<sensor_msgs::Image>());
        }

        if (msg.isType<nav_msgs::Odometry>())
        {
            std::string topic = std::string(msg.getTopic());
            bridge.HandleOdometryMessage(topic, msg.instantiate<nav_msgs::Odometry>());
        }
    }
}
#endif