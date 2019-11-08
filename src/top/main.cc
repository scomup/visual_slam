
#include <chrono>
#include <thread>
#include <memory>

#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"
#include "src/top/playable_bag.h"
#include "src/common/make_unique.h"
#include "sensor_bridge.h"
#include "geometry_msgs/PoseStamped.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

ros::Publisher pub_pose;
ros::Publisher pub_pc;
ros::Publisher pub_maker;

std::vector<geometry_msgs::Point> p_list;


#if 1
using namespace visual_slam;

top::SensorBridge *sensor_bridge;
std::shared_ptr<core::Slam> slam_core;


void ImageLCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    sensor_bridge->HandleImageMessage("/stereo/left/image_rect", msg);
}

void ImageRCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    sensor_bridge->HandleImageMessage("/stereo/right/image_rect", msg);
                auto Tcw = slam_core->Tcw();
                auto Twc = Tcw.inverse();
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.frame_id = "map";
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.pose.position.x = Twc.translation().x();
                pose_msg.pose.position.y = Twc.translation().y();
                pose_msg.pose.position.z = Twc.translation().z();
                pose_msg.pose.orientation.x = Twc.rotation().normalized().x();
                pose_msg.pose.orientation.y = Twc.rotation().normalized().y();
                pose_msg.pose.orientation.z = Twc.rotation().normalized().z();
                pose_msg.pose.orientation.w = Twc.rotation().normalized().w();
                pub_pose.publish(pose_msg);
                pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

                slam_core->getPoints(pc);
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*pc, pc_msg);
                pc_msg.header.frame_id = "map";

                pub_pc.publish(pc_msg);

                geometry_msgs::Point p;
                p.x = pose_msg.pose.position.x;
                p.y = pose_msg.pose.position.y;
                p.z = pose_msg.pose.position.z;
                p_list.push_back(p);

                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "map";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_STRIP;
                line_list.color.g = 1.0;
                line_list.color.a = 1.0;
                line_list.scale.x = 0.01;

                for (auto &p : p_list)
                {
                    line_list.points.push_back(p);
                }

                pub_maker.publish(line_list);


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_slam");
    ros::start();
    ros::NodeHandle nh;

    YAML::Node config = YAML::LoadFile("/home/liu/workspace/visual_slam/config/config.yaml");
    slam_core =  std::make_shared<core::Slam>(&config);
    sensor_bridge = new top::SensorBridge(slam_core);

    ros::Subscriber l = nh.subscribe("/stereo/left/image_rect", 1, ImageLCallback);
    ros::Subscriber r = nh.subscribe("/stereo/right/image_rect", 1, ImageRCallback);
    
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("keys", 1);
    pub_maker = nh.advertise<visualization_msgs::Marker> ("path", 1);


    ros::spin();
}



#else
using namespace visual_slam;

int main(int argc, char **argv)
{
    ::ros::init(argc, argv, "visual_slam");
    ::ros::start();
    ros::NodeHandle nh;
    pub_pose = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    pub_pc = nh.advertise<sensor_msgs::PointCloud2> ("keys", 1);
    pub_maker = nh.advertise<visualization_msgs::Marker> ("path", 1);

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

    std::shared_ptr<core::Slam> slam_core;
    slam_core =  std::make_shared<core::Slam>(&config);
    top::SensorBridge bridge(slam_core);

    int c=0;
    while (playable_bag_multiplexer.IsMessageAvailable())
    {
        if (!::ros::ok())
        {
            return 0;
        }

        const auto next_msg_tuple = playable_bag_multiplexer.GetNextMessage();
        const rosbag::MessageInstance &msg = std::get<0>(next_msg_tuple);
        if(c++<0)
            continue;

        if (msg.isType<sensor_msgs::Image>())
        {
            std::string topic = std::string(msg.getTopic());
            bridge.HandleImageMessage(topic, msg.instantiate<sensor_msgs::Image>());
            if (topic == std::string("/stereo/right/image_rect"))
            {
                auto Tcw = slam_core->Tcw();
                auto Twc = Tcw.inverse();
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.frame_id = "map";
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.pose.position.x = Twc.translation().x();
                pose_msg.pose.position.y = Twc.translation().y();
                pose_msg.pose.position.z = Twc.translation().z();
                pose_msg.pose.orientation.x = Twc.rotation().normalized().x();
                pose_msg.pose.orientation.y = Twc.rotation().normalized().y();
                pose_msg.pose.orientation.z = Twc.rotation().normalized().z();
                pose_msg.pose.orientation.w = Twc.rotation().normalized().w();
                pub_pose.publish(pose_msg);
                pcl::PointCloud<pcl::PointXYZ>::Ptr pc;

                slam_core->getPoints(pc);
                sensor_msgs::PointCloud2 pc_msg;
                pcl::toROSMsg(*pc, pc_msg);
                pc_msg.header.frame_id = "map";

                pub_pc.publish(pc_msg);

                geometry_msgs::Point p;
                p.x = pose_msg.pose.position.x;
                p.y = pose_msg.pose.position.y;
                p.z = pose_msg.pose.position.z;
                p_list.push_back(p);

                visualization_msgs::Marker line_list;
                line_list.header.frame_id = "map";
                line_list.header.stamp = ros::Time::now();
                line_list.ns = "points_and_lines";
                line_list.action = visualization_msgs::Marker::ADD;
                line_list.pose.orientation.w = 1.0;
                line_list.type = visualization_msgs::Marker::LINE_STRIP;
                line_list.color.g = 1.0;
                line_list.color.a = 1.0;
                line_list.scale.x = 0.01;

                for (auto &p : p_list)
                {
                    line_list.points.push_back(p);
                }

                pub_maker.publish(line_list);

                ros::spinOnce();
            }
        }

        if (msg.isType<nav_msgs::Odometry>())
        {
            std::string topic = std::string(msg.getTopic());
            bridge.HandleOdometryMessage(topic, msg.instantiate<nav_msgs::Odometry>());
        }
    }
}
#endif