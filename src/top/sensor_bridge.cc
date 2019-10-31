#include "sensor_bridge.h"

#include "sensor_msgs/point_cloud_conversion.h"

#include "src/common/make_unique.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>

namespace visual_slam
{

namespace top
{

using visual_slam::transform::Rigid3d;

SensorBridge::SensorBridge(std::shared_ptr<core::Slam> slam)
    : temp_image0_(nullptr),
      temp_image1_(nullptr),
      slam_(slam)
{}

std::unique_ptr<::visual_slam::sensor::OdometryData> SensorBridge::ToOdometryData(const nav_msgs::Odometry::ConstPtr &msg)
{
  double time = msg->header.stamp.toSec();
  //TODO tf_pose (liu)
  transform::Rigid3d pose({0, 0, 0}, transform::RollPitchYaw(0, 0, 0));
  return common::make_unique<sensor::OdometryData>(sensor::OdometryData{time, ToRigid3d(msg->pose.pose) * pose});
}

std::unique_ptr<sensor::ImageData> SensorBridge::ToImageData(const sensor_msgs::Image::ConstPtr &msg)
{
  double time = msg->header.stamp.toSec();

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return nullptr;
  }
  cv::Mat img;
  cv_ptr->image.copyTo(img);

  //cv::resize(img,img,cv::Size(640,400));

  return common::make_unique<sensor::ImageData>(sensor::ImageData{time, img});

return nullptr;
}


void SensorBridge::HandleImageMessage(const std::string& msg_id, const sensor_msgs::Image::ConstPtr& msg)
{
  if (msg_id == "/stereo/left/image_rect_color")
  {
    temp_image0_ = std::move(ToImageData(msg));
  }
  else if (msg_id == "/stereo/right/image_rect_color")
  {
    temp_image1_ = std::move(ToImageData(msg));
  }
  else
  {
    return;
  }
  if (temp_image0_ == nullptr || temp_image1_ == nullptr)
    return;

  if (std::fabs(temp_image0_->time - temp_image1_->time) > 0.001)
    return;

  auto img = common::make_unique<sensor::MultiImageData>(
      sensor::MultiImageData{temp_image0_->time,
                             temp_image0_->image,
                             temp_image1_->image});
  temp_image0_ = nullptr;
  temp_image1_ = nullptr;

  slam_->TrackStereoCamera(std::move(img));



}

void SensorBridge::HandleOdometryMessage(const std::string& msg_id, const nav_msgs::Odometry::ConstPtr &msg)
{
  auto odom = ToOdometryData(msg);
}


Rigid3d SensorBridge::ToRigid3d(const geometry_msgs::TransformStamped& transform) {
  return Rigid3d(ToEigen(transform.transform.translation),
                 ToEigen(transform.transform.rotation));
}

Rigid3d SensorBridge::ToRigid3d(const geometry_msgs::Pose& pose) {
  return Rigid3d({pose.position.x, pose.position.y, pose.position.z},
                 ToEigen(pose.orientation));
}

Eigen::Vector3d SensorBridge::ToEigen(const geometry_msgs::Vector3& vector3) {
  return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
}

Eigen::Quaterniond SensorBridge::ToEigen(const geometry_msgs::Quaternion& quaternion) {
  return Eigen::Quaterniond(quaternion.w, quaternion.x, quaternion.y,
                            quaternion.z);
}



} // namespace top
} // namespace visual_slam
