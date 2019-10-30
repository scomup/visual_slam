/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VISUAL_SLAM_TOP_SENSOR_BRIDGE_H_
#define VISUAL_SLAM_TOP_SENSOR_BRIDGE_H_

#include <memory>

#include "src/sensor/odometry_data.h"
#include "src/sensor/image_data.h"

#include "sensor_msgs/Image.h"
#include "nav_msgs/Odometry.h"

#include "geometry_msgs/Transform.h"
#include "geometry_msgs/TransformStamped.h"

#include "src/core/slam.h"

namespace visual_slam
{
namespace top
{
// Converts ROS messages into SensorData in tracking frame for the MapBuilder.
class SensorBridge
{
public:
  explicit SensorBridge(std::shared_ptr<core::Slam> slam);

  SensorBridge(const SensorBridge &) = delete;
  SensorBridge &operator=(const SensorBridge &) = delete;

  void HandleOdometryMessage(const std::string& msg_id, const nav_msgs::Odometry::ConstPtr &msg);
  void HandleImageMessage(const std::string& msg_id, const sensor_msgs::Image::ConstPtr& msg);

private:
  std::unique_ptr<sensor::OdometryData> ToOdometryData(const nav_msgs::Odometry::ConstPtr &msg);
  std::unique_ptr<sensor::ImageData> ToImageData( const sensor_msgs::Image::ConstPtr &msg);
  std::unique_ptr<sensor::ImageData> temp_image0_;
  std::unique_ptr<sensor::ImageData> temp_image1_;
  std::shared_ptr<core::Slam> slam_;


  transform::Rigid3d ToRigid3d(const geometry_msgs::TransformStamped &transform);
  transform::Rigid3d ToRigid3d(const geometry_msgs::Pose &pose);
  Eigen::Vector3d    ToEigen(const geometry_msgs::Vector3 &vector3);
  Eigen::Quaterniond ToEigen(const geometry_msgs::Quaternion &quaternion);

  //std::shared_ptr<CollatedMapBuilder> collated_map_builder_ptr_;
};

}
} // namespace visual_slam

#endif // VISUAL_SLAM__TOP_SENSOR_BRIDGE_H_
