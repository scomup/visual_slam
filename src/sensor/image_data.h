#ifndef VISUAL_SLAM_SENSOR_IMAGE_DATA_H_
#define VISUAL_SLAM_SENSOR_IMAGE_DATA_H_

#include <opencv2/opencv.hpp>

namespace visual_slam {
namespace sensor {

struct ImageData {
  double time;
  cv::Mat image;

bool operator<(const double rhs) const{
  return this->time < rhs;
}
};

struct MultiImageData {
  double time;
  cv::Mat image0;
  cv::Mat image1;

bool operator<(const double rhs) const{
  return this->time < rhs;
}
};


}  // namespace sensor
}  // namespace visual_slam

#endif  // VISUAL_SLAM_SENSOR_IMAGE_DATA_H_
