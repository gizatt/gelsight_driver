/**
 *  Interfaces via OpenCV webcam drivers with a GelSight
 *  unit. Provides any combination of:
 *    - Raw image output
 *    - Image output with tracking dots masked out
 *    - Normal maps output, given a configuration
 *    - Depth map generation, given a configuration
 *  depending on configuration parameters.
 *
 *  Copyright Robot Locomotion Group, 2017.
 */

#pragma once

#include <opencv2/opencv.hpp>
#include "yaml-cpp/yaml.h"

class GelsightDepthDriver {
 public:
  GelsightDepthDriver(const std::string& video_source);
  /***
     Pulls down a webcam image from the webcam,
     and updates both the internally stored latest
     webcam image, and its is_good_data mask.
  ***/
  void GetNewWebcamImage();

  /***
     Takes the latest webcam image and updates
     the internal normal map using the stored
     lookup table.
  ***/
  void UpdateNormalMap();

  /***
     Takes the latest normal map and performs
     a depth estimation using the stored config.
  ***/
  void UpdateDepthEstimate();

  const cv::Mat& get_last_raw_image() { return last_raw_image_; }

 private:
  cv::VideoCapture capture_;
  YAML::Node config_;
  cv::Mat last_raw_image_;
};