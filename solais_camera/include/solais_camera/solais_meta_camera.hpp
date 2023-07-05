// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_
#define SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_

#include <memory>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

namespace solais_camera
{
// The struct of all types of camera parameters
struct CamParam
{
  long width;
  long height;
  bool auto_exposure;
  long exposure;
  long brightness;
  long auto_white_balance;
  bool white_balance;
  long gain;
  long rgb_gain_r;
  long rgb_gain_g;
  long rgb_gain_b;
  long gamma;
  long contrast;
  long saturation;
  long hue;
  long fps;
};

constexpr const char * kCamParamTypeNames[] = {
  "width",
  "height",
  "auto_exposure",
  "exposure",
  "brightness",
  "auto_white_balance",
  "white_balance",
  "gain",
  "rgb_gain_r",
  "rgb_gain_g",
  "rgb_gain_b",
  "gamma",
  "contrast",
  "saturation",
  "hue",
  "fps"
};

// The abstract class of all types of all cameras
class MetaCamera
{
public:
  MetaCamera(std::shared_ptr<CamParam> params) : _params(params) {}
  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual bool isOpened() = 0;
  virtual bool getFrame(cv::Mat & image) = 0;
  // get error message when above api return false.
  // virtual std::string error_message() = 0;
protected:
  std::shared_ptr<CamParam> _params;
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_
