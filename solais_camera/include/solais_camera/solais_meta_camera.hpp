// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_
#define SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_

#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"

namespace solais_camera
{
// The struct of all types of camera parameters
struct CamParam
{
  int width;
  int height;
  int auto_exposure;
  int exposure;
  int brightness;
  int auto_white_balance;
  int white_balance;
  int gain;
  int rgb_gain_r;
  int rgb_gain_g;
  int rgb_gain_b;
  int gamma;
  int contrast;
  int saturation;
  int hue;
  int fps;

  int & fetchByIndex(int index)
  {
    switch (index) {
      case 0: return width;
      case 1: return height;
      case 2: return auto_exposure;
      case 3: return exposure;
      case 4: return brightness;
      case 5: return auto_white_balance;
      case 6: return white_balance;
      case 7: return gain;
      case 8: return rgb_gain_r;
      case 9: return rgb_gain_g;
      case 10: return rgb_gain_b;
      case 11: return gamma;
      case 12: return contrast;
      case 13: return saturation;
      case 14: return hue;
      case 15: return fps;
      default:
        RCLCPP_FATAL(rclcpp::get_logger("solais_camera"), "Invalid index for CamParam");
        return width;
    }
  }
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
  virtual bool open() = 0;
  virtual bool close() = 0;
  virtual bool isOpened() = 0;
  virtual bool getFrame(cv::Mat & image) = 0;
  // Parameter Management
  virtual void setParameter(const CamParam & param) = 0;
  virtual void getParameter(CamParam & param) = 0;
  // get error message when above api return false.
  // virtual std::string error_message() = 0;
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_META_CAMERA_HPP_
