// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
#define SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
#include <string>
#include <opencv2/opencv.hpp>

#include "solais_camera/solais_meta_camera.hpp"


namespace solais_camera
{
class USBCamera : public MetaCamera
{
public:
  explicit USBCamera(const std::string & device_path);
  ~USBCamera();

  bool open() override;
  bool close() override;
  bool isOpened() override;
  bool getFrame(cv::Mat & image) override;
  void setParameter(const CamParam & params) override;
  void getParameter(CamParam & params) override;

private:
  std::string _device_path;
  cv::VideoCapture _cap;
  CamParam _params;
  bool _is_opened{false};
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
