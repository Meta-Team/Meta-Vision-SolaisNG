// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
#define SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include "solais_camera/solais_meta_camera.hpp"


namespace solais_camera
{
class USBCamera : public MetaCamera
{
public:
  explicit USBCamera(std::shared_ptr<rclcpp::Node> node, const std::string & device_path, std::shared_ptr<CamParam> params);
  ~USBCamera();

  bool open() override;
  bool close() override;
  bool isOpened() override;
  bool getFrame(cv::Mat & image) override;

private:
  std::shared_ptr<rclcpp::Node> _node;
  std::string _device_path;
  cv::VideoCapture _cap;
  bool _is_opened{false};
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_USB_CAMERA_HPP_
