// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
#define SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
#include <string>
#include <memory>

#include "solais_camera/solais_usb_camera.hpp"
#include "solais_camera/solais_camera_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace solais_camera
{
class USBCameraNode : public rclcpp::Node
{
public:
  explicit USBCameraNode(const rclcpp::NodeOptions & options);
private:
  std::string device_path_;
  std::shared_ptr<USBCamera> camera_;
  std::shared_ptr<CameraServer> server_;
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
