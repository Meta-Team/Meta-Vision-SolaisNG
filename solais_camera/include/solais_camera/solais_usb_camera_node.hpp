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
  std::string _device_path;
  std::shared_ptr<USBCamera> _camera;
  std::shared_ptr<CameraServer> _server;
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
