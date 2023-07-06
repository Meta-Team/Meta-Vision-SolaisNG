// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
#define SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
#include <string>
#include <memory>

#include "solais_camera/solais_meta_camera.hpp"
#include "solais_camera/solais_usb_camera.hpp"
#include "solais_camera/solais_camera_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace solais_camera
{
class USBCameraNode
{
public:
  explicit USBCameraNode(const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MetaCamera> camera_;
  std::shared_ptr<CameraServer> server_;
  std::shared_ptr<CamParam> params_ = std::make_shared<CamParam>();  // Camera parameter is shared between MetaCamera, CameraServer and this node
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_USB_CAMERA_NODE_HPP_
