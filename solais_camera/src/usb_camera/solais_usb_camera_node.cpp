// Copyright 2023 Meta-Team
#include "solais_camera/solais_usb_camera_node.hpp"

namespace solais_camera
{
USBCameraNode::USBCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("usb_camera", options)
{
  this->declare_parameter("device_path", "/dev/video0");
  this->get_parameter("device_path", device_path_);
  camera_ = std::make_unique<USBCamera>(device_path_);
  server_ = std::make_shared<CameraServer>(std::shared_ptr<rclcpp::Node>(this), camera_);
}
}  // namespace solais_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_camera::USBCameraNode)
