#include "solais_camera/solais_mindvision_camera_node.hpp"

namespace solais_camera {
MindVisionCameraNode::MindVisionCameraNode(const rclcpp::NodeOptions & options) : node_(std::make_shared<rclcpp::Node>("mindvision_camera", options))
{
  auto config_path = node_->declare_parameter<std::string>("config_path", "");
  auto camera_serial_number = node_->declare_parameter<std::string>("camera_serial_number", "");
  camera_ = std::make_shared<MindVisionCamera>(node_, config_path, params_, camera_serial_number);
  server_ = std::make_shared<CameraServer>(node_, camera_, params_);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MindVisionCameraNode::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_camera::MindVisionCameraNode)
