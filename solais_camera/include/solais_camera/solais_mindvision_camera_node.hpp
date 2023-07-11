// Copyright 2023 Meta-Team
#ifndef SOLAIS_MINDVISION_CAMERA_NODE_HPP_
#define SOLAIS_MINDVISION_CAMERA_NODE_HPP_
#include <memory>
#include <string>

#include "solais_camera/solais_mindvision_camera.hpp"
#include "rclcpp/rclcpp.hpp"
#include "solais_camera/solais_camera_server.hpp"
#include "solais_camera/solais_meta_camera.hpp"

namespace solais_camera {
class MindVisionCameraNode {
public:
  explicit MindVisionCameraNode(const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MetaCamera> camera_;
  std::shared_ptr<CameraServer> server_;
  std::shared_ptr<CamParam> params_ = std::make_shared<CamParam>();  // Camera parameter is shared between MetaCamera, CameraServer and this node
};
}

#endif 