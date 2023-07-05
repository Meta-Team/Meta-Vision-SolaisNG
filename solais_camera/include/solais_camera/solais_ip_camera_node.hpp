// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE
#define SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE
#include <string>
#include <memory>

#include "solais_camera/solais_meta_camera.hpp"
#include "solais_camera/solais_ip_camera.hpp"
#include "solais_camera/solais_camera_server.hpp"
#include "rclcpp/rclcpp.hpp"

namespace solais_camera
{
class IPCameraNode : public rclcpp::Node
{
public:
  explicit IPCameraNode(const rclcpp::NodeOptions & options);
private:
  std::shared_ptr<IPCamera> _camera;
  std::shared_ptr<CameraServer> _server;
  std::shared_ptr<CamParam> params_ = std::make_shared<CamParam>();
};
}  // namespace solais_camera

#endif /* SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE */
