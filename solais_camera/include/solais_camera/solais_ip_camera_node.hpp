// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE
#define SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE
#include <string>
#include <memory>

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
  std::string _device_path;
  std::shared_ptr<IPCamera> _camera;
  std::shared_ptr<CameraServer> _server;
};
}  // namespace solais_camera

#endif /* SOLAIS_CAMERA_SOLAIS_IP_CAMERA_NODE */
