// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_CAMERA_CLIENT_HPP_
#define SOLAIS_CAMERA__SOLAIS_CAMERA_CLIENT_HPP_
#include <memory>
#include <rclcpp/callback_group.hpp>
#include <string>

#include <opencv2/opencv.hpp>
#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "solais_camera/solais_meta_camera.hpp"


namespace solais_camera
{
class CameraClient
{
public:
  explicit CameraClient(rclcpp::Node::SharedPtr node);
  ~CameraClient();

  void setCameraName(const std::string_view & camera_name);
  void setCameraCallback(std::function<void(const cv::Mat &, const rclcpp::Time &)> callback);
  bool connect();
  void disconnect();
  bool isConnected() const;
  bool getCameraInfo(sensor_msgs::msg::CameraInfo & cam_info) const;

private:
  // Consumer node
  rclcpp::Node::SharedPtr _node;
  // Camera name
  std::string _camera_name;
  // Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _img_sub;
  // Camera callback
  std::function<void(const cv::Mat &, const rclcpp::Time &)> _callback;
  rclcpp::CallbackGroup::SharedPtr _callback_group;
  // Executor
  rclcpp::executors::SingleThreadedExecutor::SharedPtr _executor;
  std::unique_ptr<std::thread> _executor_thread;
  // Internal Parameters
  bool _connected{false};
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_CAMERA_CLIENT_HPP_
