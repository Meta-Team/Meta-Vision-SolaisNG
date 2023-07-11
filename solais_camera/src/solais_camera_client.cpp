// Copyright 2023 Meta-Team
#include "solais_camera/solais_camera_client.hpp"
#include "rmw/qos_profiles.h"
#include "solais_camera/solais_meta_camera.hpp"

#include <algorithm>
#include <opencv2/opencv.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "solais_interfaces/srv/get_camera_info.hpp"


using namespace std::chrono_literals;

namespace solais_camera
{
CameraClient::CameraClient(rclcpp::Node::SharedPtr node)
: _node(node)
{
}

CameraClient::~CameraClient()
{
  if (_connected) {
    disconnect();
  }
}

void CameraClient::setCameraName(const std::string_view & camera_name)
{
  if (_connected) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera %s is already connected, setCameraName failed.",
      _camera_name.c_str());
    return;
  }
  _camera_name = camera_name;
}

void CameraClient::setCameraCallback(
  const std::function<void(const cv::Mat &, const std_msgs::msg::Header &)> & callback)
{
  if (_connected) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera %s is already connected, setCameraCallback failed.",
      _camera_name.c_str());
    return;
  }
  _callback = callback;
}

bool CameraClient::connect()
{
  // Check parameters
  if (_connected) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera %s is already connected, connect failed.",
      _camera_name.c_str());
    return false;
  }
  if (_camera_name.empty()) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera name is empty, connect failed.");
    return false;
  }
  if (!_callback) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera callback is empty, connect failed.");
    return false;
  }

  // Create subscription
  auto sub_opt = rclcpp::SubscriptionOptions();
  _callback_group = _node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  sub_opt.callback_group = _callback_group;
  _img_sub = _node->create_subscription<sensor_msgs::msg::Image>(
    _camera_name + "/image_raw", 1,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      auto img = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
      _callback(img, msg->header);
    },
    sub_opt
  );

  // Create executor
  _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  _executor->add_callback_group(_callback_group, _node->get_node_base_interface());
  _executor_thread = std::make_unique<std::thread>([this]() {_executor->spin();});

  // Set connected
  _connected = true;
  return true;
}

void CameraClient::disconnect()
{
  if (_connected) {
    _executor->cancel();
    _executor_thread->join();
    _executor.reset();
    _executor_thread.reset();
    _img_sub.reset();
    _camera_name = "";
    _connected = false;
  }
}

bool CameraClient::getCameraInfo(
  const std::function<void(sensor_msgs::msg::CameraInfo::ConstSharedPtr)> & service_callback)
{
  if (_camera_name.empty()) {
    RCLCPP_ERROR(
      _node->get_logger(),
      "[CameraClient] camera name is empty, get camera info failed.");
    return false;
  }
  _camera_info_sub = _node->create_subscription<sensor_msgs::msg::CameraInfo>(
    _camera_name + "/camera_info", rclcpp::SystemDefaultsQoS(),
    [this, service_callback](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      RCLCPP_INFO(_node->get_logger(), "Camera info received.");
      service_callback(camera_info);
      _camera_info_sub.reset();
    });
  return true;
}

}  // namespace solais_camera
