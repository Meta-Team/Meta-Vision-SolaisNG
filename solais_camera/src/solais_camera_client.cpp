// Copyright 2023 Meta-Team
#include "solais_camera/solais_camera_client.hpp"
#include "solais_camera/solais_meta_camera.hpp"

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include "sensor_msgs/msg/image.hpp"


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
  std::function<void(const cv::Mat &,
  const rclcpp::Time &)> callback)
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
      auto img =
      cv::Mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char *>(msg->data.data()));
      _callback(img, msg->header.stamp);
    },
    sub_opt
  );

  // Create executor
  _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  _executor->add_callback_group(_callback_group, _node->get_node_base_interface());
  _executor_thread = std::make_unique<std::thread>([&]() {_executor->spin();});

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

bool CameraClient::getCameraInfo(sensor_msgs::msg::CameraInfo & cam_info) const
{
  // @todo: implement this
  return true;
}

}  // namespace solais_camera
