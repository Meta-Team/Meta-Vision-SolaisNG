// Copyright 2023 Meta-Team
#include "solais_camera/solais_camera_server.hpp"
#include "solais_camera/solais_meta_camera.hpp"

#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"


using namespace std::chrono_literals;

namespace solais_camera
{
CameraServer::CameraServer(rclcpp::Node::SharedPtr node, std::shared_ptr<MetaCamera> camera)
: _node(node), _camera(camera)
{
  declareParameters();

  // Attempt to open the camera
  if (!_camera->open()) {
    RCLCPP_FATAL(_node->get_logger(), "Failed to open input source");
    return;
  }

  // Load camera calibration info
  _camera_info_manager = std::make_shared<camera_info_manager::CameraInfoManager>(
    _node.get(), _camera_name, _camera_info_url);
  if (_camera_info_manager->loadCameraInfo(_camera_info_url)) {
    RCLCPP_INFO(
      _node->get_logger(), "Calibration loaded from '%s'", _camera_info_url.c_str());
  } else {
    RCLCPP_INFO(
      _node->get_logger(), "Calibration file '%s' is missing", _camera_info_url.c_str());
  }
  _cam_info = std::make_shared<sensor_msgs::msg::CameraInfo>(_camera_info_manager->getCameraInfo());

  startPublisher();
  startTimer();
  startService();
  RCLCPP_INFO(_node->get_logger(), "Camera server started. Camera FPS: %d", _params.fps);
}

inline void CameraServer::declareParameters()
{
  // Fetch parameters for basic camera info
  _node->declare_parameter("camera_name", _camera_name);
  _node->declare_parameter("frame_id", _camera_frame_id);
  _node->declare_parameter("camera_info_url", _camera_info_url);
  _node->declare_parameter("use_sensor_data_qos", _use_qos_profile_sensor_data);
  _node->declare_parameter(
    "enable_camera_publisher",
    _enable_camera_publisher);
  _node->get_parameter("camera_name", _camera_name);
  _node->get_parameter("frame_id", _camera_frame_id);
  _node->get_parameter("camera_info_url", _camera_info_url);
  _node->get_parameter("use_sensor_data_qos", _use_qos_profile_sensor_data);
  _node->get_parameter(
    "enable_camera_publisher",
    _enable_camera_publisher);
  if (_camera_frame_id == "") {
    _camera_frame_id = _camera_name + "_optical";
  }
  // Set up camera paramters
  int param;
  constexpr int param_num = sizeof(CamParam) / sizeof(int);
  _camera->getParameter(_params);
  for (int i = 0; i < param_num; i++) {
    param = _params.fetchByIndex(i);
    _node->declare_parameter(kCamParamTypeNames[i], param);
    _node->get_parameter(kCamParamTypeNames[i], param);
    _params.fetchByIndex(i) = param;
  }
  _camera->setParameter(_params);
  if (_params.fps <= 0) {
    _params.fps = 30;
    _camera->setParameter(_params);
  }
}

inline void CameraServer::startPublisher()
{
  // Create Publisher
  if (_enable_camera_publisher) {
    _cam_pub = std::make_shared<image_transport::CameraPublisher>(
      image_transport::create_camera_publisher(
        _node.get(),
        _camera_name + "/image_raw",
        _use_qos_profile_sensor_data ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  } else {
    _legacy_cam_pub = std::make_shared<image_transport::Publisher>(
      image_transport::create_publisher(
        _node.get(), _camera_name + "/image_raw",
        _use_qos_profile_sensor_data ? rmw_qos_profile_sensor_data : rmw_qos_profile_default));
  }
}

inline void CameraServer::startTimer()
{
  // Start a timer that runs forever
  std::function<void()> timer_callback;
  if (_enable_camera_publisher) {
    timer_callback = [this]() {
        if (_camera->getFrame(_img)) {
          cv::waitKey(1);
          _status_ok = true;
          rclcpp::Time time_stamp = _node->now();
          // publish image msg
          if (this->_cam_pub->getNumSubscribers() > 0) {
            sensor_msgs::msg::Image::SharedPtr msg = std::make_shared<sensor_msgs::msg::Image>();
            msg->header.stamp = time_stamp;
            msg->header.frame_id = _camera_frame_id;
            msg->encoding = "bgr8";
            msg->width = _img.cols;
            msg->height = _img.rows;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(_img.step);
            msg->is_bigendian = false;
            msg->data.assign(_img.datastart, _img.dataend);
            _cam_info->header.stamp = time_stamp;
            _cam_info->header.frame_id = _camera_frame_id;
            _cam_pub->publish(*msg, *_cam_info);
          }
        } else {
          // try to reopen camera
          _status_ok = false;
          if (reopen_cnt % _params.fps == 0) {
            _camera->close();
            std::this_thread::sleep_for(100ms);
            if (_camera->open()) {
              RCLCPP_WARN(_node->get_logger(), "Reopen camera successful");
            } else {
              RCLCPP_WARN(_node->get_logger(), "Reopen camera failed!");
            }
          }
          reopen_cnt++;
        }
      };
  } else {
    timer_callback = [this]() {
        if (_camera->getFrame(_img)) {
          _status_ok = true;
          rclcpp::Time stamp = _node->now();
          // publish image msg
          if (this->_legacy_cam_pub->getNumSubscribers() > 0) {
            sensor_msgs::msg::Image::UniquePtr msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = _camera_frame_id;
            msg->encoding = "bgr8";
            msg->width = _img.cols;
            msg->height = _img.rows;
            msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(_img.step);
            msg->is_bigendian = false;
            msg->data.assign(_img.datastart, _img.dataend);
            _legacy_cam_pub->publish(std::move(msg));
            RCLCPP_INFO(_node->get_logger(), "Publishing image");
          }
        } else {
          // try to reopen camera
          _status_ok = false;
          if (reopen_cnt % _params.fps == 0) {
            _camera->close();
            std::this_thread::sleep_for(100ms);
            if (_camera->open()) {
              RCLCPP_WARN(_node->get_logger(), "Reopen camera successful.");
            } else {
              RCLCPP_WARN(_node->get_logger(), "Reopen camera failed!");
            }
          }
          reopen_cnt++;
        }
      };
  }
  auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(1000.0 / _params.fps));
  _timer = _node->create_wall_timer(period_ms, timer_callback);
}

inline void CameraServer::startService()
{
  // To be implemented
}
}  // namespace solais_camera
