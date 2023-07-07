// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_CAMERA_SERVER_HPP_
#define SOLAIS_CAMERA__SOLAIS_CAMERA_SERVER_HPP_
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "solais_camera/solais_meta_camera.hpp"


namespace solais_camera
{
class CameraServer
{
public:
  CameraServer(rclcpp::Node::SharedPtr node, std::shared_ptr<MetaCamera> camera, std::shared_ptr<CamParam> params);

private:
  // Helper Functions
  inline void declareParameters();
  inline void startPublisher();
  inline void startTimer();
  inline void startService();
  // Parent node and input source
  rclcpp::Node::SharedPtr _node;
  std::shared_ptr<MetaCamera> _camera;
  // Publishers
  std::shared_ptr<image_transport::CameraPublisher> _cam_pub;
  std::shared_ptr<image_transport::Publisher> _legacy_cam_pub;
  // Timer
  rclcpp::TimerBase::SharedPtr _timer;
  // Services
  // rclcpp::Service<sensor_msgs::srv::SetCameraInfo>::SharedPtr _get_camera_info_srv;
  // Camera Info Manager
  std::shared_ptr<camera_info_manager::CameraInfoManager> _camera_info_manager;
  sensor_msgs::msg::CameraInfo::SharedPtr _cam_info;
  // Internal Parameters
  std::string _camera_name{"default_camera"};
  std::string _camera_frame_id{""};
  std::string _camera_info_url{""};
  bool _enable_camera_publisher{false};
  bool _status_ok{false};
  bool _use_qos_profile_sensor_data{false};
  // Image Data
  cv::Mat _img;
  // Reopen Count
  int reopen_cnt{0};
  // Camera Parameters
  std::shared_ptr<CamParam> _params;
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_CAMERA_SERVER_HPP_
