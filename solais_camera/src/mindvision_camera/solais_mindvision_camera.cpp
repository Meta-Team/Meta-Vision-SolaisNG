// Copyright 2023 Meta-Team
#include "solais_camera/solais_mindvision_camera.hpp"
#include <rclcpp/logging.hpp>
#include "CameraApi.h"
#include "CameraDefine.h"
#include "CameraStatus.h"

namespace solais_camera
{
MindVisionCamera::MindVisionCamera(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & config_path,
  std::shared_ptr<CamParam> params,
  const std::string & serial_number)
: MetaCamera(params), node_(node), config_path_(config_path), serial_number_(serial_number)
{
}

bool MindVisionCamera::open()
{
  if (opened_) {
    return true;
  }

  CameraSdkStatus status;

  // Get list of camera devices
  int camera_num = 3;
  status = CameraEnumerateDevice(camera_list_, &camera_num);
  if (status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_FATAL(node_->get_logger(), "CameraEnumerateDevice failed, no device found.");
    return false;
  }

  // Find device with matching serial number
  bool found_device = false;
  for (int i = 0; i < camera_num; i++) {
    if (camera_list_[i].acSn == serial_number_) {
      status = CameraInit(&camera_list_[i], -1, -1, &handle_camera_);
      if (status != CAMERA_STATUS_SUCCESS) {
        RCLCPP_FATAL(node_->get_logger(), "CameraInit failed.");
        return false;
      }
      found_device = true;
      break;
    }
  }

  if (!found_device) {
    RCLCPP_FATAL(
      node_->get_logger(), "CameraInit failed, no device matching given serial number: [%s].",
      serial_number_.c_str());
    return false;
  }

  if (!config_path_.empty()) {
    status = CameraReadParameterFromFile(handle_camera_, const_cast<char *>(config_path_.c_str()));
    if (status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_WARN(node_->get_logger(), "Read config file %s failed", config_path_.c_str());
      return false;
    }

    int trigger_mode = 0;
    CameraGetTriggerMode(handle_camera_, &trigger_mode);
    if (trigger_mode) {
      _soft_trigger = true;
    }

    CameraSetIspOutFormat(handle_camera_, CAMERA_MEDIA_TYPE_BGR8);

  } else {
    RCLCPP_FATAL(node_->get_logger(), "No config file specified, exiting.");
    return false;
  }

  status = CameraPlay(handle_camera_);
  if (status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_FATAL(node_->get_logger(), "CameraPlay failed.");
    return false;
  }

  opened_ = true;
  return true;
}

bool MindVisionCamera::close()
{
  if (CameraUnInit(handle_camera_) != CAMERA_STATUS_SUCCESS) {
    return false;
  }
  return true;
}

bool MindVisionCamera::isOpened()
{
  return opened_;
}

bool MindVisionCamera::getFrame(cv::Mat & image)
{
  if (!opened_) {
    RCLCPP_WARN(node_->get_logger(), "Camera is not open!");
    return false;
  }

  CameraSdkStatus status = CAMERA_STATUS_SUCCESS;
  tSdkFrameHead frame_info;

  if (_soft_trigger) {
    status = CameraSoftTrigger(handle_camera_);
    if (status != CAMERA_STATUS_SUCCESS) {
      RCLCPP_FATAL(node_->get_logger(), "CameraSoftTrigger failed.");
      return false;
    }
  }

  status = CameraGetImageBuffer(handle_camera_, &frame_info, &frame_buffer_, 1000);
  if (status != CAMERA_STATUS_SUCCESS) {
    CameraReleaseImageBuffer(handle_camera_, frame_buffer_);
    RCLCPP_FATAL(node_->get_logger(), "CameraGetImageBuffer failed.");
    return false;
  }

  image = cv::Mat(frame_info.iHeight, frame_info.iWidth, CV_8UC3);

  status = CameraImageProcess(handle_camera_, frame_buffer_, image.data, &frame_info);
  if (status != CAMERA_STATUS_SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "CameraImageProcess failed.");
    return false;
  }

  CameraReleaseImageBuffer(handle_camera_, frame_buffer_);

  return true;
}

}
