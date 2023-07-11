#ifndef __SOLAIS_MINDVISION_CAMERA_HPP__
#define __SOLAIS_MINDVISION_CAMERA_HPP__
#include <memory>
#include <string>

#include <opencv2/opencv.hpp>
#include "CameraDefine.h"
#include "solais_camera/solais_meta_camera.hpp"
#include "CameraApi.h"
#include "CameraStatus.h"

namespace solais_camera
{
class MindVisionCamera : public MetaCamera
{
public:
  explicit MindVisionCamera(std::shared_ptr<rclcpp::Node> node, const std::string & config_path, std::shared_ptr<CamParam> params, const std::string & serial_number);
  ~MindVisionCamera();

  bool open() override;
  bool close() override;
  bool isOpened() override;
  bool getFrame(cv::Mat & image) override;

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::string config_path_;
  std::string serial_number_;
  cv::VideoCapture cap_;
  bool opened_{false};
  CameraHandle handle_camera_;
  tSdkCameraDevInfo camera_list_[16];
  tSdkCameraCapbility camera_info_;
  bool _soft_trigger{false};

  BYTE *frame_buffer_;
};
}  // namespace solais_camera

#endif  // __SOLAIS_MINDVISION_CAMERA_HPP__