// Copyright 2023 Meta-Team
#ifndef SOLAIS_CAMERA__SOLAIS_IP_CAMERA_HPP_
#define SOLAIS_CAMERA__SOLAIS_IP_CAMERA_HPP_
#include <string>
#include <opencv2/opencv.hpp>

#include "solais_camera/solais_meta_camera.hpp"


namespace solais_camera
{
class IPCamera : public MetaCamera
{
public:
  explicit IPCamera(const std::string & rtsp_path, std::shared_ptr<CamParam> param);
  ~IPCamera();

  bool open() override;
  bool close() override;
  bool isOpened() override;
  bool getFrame(cv::Mat & image) override;

private:
  std::string _rtsp_path;
  cv::VideoCapture _cap;
  bool _opened{false};
};
}  // namespace solais_camera

#endif  // SOLAIS_CAMERA__SOLAIS_IP_CAMERA_HPP_
