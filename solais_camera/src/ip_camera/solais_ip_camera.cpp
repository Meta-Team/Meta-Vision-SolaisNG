// Copyright 2023 Meta-Team
#include "solais_camera/solais_ip_camera.hpp"
#include <string>

namespace solais_camera
{
IPCamera::IPCamera(const std::string & rtsp_path, std::shared_ptr<CamParam> params)
: MetaCamera(params), _rtsp_path(rtsp_path)
{
}

IPCamera::~IPCamera()
{
  if (_opened) {
    _cap.release();
  }
}

bool IPCamera::open()
{
  if (_opened) {
    return true;
  }
  // open device
  if (!_cap.open(_rtsp_path)) {
    return false;
  }
  // set camera
  _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  _cap.set(cv::CAP_PROP_FRAME_WIDTH, _params->width);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _params->height);
  _opened = true;
  return true;
}

bool IPCamera::close()
{
  if (_opened) {
    _cap.release();
    _opened = false;
  }
  return true;
}

bool IPCamera::isOpened()
{
  return _opened;
}

bool IPCamera::getFrame(cv::Mat & image)
{
  if (!_opened) {
    return false;
  }
  if (!_cap.read(image)) {
    return false;
  }
  return true;
}

}  // namespace solais_camera
