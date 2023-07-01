// Copyright 2023 Meta-Team
#include "solais_camera/solais_ip_camera.hpp"
#include <string>

namespace solais_camera
{
IPCamera::IPCamera(const std::string & rtsp_path)
: _rtsp_path(rtsp_path)
{
  _params.fps = 60;
  _params.width = 640;
  _params.height = 480;
}

IPCamera::~IPCamera()
{
  if (_is_opened) {
    _cap.release();
  }
}

bool IPCamera::open()
{
  if (_is_opened) {
    return true;
  }
  // open device
  if (!_cap.open(_rtsp_path)) {
    return false;
  }
  // set camera
  _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  _cap.set(cv::CAP_PROP_FRAME_WIDTH, _params.width);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _params.height);
  _is_opened = true;
  return true;
}

bool IPCamera::close()
{
  if (_is_opened) {
    _cap.release();
    _is_opened = false;
  }
  return true;
}

bool IPCamera::isOpened()
{
  return _is_opened;
}

bool IPCamera::getFrame(cv::Mat & image)
{
  if (!_is_opened) {
    return false;
  }
  if (!_cap.read(image)) {
    return false;
  }
  return true;
}

void IPCamera::setParameter(const CamParam & params)
{
  _params = params;
}

void IPCamera::getParameter(CamParam & params)
{
  params = _params;
}

}  // namespace solais_camera
