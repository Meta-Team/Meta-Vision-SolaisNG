// Copyright 2023 Meta-Team
#include "solais_camera/solais_usb_camera.hpp"
#include <string>

namespace solais_camera
{
USBCamera::USBCamera(const std::string & device_path)
: _device_path(device_path)
{
  _params.fps = 30;
  _params.width = 640;
  _params.height = 480;
}

USBCamera::~USBCamera()
{
  if (_is_opened) {
    _cap.release();
  }
}

bool USBCamera::open()
{
  if (_is_opened) {
    return true;
  }
  // open device
  if (!_cap.open(_device_path)) {
    return false;
  }
  // set camera
  _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  _cap.set(cv::CAP_PROP_FRAME_WIDTH, _params.width);
  _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _params.height);
  _is_opened = true;
  return true;
}

bool USBCamera::close()
{
  if (_is_opened) {
    _cap.release();
    _is_opened = false;
  }
  return true;
}

bool USBCamera::isOpened()
{
  return _is_opened;
}

bool USBCamera::getFrame(cv::Mat & image)
{
  if (!_is_opened) {
    return false;
  }
  if (!_cap.read(image)) {
    return false;
  }
  return true;
}

void USBCamera::setParameter(const CamParam & params)
{
  _params = params;
}

void USBCamera::getParameter(CamParam & params)
{
  params = _params;
}

}  // namespace solais_camera
