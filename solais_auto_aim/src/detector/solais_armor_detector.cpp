// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#include "solais_auto_aim/solais_armor_detector.hpp"
#include "magic_enum.hpp"

namespace solais_auto_aim
{
ArmorDetector::ArmorDetector(
  const int & bin_threshold, const int & color, const LightParams & light,
  const ArmorParams & armor, const std::string & model_path, const double conf_threshold,
  const std::vector<Armor::ArmorNumberType> & ignored_classes)
: binary_thres_(bin_threshold), detect_color_(color), light_params_(light), armor_params_(armor)
{
  lenet_ = std::make_unique<ArmorLeNet>(model_path, conf_threshold, ignored_classes);
}

std::vector<Armor> ArmorDetector::detect(const cv::Mat & input)
{
  // Detect possible armors
  cv::Mat copy = input.clone();
  preprocessImage(copy);
  findLights(input, copy);
  matchLights();

  // Classify armors
  if (!buf_armors_.empty()) {
    lenet_->identifyNumbers(input, buf_armors_);
  }

  return buf_armors_;
}

void ArmorDetector::preprocessImage(cv::Mat & img) const
{
  cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  cv::threshold(img, img, binary_thres_, 255, cv::THRESH_BINARY);
}

void ArmorDetector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)
{
  buf_lights_.clear();
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // this->debug_lights.data.clear();

  for (const auto & contour : contours) {
    if (contour.size() < 5) {continue;}

    auto r_rect = cv::minAreaRect(contour);
    auto light = Light(r_rect);

    if (isLight(light)) {
      auto rect = light.boundingRect();
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows)
      {
        int sum_r = 0;
        int sum_b = 0;
        auto roi = rbg_img(rect);
        // Iterate through the ROI
        for (int i = 0; i < roi.rows; i++) {
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(
                contour,
                cv::Point2f(static_cast<float>(j + rect.x), static_cast<float>(i + rect.y)),
                false) >= 0)
            {
              // if point is inside contour
              sum_r += roi.at<cv::Vec3b>(i, j)[2];
              sum_b += roi.at<cv::Vec3b>(i, j)[0];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE;
        buf_lights_.emplace_back(light);
      }
    }
  }
}

bool ArmorDetector::isLight(const Light & light) const
{
  // The ratio of light (short side / long side)
  double ratio = light.width / light.length;
  bool ratio_ok = light_params_.min_ratio < ratio && ratio < light_params_.max_ratio;

  bool angle_ok = light.tilt_angle < light_params_.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  // auto_aim_interfaces::msg::DebugLight light_data;
  // light_data.center_x = light.center.x;
  // light_data.ratio = ratio;
  // light_data.angle = light.tilt_angle;
  // light_data.is_light = is_light;
  // this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

void ArmorDetector::matchLights()
{
  buf_armors_.clear();
  // this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = buf_lights_.begin(); light_1 != buf_lights_.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != buf_lights_.end(); light_2++) {
      if (light_1->color != detect_color_ || light_2->color != detect_color_) {continue;}

      if (containLight(*light_1, *light_2, buf_lights_)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != Armor::ArmorSizeType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.size_type = type;
        buf_armors_.emplace_back(armor);
      }
    }
  }
}

bool ArmorDetector::containLight(
  const Light & light_1, const Light & light_2,
  const std::vector<Light> & lights) const
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);

  for (const auto & test_light : lights) {
    if (test_light.center == light_1.center || test_light.center == light_2.center) {continue;}

    if (
      bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
      bounding_rect.contains(test_light.center))
    {
      return true;
    }
  }

  return false;
}

Armor::ArmorSizeType ArmorDetector::isArmor(const Light & light_1, const Light & light_2) const
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length :
    light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > armor_params_.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (armor_params_.min_small_center_distance <= center_distance &&
    center_distance < armor_params_.max_small_center_distance) ||
    (armor_params_.min_large_center_distance <= center_distance &&
    center_distance < armor_params_.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < armor_params_.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  Armor::ArmorSizeType type;
  if (is_armor) {
    type = center_distance >
      armor_params_.min_large_center_distance ? Armor::ArmorSizeType::LARGE : Armor::ArmorSizeType::
      SMALL;
  } else {
    type = Armor::ArmorSizeType::INVALID;
  }

  // Fill in debug information
  // auto_aim_interfaces::msg::DebugArmor armor_data;
  // armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  // armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  // armor_data.light_ratio = light_length_ratio;
  // armor_data.center_distance = center_distance;
  // armor_data.angle = angle;
  // this->debug_armors.data.emplace_back(armor_data);

  return type;
}

void ArmorDetector::drawResults(cv::Mat & img) const
{
  // Draw Lights
  for (const auto & light : buf_lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1);
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255);
    cv::line(img, light.top, light.bottom, line_color, 1);
  }

  // Draw armors
  for (const auto & armor : buf_armors_) {
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
    cv::putText(img, cv::String(magic_enum::enum_name(armor.number)),
      armor.center, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : buf_armors_) {
    std::string classification_result_str = std::string(armor.classification_result);
    cv::putText(
      img, classification_result_str, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace solais_auto_aim
