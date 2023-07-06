// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_

#include <cmath>
#include <string>
#include <vector>

#include "opencv2/opencv.hpp"
#include "solais_auto_aim/armor.hpp"
#include "solais_interfaces/msg/armor.hpp"
#include "rclcpp/rclcpp.hpp"


namespace solais_auto_aim
{

class ArmorDetector
{
public:
  struct LightParams
  {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
  };

  struct ArmorParams
  {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  ArmorDetector(
    const int & bin_threshold, const int & color, const LightParams & light,
    const ArmorParams & armor);

  std::vector<Armor> detect(const cv::Mat & input);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img) const;

  // Parameters
  int binary_thres_;
  int detect_color_;
  LightParams light_params_;
  ArmorParams armor_params_;

  // std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  // cv::Mat binary_img;
  // auto_aim_interfaces::msg::DebugLights debug_lights;
  // auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  // Helper functions
  void preprocessImage(cv::Mat & img) const;
  void findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  void matchLights();
  bool isLight(const Light & possible_light) const;
  bool containLight(
    const Light & light_1, const Light & light_2,
    const std::vector<Light> & lights) const;
  ArmorType isArmor(const Light & light_1, const Light & light_2) const;

  std::vector<Light> buf_lights_;
  std::vector<Armor> buf_armors_;
};
}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_
