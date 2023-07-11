// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_

#include <cmath>
#include <string>
#include <vector>
#include <memory>

#include "opencv2/opencv.hpp"
#include "solais_auto_aim/armor.hpp"
#include "solais_auto_aim/solais_armor_lenet.hpp"
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
    const ArmorParams & armor, const std::string & model_path, const double conf_threshold,
    const std::vector<Armor::ArmorNumberType> & ignored_classes = {});

  void detect(const cv::Mat & input, std::vector<Armor> & armors);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img, std::vector<Armor> armors) const;

  // Debug msgs
  // cv::Mat binary_img;
  // auto_aim_interfaces::msg::DebugLights debug_lights;
  // auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  // Helper functions
  void preprocessImage(cv::Mat & img) const;
  void findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  void matchLights(std::vector<Armor> & armors);
  bool isLight(const Light & possible_light) const;
  bool containLight(
    const Light & light_1, const Light & light_2,
    const std::vector<Light> & lights) const;
  Armor::ArmorSizeType isArmor(const Light & light_1, const Light & light_2) const;

  // Parameters
  int binary_thres_;
  int detect_color_;
  LightParams light_params_;
  ArmorParams armor_params_;

  std::vector<Light> buf_lights_;
  std::unique_ptr<ArmorLeNet> lenet_;
};
}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_HPP_
