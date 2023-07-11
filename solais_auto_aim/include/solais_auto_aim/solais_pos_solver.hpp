// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_POS_SOLVER_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_POS_SOLVER_HPP_
#include <array>

#include "opencv2/core.hpp"
#include "solais_auto_aim/armor.hpp"

namespace solais_auto_aim {
class PosSolver
{
public:
  PosSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);
  
  void solvePosition(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const;

  float calculateDistance2Center(const cv::Point2f & image_point);

private:
  cv::Mat camera_matrix_;
  cv::Mat distortion_coefficients_;

  // Physical parameters of armor, unit in mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float ARMOR_LIGHT_LENGTH = 55;

  // Four vertices of armor in 3d
  const std::array<cv::Point3f, 4> small_armor_points_ = {
    cv::Point3f(0, SMALL_ARMOR_WIDTH / 2 / 1000, -ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, SMALL_ARMOR_WIDTH / 2 / 1000, ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, -SMALL_ARMOR_WIDTH / 2 / 1000, ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, -SMALL_ARMOR_WIDTH / 2 / 1000, -ARMOR_LIGHT_LENGTH / 2 / 1000)
  };
  const std::array<cv::Point3f, 4> large_armor_points_ = {
    cv::Point3f(0, LARGE_ARMOR_WIDTH / 2 / 1000, -ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, LARGE_ARMOR_WIDTH / 2 / 1000, ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, -LARGE_ARMOR_WIDTH / 2 / 1000, ARMOR_LIGHT_LENGTH / 2 / 1000),
    cv::Point3f(0, -LARGE_ARMOR_WIDTH / 2 / 1000, -ARMOR_LIGHT_LENGTH / 2 / 1000)
  };
};
}

#endif  // SOLAIS_AUTO_AIM__SOLAIS_POS_SOLVER_HPP_
