// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#include "solais_auto_aim/solais_pos_solver.hpp"
#include <iostream>
#include "opencv2/calib3d.hpp"

namespace solais_auto_aim
{
PosSolver::PosSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & distortion_coefficients)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  distortion_coefficients_(cv::Mat(1, 5, CV_64F,
    const_cast<double *>(distortion_coefficients.data())).clone())
  // .clone() is necessary here since cv::Mat won't copy data, it just points to the data, and the data will be destroyed after the function call
  // TODO - refactor this code to avoid const_cast potential UB
{
}

void PosSolver::solvePosition(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec) const
{
  std::array<cv::Point2f, 4> image_armor_points = {
    armor.left_light.bottom, armor.left_light.top,
    armor.right_light.top, armor.right_light.bottom
  };
  cv::solvePnP(
    (armor.size_type == Armor::ArmorSizeType::SMALL) ? small_armor_points_ : large_armor_points_,
    image_armor_points, camera_matrix_, distortion_coefficients_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
}

float PosSolver::calculateDistance2Center(const cv::Point2f & image_point)
{
  return cv::norm(
    image_point -
    cv::Point2f(camera_matrix_.at<float>(0, 2), camera_matrix_.at<float>(1, 2)));
}

}  // namespace solais_auto_aim
