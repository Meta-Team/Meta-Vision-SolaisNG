// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__ARMOR_HPP_
#define SOLAIS_AUTO_AIM__ARMOR_HPP_

#include <algorithm>
#include <string>

#include "opencv2/core.hpp"

namespace solais_auto_aim {

const int RED = 0;
const int BLUE = 1;

const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};

struct Light : public cv::RotatedRect
{
  Light() = default;
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)
  {
    cv::Point2f p[4];
    box.points(p);
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    top = (p[0] + p[1]) / 2;
    bottom = (p[2] + p[3]) / 2;

    length = cv::norm(top - bottom);
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
  }

  int color;
  cv::Point2f top, bottom;
  double length;
  double width;
  float tilt_angle;
};

struct Armor
{
  enum class ArmorSizeType { SMALL, LARGE, INVALID };
  enum ArmorNumberType
  {
    ARMOR_1 = 0,
    ARMOR_2 = 1,
    ARMOR_3 = 2,
    ARMOR_4 = 3,
    ARMOR_5 = 4,
    ARMOR_OUTPOST = 5,
    ARMOR_GUARD = 6,
    ARMOR_BASE = 7,
    ARMOR_INVALID = 8
  };

  Armor() = default;
  Armor(const Light & l1, const Light & l2)
  {
    if (l1.center.x < l2.center.x) {
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;
  }

  // Light pairs result
  Light left_light, right_light;
  cv::Point2f center;
  ArmorSizeType size_type;

  // Number detection result
  // cv::Mat number_img;
  ArmorNumberType number;
  float confidence;
  std::string classification_result;
};

}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__ARMOR_HPP_
