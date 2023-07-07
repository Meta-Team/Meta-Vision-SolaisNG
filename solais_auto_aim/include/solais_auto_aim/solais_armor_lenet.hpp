// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_ARMOR_LENET_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_ARMOR_LENET_HPP_
#include <string>
#include <vector>

#include "solais_auto_aim/armor.hpp"
#include "opencv2/core.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/imgproc.hpp"

namespace solais_auto_aim
{
class ArmorLeNet
{
public:
  ArmorLeNet(
    const std::string & model_path, const double conf_threshold,
    const std::vector<Armor::ArmorNumberType> & ignored_classes = {});
  void identifyNumbers(const cv::Mat & src, std::vector<Armor> & armors);

private:
  double conf_threshold_;
  std::vector<Armor::ArmorNumberType> ignored_classes_;
  cv::dnn::Net lenet_;
};
}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__SOLAIS_ARMOR_LENET_HPP_
