// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#include "solais_auto_aim/solais_armor_lenet.hpp"
#include <iomanip>

namespace solais_auto_aim
{
ArmorLeNet::ArmorLeNet(
  const std::string & model_path, const double conf_threshold,
  const std::vector<Armor::ArmorNumberType> & ignored_classes)
: conf_threshold_(conf_threshold), ignored_classes_(ignored_classes)
{
  lenet_ = cv::dnn::readNetFromONNX(model_path);
}

void ArmorLeNet::identifyNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;
  const int small_armor_width = 32;
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);

  for (auto & armor : armors) {
    // Warp perspective transform
    std::array<cv::Point2f, 4> lights_vertices = {
      armor.left_light.bottom,
      armor.left_light.top,
      armor.right_light.top,
      armor.right_light.bottom
    };

    const int top_light_y = (warp_height - light_length) / 2 - 1;
    const int bottom_light_y = top_light_y + light_length;
    const int warp_width = armor.size_type ==
      Armor::ArmorSizeType::SMALL ? small_armor_width : large_armor_width;

    std::array<cv::Point2f, 4> target_vertices = {
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_BGR2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat image = number_image.clone();

    // Normalize
    image = image / 255.0;

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);

    // Set the input blob for the neural network
    lenet_.setInput(blob);
    // Forward pass the image blob through the model
    cv::Mat outputs = lenet_.forward();

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);
    auto sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    armor.confidence = static_cast<float>(confidence);
    armor.number = static_cast<Armor::ArmorNumberType>(label_id);

    std::stringstream result_ss;
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1) <<
      armor.confidence * 100.0 << "%";
    armor.classification_result = result_ss.str();
  }

  std::erase_if(
    armors, [this](
      const Armor & armor) {
      // Filter confidnece threshold
      if (armor.confidence < conf_threshold_) {
        return true;
      }

      // Filter ignored (invalid) classes
      for (const auto & ignore_class : ignored_classes_) {
        if (armor.number == ignore_class) {
          return true;
        }
      }

      // Filter possible mismatched types
      bool mismatch_armor_type = false;
      if (armor.size_type == Armor::ArmorSizeType::LARGE) {
        mismatch_armor_type =
        (armor.number == Armor::ARMOR_OUTPOST) || (armor.number == Armor::ARMOR_2) ||
        (armor.number == Armor::ARMOR_GUARD);
      } else if (armor.size_type == Armor::ArmorSizeType::SMALL) {
        // mismatch_armor_type =
        // (armor.number == Armor::ARMOR_1) || (armor.number == Armor::ARMOR_BASE);
        mismatch_armor_type = (armor.number == Armor::ARMOR_1);
      }
      return mismatch_armor_type;
    });
}
}  // namespace solais_auto_aim
