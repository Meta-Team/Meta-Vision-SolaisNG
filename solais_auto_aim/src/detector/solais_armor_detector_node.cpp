// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#include "solais_auto_aim/solais_armor_detector_node.hpp"
#include <vector>
#include <memory>
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/node.hpp"

namespace solais_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("armor_detector", options);
  camera_client_ = std::make_unique<solais_camera::CameraClient>(node_);

  RCLCPP_INFO(node_->get_logger(), "Starting Armor Detector Node...");

  initDetector();

  cv::namedWindow("image", cv::WINDOW_NORMAL);
  cv::namedWindow("detected", cv::WINDOW_NORMAL);

  auto camera_name = node_->declare_parameter("camera_name", "default_camera");
  camera_client_->setCameraName(camera_name);
  camera_client_->setCameraCallback(
    [this](const cv::Mat & img, const rclcpp::Time & stamp) {
      imageCallback(img, stamp);
    });
  camera_client_->connect();

  armors_pub_ = node_->create_publisher<solais_interfaces::msg::Armors>(
    "/detector/armors",
    rclcpp::SensorDataQoS());
}

inline void ArmorDetectorNode::initDetector()
{
  // Set threshold
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.integer_range.resize(1);
  param_desc.integer_range[0].step = 1;
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 255;
  auto binary_thres = (node_->declare_parameter("binary_thres", 160, param_desc));

  // Set enemy color
  param_desc.description = "0-RED, 1-BLUE";
  param_desc.integer_range[0].from_value = 0;
  param_desc.integer_range[0].to_value = 1;
  auto detect_color = node_->declare_parameter("detect_color", RED, param_desc);

  // Set armor light params
  ArmorDetector::LightParams light_params = {
    .min_ratio = node_->declare_parameter("light.min_ratio", 0.1),
    .max_ratio = node_->declare_parameter("light.max_ratio", 0.4),
    .max_angle = node_->declare_parameter("light.max_angle", 40.0)};

  // Set armor params
  ArmorDetector::ArmorParams armor_params = {
    .min_light_ratio = node_->declare_parameter("armor.min_light_ratio", 0.7),
    .min_small_center_distance = node_->declare_parameter("armor.min_small_center_distance", 0.8),
    .max_small_center_distance = node_->declare_parameter("armor.max_small_center_distance", 3.2),
    .min_large_center_distance = node_->declare_parameter("armor.min_large_center_distance", 3.2),
    .max_large_center_distance = node_->declare_parameter("armor.max_large_center_distance", 5.5),
    .max_angle = node_->declare_parameter("armor.max_angle", 35.0)};

  // Set LeNet params
  auto pkg_path = ament_index_cpp::get_package_share_directory("solais_auto_aim");
  RCLCPP_INFO_STREAM(node_->get_logger(), "Package path: " << pkg_path);
  auto model_path = pkg_path + "/models/chenjunn_mlp.onnx";
  double lenet_threshold = node_->declare_parameter("classifier_threshold", 0.7);
  //  @todo: implement parameter for ignored classes
  auto ignored_classes = std::vector({Armor::ARMOR_INVALID});

  armor_detector_ = std::make_unique<ArmorDetector>(
    binary_thres, detect_color, light_params,
    armor_params, model_path, lenet_threshold, ignored_classes);
}

void ArmorDetectorNode::imageCallback(const cv::Mat & img, const rclcpp::Time & stamp)
{
  // Debug code for displaying image
  cv::imshow("image", img);
  cv::waitKey(1);

  // Armor detection
  auto armors = armor_detector_->detect(img);
  auto final_time = node_->now();
  auto latency = (final_time - stamp).seconds() * 1000;
  RCLCPP_INFO_STREAM(node_->get_logger(), "Latency: " << latency << "ms");

  cv::Mat img_copy = img.clone();
  armor_detector_->drawResults(img_copy);
  cv::imshow("detected", img_copy);
}

}  // namespace solais_auto_aim


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_auto_aim::ArmorDetectorNode)
