// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#include "solais_auto_aim/solais_armor_detector_node.hpp"
#include <vector>
#include <memory>
#include "opencv2/highgui.hpp"
#include "rclcpp/logging.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/node.hpp"
#include "solais_auto_aim/armor.hpp"
#include "solais_auto_aim/solais_pos_solver.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace solais_auto_aim
{
ArmorDetectorNode::ArmorDetectorNode(const rclcpp::NodeOptions & options)
{
  node_ = std::make_shared<rclcpp::Node>("armor_detector", options);
  RCLCPP_INFO(node_->get_logger(), "Starting Armor Detector Node...");

  armors_.reserve(100);  // Pre-allocate 100 armors
  camera_client_ = std::make_unique<solais_camera::CameraClient>(node_);
  auto camera_name = node_->declare_parameter("camera_name", "default_camera");
  camera_client_->setCameraName(camera_name);
  camera_client_->setCameraCallback(
    [this](const cv::Mat & img, const std_msgs::msg::Header & header) {
      imageCallback(img, header);
    });
  initDetector();

  cv::namedWindow("detected", cv::WINDOW_NORMAL);
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
  //  TODO - implement parameter for ignored classes
  auto ignored_classes = std::vector({Armor::ARMOR_INVALID});

  armor_detector_ = std::make_unique<ArmorDetector>(
    binary_thres, detect_color, light_params,
    armor_params, model_path, lenet_threshold, ignored_classes);
  
  using std::chrono_literals::operator""s;
  cam_info_timer_ = node_->create_wall_timer(5s, [this](){
    camera_client_->getCameraInfo([this](const auto camera_info){
      pos_solver_ = std::make_unique<PosSolver>(camera_info->k, camera_info->d);
    });
    cam_info_timer_->cancel();
  });
}

void ArmorDetectorNode::imageCallback(const cv::Mat & img, const std_msgs::msg::Header & header)
{
  // auto latency = (node_->now() - header.stamp).seconds() * 1000;
  // RCLCPP_INFO_STREAM(node_->get_logger(), "Comm latency: " << latency << "ms");
  armors_.clear();

  // Armor detection
  armor_detector_->detect(img, armors_);

  // Debug code for showing the result
  // TODO - implement debug mode
  cv::Mat img_copy = img.clone();
  armor_detector_->drawResults(img_copy, armors_);
  cv::imshow("detected", img_copy);
  cv::waitKey(1);

  if (pos_solver_.get() == nullptr) {
    // This may happen 
    RCLCPP_WARN(node_->get_logger(), "Camera info not received yet");
    return;
  }

  armors_msg_.header = header;
  armors_msg_.armors.clear();

  // Solve PnP
  solais_interfaces::msg::Armor armor_msg;
  for (const auto & armor : armors_)
  {
    cv::Mat rotation_vec;
    cv::Mat translation_vec;
    pos_solver_->solvePosition(armor, rotation_vec, translation_vec);
    armor_msg.type = ARMOR_TYPE_STR[static_cast<int>(armor.size_type)];
    armor_msg.number = static_cast<int>(armor.number);

    armor_msg.pose.position.x = translation_vec.at<double>(0);
    armor_msg.pose.position.y = translation_vec.at<double>(1);
    armor_msg.pose.position.z = translation_vec.at<double>(2);

    cv::Mat rotation_mat;
    cv::Rodrigues(rotation_vec, rotation_mat);
    tf2::Matrix3x3 tf2_rotation_mat(
      rotation_mat.at<double>(0, 0), rotation_mat.at<double>(0, 1),
      rotation_mat.at<double>(0, 2), rotation_mat.at<double>(1, 0),
      rotation_mat.at<double>(1, 1), rotation_mat.at<double>(1, 2),
      rotation_mat.at<double>(2, 0), rotation_mat.at<double>(2, 1),
      rotation_mat.at<double>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rotation_mat.getRotation(tf2_quaternion);
    armor_msg.pose.orientation = tf2::toMsg(tf2_quaternion);

    armor_msg.distance_to_image_center = pos_solver_->calculateDistance2Center(armor.center);

    armors_msg_.armors.push_back(armor_msg);
  }

  armors_pub_->publish(armors_msg_);
}

}  // namespace solais_auto_aim


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_auto_aim::ArmorDetectorNode)
