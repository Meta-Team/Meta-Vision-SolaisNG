// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include "solais_auto_aim/solais_armor_detector.hpp"
#include "solais_auto_aim/solais_aiming_solver.hpp"
#include "solais_camera/solais_camera_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "solais_interfaces/msg/armors.hpp"

namespace solais_auto_aim
{

class ArmorDetectorNode
{
public:
  explicit ArmorDetectorNode(const rclcpp::NodeOptions & options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const
  {
    return node_->get_node_base_interface();
  }

private:
  inline void initDetector();

  void imageCallback(const cv::Mat & img, const rclcpp::Time & stamp);

  rclcpp::Node::SharedPtr node_;

  std::shared_ptr<ArmorDetector> armor_detector_;

  // Receive image from solais_camera_client
  std::shared_ptr<solais_camera::CameraClient> camera_client_;

  // Publish armor message
  solais_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<solais_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Camera info part
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  cv::Point2f cam_center_;
  std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
  std::unique_ptr<AimingSolver> aiming_solver_;
};

}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_
