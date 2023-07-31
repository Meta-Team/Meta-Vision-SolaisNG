// Copyright 2023 Meta-Team
// Licensed under the MIT License.
#ifndef SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_
#define SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_

#include <memory>
#include <rclcpp/node_interfaces/node_base_interface.hpp>
#include <rclcpp/timer.hpp>
#include "solais_auto_aim/solais_armor_detector.hpp"
#include "solais_auto_aim/solais_pos_solver.hpp"
#include "solais_camera/solais_camera_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "solais_interfaces/msg/armors.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

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

  inline void createRvizMarker();

  void imageCallback(const cv::Mat & img, const std_msgs::msg::Header & header);

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<ArmorDetector> armor_detector_;
  std::vector<Armor> armors_;  // Pre-allocated armors

  // Receive image from solais_camera_client
  std::unique_ptr<solais_camera::CameraClient> camera_client_;

  // Publish armor message
  solais_interfaces::msg::Armors armors_msg_;
  rclcpp::Publisher<solais_interfaces::msg::Armors>::SharedPtr armors_pub_;

  // Camera info
  rclcpp::TimerBase::SharedPtr cam_info_timer_;
  cv::Point2f cam_center_;

  // Rviz Marker
  visualization_msgs::msg::Marker armor_marker_msg_;
  visualization_msgs::msg::Marker class_marker_msg_;
  visualization_msgs::msg::MarkerArray marker_array_msg_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Position Solver
  std::unique_ptr<PosSolver> pos_solver_;

  // Debug Mode
  bool debug_mode_{false};
  image_transport::Publisher debug_img_pub_;
};

}  // namespace solais_auto_aim

#endif  // SOLAIS_AUTO_AIM__SOLAIS_ARMOR_DETECTOR_NODE_HPP_
