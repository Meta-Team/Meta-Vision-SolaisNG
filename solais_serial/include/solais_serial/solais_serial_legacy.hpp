// Copyright 2023 Meta-Team
#ifndef SOLAIS_SERIAL
#define SOLAIS_SERIAL

#include <bits/stdint-uintn.h>
#include <rclcpp/subscription.hpp>
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Dense>

// #include "tf2_ros/buffer.h"

namespace solais_serial
{
class SerialNodeLegacy
{
public:
  explicit SerialNodeLegacy(const rclcpp::NodeOptions & options);
  ~SerialNodeLegacy();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  void declareParameters();

  void receivePackage();

  void sendPackage(const auto_aim_interfaces::msg::Target::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<IoContext> io_context_;
  std::string device_name_;
  long baud_rate_;
  std::shared_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr armors_sub_;
  double timestamp_offset_ = 0;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Debug
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  visualization_msgs::msg::Marker aiming_point_;


  // For projectile prediction
  float cur_pitch_ = 0.;
  float cur_yaw_ = 0.;
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_pitch_;
  double offset_yaw_;
  double offset_time_;
  double shoot_speed_;
  double friction_{0.001};

  std::string solver_type_;
  std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver_;
};


struct __attribute__((packed, aligned(1))) SentPackage
{
  uint8_t sof = 0x5A;  // Start of frame
  float yaw;
  float pitch;
  uint8_t crc8;
};

struct __attribute__((packed, aligned(1))) ReceivedPackage
{
  uint8_t sof = 0x5A;  // Start of frame
  float yaw;
  float pitch;
  uint16_t crc16;
};

enum class CommandID : uint8_t
{
  VISION_CONTROL_CMD_ID = 0,
  CMD_ID_COUNT
};

std::vector<uint8_t> Pak2Vector(const SentPackage & data)
{
  std::vector<uint8_t> package(sizeof(SentPackage));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(SentPackage), package.begin());
  return package;
}

ReceivedPackage Vector2Pak(const std::vector<uint8_t> & data)
{
  ReceivedPackage package;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&package));
  return package;
}

}  // namespace solais_serial


#endif
