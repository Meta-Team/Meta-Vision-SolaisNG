// Copyright 2023 Meta-Team
#ifndef SOLAIS_SERIAL
#define SOLAIS_SERIAL

#include <bits/stdint-uintn.h>
#include <rclcpp/subscription.hpp>
#include "rclcpp/rclcpp.hpp"
#include "serial_driver/serial_driver.hpp"
#include "serial_driver/serial_port.hpp"
#include "solais_interfaces/msg/armors.hpp"

namespace solais_serial
{
class SerialNodeLegacy
{
public:
  SerialNodeLegacy(const rclcpp::NodeOptions & options);
  ~SerialNodeLegacy();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

private:
  void declareParameters();

  void armorCallback(const solais_interfaces::msg::Armors::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  std::unique_ptr<IoContext> io_context_;
  std::string device_name_;
  long baud_rate_;
  std::shared_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
  rclcpp::Subscription<solais_interfaces::msg::Armors>::SharedPtr armors_sub_;
};

struct __attribute__((packed, aligned(1))) VisionCommand
{

  enum VisionFlag : uint8_t
  {
    NONE = 0,
    DETECTED = 1,
    TOP_KILLER_TRIGGERED = 2,
  };

  uint8_t flag;
  uint16_t frameTime;               // [0.1ms]
  int16_t yawDelta;                 // yaw relative angle [deg] * 100
  int16_t pitchDelta;               // pitch relative angle [deg] * 100
  int16_t distance;                 // [mm]
  int16_t avgLightAngle;            // [deg] * 100
  int16_t imageX;                   // pixel
  int16_t imageY;                   // pixel
  int16_t remainingTimeToTarget;    // [ms]
  int16_t period;                   // [ms]
};

struct __attribute__((packed, aligned(1))) Package
{
  uint8_t sof;
  uint8_t cmdID;
  union {
    VisionCommand command;
  };
  uint8_t crc8;
};

enum class CommandID : uint8_t
{
  VISION_CONTROL_CMD_ID = 0,
  CMD_ID_COUNT
};

std::vector<uint8_t> toVector(const Package & data)
{
  std::vector<uint8_t> package(sizeof(Package));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data),
    reinterpret_cast<const uint8_t *>(&data) + sizeof(Package), package.begin());
  return package;
}

}  // namespace solais_serial


#endif
