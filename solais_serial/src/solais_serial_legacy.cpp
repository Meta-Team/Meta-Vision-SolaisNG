#include "solais_serial/solais_serial_legacy.hpp"
#include <bits/stdint-uintn.h>
#include <vector>
#include "rclcpp/qos.hpp"
#include "serial_driver/serial_port.hpp"
#include "solais_serial/crc.h"

namespace solais_serial
{
SerialNodeLegacy::SerialNodeLegacy(const rclcpp::NodeOptions & options)
{
  io_context_ = std::make_unique<IoContext>(2);
  serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
  node_ = std::make_shared<rclcpp::Node>("solais_serial_legacy", options);

  declareParameters();

  //  Open serial port
  serial_driver_->init_port(device_name_, *device_config_);
  if (!serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
  }

  armors_sub_ = node_->create_subscription<solais_interfaces::msg::Armors>(
    "/detector/armors", rclcpp::SensorDataQoS(), [this](const solais_interfaces::msg::Armors::SharedPtr msg) {
      armorCallback(msg);
    });
}

SerialNodeLegacy::~SerialNodeLegacy()
{
  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (io_context_) {
    io_context_->waitForExit();
  }
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SerialNodeLegacy::get_node_base_interface()
const
{
  return node_->get_node_base_interface();
}

void SerialNodeLegacy::declareParameters()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  device_name_ = node_->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  baud_rate_ = node_->declare_parameter<long>("baud_rate", 115200);

  FlowControl flow_control;
  const std::string flow_control_str = node_->declare_parameter<std::string>("flow_control", "");
  if (flow_control_str == "hardware") {
    flow_control = FlowControl::HARDWARE;
  } else if (flow_control_str == "software") {
    flow_control = FlowControl::SOFTWARE;
  } else {
    flow_control = FlowControl::NONE;
  }

  Parity parity;
  const std::string parity_str = node_->declare_parameter<std::string>("parity", "");
  if (parity_str == "even") {
    parity = Parity::EVEN;
  } else if (parity_str == "odd") {
    parity = Parity::ODD;
  } else {
    parity = Parity::NONE;
  }

  StopBits stop_bits;
  const std::string stop_bits_str = node_->declare_parameter<std::string>("stop_bits", "");
  if (stop_bits_str == "1") {
    stop_bits = StopBits::ONE;
  } else if (stop_bits_str == "2") {
    stop_bits = StopBits::TWO;
  } else {
    stop_bits = StopBits::ONE;
  }

  device_config_ = std::make_shared<drivers::serial_driver::SerialPortConfig>( baud_rate_, flow_control, parity, stop_bits);
}

void SerialNodeLegacy::armorCallback(const solais_interfaces::msg::Armors::SharedPtr msg)
{
  try {
    Package package;

    package.sof = 0xA5;
    package.cmdID = static_cast<uint8_t>(CommandID::VISION_CONTROL_CMD_ID);
    package.command.flag = 0;
    package.command.flag |= VisionCommand::DETECTED;
    package.command.frameTime = msg->header.stamp.nanosec / 100000;
    package.command.yawDelta = 0;
    package.command.pitchDelta = 0;
    package.command.distance = 0;
    package.command.avgLightAngle = 0;
    package.command.imageX = 0;
    package.command.imageY = 0;
    package.command.remainingTimeToTarget = 0;
    package.command.period = 0;
    appendCRC8CheckSum((uint8_t *) (&package), sizeof(uint8_t) * 2 + sizeof(VisionCommand) + sizeof(uint8_t));

    std::vector<uint8_t> package_vec = toVector(package);
    serial_driver_->port()->send(package_vec);

    auto latency = (node_->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO(node_->get_logger(), "Total latency: %f ms", latency);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error sending data: %s", e.what());
  }
}

}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_serial::SerialNodeLegacy)
