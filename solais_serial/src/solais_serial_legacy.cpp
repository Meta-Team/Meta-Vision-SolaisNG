#include "solais_serial/solais_serial_legacy.hpp"
#include <bits/stdint-uintn.h>
#include <rclcpp/logging.hpp>
#include <vector>
#include "rclcpp/qos.hpp"
#include "serial_driver/serial_port.hpp"
#include "solais_serial/crc.h"
#include <limits>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "rmoss_projectile_motion/gaf_projectile_solver.hpp"

namespace solais_serial
{
SerialNodeLegacy::SerialNodeLegacy(const rclcpp::NodeOptions & options)
{
  io_context_ = std::make_unique<IoContext>(2);
  serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
  node_ = std::make_shared<rclcpp::Node>("solais_serial_legacy", options);

  declareParameters();

  timestamp_offset_ = node_->declare_parameter("timestamp_offset", 0.0);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);

  marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

  //  Open serial port
  serial_driver_->init_port(device_name_, *device_config_);
  if (!serial_driver_->port()->is_open()) {
    serial_driver_->port()->open();
    receive_thread_ = std::thread(&SerialNodeLegacy::receivePackage, this);
  }

  RCLCPP_INFO(node_->get_logger(), "Projectile motion solver type: %s", solver_type_.c_str());
  if (solver_type_ == "gravity") {
    solver_ = std::make_shared<rmoss_projectile_motion::GravityProjectileSolver>(shoot_speed_);
  } else if (solver_type_ == "gaf") {
    friction_ = node_->declare_parameter("projectile.friction", 0.001);
    solver_ =
      std::make_shared<rmoss_projectile_motion::GafProjectileSolver>(shoot_speed_, friction_);
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Unknown solver type: %s", solver_type_.c_str());
    return;
  }

  aiming_point_.header.frame_id = "odom";
  aiming_point_.ns = "aiming_point";
  aiming_point_.type = visualization_msgs::msg::Marker::SPHERE;
  aiming_point_.action = visualization_msgs::msg::Marker::ADD;
  aiming_point_.scale.x = aiming_point_.scale.y = aiming_point_.scale.z = 0.12;
  aiming_point_.color.r = 1.0;
  aiming_point_.color.g = 1.0;
  aiming_point_.color.b = 1.0;
  aiming_point_.color.a = 1.0;
  aiming_point_.lifetime = rclcpp::Duration::from_seconds(0.1);


  armors_sub_ = node_->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(), [this](const auto_aim_interfaces::msg::Target::SharedPtr msg) {
      sendPackage(msg);
    });
}

SerialNodeLegacy::~SerialNodeLegacy()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

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
  offset_x_ = node_->declare_parameter("projectile.offset_x", 0.0);
  offset_y_ = node_->declare_parameter("projectile.offset_y", 0.0);
  offset_z_ = node_->declare_parameter("projectile.offset_z", 0.0);
  offset_pitch_ = node_->declare_parameter("projectile.offset_pitch", 0.0);
  offset_yaw_ = node_->declare_parameter("projectile.offset_yaw", 0.0);
  offset_time_ = node_->declare_parameter("projectile.offset_time", 0.0);
  shoot_speed_ = node_->declare_parameter("projectile.initial_speed", 15.0);
  solver_type_ = node_->declare_parameter("projectile.solver_type", "gravity");

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

void SerialNodeLegacy::receivePackage()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceivedPackage));

  while (rclcpp::ok()) {
    try {
      serial_driver_->port()->receive(header);

      if (header[0] == 0x5A) {
        data.resize(sizeof(ReceivedPackage) - 1);
        serial_driver_->port()->receive(data);

        data.insert(data.begin(), header[0]);
        ReceivedPackage package = Vector2Pak(data);

        bool crc16_check = verifyCRC16CheckSum(reinterpret_cast<uint8_t *>(&package), sizeof(ReceivedPackage));

        if (crc16_check) {
          // RCLCPP_INFO(node_->get_logger(), "Yaw: %f, Pitch: %f", package.yaw, package.pitch);
          cur_pitch_ = package.pitch / 180. * M_PI;
          cur_yaw_ = package.yaw / 180. * M_PI;

          geometry_msgs::msg::TransformStamped t;
            timestamp_offset_ = node_->get_parameter("timestamp_offset").as_double();
            t.header.stamp = node_->now() + rclcpp::Duration::from_seconds(timestamp_offset_);
            t.header.frame_id = "odom";
            t.child_frame_id = "gimbal_link";
            tf2::Quaternion q;
            q.setRPY(0., package.pitch / 180. * M_PI, package.yaw / 180. * M_PI);
            t.transform.rotation = tf2::toMsg(q);
            tf_broadcaster_->sendTransform(t);
        } else {
          RCLCPP_ERROR(node_->get_logger(), "CRC16 check failed");
        }
      } else {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *(node_->get_clock()), 20, "Invalid header: %02X", header[0]);
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *(node_->get_clock()), 20, "Error receiving data: %s", e.what());
      reopenPort();
    }
  }
}

void SerialNodeLegacy::reopenPort()
{
  RCLCPP_WARN(node_->get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(node_->get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void SerialNodeLegacy::sendPackage(const auto_aim_interfaces::msg::Target::SharedPtr msg)
{
  if (!msg->tracking) {
    return;
  }

  rclcpp::Time target_time = msg->header.stamp;
  auto center_position =
    Eigen::Vector3d(
    msg->position.x + offset_x_, msg->position.y + offset_y_,
    msg->position.z + offset_z_);
  auto center_velocity = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);

  // Calculate each target position at current time & predict time.
  double min_yaw = std::numeric_limits<double>::max();
  double min_dis = std::numeric_limits<double>::max();
  double hit_yaw = cur_yaw_, hit_pitch = cur_pitch_;
  bool is_current_pair = true;
  double r = 0., target_dz = 0., fly_time = 0.;
  double target_pitch, target_yaw;
  Eigen::Vector3d target_position, target_predict_position;
  double final_x = 0., final_y = 0., final_z = 0.;

  if (msg->armors_num <= 0)
   return;

  for (int i = 0; i < msg->armors_num; ++i) {
    double tmp_yaw = msg->yaw + i * (2 * M_PI / msg->armors_num);
    if (msg->armors_num == 4) {
      r = is_current_pair ? msg->radius_1 : msg->radius_2;
      is_current_pair = !is_current_pair;
      target_dz = is_current_pair ? 0. : msg->dz;
    } else {
      r = msg->radius_1;
      target_dz = 0.;
    }
    target_position = center_position + Eigen::Vector3d(
      -r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw),
      target_dz);

    // Use distance to calculate the time offset. (Approximate)
    fly_time = target_position.head(2).norm() / shoot_speed_ + offset_time_;
    tmp_yaw = tmp_yaw + msg->v_yaw * fly_time;
    target_predict_position = center_position + center_velocity * fly_time +
      Eigen::Vector3d(
      -r * std::cos(tmp_yaw), -r * std::sin(tmp_yaw),
      target_dz);

    solver_->solve(
      target_predict_position.head(2).norm(), target_predict_position.z(),
      target_pitch);
    target_pitch = -target_pitch;  // Right-handed system
    target_yaw = std::atan2(target_predict_position.y(), target_predict_position.x());

    // Choose the target with minimum yaw error.
    if (::abs(
        ::fmod(
          tmp_yaw,
          M_PI) - cur_yaw_) < min_yaw && target_predict_position.head(2).norm() < min_dis)
    {
      min_yaw = ::abs(::fmod(tmp_yaw, M_PI) - cur_yaw_);
      min_dis = target_predict_position.head(2).norm();
      hit_yaw = target_yaw;
      hit_pitch = target_pitch;
      final_x = target_predict_position.x();
      final_y = target_predict_position.y();
      final_z = target_predict_position.z();
    }
  }

  // Publish debug marker
  aiming_point_.header.stamp = node_->now();
  aiming_point_.pose.position.x = final_x;
  aiming_point_.pose.position.y = final_y;
  aiming_point_.pose.position.z = final_z;
  marker_pub_->publish(aiming_point_);

  try {
    SentPackage package;

    package.sof = 0xA5;
    // hit_pitch = 0;
    package.pitch = (hit_pitch + offset_pitch_) / M_PI * 180;
    package.yaw = (hit_yaw + offset_yaw_) / M_PI * 180;
    appendCRC8CheckSum((uint8_t *) (&package), sizeof(SentPackage));

    std::vector<uint8_t> package_vec = Pak2Vector(package);
    serial_driver_->port()->send(package_vec);

    RCLCPP_INFO(node_->get_logger(), " Target Yaw: %f, Target Pitch: %f", package.yaw, package.pitch);

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
