#include "rclcpp/rclcpp.hpp"
#include "solais_interpreter/solais_interpreter.hpp"

#include "solais_interpreter/crc.h"
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "rmoss_projectile_motion/gravity_projectile_solver.hpp"
#include "rmoss_projectile_motion/gaf_projectile_solver.hpp"

SolaisInterpreter::SolaisInterpreter(const rclcpp::NodeOptions & options)
{
    node_ = std::make_shared<rclcpp::Node>("solais_interpreter", options);
    declareParameters();
    timestamp_offset_ = node_->declare_parameter("timestamp_offset", 0.0);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*node_);
    marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("/aiming_point", 10);

    receive_thread_ = std::thread(&SolaisInterpreter::receivePackage, this);

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

SolaisInterpreter::~SolaisInterpreter()
{
    if (receive_thread_.joinable())
        receive_thread_.join();
}

auto SolaisInterpreter::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

void SolaisInterpreter::declareParameters()
{
    offset_x_ = node_->declare_parameter("projectile.offset_x", 0.0);
    offset_y_ = node_->declare_parameter("projectile.offset_y", 0.0);
    offset_z_ = node_->declare_parameter("projectile.offset_z", 0.0);
    offset_pitch_ = node_->declare_parameter("projectile.offset_pitch", 0.0);
    offset_yaw_ = node_->declare_parameter("projectile.offset_yaw", 0.0);
    offset_time_ = node_->declare_parameter("projectile.offset_time", 0.0);
    shoot_speed_ = node_->declare_parameter("projectile.initial_speed", 15.0);
    solver_type_ = node_->declare_parameter("projectile.solver_type", "gravity");
}

void SolaisInterpreter::receivePackage()
{
    // TODO: Implement this function
}

void SolaisInterpreter::sendPackage(const auto_aim_interfaces::msg::Target::SharedPtr msg)
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

    for (int i = 0; i < msg->armors_num; ++i)
    {
        double tmp_yaw = msg->yaw + i * (2 * M_PI / msg->armors_num);
        if (msg->armors_num == 4)
        {
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
        if (abs(fmod(tmp_yaw,M_PI) - cur_yaw_cropped_) < min_yaw
            && target_predict_position.head(2).norm() < min_dis)
        {
            min_yaw = ::abs(::fmod(tmp_yaw, M_PI) - cur_yaw_cropped_);
            min_dis = target_predict_position.head(2).norm();
            hit_yaw = target_yaw;
            hit_pitch = target_pitch;
            final_x = target_predict_position.x();
            final_y = target_predict_position.y();
            final_z = target_predict_position.z();
        }

        // Publish debug marker
        aiming_point_.header.stamp = node_->now();
        aiming_point_.pose.position.x = final_x;
        aiming_point_.pose.position.y = final_y;
        aiming_point_.pose.position.z = final_z;
        marker_pub_->publish(aiming_point_);

        // MY_TODO: Publish the target position to the topic
    }
}

double SolaisInterpreter::calculateMinAngleDiff(double & angle1, double & angle2)
{
    double diff = angle1 - angle2;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    } else if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return diff;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(SolaisInterpreter)