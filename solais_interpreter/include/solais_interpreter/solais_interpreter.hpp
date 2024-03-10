#pragma once

#include "rclcpp/rclcpp.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/detail/vector3__struct.hpp>
#include <rclcpp/subscription.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <Eigen/Dense>

class SolaisInterpreter
{
public:
    explicit SolaisInterpreter(const rclcpp::NodeOptions & options);
    ~SolaisInterpreter();
    auto get_node_base_interface() const;

private:
    void declareParameters();

    void rx_msg(const geometry_msgs::msg::Vector3::SharedPtr msg);

    void tx_msg(const auto_aim_interfaces::msg::Target::SharedPtr msg);

    double calculateMinAngleDiff(double& angle1, double& angle2);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr armors_sub_;
    double timestamp_offset_ = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr imu_sub_;

    // Debug
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    visualization_msgs::msg::Marker aiming_point_;
    // std::thread receive_thread_;

    // For projectile prediction
    double cur_pitch_ = 0.;
    double cur_yaw_ = 0.;
    double cur_yaw_cropped_ = 0.;  // cur_yaw_ in range [-pi, pi)
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