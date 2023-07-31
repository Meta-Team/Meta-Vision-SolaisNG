#ifndef PROJECTILE_PREDICTION_HPP
#define PROJECTILE_PREDICTION_HPP
#include "rclcpp/rclcpp.hpp"

namespace solais_serial {
class ProjectilePrediction
{
public:
	ProjectilePrediction(rclcpp::Node::SharedPtr node);
	
private:
	rclcpp::Node::SharedPtr node_;
};
}

#endif // PROJECTILE_PREDICTION_HPP