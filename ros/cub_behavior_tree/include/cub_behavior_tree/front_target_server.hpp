#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace cub_behavior_tree
{

std::shared_ptr<rclcpp::Node> create_front_target_server(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

}  // namespace cub_behavior_tree
