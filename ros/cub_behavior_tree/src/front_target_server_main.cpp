#include <rclcpp/rclcpp.hpp>
#include "cub_behavior_tree/front_target_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(cub_behavior_tree::create_front_target_server());
  rclcpp::shutdown();
  return 0;
}
