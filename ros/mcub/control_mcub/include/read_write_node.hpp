// Copyright 2021 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef READ_WRITE_NODE_HPP_
#define READ_WRITE_NODE_HPP_

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/empty.hpp"


class ReadWriteNode : public rclcpp::Node
{
public:
  using Twist = geometry_msgs::msg::Twist;
  using Odometry = nav_msgs::msg::Odometry;
  using Empty = std_msgs::msg::Empty;

  ReadWriteNode();
  virtual ~ReadWriteNode();

  void timer_callback();


private:
  rclcpp::Subscription<Twist>::SharedPtr set_twist_subscriber_;
  rclcpp::Subscription<Empty>::SharedPtr clear_odom_subscriber_;
  rclcpp::Publisher<Odometry>::SharedPtr odometry_publisher_;


  rclcpp::TimerBase::SharedPtr timer_;
  Twist twist_request;
  Odometry odom;
  int32_t last_position1;
  int32_t last_position2;
  double theta_=0.0;

  int dt_millis = 50;
};

#endif  // READ_WRITE_NODE_HPP_
