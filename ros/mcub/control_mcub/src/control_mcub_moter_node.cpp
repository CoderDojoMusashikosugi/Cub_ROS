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

/*******************************************************************************
// This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
// For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
// To test this example, please follow the commands below.
//
// Open terminal #1
// $ ros2 run dynamixel_sdk_examples read_write_node
//
// Open terminal #2 (run one of below commands at a time)
// $ ros2 topic pub -1 /set_position dynamixel_sdk_custom_interfaces/SetPosition "{id: 1, position: 1000}"
// $ ros2 service call /get_position dynamixel_sdk_custom_interfaces/srv/GetPosition "id: 1"
//
// Author: Will Son
*******************************************************************************/

/******************************************************************************/
/* include                                                                    */
/******************************************************************************/
#include <cstdio>
#include <memory>
#include <string>
#include <math.h>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/srv/get_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"

#include "read_write_node.hpp"

/******************************************************************************/
/* define                                                                     */
/******************************************************************************/
/* Control table address for X series                                         */
#define ADDR_DRIVE_MODE                 10
#define ADDR_OPERATING_MODE             11
#define ADDR_TORQUE_ENABLE				64
#define ADDR_GOAL_CURRENT		  		102	 // Does NOT exist in Rot motors
#define ADDR_GOAL_VELOCITY				104
#define ADDR_GOAL_POSITION				116
#define ADDR_PRESENT_CURRENT			126	 // Represents "Present Load" in Rot motors
#define ADDR_PRESENT_VELOCITY			128
#define ADDR_PRESENT_POSITION			132

/* Motors ID */
#define DXL1_ID                         1
#define DXL2_ID                         2
#define DXL3_ID                         3
#define DXL4_ID                         4

/* TORQUE ENABLE/DISABLE */
#define TORQUE_ENABLE                   1	 // Value for enabling the torque
#define TORQUE_DISABLE                  0	 // Value for disabling the torque

/* DRIVE_MODE*/
#define NORMAL_MODE     0
#define REVERSE_MODE    1

/* OPERATING_MODE */
#define CURRENT_BASED_POSITION_CONTROL 5
#define POSITION_CONTROL				       3
#define VELOCITY_CONTROL				       1
#define TORQUE_CONTROL					       0

/* Protocol version */ 
#define PROTOCOL_VERSION 2.0  // Default Protocol version of DYNAMIXEL X series.

/* Default setting */
#define BAUDRATE 57600  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/ttyAMA0"  // [Linux]: "/dev/ttyUSB*", [Windows]: "COM*"

dynamixel::PortHandler * portHandler;
dynamixel::PacketHandler * packetHandler;

uint8_t dxl_error = 0;
uint32_t goal_position = 0;
int dxl_comm_result = COMM_TX_FAIL;


/******************************************************************************/
/* Constructor                                                                */
/******************************************************************************/
ReadWriteNode::ReadWriteNode()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  this->declare_parameter("qos_depth", 10);
  int8_t qos_depth = 0;
  this->get_parameter("qos_depth", qos_depth);

  const auto QOS_RKL10V =
    rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();
  
  odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  set_twist_subscriber_ =
    this->create_subscription<Twist>(
    "cmd_vel_atom",
    QOS_RKL10V,
    [this](const Twist::SharedPtr msg) -> void
    {
      this->twist_request = *msg;
    }
  );

  clear_odom_subscriber_ =
    this->create_subscription<Empty>(
    "clear_odom",
    QOS_RKL10V,
    [this]([[maybe_unused]] const Empty::SharedPtr msg) -> void
    {
      this->odom = nav_msgs::msg::Odometry();
    }
  );

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(dt_millis),
    std::bind(&ReadWriteNode::timer_callback, this)
  );
}

ReadWriteNode::~ReadWriteNode()
{
}

void ReadWriteNode::timer_callback(){
        // caluclate velocity for each dinamixel from twist value
      float cub_d = 62.5;  // [mm] distance between center and wheel
      float motor_vel_unit = 0.229;  //[rpm]
      double r_vel_m = this->twist_request.linear.x + cub_d * this->twist_request.angular.z / 1000.0; // [m/s]
      double l_vel_m = this->twist_request.linear.x - cub_d * this->twist_request.angular.z / 1000.0; // [m/s]
      
      float diameter = 40; // [mm] diameter of wheel
      double r_vel_r = r_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]
      double l_vel_r = l_vel_m / (diameter / 1000.0 / 2.0); // [rad/s]

      int32_t r_goal_vel = (int32_t)(r_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // right goal velocity
      int32_t l_goal_vel = (int32_t)(l_vel_r / (2 * M_PI) * 60.0 / motor_vel_unit); // left goal velocity

      uint8_t dxl_error = 0;

      RCLCPP_INFO(this->get_logger(), "input linear.x = '%f' angur = '%f' \n"
                                      "r_vel_m = %lf l_vel_m = %lf\n"
                                      "r_vel_r = %lf l_vel_r = %lf\n"
                                      "r_goal_vel = %d l_goal_vel = %d",
                                      this->twist_request.linear.x, this->twist_request.angular.z,
                                      r_vel_m, l_vel_m, r_vel_r, l_vel_r,
                                      r_goal_vel, l_goal_vel);

      // Write Goal Position (length : 4 bytes)
      // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        DXL1_ID,
        ADDR_GOAL_VELOCITY,
        (uint32_t)r_goal_vel,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", DXL1_ID, r_goal_vel);
      }
      
      dxl_comm_result =
      packetHandler->write4ByteTxRx(
        portHandler,
        DXL2_ID,
        ADDR_GOAL_VELOCITY,
        (uint32_t)l_goal_vel,
        &dxl_error
      );

      if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
      } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
      } else {
        RCLCPP_INFO(this->get_logger(), "Set [ID: %d] [Goal Position: %d]", DXL2_ID, l_goal_vel);
      }


      // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
      int32_t position1 = 0;

      // Read Present Velocity (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position1, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "getposition : [ID:%d] -> [position:%d]", DXL1_ID, position1);
      } else {
        RCLCPP_INFO(this->get_logger(), "Failed to get position! Result: %d", dxl_comm_result);
      }

      // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
      int32_t position2 = 0;

      // Read Present position (length : 4 bytes) and Convert uint32 -> int32
      // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
      dxl_comm_result = packetHandler->read4ByteTxRx(
        portHandler, DXL2_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position2, &dxl_error);
      if (dxl_comm_result == COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "getposition : [ID:%d] -> [position:%d]", DXL2_ID, position2);
      } else {
        RCLCPP_INFO(this->get_logger(), "Failed to get position! Result: %d", dxl_comm_result);
      }


      if(abs(position1-last_position1) >= 2048 && abs(position1-last_position1) >= 2048){
        last_position1 = position1;
        last_position2 = position2;
        RCLCPP_INFO(this->get_logger(), "reset last_position %d,%d", last_position1,last_position2);
      }else{
        int32_t diff1 = position1-last_position1;
        int32_t diff2 = position2-last_position2;
        last_position1 = position1;
        last_position2 = position2;
        RCLCPP_INFO(this->get_logger(), "position diff %d,%d", diff1,diff2);

        static int rotation = 4096;
        static double circle = diameter / 1000.0 * M_PI;

        double diff1_meter = double(diff1)/rotation*circle;
        double diff2_meter = double(diff2)/rotation*circle;

        double diff_linear = (diff1_meter+diff2_meter)/2.0;
        double diff_angular = (diff1_meter-diff2_meter)/(cub_d/1000.0)/2.0;

        // odom.pose.pose.position.z += diff_angular;
        theta_ += diff_angular;
        double diff_x = cos(theta_)*diff_linear;
        double diff_y = sin(theta_)*diff_linear;

        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x += diff_x;
        odom.pose.pose.position.y += diff_y;
        odom.pose.pose.orientation.z = sin(theta_ / 2.0);
        odom.pose.pose.orientation.w = cos(theta_ / 2.0);
      }
      odometry_publisher_->publish(odom);

}

/******************************************************************************/
/* Function                                                                   */
/******************************************************************************/
void setupDynamixel()
{
  /* Use Position Control Mode                */
  /* #define CURRENT_BASED_POSITION_CONTROL 5 */
  /* #define POSITION_CONTROL				        3 */
  /* #define VELOCITY_CONTROL				        1 */
  /* #define TORQUE_CONTROL					        0 */
  
  // setup Dynamixel ID1
  // set Drive Mode as Reverse Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    DXL1_ID,
    ADDR_DRIVE_MODE,
    REVERSE_MODE,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set DRIVE Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set DRIVE Mode.");
  }
  // set OPERATING_MODE as VELOCITY_CONTROL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    DXL1_ID,
    ADDR_OPERATING_MODE,
    VELOCITY_CONTROL,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Velocity Control Mode.");
  }

  // setup Dynamixel ID2
  // set Drive Mode as Reverse Mode
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    DXL2_ID,
    ADDR_DRIVE_MODE,
    NORMAL_MODE,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set DRIVE Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set DRIVE Mode.");
  }
  // set OPERATING_MODE as VELOCITY_CONTROL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    DXL2_ID,
    ADDR_OPERATING_MODE,
    VELOCITY_CONTROL,
    &dxl_error
  );
  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set Velocity Control Mode.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set Velocity Control Mode.");
  }

  // Enable Torque of DYNAMIXEL
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_ENABLE,  /* Torque ON */
    &dxl_error
  );

  if (dxl_comm_result != COMM_SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to enable torque.");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to enable torque.");
  }
}

int main(int argc, char * argv[])
{
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open Serial Port
  dxl_comm_result = portHandler->openPort();
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
  }

  // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
  dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
  if (dxl_comm_result == false) {
    RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
    return -1;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
  }

  setupDynamixel();

  rclcpp::init(argc, argv);

  auto readwritenode = std::make_shared<ReadWriteNode>();
  rclcpp::spin(readwritenode);

  // Disable Torque of DYNAMIXEL
  packetHandler->write1ByteTxRx(
    portHandler,
    BROADCAST_ID,
    ADDR_TORQUE_ENABLE,
    TORQUE_DISABLE,
    &dxl_error
  );
  portHandler->closePort();
	
	rclcpp::shutdown();

  return 0;
}
