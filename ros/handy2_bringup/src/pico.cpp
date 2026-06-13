#include <chrono>
#include <memory>
#include <string>

#include "pico_comm.h"
#include <iostream>
#include <unistd.h>
#include <time.h>

#include "rclcpp/rclcpp.hpp"
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses a fancy C++11 lambda
* function to shorten the callback syntax, at the expense of making the
* code somewhat more difficult to understand at first glance. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("pico_node")
  {
    this->declare_parameter("shutter_on_us", shutter_on_us_default_);
    this->declare_parameter("shutter_offset_us", shutter_offset_us_default_);
    shutter_on_us_ = this->get_parameter("shutter_on_us").as_int();
    shutter_offset_us_ = this->get_parameter("shutter_offset_us").as_int();
    RCLCPP_INFO(this->get_logger(), "Setting Shutter Params: ON=%d us, Offset=%d us", shutter_on_us_, shutter_offset_us_);

    // Register parameter change callback
    callback_handle_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        uint32_t target_on_us = shutter_on_us_;
        uint32_t target_offset_us = shutter_offset_us_;
        bool changed = false;

        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "shutter_on_us") {
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              target_on_us = parameter.as_int();
              changed = true;
            }
          } else if (parameter.get_name() == "shutter_offset_us") {
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              target_offset_us = parameter.as_int();
              changed = true;
            }
          }
        }

        if (changed && initialized_) {
          bool verified = false;
          int retries = 5; // Increased retries for robustness against firmware race
          while (retries-- > 0) {
            if (picoComm::set_shutter_params(target_on_us, target_offset_us)) {
              // Wait for Pico to potentially overwrite or for sysfs to settle
              std::this_thread::sleep_for(100ms); 
              
              uint32_t read_on, read_offset;
              if (picoComm::read_shutter_params(read_on, read_offset)) {
                if (read_on == target_on_us && read_offset == target_offset_us) {
                  verified = true;
                  break;
                } else {
                  RCLCPP_WARN(this->get_logger(), "Verification failed: Expected ON=%u, Offset=%u but got ON=%u, Offset=%u. Retrying...", 
                    target_on_us, target_offset_us, read_on, read_offset);
                }
              } else {
                RCLCPP_WARN(this->get_logger(), "Failed to read back parameters for verification. Retrying...");
              }
            } else {
              RCLCPP_WARN(this->get_logger(), "Failed to write parameters to Pico. Retrying...");
            }
            std::this_thread::sleep_for(150ms);
          }

          if (verified) {
            shutter_on_us_ = target_on_us;
            shutter_offset_us_ = target_offset_us;
            RCLCPP_INFO(this->get_logger(), "Shutter params updated and verified: ON=%d us, Offset=%d us", shutter_on_us_, shutter_offset_us_);
          } else {
            result.successful = false;
            result.reason = "Failed to verify parameter update on hardware after retries. Hardware may be overriding values.";
            RCLCPP_ERROR(this->get_logger(), "%s", result.reason.c_str());
          }
        } else if (changed && !initialized_) {
          shutter_on_us_ = target_on_us;
          shutter_offset_us_ = target_offset_us;
        }

        return result;
      });

    // Use a timer to retry init without blocking constructor
    init_timer_ = this->create_wall_timer(
      1s,
      [this]() {
        if (!picoComm::init()) {
          RCLCPP_WARN(this->get_logger(), "failed to initialize picoComm, retrying in 1 second...");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "picoComm initialized successfully.");
        initialized_ = true;
        
        // Initial application of parameters with verification
        bool verified = false;
        int retries = 5;
        while (retries-- > 0) {
            if (picoComm::set_shutter_params(shutter_on_us_, shutter_offset_us_)) {
                std::this_thread::sleep_for(100ms);
                uint32_t read_on, read_offset;
                if (picoComm::read_shutter_params(read_on, read_offset) && 
                    read_on == shutter_on_us_ && read_offset == shutter_offset_us_) {
                    verified = true;
                    break;
                }
            }
            std::this_thread::sleep_for(150ms);
        }

        if (verified) {
            RCLCPP_INFO(this->get_logger(), "Initial shutter params applied and verified.");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to verify initial shutter params. Hardware might be reverting them.");
        }

        // Cancel timer once initialized
        init_timer_->cancel();
      });
  }

private:
  uint32_t shutter_on_us_;
  const int shutter_on_us_default_ = 2000;
  uint32_t shutter_offset_us_;
  const int shutter_offset_us_default_ = 0;
  bool initialized_ = false;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  rclcpp::TimerBase::SharedPtr init_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  RCLCPP_INFO(node->get_logger(), "pico_node is now spinning!");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
