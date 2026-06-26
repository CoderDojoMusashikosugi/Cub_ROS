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
  : Node("minimal_publisher")
  {
    this->declare_parameter("shutter_on_us", shutter_on_us_default_);
    this->declare_parameter("shutter_offset_us", shutter_offset_us_default_);
    shutter_on_us_ = this->get_parameter("shutter_on_us").as_int();
    shutter_offset_us_ = this->get_parameter("shutter_offset_us").as_int();
    std::cout << "Setting Shutter Params: ON=" << shutter_on_us_ << "us, Offset=" << shutter_offset_us_ << "us" << std::endl;

    while (!picoComm::init()) {
        std::cout << "failed to initialize picoComm, retrying in 1 second..." << std::endl;
        std::this_thread::sleep_for(1s);
    }
    std::cout << "picoComm initialized successfully." << std::endl;

    if (!picoComm::set_shutter_params(shutter_on_us_, shutter_offset_us_)) {
        std::cerr << "Failed to set shutter params." << std::endl;
    }
  }

private:
  uint32_t shutter_on_us_;
  const int shutter_on_us_default_ = 2000;
  uint32_t shutter_offset_us_;
  const int shutter_offset_us_default_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
