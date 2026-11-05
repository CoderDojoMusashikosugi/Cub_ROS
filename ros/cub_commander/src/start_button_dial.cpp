#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"

class StartButtonDial : public rclcpp::Node
{
public:
  StartButtonDial()
  : Node("start_button_dial_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("/proceed_to_next_group", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "/dial/button", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        // The Dial repeats its state every 100 ms; publish only on a rising edge.
        if (msg->data && !previous_pressed_) {
          publisher_->publish(std_msgs::msg::Empty());
          RCLCPP_INFO(this->get_logger(),
                      "Dial button pressed - published to /proceed_to_next_group");
        }
        previous_pressed_ = msg->data;
      });
  }

private:
  bool previous_pressed_ = false;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StartButtonDial>());
  rclcpp::shutdown();
  return 0;
}
