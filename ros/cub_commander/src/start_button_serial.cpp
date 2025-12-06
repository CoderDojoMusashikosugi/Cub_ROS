#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include <libserial/SerialPort.h>

class UpEdge
{
public:
  bool operator()(bool value){
    bool retval = (before==false && value==true);
    before=value;
    return retval;
  }
private:
  bool before=false;
};

class StartButtonSerial : public rclcpp::Node
{
public:
  StartButtonSerial()
  : Node("start_button_serial_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Empty>("/proceed_to_next_group", 10);

    if (open_serial_port()) {
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        std::bind(&StartButtonSerial::read_serial, this)
      );
      RCLCPP_INFO(this->get_logger(), "Button serial port opened successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open button serial port");
      rclcpp::shutdown();
    }
  }

private:
  LibSerial::SerialPort serial_port_;
  std::string read_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr publisher_;
  UpEdge button_pressed_;

  bool open_serial_port()
  {
    try {
      serial_port_.Open("/dev/ttyATOM");
      serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
      return true;
    } catch (const LibSerial::OpenFailed&) {
      return false;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Serial port error: %s", e.what());
      return false;
    }
  }

  void read_serial()
  {
    if (!serial_port_.IsOpen()) {
      return;
    }

    try {
      while (serial_port_.IsDataAvailable()) {
        char byte;
        serial_port_.ReadByte(byte, 0);

        if (byte == '\n') {
          process_data();
          read_buffer_.clear();
        } else {
          read_buffer_ += byte;
          if (read_buffer_.size() > 100) {
            read_buffer_.clear();
          }
        }
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 5000,
                          "Serial read error: %s", e.what());
    }
  }

  void process_data()
  {
    if (read_buffer_.size() < 5) {
      return;
    }

    if (read_buffer_.substr(0, 4) == "BTN:") {
      char state_char = read_buffer_[4];
      if (state_char == '0' || state_char == '1') {
        bool pressed = (state_char == '1');
        if (button_pressed_(pressed)) {
          publisher_->publish(std_msgs::msg::Empty());
          RCLCPP_INFO(this->get_logger(), "Button pressed - published to /proceed_to_next_group");
        }
      }
    }
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StartButtonSerial>());
  rclcpp::shutdown();
  return 0;
}
