#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <libserial/SerialPort.h>

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>

// Status Bitmask Definitions (matching Spresense firmware)
constexpr uint8_t STATUS_CLOCK_SYNCHRONIZED = 0x01; // Bit 0: Clock is currently synchronized
constexpr uint8_t STATUS_SYNC_EVER           = 0x02; // Bit 1: Synchronized at least once since boot
constexpr uint8_t STATUS_SYNC_WITHIN_2S      = 0x04; // Bit 2: Last sync was within 2 seconds
constexpr uint8_t STATUS_SYNC_WITHIN_1M      = 0x08; // Bit 3: Last sync was within 1 minute (60s)
constexpr uint8_t STATUS_SYNC_WITHIN_1H      = 0x10; // Bit 4: Last sync was within 1 hour (3600s)
constexpr uint8_t STATUS_DRDY_TIMED          = 0x20; // Bit 5: Timestamp derived from IMU DRDY interrupt (D18)

// Data structure for binary communication with Spresense
// Total struct size: 41 bytes
struct __attribute__((packed)) SyncedIMUData {
  uint64_t utc_timestamp_us; // Synchronized UTC timestamp in microseconds (0 if not synchronized)
  uint32_t sensor_timestamp; // 19.2MHz clock timestamp from cxd5602pwbimu_data_t
  float temp;                // Temperature [degC]
  float gx;                  // Gyro X [rad/s]
  float gy;                  // Gyro Y [rad/s]
  float gz;                  // Gyro Z [rad/s]
  float ax;                  // Accel X [G]
  float ay;                  // Accel Y [G]
  float az;                  // Accel Z [G]
  uint8_t status;            // Status bit flags
};

const uint8_t HEADER[] = {0xAA, 0xBB, 0xCC, 0xDD};

// Helper function to calculate XOR checksum
uint8_t calculate_checksum(const uint8_t* data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

// Helper function to convert integer baud rate to LibSerial BaudRate enum
LibSerial::BaudRate get_libserial_baudrate(int baudrate) {
  switch (baudrate) {
    case 110: return LibSerial::BaudRate::BAUD_110;
    case 300: return LibSerial::BaudRate::BAUD_300;
    case 600: return LibSerial::BaudRate::BAUD_600;
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 460800: return LibSerial::BaudRate::BAUD_460800;
    case 500000: return LibSerial::BaudRate::BAUD_500000;
    case 576000: return LibSerial::BaudRate::BAUD_576000;
    case 921600: return LibSerial::BaudRate::BAUD_921600;
    case 1000000: return LibSerial::BaudRate::BAUD_1000000;
    case 1152000: return LibSerial::BaudRate::BAUD_1152000;
    case 1500000: return LibSerial::BaudRate::BAUD_1500000;
    case 2000000: return LibSerial::BaudRate::BAUD_2000000;
    default: throw std::invalid_argument("Invalid baud rate");
  }
}

class SpresenseImuNode : public rclcpp::Node {
public:
  SpresenseImuNode() : Node("spresense_imu_node") {
    // Declare and get parameters
    serial_port_name_ = this->declare_parameter<std::string>("serial_port", "/dev/ttyMULIMU");
    baud_rate_ = this->declare_parameter<int>("baud_rate", 230400);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "imu_link");
    publish_temperature_ = this->declare_parameter<bool>("publish_temperature", true);
    use_gnss_time_ = this->declare_parameter<bool>("use_gnss_time", true);
    gravity_acceleration_ = this->declare_parameter<double>("gravity_acceleration", 9.80665);

    RCLCPP_INFO(this->get_logger(), "Opening serial port: %s at %d baud", serial_port_name_.c_str(), baud_rate_);
    RCLCPP_INFO(this->get_logger(), "IMU frame_id: '%s', GNSS sync time: %s",
                frame_id_.c_str(), use_gnss_time_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(this->get_logger(), "sizeof(SyncedIMUData): %zu bytes, packet size: %zu bytes",
                sizeof(SyncedIMUData), sizeof(HEADER) + sizeof(SyncedIMUData) + 1);

    // Create publishers
    imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 20);
    if (publish_temperature_) {
      temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
    }

    // Open serial port
    try {
      serial_port_.Open(serial_port_name_);
      serial_port_.SetDTR(true);
      serial_port_.SetBaudRate(get_libserial_baudrate(baud_rate_));
      serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
      serial_port_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
      serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);
      serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    } catch (const LibSerial::OpenFailed& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port '%s': %s",
                   serial_port_name_.c_str(), e.what());
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Serial port opened successfully. Waiting for IMU packets...");

    // Create timer for non-blocking serial reading
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&SpresenseImuNode::read_serial_data, this)
    );
  }

  ~SpresenseImuNode() override {
    if (serial_port_.IsOpen()) {
      serial_port_.Close();
      RCLCPP_INFO(this->get_logger(), "Serial port closed.");
    }
  }

private:
  void read_serial_data() {
    if (!serial_port_.IsOpen()) {
      return;
    }

    std::vector<uint8_t> byte_buffer;
    try {
      serial_port_.Read(byte_buffer, 256, 50); // Read up to 256 bytes with 50ms timeout
    } catch (const LibSerial::ReadTimeout&) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No data from Spresense IMU (timeout). Check wiring and power.");
      return;
    }

    if (!byte_buffer.empty()) {
      process_serial_buffer(byte_buffer);
    }
  }

  void process_serial_buffer(const std::vector<uint8_t>& new_bytes) {
    data_buffer_.insert(data_buffer_.end(), new_bytes.begin(), new_bytes.end());

    constexpr size_t PACKET_SIZE = sizeof(HEADER) + sizeof(SyncedIMUData) + 1; // 4 + 41 + 1 = 46

    while (true) {
      if (data_buffer_.size() < sizeof(HEADER)) {
        break; // Wait for at least header size
      }

      // Search for header in buffer
      size_t header_start_pos = std::string::npos;
      for (size_t i = 0; i <= data_buffer_.size() - sizeof(HEADER); ++i) {
        if (std::memcmp(data_buffer_.data() + i, HEADER, sizeof(HEADER)) == 0) {
          header_start_pos = i;
          break;
        }
      }

      if (header_start_pos == std::string::npos) {
        // No header found; retain only the last (sizeof(HEADER) - 1) bytes
        size_t keep = sizeof(HEADER) - 1;
        if (data_buffer_.size() > keep) {
          data_buffer_.erase(data_buffer_.begin(), data_buffer_.end() - keep);
        }
        break;
      }

      // Discard any garbage bytes before the header
      if (header_start_pos > 0) {
        data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + header_start_pos);
      }

      // Check if a full packet is available
      if (data_buffer_.size() < PACKET_SIZE) {
        break; // Wait for more data
      }

      // Extract payload and checksum
      SyncedIMUData received_data;
      std::memcpy(&received_data, data_buffer_.data() + sizeof(HEADER), sizeof(SyncedIMUData));
      uint8_t received_checksum = data_buffer_[sizeof(HEADER) + sizeof(SyncedIMUData)];

      uint8_t calculated_checksum = calculate_checksum(
        reinterpret_cast<const uint8_t*>(&received_data), sizeof(SyncedIMUData)
      );

      if (calculated_checksum == received_checksum) {
        process_imu_data(received_data);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "Checksum mismatch! Calc: 0x%02X, Recv: 0x%02X. Discarding corrupt packet.",
                    calculated_checksum, received_checksum);
      }

      // Remove the processed packet from buffer
      data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + PACKET_SIZE);
    }
  }

  void process_imu_data(const SyncedIMUData& data) {
    rclcpp::Time stamp;
    bool using_gnss_time = false;

    // Check if GNSS clock is synchronized and fresh (within last 2 seconds)
    bool is_gnss_fresh = (data.status & STATUS_SYNC_WITHIN_2S) && (data.utc_timestamp_us > 0);

    if (use_gnss_time_ && is_gnss_fresh) {
      // Convert microseconds to nanoseconds
      stamp = rclcpp::Time(static_cast<int64_t>(data.utc_timestamp_us * 1000ULL));
      using_gnss_time = true;

      if (!was_gnss_synced_) {
        RCLCPP_INFO(this->get_logger(),
                    "GNSS time synchronization locked! Using GNSS UTC timestamps (status=0x%02X).",
                    data.status);
        was_gnss_synced_ = true;
      }
    } else {
      // Fallback to ROS system time
      stamp = this->get_clock()->now();

      if (was_gnss_synced_) {
        RCLCPP_WARN(this->get_logger(),
                    "GNSS time sync lost (status=0x%02X). Falling back to ROS system time.",
                    data.status);
        was_gnss_synced_ = false;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Waiting for GNSS sync (status=0x%02X). Using ROS system time.",
                             data.status);
      }
    }

    bool drdy_active = (data.status & STATUS_DRDY_TIMED) != 0;
    if (!was_drdy_logged_ || drdy_active != was_drdy_active_) {
      if (drdy_active) {
        RCLCPP_INFO(this->get_logger(),
                    "IMU DRDY edge capture (D18) ACTIVE. Timestamps locked to DRDY interrupt edge.");
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "IMU DRDY (D18) not detected / unconnected. Falling back to data arrival time.");
      }
      was_drdy_active_ = drdy_active;
      was_drdy_logged_ = true;
    }

    // Publish Imu message
    auto imu_msg = std::make_unique<sensor_msgs::msg::Imu>();
    imu_msg->header.stamp = stamp;
    imu_msg->header.frame_id = frame_id_;

    // Angular velocity: already in rad/s from cxd5602pwbimu driver
    imu_msg->angular_velocity.x = data.gx;
    imu_msg->angular_velocity.y = data.gy;
    imu_msg->angular_velocity.z = data.gz;

    // Linear acceleration: convert from G to m/s^2
    imu_msg->linear_acceleration.x = data.ax * gravity_acceleration_;
    imu_msg->linear_acceleration.y = data.ay * gravity_acceleration_;
    imu_msg->linear_acceleration.z = data.az * gravity_acceleration_;

    // Orientation is not estimated on this node; flag as unknown
    imu_msg->orientation.w = 1.0;
    imu_msg->orientation.x = 0.0;
    imu_msg->orientation.y = 0.0;
    imu_msg->orientation.z = 0.0;
    imu_msg->orientation_covariance[0] = -1.0;
    imu_msg->angular_velocity_covariance[0] = -1.0;
    imu_msg->linear_acceleration_covariance[0] = -1.0;

    imu_raw_pub_->publish(std::move(imu_msg));

    // Optionally publish Temperature message
    if (publish_temperature_ && temp_pub_) {
      auto temp_msg = std::make_unique<sensor_msgs::msg::Temperature>();
      temp_msg->header.stamp = stamp;
      temp_msg->header.frame_id = frame_id_;
      temp_msg->temperature = data.temp;
      temp_msg->variance = 0.0;
      temp_pub_->publish(std::move(temp_msg));
    }
  }

  // Members
  std::string serial_port_name_;
  int baud_rate_;
  std::string frame_id_;
  bool publish_temperature_;
  bool use_gnss_time_;
  double gravity_acceleration_;
  bool was_gnss_synced_{false};
  bool was_drdy_active_{false};
  bool was_drdy_logged_{false};

  LibSerial::SerialPort serial_port_;
  std::vector<uint8_t> data_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SpresenseImuNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
