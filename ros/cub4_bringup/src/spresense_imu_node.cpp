#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <sys/ioctl.h>

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
  float ax;                  // Accel X [m/s^2]
  float ay;                  // Accel Y [m/s^2]
  float az;                  // Accel Z [m/s^2]
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

// Helper function to format status bitmask into "[xxxxxx]" style string
// Bit 5: STATUS_DRDY_TIMED
// Bit 4: STATUS_SYNC_WITHIN_1H
// Bit 3: STATUS_SYNC_WITHIN_1M
// Bit 2: STATUS_SYNC_WITHIN_2S
// Bit 1: STATUS_SYNC_EVER
// Bit 0: STATUS_CLOCK_SYNCHRONIZED
std::string format_status_bits(uint8_t status) {
  std::string s = "------";
  if (status & STATUS_DRDY_TIMED)          s[0] = 'x'; // Bit 5
  if (status & STATUS_SYNC_WITHIN_1H)      s[1] = 'x'; // Bit 4
  if (status & STATUS_SYNC_WITHIN_1M)      s[2] = 'x'; // Bit 3
  if (status & STATUS_SYNC_WITHIN_2S)      s[3] = 'x'; // Bit 2
  if (status & STATUS_SYNC_EVER)           s[4] = 'x'; // Bit 1
  if (status & STATUS_CLOCK_SYNCHRONIZED)  s[5] = 'x'; // Bit 0
  return "[" + s + "]";
}

// Helper function to convert integer baud rate to termios speed_t
speed_t get_termios_baudrate(int baudrate) {
  switch (baudrate) {
    case 110: return B110;
    case 300: return B300;
    case 600: return B600;
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 576000: return B576000;
    case 921600: return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
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
    data_timeout_sec_ = this->declare_parameter<double>("data_timeout_sec", 5.0);

    RCLCPP_INFO(this->get_logger(), "Opening serial port: %s at %d baud (timeout: %.1fs)",
                serial_port_name_.c_str(), baud_rate_, data_timeout_sec_);
    RCLCPP_INFO(this->get_logger(), "IMU frame_id: '%s', GNSS sync time: %s",
                frame_id_.c_str(), use_gnss_time_ ? "ENABLED" : "DISABLED");
    RCLCPP_INFO(this->get_logger(), "sizeof(SyncedIMUData): %zu bytes, packet size: %zu bytes",
                sizeof(SyncedIMUData), sizeof(HEADER) + sizeof(SyncedIMUData) + 1);

    // Create publishers
    imu_raw_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 20);
    if (publish_temperature_) {
      temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
    }

    // Attempt initial open (non-fatal if port is not yet available)
    try_open_serial();

    // Create timer for non-blocking serial reading and reconnection
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&SpresenseImuNode::read_serial_data, this)
    );
  }

  ~SpresenseImuNode() override {
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Serial port closed.");
  }

private:
  void close_serial() {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  bool try_open_serial() {
    close_serial();

    // Check if device file exists first
    if (access(serial_port_name_.c_str(), F_OK) != 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for serial port '%s' to appear...",
                           serial_port_name_.c_str());
      return false;
    }

    int fd = ::open(serial_port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for serial port '%s': %s",
                           serial_port_name_.c_str(), std::strerror(errno));
      return false;
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for serial port '%s': %s",
                           serial_port_name_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    speed_t speed;
    try {
      speed = get_termios_baudrate(baud_rate_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "%s", e.what());
      ::close(fd);
      return false;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    // 8 data bits, 1 stop bit, no parity, no hardware flow control
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cflag |= (CLOCAL | CREAD);

    // Raw input
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
    // Raw output
    tio.c_oflag &= ~OPOST;
    // Raw mode (non-canonical, no echo, no signals)
    tio.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Waiting for serial port '%s': %s",
                           serial_port_name_.c_str(), std::strerror(errno));
      ::close(fd);
      return false;
    }

    // Set DTR high
    int status = 0;
    if (ioctl(fd, TIOCMGET, &status) == 0) {
      status |= TIOCM_DTR;
      ioctl(fd, TIOCMSET, &status);
    }

    serial_fd_ = fd;
    data_buffer_.clear();
    was_gnss_synced_ = false;
    was_drdy_logged_ = false;
    was_drdy_active_ = false;
    is_connected_ = true;
    last_data_received_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(this->get_logger(),
                "Serial port '%s' opened successfully. Waiting for IMU packets...",
                serial_port_name_.c_str());
    return true;
  }

  void disconnect_serial(const std::string& reason) {
    RCLCPP_WARN(this->get_logger(),
                "Serial port '%s' disconnected (%s). Will attempt to reconnect...",
                serial_port_name_.c_str(), reason.c_str());
    is_connected_ = false;
    close_serial();

    data_buffer_.clear();
    was_gnss_synced_ = false;
    was_drdy_logged_ = false;
    was_drdy_active_ = false;
  }

  void read_serial_data() {
    auto now = std::chrono::steady_clock::now();

    if (!is_connected_ || serial_fd_ < 0) {
      if (now - last_reconnect_attempt_ >= std::chrono::seconds(1)) {
        last_reconnect_attempt_ = now;
        try_open_serial();
      }
      return;
    }

    // Check if device file still exists
    if (access(serial_port_name_.c_str(), F_OK) != 0) {
      disconnect_serial("device file disappeared");
      return;
    }

    struct pollfd pfd;
    pfd.fd = serial_fd_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    // Wait up to 50ms for data
    int poll_ret = poll(&pfd, 1, 50);

    if (poll_ret < 0) {
      if (errno == EINTR) {
        return;
      }
      disconnect_serial(std::strerror(errno));
      return;
    }

    if (poll_ret == 0) {
      // Timeout: Check if data timeout threshold exceeded
      double elapsed_sec = std::chrono::duration<double>(now - last_data_received_).count();
      if (elapsed_sec >= data_timeout_sec_) {
        std::ostringstream ss;
        ss << "data timeout (>" << data_timeout_sec_ << "s)";
        disconnect_serial(ss.str());
        return;
      }
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "No data from Spresense IMU (timeout). Check wiring and power.");
      return;
    }

    // Check for errors or disconnection on file descriptor
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
      disconnect_serial("device hangup/error detected by poll");
      return;
    }

    if (pfd.revents & POLLIN) {
      uint8_t buffer[256];
      ssize_t bytes_read = ::read(serial_fd_, buffer, sizeof(buffer));
      if (bytes_read > 0) {
        last_data_received_ = now;
        std::vector<uint8_t> byte_buffer(buffer, buffer + bytes_read);
        process_serial_buffer(byte_buffer);
      } else if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          disconnect_serial(std::strerror(errno));
        }
      } else if (bytes_read == 0) {
        // EOF indicates device disconnected
        disconnect_serial("end of file (device disconnected)");
      }
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

    // Check if GNSS clock is synchronized and fresh (within last 2 seconds)
    bool is_gnss_fresh = (data.status & STATUS_SYNC_WITHIN_2S) && (data.utc_timestamp_us > 0);

    if (use_gnss_time_ && is_gnss_fresh) {
      // Convert microseconds to nanoseconds
      stamp = rclcpp::Time(static_cast<int64_t>(data.utc_timestamp_us * 1000ULL));

      if (!was_gnss_synced_) {
        RCLCPP_INFO(this->get_logger(),
                    "%s GNSS time synchronization locked! Using GNSS UTC timestamps.",
                    format_status_bits(data.status).c_str());
        was_gnss_synced_ = true;
      }
    } else {
      // Fallback to ROS system time
      stamp = this->get_clock()->now();

      if (was_gnss_synced_) {
        RCLCPP_WARN(this->get_logger(),
                    "%s GNSS time sync lost. Falling back to ROS system time.",
                    format_status_bits(data.status).c_str());
        was_gnss_synced_ = false;
      } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "%s Waiting for GNSS sync. Using ROS system time.",
                             format_status_bits(data.status).c_str());
      }
    }

    bool drdy_active = (data.status & STATUS_DRDY_TIMED) != 0;
    if (!was_drdy_logged_ || drdy_active != was_drdy_active_) {
      if (drdy_active) {
        RCLCPP_INFO(this->get_logger(),
                    "%s IMU DRDY edge capture (D18) ACTIVE. Timestamps locked to DRDY interrupt edge.",
                    format_status_bits(data.status).c_str());
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "%s IMU DRDY (D18) not detected / unconnected. Falling back to data arrival time.",
                    format_status_bits(data.status).c_str());
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

    // Linear acceleration: already in m/s^2 from cxd5602pwbimu driver
    imu_msg->linear_acceleration.x = data.ax;
    imu_msg->linear_acceleration.y = data.ay;
    imu_msg->linear_acceleration.z = data.az;

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
  double data_timeout_sec_{5.0};
  bool was_gnss_synced_{false};
  bool was_drdy_active_{false};
  bool was_drdy_logged_{false};
  bool is_connected_{false};
  std::chrono::steady_clock::time_point last_reconnect_attempt_{};
  std::chrono::steady_clock::time_point last_data_received_{};

  int serial_fd_{-1};
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
