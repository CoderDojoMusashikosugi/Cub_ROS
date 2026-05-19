#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <ctime>
#include <iomanip>
#include <sstream>

// NTP SHM structure
struct shmTime {
  int    mode; /* 0 - if valid is set:
                *      use values,
                *      clear valid
                * 1 - if valid is set:
                *      if count before and after are equal:
                *        use values,
                *        clear valid
                */
  volatile int count;
  time_t clockTimeStampSec;
  int    clockTimeStampUSec;
  time_t receiveTimeStampSec;
  int    receiveTimeStampUSec;
  int    leap;
  int    precision;
  int    nsamples;
  volatile int valid;
  unsigned     clockTimeStampNSec;
  unsigned     receiveTimeStampNSec;
  int          dummy[8];
};

class GNSSTimeNode : public rclcpp::Node {
public:
    GNSSTimeNode() : Node("gnss_time_node") {
        this->declare_parameter("port", "/dev/serial0");
        this->declare_parameter("baudrate", 9600);
        this->declare_parameter("shm_unit", 0);

        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();
        int shm_unit = this->get_parameter("shm_unit").as_int();

        // Open serial port
        fd_ = open(port.c_str(), O_RDONLY | O_NOCTTY);
        if (fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port.c_str());
            return;
        }

        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B9600); // Assuming 9600 for UM982
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        options.c_iflag &= ~(IXON | IXOFF | IXANY);
        options.c_oflag &= ~OPOST;
        tcsetattr(fd_, TCSANOW, &options);

        // Access NTP SHM
        key_t key = 0x4e545030 + shm_unit; // NTP0, NTP1, etc.
        // Try to create or access the SHM segment
        int shmid = shmget(key, sizeof(struct shmTime), IPC_CREAT | 0666);
        if (shmid < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to access/create SHM unit %d: %s", shm_unit, strerror(errno));
        } else {
            shm_ptr_ = (struct shmTime *)shmat(shmid, NULL, 0);
            if (shm_ptr_ == (void *)-1) {
                RCLCPP_ERROR(this->get_logger(), "Failed to attach SHM: %s", strerror(errno));
                shm_ptr_ = nullptr;
            } else {
                // Initialize SHM if we just created it
                if (shm_ptr_->valid == 0) {
                    shm_ptr_->mode = 0;
                    shm_ptr_->precision = -1;
                    shm_ptr_->nsamples = 3;
                }
                RCLCPP_INFO(this->get_logger(), "Successfully attached to SHM unit %d", shm_unit);
            }
        }

        thread_ = std::thread(&GNSSTimeNode::readLoop, this);
    }

    ~GNSSTimeNode() {
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) close(fd_);
        if (shm_ptr_) shmdt(shm_ptr_);
    }

private:
    void readLoop() {
        char buf[1024];
        std::string line;
        while (rclcpp::ok()) {
            int n = read(fd_, buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                for (int i = 0; i < n; ++i) {
                    if (buf[i] == '\n' || buf[i] == '\r') {
                        if (!line.empty()) {
                            parseNMEA(line);
                            line.clear();
                        }
                    } else {
                        line += buf[i];
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    void parseNMEA(const std::string& line) {
        // $GPRMC,014332.00,A,3540.0000,N,13945.0000,E,0.0,0.0,090526,,,A*59
        if (line.substr(0, 6) == "$GPRMC") {
            std::vector<std::string> fields = split(line, ',');
            if (fields.size() >= 10 && fields[2] == "A") { // Status A = active
                std::string time_str = fields[1]; // HHMMSS.SS
                std::string date_str = fields[9]; // DDMMYY

                if (time_str.size() >= 6 && date_str.size() == 6) {
                    struct tm tm = {0};
                    tm.tm_hour = std::stoi(time_str.substr(0, 2));
                    tm.tm_min = std::stoi(time_str.substr(2, 2));
                    tm.tm_sec = std::stoi(time_str.substr(4, 2));
                    tm.tm_mday = std::stoi(date_str.substr(0, 2));
                    tm.tm_mon = std::stoi(date_str.substr(2, 2)) - 1;
                    tm.tm_year = std::stoi(date_str.substr(4, 2)) + 100; // Assuming 2000s

                    time_t gnss_time = timegm(&tm);
                    double frac_sec = 0;
                    if (time_str.size() > 6) {
                        frac_sec = std::stod(time_str.substr(6));
                    }

                    updateSHM(gnss_time, frac_sec);
                }
            }
        }
    }

    void updateSHM(time_t gnss_time, double frac_sec) {
        if (!shm_ptr_) return;

        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);

        shm_ptr_->count++;
        shm_ptr_->clockTimeStampSec = gnss_time;
        shm_ptr_->clockTimeStampUSec = (int)(frac_sec * 1000000);
        shm_ptr_->clockTimeStampNSec = (unsigned)(frac_sec * 1000000000);
        shm_ptr_->receiveTimeStampSec = ts.tv_sec;
        shm_ptr_->receiveTimeStampUSec = ts.tv_nsec / 1000;
        shm_ptr_->receiveTimeStampNSec = ts.tv_nsec;
        shm_ptr_->leap = 0;
        shm_ptr_->precision = -1; // ~0.5s precision for NMEA
        shm_ptr_->valid = 1;
        shm_ptr_->count++;

        RCLCPP_INFO(this->get_logger(), "Updated SHM with GNSS time: %ld.%09u", (long)gnss_time, shm_ptr_->clockTimeStampNSec);
    }

    std::vector<std::string> split(const std::string& s, char delimiter) {
        std::vector<std::string> tokens;
        std::string token;
        std::istringstream tokenStream(s);
        while (std::getline(tokenStream, token, delimiter)) {
            tokens.push_back(token);
        }
        return tokens;
    }

    int fd_ = -1;
    struct shmTime *shm_ptr_ = nullptr;
    std::thread thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GNSSTimeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
