#include <functional>
#include <stdexcept>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <signal.h>
#include <stdio.h>
# include <termios.h>
# include <unistd.h>

static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_B = 0x62;
static constexpr char KEYCODE_C = 0x63;
static constexpr char KEYCODE_D = 0x64;
static constexpr char KEYCODE_E = 0x65;
static constexpr char KEYCODE_F = 0x66;
static constexpr char KEYCODE_G = 0x67;
static constexpr char KEYCODE_Q = 0x71;
static constexpr char KEYCODE_R = 0x72;
static constexpr char KEYCODE_T = 0x74;
static constexpr char KEYCODE_V = 0x76;
static constexpr char KEYCODE_SPACE = 0x20;

bool running = true;

class KeyboardReader final
{
public:
  KeyboardReader()
  {
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
      throw std::runtime_error("Failed to set new console mode");
    }
  }

  char readOne()
  {
    char c = 0;

    int rc = read(0, &c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }

    return c;
  }

  ~KeyboardReader()
  {
    tcsetattr(0, TCSANOW, &cooked_);
  }

private:
  struct termios cooked_;
};

class TeleopmCub final
{
public:
  TeleopmCub()
  {
    nh_ = rclcpp::Node::make_shared("teleop_mcub");
    nh_->declare_parameter("scale_angular", 1.0);
    nh_->declare_parameter("scale_linear", 1.0);

    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("set_twist", 1);
  }

  int keyLoop()
  {
    char c;
    bool stop_flg = true;

    std::thread{std::bind(&TeleopmCub::spin, this)}.detach();

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the mcub.");
    puts("'Q' to quit.");

    while (running)
    {
      // get the next event from the keyboard
      try
      {
        c = input_.readOne();
      }
      catch (const std::runtime_error &)
      {
        perror("read():");
        return -1;
      }

      double linear = 0.0;
      double angular = 0.0;

      RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

      switch(c)
      {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        angular = 1.0;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        angular = -1.0;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        linear = 0.2;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        linear = -0.2;
        break;
      case KEYCODE_SPACE:
        RCLCPP_DEBUG(nh_->get_logger(), "SPACE");
        linear = 0;
        angular = 0;
        stop_flg = true;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        stop_flg = true;
        running = false;
        break;
      default:
        // This can happen if the read returned when there was no data, or
        // another key was pressed.  In these cases, just silently ignore the
        // press.
        break;
      }

      if (running && (linear != 0.0 || angular != 0.0))
      {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = nh_->get_parameter("scale_angular").as_double() * angular;
        twist.linear.x = nh_->get_parameter("scale_linear").as_double() * linear;
        twist_pub_->publish(twist);
        stop_flg = false;
      }
      if (stop_flg)
      {
        geometry_msgs::msg::Twist twist;
        twist.angular.z = 0;
        twist.linear.x = 0;
        twist_pub_->publish(twist);
        stop_flg = false;
      }
    }

    return 0;
  }

private:
  void spin()
  {
    rclcpp::spin(nh_);
  }


  rclcpp::Node::SharedPtr nh_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;

  KeyboardReader input_;
};

void quit(int sig)
{
  (void)sig;
  running = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  signal(SIGINT, quit);

  TeleopmCub teleop_mcub;

  int rc = teleop_mcub.keyLoop();

  rclcpp::shutdown();

  return rc;
}