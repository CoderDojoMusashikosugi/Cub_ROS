#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cub_commander/joy_to_dualsense.hpp"
#include <math.h>
#include <algorithm>
using std::placeholders::_1;
using namespace std;

inline double radians(double value)
{
  return value*M_PI/180.0;
}

inline double degrees(double value)
{
  return value*180.0/M_PI;
}

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

class CubCommander : public rclcpp::Node
{
  public:
    CubCommander()
    : Node("cub_commander_node")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&CubCommander::topic_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      cmd_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CubCommander::publishCmdVel, this)
      );

    }

  private:
    JoyToDualSense joy;
    UpEdge upPressed;
    UpEdge downPressed;
    UpEdge leftPressed;
    UpEdge rightPressed;

    double linear = 0.4;
    double linear_step = 0.1;
    double angular = radians(60.0);
    double angular_step = radians(20.0);
    
    geometry_msgs::msg::Twist cmd_vel_;

    void topic_callback(const sensor_msgs::msg::Joy & msg)
    {
      joy.update(msg);

      if(upPressed(joy.up())){
        linear = clamp(linear + linear_step,linear_step,1.5);
        RCLCPP_INFO(this->get_logger(), "linear: %f", linear);
      }
      if(downPressed(joy.down())){
        linear = clamp(linear - linear_step,linear_step,1.5);
        RCLCPP_INFO(this->get_logger(), "linear: %f", linear);
      }
      if(leftPressed(joy.left())){
        angular = clamp(angular + angular_step,angular_step,radians(360.0));
        RCLCPP_INFO(this->get_logger(), "angular: %f", degrees(angular));
      }
      if(rightPressed(joy.right())){
        angular = clamp(angular - angular_step,angular_step,radians(360.0));
        RCLCPP_INFO(this->get_logger(), "angular: %f", degrees(angular));
      }
      cmd_vel_.linear.x = linear*joy.lx();
      cmd_vel_.angular.z = angular*joy.ly();
    }

    void publishCmdVel()
    {
      publisher_->publish(cmd_vel_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubCommander>());
  rclcpp::shutdown();
  return 0;
}