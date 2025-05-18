#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "cub_commander/joypad.hpp"
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
      cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CubCommander::cmd_vel_nav_callback, this, _1));
      publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_atom", 10);
      start_button_pub_ = this->create_publisher<std_msgs::msg::Empty>("/proceed_to_next_group", 10);
      cmd_vel_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&CubCommander::publishCmdVel, this)
      );

      joy_stamp_ = this->get_clock()->now();
      cmd_vel_stamp_ = this->get_clock()->now();
    }

  private:
    Joypad joy;
    UpEdge upPressed;
    UpEdge downPressed;
    UpEdge leftPressed;
    UpEdge rightPressed;
    UpEdge TouchpadPressed;

    double linear = 1.2;
    double linear_step = 0.1;
    double angular = radians(60.0);
    double angular_step = radians(20.0);
    
    geometry_msgs::msg::Twist cmd_vel_;
    rclcpp::Time joy_stamp_;
    rclcpp::Time cmd_vel_stamp_;

    bool autonomous = true;

    void topic_callback(const sensor_msgs::msg::Joy & msg)
    {
      joy.update(msg);
      // joy.print();

      joy_stamp_ = this->get_clock()->now();

      // 自動と手動の切り替え
      if(joy.l1() || joy.l2()){
        autonomous = false;
      }else{
        autonomous = true;
      }

      // 横断歩道での再スタートボタンのJoycon代替版
      if(TouchpadPressed(joy.touchpad())){
        std_msgs::msg::Empty empty;
        start_button_pub_->publish(empty);
      }

      // 手動操縦関連
      if(upPressed(joy.up())){
        linear = clamp(linear + linear_step,linear_step,1.5);
        // RCLCPP_INFO(this->get_logger(), "linear: %f", linear);
      }
      if(downPressed(joy.down())){
        linear = clamp(linear - linear_step,linear_step,1.5);
        // RCLCPP_INFO(this->get_logger(), "linear: %f", linear);
      }
      if(leftPressed(joy.left())){
        angular = clamp(angular + angular_step,angular_step,radians(360.0));
        // RCLCPP_INFO(this->get_logger(), "angular: %f", degrees(angular));
      }
      if(rightPressed(joy.right())){
        angular = clamp(angular - angular_step,angular_step,radians(360.0));
        // RCLCPP_INFO(this->get_logger(), "angular: %f", degrees(angular));
      }
      if(autonomous==false){
        cmd_vel_.linear.x = linear*joy.lx();
        cmd_vel_.angular.z = angular*joy.ly();
      }
    }

    void cmd_vel_nav_callback(const geometry_msgs::msg::Twist & msg){
      cmd_vel_stamp_ = this->get_clock()->now();

      if(autonomous){
        cmd_vel_.linear.x  = msg.linear.x;
        cmd_vel_.angular.z = msg.angular.z;
      }
    }

    void publishCmdVel()
    {
      // コントローラの接続が切れたらマニュアルモードを強制終了する。
      if(!joy.connected()){
        autonomous = true;
      }
      // autonomousの時、navigationから３秒以上データが来てなければ、暴走防止に停止命令を入れておく。
      if(autonomous && (this->now() - cmd_vel_stamp_).seconds() >= 3.0){
        cmd_vel_.linear.x  = 0.0;
        cmd_vel_.angular.z = 0.0;
      }
      publisher_->publish(cmd_vel_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; 
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_button_pub_; 
    rclcpp::TimerBase::SharedPtr cmd_vel_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CubCommander>());
  rclcpp::shutdown();
  return 0;
}