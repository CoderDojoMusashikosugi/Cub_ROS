#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <math.h>
#include <algorithm>
using std::placeholders::_1;
using namespace std;

class Odom2tf : public rclcpp::Node
{
  public:
    Odom2tf()
    : Node("odom_to_tf")
    {
      subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Odom2tf::topic_callback, this, _1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry & msg)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = msg.header.stamp;
      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = msg.pose.pose.position.x;
      t.transform.translation.y = msg.pose.pose.position.y;
      t.transform.translation.z = msg.pose.pose.position.z;

      t.transform.rotation.x = msg.pose.pose.orientation.x;
      t.transform.rotation.y = msg.pose.pose.orientation.y;
      t.transform.rotation.z = msg.pose.pose.orientation.z;
      t.transform.rotation.w = msg.pose.pose.orientation.w;

      printf("%f,%f\n",t.transform.translation.x,t.transform.translation.y);

      tf_broadcaster_->sendTransform(t);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odom2tf>());
  rclcpp::shutdown();
  return 0;
}