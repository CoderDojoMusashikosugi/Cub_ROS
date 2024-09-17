#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>

class mCubPublishOdometryNode : public rclcpp::Node
{
public:
    mCubPublishOdometryNode() : Node("mcub_publish_odometry_node")
    {
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&mCubPublishOdometryNode::publish_odometry, this));

        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;

        previous_left_encoder_ = 0;
        previous_right_encoder_ = 0;

        wheel_radius_ = 0.1;  // ホイールの半径 [m]
        wheel_base_ = 0.5;    // ホイールベース（左右のホイール間の距離） [m]
    }

private:
    void publish_odometry()
    {
        // ここでモーターの現在のエンコーダの値を取得する
        int left_encoder = get_left_encoder();
        int right_encoder = get_right_encoder();

        // エンコーダの変化量を計算
        int delta_left = left_encoder - previous_left_encoder_;
        int delta_right = right_encoder - previous_right_encoder_;

        // エンコーダの値を更新
        previous_left_encoder_ = left_encoder;
        previous_right_encoder_ = right_encoder;

        // ホイールの回転量から移動距離を計算
        double delta_left_dist = delta_left * (2 * M_PI * wheel_radius_);
        double delta_right_dist = delta_right * (2 * M_PI * wheel_radius_);

        // ロボットの直線移動距離と角度変化量を計算
        double delta_s = (delta_right_dist + delta_left_dist) / 2.0;
        double delta_theta = (delta_right_dist - delta_left_dist) / wheel_base_;

        // オドメトリを更新
        x_ += delta_s * cos(theta_);
        y_ += delta_s * sin(theta_);
        theta_ += delta_theta;

        // オドメトリメッセージを作成
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";

        // ポーズを設定
        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;

        // クォータニオンに変換
        tf2::Quaternion quat;
        quat.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = quat.x();
        odom_msg.pose.pose.orientation.y = quat.y();
        odom_msg.pose.pose.orientation.z = quat.z();
        odom_msg.pose.pose.orientation.w = quat.w();

        // ツイストを設定
        odom_msg.child_frame_id = "base_footprint";
        odom_msg.twist.twist.linear.x = delta_s / 0.1;  // 0.1秒ごとに発行するので
        odom_msg.twist.twist.angular.z = delta_theta / 0.1;

        // オドメトリを発行
        odom_publisher_->publish(odom_msg);
    }

    int get_left_encoder()
    {
        // 実際のエンコーダの値を取得するコードに置き換える
        return 0;
    }

    int get_right_encoder()
    {
        // 実際のエンコーダの値を取得するコードに置き換える
        return 0;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_, y_, theta_;
    int previous_left_encoder_, previous_right_encoder_;
    double wheel_radius_, wheel_base_;
};
