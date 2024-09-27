#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class WheelOdometryNode : public rclcpp::Node
{
public:
    WheelOdometryNode()
    : Node("wheel_odometry_node")
    {
        // オドメトリのパブリッシャーの初期化
        odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        // サブスクライバーの初期化
        wheel_position_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "wheel_positions", 10, std::bind(&WheelOdometryNode::wheelPositionCallback, this, std::placeholders::_1));

        // TFブロードキャスターの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初期化
        last_left_wheel_position_ = 0;
        last_right_wheel_position_ = 0;
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

private:
    void wheelPositionCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        
        // メッセージから車輪の位置を取得
        int32_t left_wheel_position = msg->data[0];
        int32_t right_wheel_position = msg->data[1];

        if((last_left_wheel_position_ == 0) && (last_right_wheel_position_ == 0)) {
            last_left_wheel_position_ = left_wheel_position;
            last_right_wheel_position_ = right_wheel_position;
        }

#ifdef CUB_TARGET_CUB2
        int32_t delta_left_pos;   // pulse
        int32_t delta_right_pos;  // pulse
        // オーバーフロー処理 left
        if (last_left_wheel_position_ > 30000 && left_wheel_position < 3000) {
            // 正方向にオーバーフロー
            delta_left_pos = (32767 - last_left_wheel_position_) + left_wheel_position + 1;
        } 
        // アンダーフロー処理
        else if (last_left_wheel_position_ < 3000 && left_wheel_position > 30000) {
            // 逆方向にアンダーフロー
            delta_left_pos = -((32767 - left_wheel_position) + last_left_wheel_position_ + 1);
        } 
        // 通常の範囲内の変化
        else {
            delta_left_pos = left_wheel_position - last_left_wheel_position_;
        }

        // オーバーフロー処理 right
        if (last_right_wheel_position_ > 30000 && right_wheel_position < 3000) {
            // 正方向にオーバーフロー
            delta_right_pos = (32767 - last_right_wheel_position_) + right_wheel_position + 1;
        } 
        // アンダーフロー処理
        else if (last_right_wheel_position_ < 3000 && right_wheel_position > 30000) {
            // 逆方向にアンダーフロー
            delta_right_pos = -((32767 - right_wheel_position) + last_right_wheel_position_ + 1);
        } 
        // 通常の範囲内の変化
        else {
            delta_right_pos = right_wheel_position - last_right_wheel_position_;
        }
#elif defined(CUB_TARGET_MCUB)
        int32_t delta_left_pos = (left_wheel_position - last_left_wheel_position_);     // pulse
        int32_t delta_right_pos = (right_wheel_position - last_right_wheel_position_);  // pulse
#endif
        double delta_left = delta_left_pos * pulse2meter_param_;    // [m]
        double delta_right = delta_right_pos * pulse2meter_param_;  // [m]
        double delta_center = (delta_left + delta_right) / 2.0; //[m]

        // 角度の更新
        double delta_theta = (delta_right - delta_left) / wheel_distance_;  // [rad]
        theta_ += delta_theta;

        // 位置の更新
        double delta_x = delta_center * cos(theta_);
        double delta_y = delta_center * sin(theta_);
        x_ += delta_x;
        y_ += delta_y;

        RCLCPP_DEBUG(this->get_logger(), "delta (l,r) = (%lf, %lf), delta_center = %lf delta_theta = %lf", delta_left, delta_right, delta_center, delta_theta);

        // オドメトリメッセージの作成
        auto odometry_msg = nav_msgs::msg::Odometry();
        odometry_msg.header.stamp = this->get_clock()->now();
        odometry_msg.header.frame_id = "odom";
        odometry_msg.child_frame_id = "base_link";
        odometry_msg.pose.pose.position.x = x_;
        odometry_msg.pose.pose.position.y = y_;
        odometry_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        odometry_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

        // オドメトリメッセージのパブリッシュ
        odometry_publisher_->publish(odometry_msg);

        // TF変換のブロードキャスト
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = odometry_msg.header.stamp;
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = x_;
        transform_stamped.transform.translation.y = y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = odometry_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform_stamped);

        // 現在の車輪位置を保存
        last_left_wheel_position_ = left_wheel_position;
        last_right_wheel_position_ = right_wheel_position;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_position_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int32_t last_left_wheel_position_;
    int32_t last_right_wheel_position_;

    // オドメトリの計算
#ifdef CUB_TARGET_CUB2
    // CUB_TARGET が cub2 の場合のコード
    std::cout << "CUB_TARGET is cub2" << std::endl;
    const double wheel_distance_ = 110.0 / 1000.0;   // 車輪間距離 [mm]
    const double wheel_circumference_ = 150 * M_PI / 1000.0;       // 車輪の円周 [mm]　直径 * pi
    const double ang_res_ = 1;    // [deg/pulse] motor pluse resolution
    const int16_t POS_MAX = 32767; //車輪のエンコーダーの最大値
#elif defined(CUB_TARGET_MCUB)
    // CUB_TARGET が mcub の場合のコード
    const double wheel_distance_ = 125.0 / 1000.0;   // 車輪間距離 [mm]
    const double wheel_circumference_ = 40 * M_PI / 1000.0;       // 車輪の円周 [mm]　直径 * pi
    const double ang_res_ = 0.088;    // [deg/pulse] motor pluse resolution
#endif

    const double pulse2meter_param_ = ang_res_ * wheel_circumference_ / 360.0;

    double x_, y_, theta_; // 現在のオドメトリの位置と角度
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
