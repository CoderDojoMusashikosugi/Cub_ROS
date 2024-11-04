#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <rclcpp/qos.hpp>

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
        rclcpp::QoS best_effort(10);
        best_effort.best_effort();
        wheel_position_subscription_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel_positions", best_effort, std::bind(&WheelOdometryNode::wheelPositionCallback, this, std::placeholders::_1));

        // TFブロードキャスターの初期化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // 初期化
        last_left_wheel_pos_ = 0;
        last_right_wheel_pos_ = 0;
#ifdef CUB_TARGET_CUB2
        last_left_rear_wheel_pos_ = 0;
        last_right_rear_wheel_pos_ = 0;
#endif
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
        if((last_left_wheel_pos_ == 0) && (last_right_wheel_pos_ == 0)) {
            last_left_wheel_pos_ = left_wheel_position;
            last_right_wheel_pos_ = right_wheel_position;
        }
#ifdef CUB_TARGET_CUB2
        // メッセージから車輪の位置を取得
        int32_t left_wheel_position_2 = msg->data[2];   // 不正値読み出し時のバックアップ
        int32_t right_wheel_position_2 = msg->data[3];  // 不正値読み出し時のバックアップ

        if((last_left_rear_wheel_pos_ == 0) && (last_right_rear_wheel_pos_ == 0)) {
            last_left_rear_wheel_pos_ = left_wheel_position_2;
            last_right_rear_wheel_pos_ = right_wheel_position_2;
        }

        int32_t delta_left_pos;   // pulse
        int32_t delta_right_pos;  // pulse
        int32_t delta_left_pos_2;   // pulse    不正値読み出し時のバックアップ
        int32_t delta_right_pos_2;  // pulse    不正値読み出し時のバックアップ
        
        delta_left_pos = calc_delta_pos(last_left_wheel_pos_, left_wheel_position);
        delta_left_pos_2 = calc_delta_pos(last_left_rear_wheel_pos_, left_wheel_position_2);
        RCLCPP_DEBUG(this->get_logger(), "left pos(last, cur, delta) (%d, %d, %d) left rear(%d, %d, %d)", last_left_wheel_pos_, left_wheel_position, delta_left_pos,
            last_left_rear_wheel_pos_, left_wheel_position_2, delta_left_pos_2);
        if (isElementInVector(msg->data, left_wheel_position)) {  // posが他の要素と同じ値だった場合は、不正な値呼び出しとし、他の値を使用する
            delta_left_pos = -delta_left_pos_2;
            left_wheel_position = last_left_wheel_pos_ += delta_left_pos_2;
            if (left_wheel_position < 0) left_wheel_position += POS_MAX;
            RCLCPP_WARN(this->get_logger(), "msg->data[0] is invalid value %d", msg->data[0]);
        } else {
            delta_left_pos = -delta_left_pos;
        }

        delta_right_pos = calc_delta_pos(last_right_wheel_pos_, right_wheel_position);
        delta_right_pos_2 = calc_delta_pos(last_right_rear_wheel_pos_, right_wheel_position_2);
        RCLCPP_DEBUG(this->get_logger(), "right pos(last, cur, delta) (%d, %d, %d) right rear(%d, %d, %d)", last_right_wheel_pos_, right_wheel_position, delta_right_pos,
            last_right_rear_wheel_pos_, right_wheel_position_2, delta_right_pos_2);
        if (isElementInVector(msg->data, right_wheel_position)) {  // posが他の要素と同じ値だった場合は、不正な値呼び出しとし、他の値を使用する
            delta_right_pos = delta_right_pos_2;
            right_wheel_position = last_right_wheel_pos_ += delta_right_pos_2;
            if (right_wheel_position < 0) right_wheel_position += POS_MAX;
            RCLCPP_WARN(this->get_logger(), "msg->data[1] is invalid value %d", msg->data[1]);
        }


#elif defined(CUB_TARGET_MCUB)
        int32_t delta_left_pos = (left_wheel_position - last_left_wheel_pos_);     // pulse
        int32_t delta_right_pos = (right_wheel_position - last_right_wheel_pos_);  // pulse
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
        last_left_wheel_pos_ = left_wheel_position;
        last_right_wheel_pos_ = right_wheel_position;
#ifdef CUB_TARGET_CUB2
        last_left_rear_wheel_pos_ = left_wheel_position_2;
        last_right_rear_wheel_pos_ = right_wheel_position_2;
#endif
    }
    
    int32_t calc_delta_pos(int32_t _last_pos, int32_t _cur_pos)
    {
        const int32_t upper_th = 22000;
        const int32_t lower_th = 10000;
        int32_t delta_pos = 0;
        // オーバーフロー処理
        if (_last_pos > upper_th && _cur_pos < lower_th) {
            // 正方向にオーバーフロー
            delta_pos = (32767 - _last_pos) + _cur_pos + 1;
        } 
        // アンダーフロー処理
        else if (_last_pos < lower_th && _cur_pos > upper_th) {
            // 逆方向にアンダーフロー
            delta_pos = -((32767 - _cur_pos) + _last_pos + 1);
        } 
        // 通常の範囲内の変化
        else {
            delta_pos = _cur_pos - _last_pos;
        }
        return delta_pos;
    }

    bool isElementInVector(const std::vector<int>& vec, int element) {
        int count = 0;
        for (const auto& v : vec) {
            if (v == element) {
                count++;
            }
        }
        return (count>=2)? true : false;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr wheel_position_subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    int32_t last_left_wheel_pos_;
    int32_t last_right_wheel_pos_;

    // オドメトリの計算
#ifdef CUB_TARGET_CUB2
    // CUB_TARGET が cub2 の場合のコード
    const double wheel_distance_ = 110.0 / 1000.0;   // 車輪間距離 [mm]
    const double wheel_circumference_ = 150 * M_PI / 1000.0;       // 車輪の円周 [mm]　直径 * pi
    const int16_t POS_MAX = 32767; //車輪のエンコーダーの最大値
    const double ang_res_ = 1.0/(double)POS_MAX;    // [deg/pulse] motor pluse resolution
    const double pulse2meter_param_ = ang_res_ * wheel_circumference_;
    int32_t last_left_rear_wheel_pos_;
    int32_t last_right_rear_wheel_pos_;
#elif defined(CUB_TARGET_MCUB)
    // CUB_TARGET が mcub の場合のコード
    const double wheel_distance_ = 125.0 / 1000.0;   // 車輪間距離 [mm]
    const double wheel_circumference_ = 40 * M_PI / 1000.0;       // 車輪の円周 [mm]　直径 * pi
    const double ang_res_ = 0.088;    // [deg/pulse] motor pluse resolution
    const double pulse2meter_param_ = ang_res_ * wheel_circumference_ / 360.0;
#endif

    double x_, y_, theta_; // 現在のオドメトリの位置と角度
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
