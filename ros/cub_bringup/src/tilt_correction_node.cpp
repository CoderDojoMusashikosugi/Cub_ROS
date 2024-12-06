#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

class TiltCorrectionNode : public rclcpp::Node
{
public:
  TiltCorrectionNode()
  : Node("tilt_correction_node")
  {
    // パラメータの宣言と取得（デフォルト値は0.0度）
    this->declare_parameter<double>("tilt_angle_degrees", 0.0);
    this->get_parameter("tilt_angle_degrees", tilt_angle_degrees_);

    // 度をラジアンに変換
    tilt_angle_radians_ = tilt_angle_degrees_ * M_PI / 180.0;

    RCLCPP_INFO(this->get_logger(), "Initial tilt angle: %.2f degrees", tilt_angle_degrees_);

    // オリジナルの点群トピックを購読
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10,
      std::bind(&TiltCorrectionNode::pointCloudCallback, this, std::placeholders::_1));

    // 補正後の点群をパブリッシュ
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points_corrected", 10);

    // パラメータの動的変更を可能にするコールバックの設定
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&TiltCorrectionNode::onParameterChange, this, std::placeholders::_1));
  }

private:
  // 点群のコールバック関数
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // ROS2のPointCloud2メッセージをPCLのPointCloudに変換
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    // Y軸を中心に回転行列を作成
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(static_cast<float>(tilt_angle_radians_), Eigen::Vector3f::UnitY()));

    // 変換を適用
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_corrected(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::transformPointCloud(*pcl_cloud, *pcl_cloud_corrected, transform);

    // 補正後のPointCloudをROS2のPointCloud2メッセージに変換
    sensor_msgs::msg::PointCloud2 corrected_msg;
    pcl::toROSMsg(*pcl_cloud_corrected, corrected_msg);
    corrected_msg.header = msg->header; // 元のヘッダーを保持

    // 補正後の点群をパブリッシュ
    publisher_->publish(corrected_msg);
  }

  // パラメータ変更時のコールバック関数
  rcl_interfaces::msg::SetParametersResult onParameterChange(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    for (const auto &param : params) {
      if (param.get_name() == "tilt_angle_degrees") {
        double new_angle = param.as_double();
        tilt_angle_degrees_ = new_angle;
        tilt_angle_radians_ = new_angle * M_PI / 180.0;
        RCLCPP_INFO(this->get_logger(), "Updated tilt angle: %.2f degrees", tilt_angle_degrees_);
      }
    }

    return result;
  }

  // メンバ変数
  double tilt_angle_degrees_;
  double tilt_angle_radians_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TiltCorrectionNode>());
  rclcpp::shutdown();
  return 0;
}
