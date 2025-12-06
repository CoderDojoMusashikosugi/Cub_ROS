#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

namespace cub_diagnostics
{

class FixStatusNode : public rclcpp::Node
{
public:
  explicit FixStatusNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("fix_status", options), updater_(this)
  {
    hardware_id_ = get_parameter_or_declare("hardware_id", std::string("gps"));
    updater_.setHardwareID(hardware_id_);

    const double update_period = get_parameter_or_declare("update_period", 1.0);
    updater_.setPeriod(update_period);

    const std::string topic = get_parameter_or_declare("topic", std::string("/fix"));
    timeout_ = get_parameter_or_declare("timeout", 2.0);

    updater_.add("GPS Fix Status", this, &FixStatusNode::produce_fix_status);

    fix_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      topic, 10,
      [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
        last_fix_ = msg;
        last_received_time_ = now();
      });

    RCLCPP_INFO(get_logger(), "FixStatusNode started, monitoring topic: %s", topic.c_str());
  }

private:
  void produce_fix_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    if (!last_fix_) {
      stat.summary(DiagnosticStatus::WARN, "No GPS fix message received yet");
      return;
    }

    const auto time_since_last = (now() - last_received_time_).seconds();
    if (time_since_last > timeout_) {
      stat.summary(
        DiagnosticStatus::ERROR,
        "GPS fix message timeout (" + std::to_string(time_since_last) + "s)");
      stat.addf("Time since last message", "%.2f s", time_since_last);
      stat.addf("Timeout threshold", "%.2f s", timeout_);
      return;
    }

    const int8_t status = last_fix_->status.status;
    int level = DiagnosticStatus::OK;
    std::string message;

    // status values from sensor_msgs/NavSatStatus:
    // STATUS_NO_FIX = -1  : unable to fix position
    // STATUS_FIX = 0      : unaugmented fix
    // STATUS_SBAS_FIX = 1 : with satellite-based augmentation
    // STATUS_GBAS_FIX = 2 : with ground-based augmentation
    if (status == -1) {
      level = DiagnosticStatus::ERROR;
      message = "No GPS fix (status=-1)";
    } else if (status == 0 || status == 1) {
      level = DiagnosticStatus::WARN;
      if (status == 0) {
        message = "GPS unaugmented fix (status=0)";
      } else {
        message = "GPS SBAS fix (status=1)";
      }
    } else if (status == 2) {
      level = DiagnosticStatus::OK;
      message = "GPS GBAS fix (status=2)";
    } else {
      level = DiagnosticStatus::WARN;
      message = "Unknown GPS status (" + std::to_string(status) + ")";
    }

    stat.summary(level, message);
    stat.add("Status", std::to_string(status));
    stat.add("Latitude", std::to_string(last_fix_->latitude));
    stat.add("Longitude", std::to_string(last_fix_->longitude));
    stat.add("Altitude", std::to_string(last_fix_->altitude));
    stat.addf("Time since last message", "%.2f s", time_since_last);
  }

  template<typename T>
  T get_parameter_or_declare(const std::string & name, const T & default_value)
  {
    T value = default_value;
    if (this->has_parameter(name)) {
      this->get_parameter(name, value);
    } else {
      value = this->declare_parameter<T>(name, default_value);
    }
    return value;
  }

  diagnostic_updater::Updater updater_;
  std::string hardware_id_;
  double timeout_{2.0};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  sensor_msgs::msg::NavSatFix::SharedPtr last_fix_;
  rclcpp::Time last_received_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace cub_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<cub_diagnostics::FixStatusNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
