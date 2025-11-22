#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include <string>
#include <vector>
#include <algorithm>

namespace cub_diagnostics
{

class DiagRvizConverter : public rclcpp::Node
{
public:
  DiagRvizConverter()
  : Node("diag_rviz_converter")
  {
    // Subscribe to /diagnostics_agg topic
    diag_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics_agg",
      10,
      std::bind(&DiagRvizConverter::diagCallback, this, std::placeholders::_1));

    // Publish formatted overlay text
    overlay_pub_ = this->create_publisher<rviz_2d_overlay_msgs::msg::OverlayText>(
      "/diag_overlay",
      10);

    RCLCPP_INFO(this->get_logger(), "Diagnostic RViz Converter Node started");
  }

private:
  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    std::string formatted_text;
    uint8_t robot_status = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::vector<std::string> error_items;
    std::vector<std::string> warn_items;

    // Search for "Robot" group and collect error/warning items
    for (const auto & status : msg->status) {
      // Check if this is the Robot group
      if (status.name == "/Robot" || status.name == "Robot") {
        robot_status = status.level;
      }

      // Check if this item is under Robot group and has error/warning
      if (status.name.find("/Robot/") == 0 || status.name.find("Robot/") == 0) {
        if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
          // Extract the item name (everything after "Robot/")
          size_t pos = status.name.find("Robot/");
          std::string item_name = status.name.substr(pos + 6);  // 6 = length of "Robot/"
          error_items.push_back(item_name + ": " + status.message);
        } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
          size_t pos = status.name.find("Robot/");
          std::string item_name = status.name.substr(pos + 6);
          warn_items.push_back(item_name + ": " + status.message);
        }
      }
    }

    // Calculate number of lines to display
    // 1 line for status + error items + warning items
    int num_lines = 1 + error_items.size() + warn_items.size();

    // Create OverlayText message
    auto overlay_msg = rviz_2d_overlay_msgs::msg::OverlayText();

    // Set display properties
    overlay_msg.action = rviz_2d_overlay_msgs::msg::OverlayText::ADD;
    overlay_msg.width = 1000;
    overlay_msg.text_size = 16.0;
    overlay_msg.line_width = 8;

    // Calculate height based on number of lines
    // text_size + line_width for each line, plus some padding
    int line_height = static_cast<int>(overlay_msg.text_size) + overlay_msg.line_width;
    overlay_msg.height = num_lines * line_height + 3;  // 10px padding

    overlay_msg.horizontal_alignment = rviz_2d_overlay_msgs::msg::OverlayText::LEFT;
    overlay_msg.vertical_alignment = rviz_2d_overlay_msgs::msg::OverlayText::TOP;
    overlay_msg.horizontal_distance = 10;
    overlay_msg.vertical_distance = 10;
    overlay_msg.font = "DejaVu Sans Mono";

    // Set colors based on status
    switch (robot_status) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        formatted_text = "Diag: OK";
        overlay_msg.width = 150;
        // Green text
        overlay_msg.fg_color.r = 0.0;
        overlay_msg.fg_color.g = 1.0;
        overlay_msg.fg_color.b = 0.0;
        overlay_msg.fg_color.a = 1.0;
        // Dark green background
        overlay_msg.bg_color.r = 0.0;
        overlay_msg.bg_color.g = 0.3;
        overlay_msg.bg_color.b = 0.0;
        overlay_msg.bg_color.a = 0.8;
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        formatted_text = "Diag: WARN";
        // Yellow text
        overlay_msg.fg_color.r = 1.0;
        overlay_msg.fg_color.g = 1.0;
        overlay_msg.fg_color.b = 0.0;
        overlay_msg.fg_color.a = 1.0;
        // Dark yellow/orange background
        overlay_msg.bg_color.r = 0.4;
        overlay_msg.bg_color.g = 0.3;
        overlay_msg.bg_color.b = 0.0;
        overlay_msg.bg_color.a = 0.8;
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        formatted_text = "Diag: ERROR";
        // Red text
        overlay_msg.fg_color.r = 1.0;
        overlay_msg.fg_color.g = 0.0;
        overlay_msg.fg_color.b = 0.0;
        overlay_msg.fg_color.a = 1.0;
        // Dark red background
        overlay_msg.bg_color.r = 0.4;
        overlay_msg.bg_color.g = 0.0;
        overlay_msg.bg_color.b = 0.0;
        overlay_msg.bg_color.a = 0.8;
        break;
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
        formatted_text = "Diag: STALE";
        // Gray text
        overlay_msg.fg_color.r = 0.7;
        overlay_msg.fg_color.g = 0.7;
        overlay_msg.fg_color.b = 0.7;
        overlay_msg.fg_color.a = 1.0;
        // Dark gray background
        overlay_msg.bg_color.r = 0.2;
        overlay_msg.bg_color.g = 0.2;
        overlay_msg.bg_color.b = 0.2;
        overlay_msg.bg_color.a = 0.8;
        break;
      default:
        formatted_text = "Diag: UNKNOWN";
        // White text
        overlay_msg.fg_color.r = 1.0;
        overlay_msg.fg_color.g = 1.0;
        overlay_msg.fg_color.b = 1.0;
        overlay_msg.fg_color.a = 1.0;
        // Dark background
        overlay_msg.bg_color.r = 0.2;
        overlay_msg.bg_color.g = 0.2;
        overlay_msg.bg_color.b = 0.2;
        overlay_msg.bg_color.a = 0.8;
        break;
    }

    // Add error items first (higher priority)
    for (const auto & error_item : error_items) {
      formatted_text += "\n" + error_item;
    }

    // Then add warning items
    for (const auto & warn_item : warn_items) {
      formatted_text += "\n" + warn_item;
    }

    // Set the formatted text
    overlay_msg.text = formatted_text;

    // Publish the overlay message
    overlay_pub_->publish(overlay_msg);
  }

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Publisher<rviz_2d_overlay_msgs::msg::OverlayText>::SharedPtr overlay_pub_;
};

}  // namespace cub_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cub_diagnostics::DiagRvizConverter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
