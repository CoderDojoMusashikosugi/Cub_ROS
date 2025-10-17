#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <dirent.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cub_diagnostics
{
namespace
{
constexpr const char * kThermalBasePath = "/sys/devices/virtual/thermal";

std::string trim(const std::string & input)
{
  const char whitespace[] = " \t\n\r";
  const auto start = input.find_first_not_of(whitespace);
  if (start == std::string::npos) {
    return {};
  }
  const auto end = input.find_last_not_of(whitespace);
  return input.substr(start, end - start + 1);
}

bool read_first_line(const std::string & path, std::string & out)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  if (!std::getline(file, line)) {
    return false;
  }

  out = trim(line);
  return true;
}

bool starts_with(const char * text, const char * prefix)
{
  if (!text || !prefix) {
    return false;
  }
  const auto prefix_length = std::strlen(prefix);
  return std::strncmp(text, prefix, prefix_length) == 0;
}

int max_level(int a, int b)
{
  return (a > b) ? a : b;
}
}  // namespace

class JetsonStateNode : public rclcpp::Node
{
  struct ThermalZone
  {
    std::string type;
    std::string temp_path;
  };

public:
  explicit JetsonStateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("jetson_state", options), updater_(this)
  {
    hardware_id_ = get_parameter_or_declare("hardware_id", std::string("jetson"));
    updater_.setHardwareID(hardware_id_);

    const double update_period = get_parameter_or_declare("update_period", 1.0);
    updater_.setPeriod(update_period);

    warn_temperature_ = get_parameter_or_declare("warn_temperature", 60.0);
    error_temperature_ = get_parameter_or_declare("error_temperature", 80.0);
    requested_types_ = get_parameter_or_declare(
      "thermal_zones",
      std::vector<std::string>{
        "cpu-thermal",
        "gpu-thermal",
        "soc0-thermal",
        "soc1-thermal",
        "soc2-thermal",
        "tj-thermal"
      });

    load_thermal_zones();

    updater_.add("Jetson Thermal State", this, &JetsonStateNode::produce_thermal_status);
  }

private:
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

  void load_thermal_zones()
  {
    zones_.clear();
    missing_types_.clear();

    std::unordered_map<std::string, std::string> discovered;

    DIR * dir = opendir(kThermalBasePath);
    if (!dir) {
      RCLCPP_WARN(
        get_logger(), "Unable to open %s for thermal zone discovery: %s",
        kThermalBasePath, std::strerror(errno));
    } else {
      struct dirent * entry = nullptr;
      while ((entry = readdir(dir)) != nullptr) {
        if (entry->d_name[0] == '.') {
          continue;
        }
        if (!starts_with(entry->d_name, "thermal_zone")) {
          continue;
        }

        std::string zone_path = std::string(kThermalBasePath) + "/" + entry->d_name;
        std::string type_path = zone_path + "/type";
        std::string type_value;
        if (!read_first_line(type_path, type_value)) {
          RCLCPP_WARN(
            get_logger(), "Failed to read thermal zone type at %s",
            type_path.c_str());
          continue;
        }

        discovered[type_value] = zone_path + "/temp";
      }
      closedir(dir);
    }

    for (const auto & type : requested_types_) {
      auto it = discovered.find(type);
      if (it != discovered.end()) {
        zones_.push_back(ThermalZone{type, it->second});
      } else {
        missing_types_.push_back(type);
      }
    }

    if (zones_.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "No requested thermal zones found. Verify names against contents of %s",
        kThermalBasePath);
    } else {
      std::ostringstream oss;
      bool first = true;
      for (const auto & zone : zones_) {
        if (!first) {
          oss << ", ";
        }
        oss << zone.type;
        first = false;
      }
      RCLCPP_INFO(
        get_logger(),
        "Monitoring Jetson thermal zones: %s",
        oss.str().c_str());
    }

    if (!missing_types_.empty()) {
      std::ostringstream oss;
      bool first = true;
      for (const auto & type : missing_types_) {
        if (!first) {
          oss << ", ";
        }
        oss << type;
        first = false;
      }
      RCLCPP_WARN(
        get_logger(), "Requested thermal zones not found: %s",
        oss.str().c_str());
    }
  }

  int level_from_temperature(double temperature_c) const
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    if (temperature_c >= error_temperature_) {
      return DiagnosticStatus::ERROR;
    }
    if (temperature_c >= warn_temperature_) {
      return DiagnosticStatus::WARN;
    }
    return DiagnosticStatus::OK;
  }

  bool read_temperature(const std::string & path, double & temperature_c) const
  {
    std::string value_str;
    if (!read_first_line(path, value_str)) {
      return false;
    }

    if (value_str.empty()) {
      return false;
    }

    try {
      const long raw_value = std::stol(value_str);
      temperature_c = static_cast<double>(raw_value) / 1000.0;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        get_logger(),
        "Failed to parse temperature from %s: %s (value='%s')",
        path.c_str(), e.what(), value_str.c_str());
      return false;
    }
  }

  std::string join_missing_types() const
  {
    if (missing_types_.empty()) {
      return {};
    }
    std::ostringstream oss;
    bool first = true;
    for (const auto & type : missing_types_) {
      if (!first) {
        oss << ", ";
      }
      oss << type;
      first = false;
    }
    return oss.str();
  }

  void produce_thermal_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    if (zones_.empty()) {
      stat.summary(DiagnosticStatus::WARN, "No thermal zones configured or available");
      if (!missing_types_.empty()) {
        stat.add("Missing zone types", join_missing_types());
      }
      stat.addf("Warn threshold (C)", "%.1f", warn_temperature_);
      stat.addf("Error threshold (C)", "%.1f", error_temperature_);
      return;
    }

    int overall_level = DiagnosticStatus::OK;
    double max_temperature = std::numeric_limits<double>::lowest();
    std::string hottest_zone;
    bool any_success = false;

    for (const auto & zone : zones_) {
      double temperature_c = 0.0;
      if (!read_temperature(zone.temp_path, temperature_c)) {
        stat.add(zone.type + " status", "read failed");
        overall_level = max_level(overall_level, DiagnosticStatus::ERROR);
        continue;
      }

      any_success = true;
      stat.addf(zone.type + " (C)", "%.2f", temperature_c);
      const int level = level_from_temperature(temperature_c);
      overall_level = max_level(overall_level, level);

      if (temperature_c > max_temperature) {
        max_temperature = temperature_c;
        hottest_zone = zone.type;
      }
    }

    if (!missing_types_.empty()) {
      stat.add("Missing zone types", join_missing_types());
    }

    if (!any_success) {
      stat.summary(DiagnosticStatus::ERROR, "Failed to read all thermal zones");
    } else {
      std::ostringstream message;
      message << "Max " << hottest_zone << " " << std::fixed << std::setprecision(1)
              << max_temperature << " C";
      stat.summary(overall_level, message.str());
    }

    stat.addf("Warn threshold (C)", "%.1f", warn_temperature_);
    stat.addf("Error threshold (C)", "%.1f", error_temperature_);
  }

  diagnostic_updater::Updater updater_;
  std::string hardware_id_;
  double warn_temperature_{80.0};
  double error_temperature_{95.0};
  std::vector<std::string> requested_types_;
  std::vector<ThermalZone> zones_;
  std::vector<std::string> missing_types_;
};

}  // namespace cub_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<cub_diagnostics::JetsonStateNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
