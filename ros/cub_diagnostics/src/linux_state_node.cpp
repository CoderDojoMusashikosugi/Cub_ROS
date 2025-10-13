#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <sys/statvfs.h>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cub_diagnostics
{
namespace
{
constexpr const char * kProcStatPath = "/proc/stat";
constexpr const char * kProcMemInfoPath = "/proc/meminfo";

std::string format_percentage(double ratio)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(1) << ratio * 100.0;
  return oss.str();
}

std::string format_bytes(uint64_t bytes)
{
  static const char * units[] = {"B", "KiB", "MiB", "GiB", "TiB"};
  double value = static_cast<double>(bytes);
  size_t idx = 0;
  while (value >= 1024.0 && idx + 1 < (sizeof(units) / sizeof(units[0]))) {
    value /= 1024.0;
    ++idx;
  }

  std::ostringstream oss;
  int precision = (value >= 100.0) ? 0 : (value >= 10.0 ? 1 : 2);
  oss << std::fixed << std::setprecision(precision) << value << ' ' << units[idx];
  return oss.str();
}

int max_level(int a, int b)
{
  return (a > b) ? a : b;
}

int level_from_ratio(double ratio, double warn, double error)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (ratio >= error) {
    return DiagnosticStatus::ERROR;
  }
  if (ratio >= warn) {
    return DiagnosticStatus::WARN;
  }
  return DiagnosticStatus::OK;
}

struct CpuTimes
{
  uint64_t idle{0};
  uint64_t total{0};
  bool valid{false};
};

bool read_cpu_times(CpuTimes & out)
{
  std::ifstream file(kProcStatPath);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  if (!std::getline(file, line)) {
    return false;
  }

  std::istringstream iss(line);
  std::string label;
  iss >> label;
  if (label != "cpu") {
    return false;
  }

  std::vector<uint64_t> values;
  uint64_t value = 0;
  while (iss >> value) {
    values.push_back(value);
  }

  if (values.size() < 4) {
    return false;
  }

  uint64_t idle = values[3];
  uint64_t iowait = (values.size() > 4) ? values[4] : 0;
  uint64_t idle_all = idle + iowait;

  uint64_t total = 0;
  for (auto v : values) {
    total += v;
  }

  out.idle = idle_all;
  out.total = total;
  out.valid = true;
  return true;
}

struct MemInfo
{
  uint64_t mem_total{0};
  uint64_t mem_available{0};
  uint64_t mem_free{0};
  uint64_t buffers{0};
  uint64_t cached{0};
  uint64_t swap_total{0};
  uint64_t swap_free{0};
};

bool read_mem_info(MemInfo & info)
{
  std::ifstream file(kProcMemInfoPath);
  if (!file.is_open()) {
    return false;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::string key;
    uint64_t value = 0;
    std::string unit;
    if (!(iss >> key >> value)) {
      continue;
    }
    if (!key.empty() && key.back() == ':') {
      key.pop_back();
    }

    if (!(iss >> unit)) {
      unit.clear();
    }

    const uint64_t bytes = value * 1024ULL;

    if (key == "MemTotal") {
      info.mem_total = bytes;
    } else if (key == "MemAvailable") {
      info.mem_available = bytes;
    } else if (key == "MemFree") {
      info.mem_free = bytes;
    } else if (key == "Buffers") {
      info.buffers = bytes;
    } else if (key == "Cached") {
      info.cached = bytes;
    } else if (key == "SwapTotal") {
      info.swap_total = bytes;
    } else if (key == "SwapFree") {
      info.swap_free = bytes;
    }
  }

  if (info.mem_total > 0 && info.mem_available == 0) {
    info.mem_available = info.mem_free + info.buffers + info.cached;
  }

  return info.mem_total > 0;
}

}  // namespace

class LinuxStateNode : public rclcpp::Node
{
public:
  explicit LinuxStateNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("linux_state", options), updater_(this)
  {
    hardware_id_ = get_parameter_or_declare("hardware_id", std::string("linux"));
    updater_.setHardwareID(hardware_id_);

    const double update_period = get_parameter_or_declare("update_period", 1.0);
    updater_.setPeriod(update_period);

    cpu_warn_ = get_parameter_or_declare("cpu.warn", 0.8);
    cpu_error_ = get_parameter_or_declare("cpu.error", 0.95);
    memory_warn_ = get_parameter_or_declare("memory.warn", 0.8);
    memory_error_ = get_parameter_or_declare("memory.error", 0.95);
    swap_warn_ = get_parameter_or_declare("swap.warn", 0.5);
    swap_error_ = get_parameter_or_declare("swap.error", 0.8);
    storage_paths_ = get_parameter_or_declare("storage.paths", std::vector<std::string>{"/"});
    storage_warn_ = get_parameter_or_declare("storage.warn", 0.8);
    storage_error_ = get_parameter_or_declare("storage.error", 0.95);

    updater_.add("Linux CPU Usage", this, &LinuxStateNode::produce_cpu_status);
    updater_.add("Linux Memory Usage", this, &LinuxStateNode::produce_memory_status);
    updater_.add("Linux Swap Usage", this, &LinuxStateNode::produce_swap_status);
    updater_.add("Linux Storage Usage", this, &LinuxStateNode::produce_storage_status);
  }

private:
  void produce_cpu_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    CpuTimes current;
    if (!read_cpu_times(current)) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Failed to read /proc/stat");
      return;
    }

    if (!previous_cpu_.valid) {
      previous_cpu_ = current;
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Collecting initial CPU sample");
      return;
    }

    const uint64_t total_delta = current.total - previous_cpu_.total;
    const uint64_t idle_delta = current.idle - previous_cpu_.idle;
    previous_cpu_ = current;

    if (total_delta == 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "No CPU usage delta yet");
      return;
    }

    const double usage = 1.0 - static_cast<double>(idle_delta) / static_cast<double>(total_delta);
    const int level = level_from_ratio(usage, cpu_warn_, cpu_error_);

    std::ostringstream message;
    message << "CPU usage " << format_percentage(usage) << "%";
    stat.summary(level, message.str());
    stat.add("Usage", format_percentage(usage) + "%");
    stat.addf("Warn threshold", "%.1f%%", cpu_warn_ * 100.0);
    stat.addf("Error threshold", "%.1f%%", cpu_error_ * 100.0);
  }

  void produce_memory_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    MemInfo info;
    if (!read_mem_info(info)) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Failed to read /proc/meminfo");
      return;
    }

    if (info.mem_total == 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "MemTotal reported as zero");
      return;
    }

    const uint64_t mem_used = info.mem_total - info.mem_available;
    const double usage = static_cast<double>(mem_used) / static_cast<double>(info.mem_total);
    const int level = level_from_ratio(usage, memory_warn_, memory_error_);

    std::ostringstream message;
    message << "Memory usage " << format_percentage(usage) << "%";
    stat.summary(level, message.str());
    stat.add("Usage", format_percentage(usage) + "%");
    stat.add("Total", format_bytes(info.mem_total));
    stat.add("Used", format_bytes(mem_used));
    stat.add("Available", format_bytes(info.mem_available));
    stat.addf("Warn threshold", "%.1f%%", memory_warn_ * 100.0);
    stat.addf("Error threshold", "%.1f%%", memory_error_ * 100.0);

    last_mem_info_ = info;
  }

  void produce_swap_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    MemInfo info = last_mem_info_;
    if (info.swap_total == 0) {
      // Refresh if previous read missed swap values
      read_mem_info(info);
    }

    if (info.swap_total == 0) {
      stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Swap not configured");
      return;
    }

    const uint64_t swap_used = info.swap_total - info.swap_free;
    const double usage = static_cast<double>(swap_used) / static_cast<double>(info.swap_total);
    const int level = level_from_ratio(usage, swap_warn_, swap_error_);

    std::ostringstream message;
    message << "Swap usage " << format_percentage(usage) << "%";
    stat.summary(level, message.str());
    stat.add("Usage", format_percentage(usage) + "%");
    stat.add("Total", format_bytes(info.swap_total));
    stat.add("Used", format_bytes(swap_used));
    stat.add("Free", format_bytes(info.swap_free));
    stat.addf("Warn threshold", "%.1f%%", swap_warn_ * 100.0);
    stat.addf("Error threshold", "%.1f%%", swap_error_ * 100.0);
  }

  void produce_storage_status(diagnostic_updater::DiagnosticStatusWrapper & stat)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;

    if (storage_paths_.empty()) {
      stat.summary(DiagnosticStatus::WARN, "No storage paths configured");
      return;
    }

    int overall_level = DiagnosticStatus::OK;
    std::vector<std::string> messages;

    for (const auto & path : storage_paths_) {
      struct statvfs vfs
      {
      };
      if (statvfs(path.c_str(), &vfs) != 0) {
        overall_level = max_level(overall_level, DiagnosticStatus::ERROR);
        messages.push_back(path + ": statvfs failed");
        continue;
      }

      const uint64_t total = static_cast<uint64_t>(vfs.f_frsize) * static_cast<uint64_t>(vfs.f_blocks);
      const uint64_t available = static_cast<uint64_t>(vfs.f_frsize) * static_cast<uint64_t>(vfs.f_bavail);
      if (total == 0) {
        overall_level = max_level(overall_level, DiagnosticStatus::WARN);
        messages.push_back(path + ": total size reported as zero");
        continue;
      }

      const uint64_t used = total - available;
      const double usage = static_cast<double>(used) / static_cast<double>(total);
      const int level = level_from_ratio(usage, storage_warn_, storage_error_);
      overall_level = max_level(overall_level, level);

      std::ostringstream oss;
      oss << path << "=" << format_percentage(usage) << "% used";
      messages.push_back(oss.str());

      stat.add(path + " used", format_bytes(used));
      stat.add(path + " available", format_bytes(available));
      stat.add(path + " total", format_bytes(total));
    }

    std::string summary_msg;
    if (messages.empty()) {
      summary_msg = "No storage data";
    } else {
      std::ostringstream oss;
      bool first = true;
      for (const auto & msg : messages) {
        if (!first) {
          oss << "; ";
        }
        oss << msg;
        first = false;
      }
      summary_msg = oss.str();
    }

    stat.summary(overall_level, summary_msg);
    stat.addf("Warn threshold", "%.1f%%", storage_warn_ * 100.0);
    stat.addf("Error threshold", "%.1f%%", storage_error_ * 100.0);
  }

  diagnostic_updater::Updater updater_;
  std::string hardware_id_;

  double cpu_warn_{0.8};
  double cpu_error_{0.95};
  double memory_warn_{0.8};
  double memory_error_{0.95};
  double swap_warn_{0.5};
  double swap_error_{0.8};
  std::vector<std::string> storage_paths_;
  double storage_warn_{0.8};
  double storage_error_{0.95};

  CpuTimes previous_cpu_{};
  MemInfo last_mem_info_{};

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
};

}  // namespace cub_diagnostics

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<cub_diagnostics::LinuxStateNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
