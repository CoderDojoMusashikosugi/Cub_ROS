#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <limits>
#include <map>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "diagnostic_updater/diagnostic_updater.hpp"
#include "diagnostic_updater/update_functions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp/qos.hpp"

namespace
{
std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(input);
  std::string item;
  while (std::getline(ss, item, delimiter)) {
    if (!item.empty()) {
      tokens.emplace_back(item);
    }
  }
  return tokens;
}

double as_double(const rclcpp::Parameter & param, bool & valid)
{
  valid = true;
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return param.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(param.as_int());
    default:
      valid = false;
      return 0.0;
  }
}

int as_int(const rclcpp::Parameter & param, bool & valid)
{
  valid = true;
  switch (param.get_type()) {
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return param.as_int();
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return static_cast<int>(param.as_double());
    default:
      valid = false;
      return 0;
  }
}
}  // namespace

struct TopicConfig
{
  std::string key;
  std::string topic_name;
  std::string type_name;
  std::string diagnostic_name;
  double min_hz{0.0};
  double max_hz{std::numeric_limits<double>::infinity()};
  double expected_hz{std::numeric_limits<double>::quiet_NaN()};
  double tolerance{0.1};
  int window_size{10};
  double stale_timeout{std::numeric_limits<double>::quiet_NaN()};
  bool min_hz_set{false};
  bool max_hz_set{false};
  bool expected_hz_set{false};
  bool stale_timeout_set{false};
  int depth{10};
  std::string reliability{"best_effort"};
  std::string durability{"volatile"};
};

class TopicMonitor : public diagnostic_updater::DiagnosticTask
{
public:
  TopicMonitor(TopicConfig config, const rclcpp::Clock::SharedPtr & clock)
  : diagnostic_updater::DiagnosticTask(config.diagnostic_name.empty() ?
      std::string("TopicMonitor ") + config.topic_name : config.diagnostic_name),
    config_(std::move(config)),
    min_freq_(config_.min_hz),
    max_freq_(config_.max_hz),
    frequency_status_(
      diagnostic_updater::FrequencyStatusParam(
        &min_freq_, &max_freq_, config_.tolerance, config_.window_size),
      getName(), clock),
    clock_(clock)
  {
    if (config_.diagnostic_name.empty()) {
      config_.diagnostic_name = getName();
    }
  }

  void tick()
  {
    last_message_time_ = clock_->now();
    ++message_count_;
    frequency_status_.tick();
  }

  void run(diagnostic_updater::DiagnosticStatusWrapper & stat) override
  {
    frequency_status_.run(stat);

    stat.add("Topic", config_.topic_name);
    stat.add("Type", config_.type_name);
    stat.addf("Configured window size", "%d", config_.window_size);

    if (config_.expected_hz_set) {
      stat.addf("Expected frequency (Hz)", "%0.3f", config_.expected_hz);
    }

    if (!std::isnan(config_.stale_timeout) && config_.stale_timeout > 0.0) {
      stat.addf("Stale timeout (s)", "%0.3f", config_.stale_timeout);
    }

    stat.add("QoS reliability", config_.reliability);
    stat.add("QoS durability", config_.durability);
    stat.addf("QoS depth", "%d", config_.depth);

    stat.addf("Messages received", "%llu", static_cast<unsigned long long>(message_count_));

    if (message_count_ == 0) {
      stat.add("Last message age (s)", "n/a");
    } else {
      const double age = (clock_->now() - last_message_time_).seconds();
      stat.addf("Last message age (s)", "%0.3f", age);
    }
  }

  const TopicConfig & config() const {return config_;}

private:
  TopicConfig config_;
  double min_freq_;
  double max_freq_;
  diagnostic_updater::FrequencyStatus frequency_status_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_message_time_{};
  uint64_t message_count_{0};
};

class TopicHzNode : public rclcpp::Node
{
public:
  explicit TopicHzNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("topic_diagnostics", options), updater_(this)
  {
    const auto hardware_id = get_or_declare<std::string>("hardware_id", "mcub");
    updater_.setHardwareID(hardware_id);

    const double update_period = get_or_declare<double>("update_period", 1.0);
    updater_.setPeriod(update_period);

    load_topics();

    if (monitors_.empty()) {
      RCLCPP_WARN(get_logger(), "No topic diagnostics configured. See config/topic_diagnostics.yaml");
    }
  }

private:
  template<typename T>
  T get_or_declare(const std::string & name, const T & default_value)
  {
    T value = default_value;
    if (this->has_parameter(name)) {
      this->get_parameter(name, value);
    } else {
      value = this->declare_parameter<T>(name, default_value);
    }
    return value;
  }

  void load_topics()
  {
    const auto parameter_names = this->list_parameters({"topics"}, 100);
    if (parameter_names.names.empty()) {
      return;
    }

    std::unordered_map<std::string, TopicConfig> configs;

    constexpr char prefix[] = "topics.";

    for (const auto & full_name : parameter_names.names) {
      if (full_name.rfind(prefix, 0) != 0) {
        continue;
      }

      const std::string composed_key = full_name.substr(sizeof(prefix) - 1);
      if (composed_key.empty()) {
        continue;
      }

      rclcpp::Parameter param;
      if (!this->get_parameter(full_name, param)) {
        continue;
      }

      const auto tokens = split(composed_key, '.');
      if (tokens.empty()) {
        continue;
      }

      auto & config = configs[tokens.front()];
      config.key = tokens.front();

      if (tokens.size() == 1) {
        continue;
      }

      const auto field = tokens[1];

      if (field == "topic") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          config.topic_name = param.as_string();
        }
      } else if (field == "type") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          config.type_name = param.as_string();
        }
      } else if (field == "diagnostic_name") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          config.diagnostic_name = param.as_string();
        }
      } else if (field == "expected_hz") {
        bool valid = false;
        const double value = as_double(param, valid);
        if (valid && value > 0.0) {
          config.expected_hz = value;
          config.expected_hz_set = true;
        }
      } else if (field == "min_hz") {
        bool valid = false;
        const double value = as_double(param, valid);
        if (valid && value >= 0.0) {
          config.min_hz = value;
          config.min_hz_set = true;
        }
      } else if (field == "max_hz") {
        bool valid = false;
        const double value = as_double(param, valid);
        if (valid && value > 0.0) {
          config.max_hz = value;
          config.max_hz_set = true;
        }
      } else if (field == "tolerance") {
        bool valid = false;
        const double value = as_double(param, valid);
        if (valid && value >= 0.0) {
          config.tolerance = value;
        }
      } else if (field == "window_size") {
        bool valid = false;
        const int value = as_int(param, valid);
        if (valid && value > 0) {
          config.window_size = value;
        }
      } else if (field == "stale_timeout_sec") {
        bool valid = false;
        const double value = as_double(param, valid);
        if (valid && value > 0.0) {
          config.stale_timeout = value;
          config.stale_timeout_set = true;
        }
      } else if (field == "qos" && tokens.size() >= 3) {
        const auto qos_field = tokens[2];
        if (qos_field == "reliability" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          config.reliability = param.as_string();
        } else if (qos_field == "durability" && param.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          config.durability = param.as_string();
        } else if (qos_field == "depth") {
          bool valid = false;
          const int value = as_int(param, valid);
          if (valid && value > 0) {
            config.depth = value;
          }
        }
      }
    }

    for (auto & entry : configs) {
      auto & config = entry.second;

      if (config.topic_name.empty()) {
        RCLCPP_WARN(get_logger(), "Skipping topic diagnostics '%s': missing topic name", config.key.c_str());
        continue;
      }

      if (config.type_name.empty()) {
        RCLCPP_WARN(get_logger(), "Skipping topic diagnostics '%s': missing type", config.key.c_str());
        continue;
      }

      if (config.expected_hz_set) {
        if (!config.min_hz_set) {
          config.min_hz = config.expected_hz;
          config.min_hz_set = true;
        }
        if (!config.max_hz_set) {
          config.max_hz = config.expected_hz;
          config.max_hz_set = true;
        }
      }

      if (config.stale_timeout_set && !config.min_hz_set) {
        config.min_hz = 1.0 / config.stale_timeout;
        config.min_hz_set = true;
      }

      if (config.max_hz < config.min_hz) {
        config.max_hz = config.min_hz;
      }

      if (config.window_size <= 0) {
        config.window_size = 10;
      }

      rclcpp::QoS qos(rclcpp::KeepLast(config.depth));
      auto to_lower = [](std::string value) {
        std::transform(
          value.begin(), value.end(), value.begin(),
          [](unsigned char c) {return static_cast<char>(std::tolower(c));});
        return value;
      };

      const auto reliability_lower = to_lower(config.reliability);
      if (reliability_lower == "reliable") {
        qos.reliable();
        config.reliability = "reliable";
      } else {
        qos.best_effort();
        config.reliability = "best_effort";
      }

      const auto durability_lower = to_lower(config.durability);
      if (durability_lower == "transient_local") {
        qos.transient_local();
        config.durability = "transient_local";
      } else {
        qos.durability_volatile();
        config.durability = "volatile";
      }

      auto monitor = std::make_shared<TopicMonitor>(config, this->get_clock());
      updater_.add(*monitor);
      monitors_.push_back(monitor);

      auto subscription = this->create_generic_subscription(
        config.topic_name,
        config.type_name,
        qos,
        [monitor](std::shared_ptr<rclcpp::SerializedMessage>) {
          monitor->tick();
        });

      if (!subscription) {
        RCLCPP_ERROR(
          get_logger(), "Failed to create subscription for topic '%s' (%s)",
          config.topic_name.c_str(), config.type_name.c_str());
        continue;
      }

      subscriptions_.push_back(subscription);

      const std::string max_hz_str = std::isfinite(config.max_hz) ?
        std::to_string(config.max_hz) : std::string("inf");
      RCLCPP_INFO(
        get_logger(), "Monitoring %s [%s] min %.2f Hz max %s Hz", config.topic_name.c_str(),
        config.type_name.c_str(), config.min_hz, max_hz_str.c_str());
    }
  }

  diagnostic_updater::Updater updater_;
  std::vector<std::shared_ptr<TopicMonitor>> monitors_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions()
    .allow_undeclared_parameters(true)
    .automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<TopicHzNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
