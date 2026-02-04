#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace handy1_bringup
{
    class LivoxFrequencyChecker : public rclcpp::Node
    {
    public:
        explicit LivoxFrequencyChecker(const rclcpp::NodeOptions & options)
        : Node("livox_frequency_checker", options), count_(0), start_time_(this->now()), last_msg_time_(this->now())
        {
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10,
                std::bind(&LivoxFrequencyChecker::callback, this, std::placeholders::_1));
            
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&LivoxFrequencyChecker::timer_callback, this));
        }

    private:
        void callback(const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/)
        {
            count_++;
            last_msg_time_ = this->now();
        }

        void timer_callback()
        {
            auto now = this->now();
            double duration = (now - start_time_).seconds();
            double time_since_last = (now - last_msg_time_).seconds();

            if (duration > 0.0) {
                double fps = count_ / duration;
                if (fps < 5.0) {
                    RCLCPP_WARN(this->get_logger(), "Frequency is low: %.2f Hz", fps);
                }
            }

            if (time_since_last > 1.0) {
                RCLCPP_WARN(this->get_logger(), "No data received for %.2f seconds", time_since_last);
            }
            
            count_ = 0;
            start_time_ = now;
        }

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        rclcpp::Time start_time_;
        rclcpp::Time last_msg_time_;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(handy1_bringup::LivoxFrequencyChecker)
