#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace handy1_bringup
{
    class FrequencyChecker : public rclcpp::Node
    {
    public:
        explicit FrequencyChecker(const rclcpp::NodeOptions & options)
        : Node("frequency_checker", options), count_(0), start_time_(this->now())
        {
            sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "input_image", 10,
                std::bind(&FrequencyChecker::callback, this, std::placeholders::_1));
            
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&FrequencyChecker::timer_callback, this));
        }

    private:
        void callback(const sensor_msgs::msg::Image::SharedPtr /*msg*/)
        {
            count_++;
        }

        void timer_callback()
        {
            auto now = this->now();
            double duration = (now - start_time_).seconds();
            if (duration > 0.0) {
                double fps = count_ / duration;
                RCLCPP_INFO(this->get_logger(), "Frequency: %.2f Hz", fps);
            }
            count_ = 0;
            start_time_ = now;
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
        rclcpp::TimerBase::SharedPtr timer_;
        size_t count_;
        rclcpp::Time start_time_;
    };
}

RCLCPP_COMPONENTS_REGISTER_NODE(handy1_bringup::FrequencyChecker)