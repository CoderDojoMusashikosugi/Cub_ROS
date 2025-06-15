#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

class ClearOdomPublisher : public rclcpp::Node
{
public:
    ClearOdomPublisher() : Node("clear_odom_publisher")
    {
        auto publisher = this->create_publisher<std_msgs::msg::Empty>("/clear_odom", 10);

        std_msgs::msg::Empty msg;
        RCLCPP_INFO(this->get_logger(), "published clear odom command");
        publisher->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ClearOdomPublisher>();

    rclcpp::spin_some(node);

    rclcpp::shutdown();
    return 0;
}