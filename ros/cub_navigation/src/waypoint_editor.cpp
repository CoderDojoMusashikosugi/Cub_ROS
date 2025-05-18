#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

struct Waypoint {
    double x, y, yaw;
};

class WaypointEditor : public rclcpp::Node {
public:
    WaypointEditor() : Node("waypoint_editor") {
        // Subscriber to PoseStamped
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&WaypointEditor::poseCallback, this, std::placeholders::_1));
        point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point", 10, std::bind(&WaypointEditor::pointCallback, this, std::placeholders::_1));

        // Publisher for markers
        line_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoint_lines", 10);
        arrow_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/waypoint_arrows", 10);

        // YAML file path
        wp_file_ = "/home/cub/wp.yaml";

        // Ensure the file exists
        std::ofstream file(wp_file_, std::ios::app);
        file.close();

        RCLCPP_INFO(this->get_logger(), "Waypoint Manager Node Initialized.");

        initial_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&WaypointEditor::publishInitialMarkers, this));

    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr line_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arrow_marker_pub_;
    std::string wp_file_;
    rclcpp::TimerBase::SharedPtr initial_timer_;

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double yaw = calculateYaw(msg->pose.orientation);

        RCLCPP_INFO(this->get_logger(), "Received Pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

        // Save waypoint to YAML
        saveToYaml(x, y, yaw);

        // Publish markers to RViz
        publishLineMarkers();
        publishArrowMarkers();
    }

    void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // Publish markers to RViz
        publishLineMarkers();
        publishArrowMarkers();
    }

    double calculateYaw(const geometry_msgs::msg::Quaternion &q) {
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    void saveToYaml(double x, double y, double yaw) {
        // Load existing waypoints
        YAML::Node data;
        try {
            data = YAML::LoadFile(wp_file_);
        } catch (const std::exception &e) {
            data = YAML::Node(YAML::NodeType::Sequence);
        }

        // Append new waypoint
        YAML::Node new_wp;
        new_wp["x"] = x;
        new_wp["y"] = y;
        new_wp["yaw"] = yaw;
        data.push_back(new_wp);

        // Save back to file
        std::ofstream file(wp_file_);
        file << data;
        file.close();

        RCLCPP_INFO(this->get_logger(), "Waypoint saved: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);
    }

    void publishLineMarkers() {
        // Load waypoints from YAML
        YAML::Node data = YAML::LoadFile(wp_file_);

        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "waypoint_lines";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Line width
        marker.scale.x = 0.1;

        // Line color (blue)
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        // Add points to the line strip
        for (const auto &node : data) {
            geometry_msgs::msg::Point point;
            point.x = node["x"].as<double>();
            point.y = node["y"].as<double>();
            point.z = 0.0; // Keep z-axis as 0 for 2D display
            marker.points.push_back(point);
        }

        // Publish the line marker
        line_marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Published line marker connecting %zu waypoints to RViz.", marker.points.size());
    }

    void clearArrowMarkers() {
        // Load waypoints from YAML
        YAML::Node data = YAML::LoadFile(wp_file_);
        int id = 0;
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "waypoint_arrows";
        marker.id = 0;
        marker.action = visualization_msgs::msg::Marker::DELETEALL;
        // Publish the arrow marker
        arrow_marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "clear all arrow markers to RViz.");
    }
    void publishArrowMarkers() {
        clearArrowMarkers();
        // Load waypoints from YAML
        YAML::Node data = YAML::LoadFile(wp_file_);
        int id = 0;

        for (const auto &node : data) {
            Waypoint wp;
            wp.x = node["x"].as<double>();
            wp.y = node["y"].as<double>();
            wp.yaw = node["yaw"].as<double>();

            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoint_arrows";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::ARROW;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = wp.x;
            marker.pose.position.y = wp.y;
            marker.pose.position.z = 0.0;
            marker.pose.orientation = yawToQuaternion(wp.yaw);
            marker.scale.x = 0.5; // Arrow length
            marker.scale.y = 0.1; // Arrow width
            marker.scale.z = 0.1; // Arrow height
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 1.0;

            // Publish the arrow marker
            arrow_marker_pub_->publish(marker);
        }

        RCLCPP_INFO(this->get_logger(), "Published %d markers to RViz.", id);
    }

    void publishInitialMarkers(){
        publishLineMarkers();
        publishArrowMarkers();
        initial_timer_->cancel();
    }

    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
        geometry_msgs::msg::Quaternion q;
        q.w = std::cos(yaw / 2.0);
        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        return q;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointEditor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}