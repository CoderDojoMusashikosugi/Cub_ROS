#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
// #include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <vector>
#include <string>

class JsonLineVisualizer : public rclcpp::Node {
public:
    JsonLineVisualizer() : Node("json_line_visualizer") {
        // Publisher for line markers
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/wp_vis", 10);

        // Load the JSON file
        std::string json_file = "/home/cub/colcon_ws/src/cub/cub_behavior_tree/routes/furo_waypoints.yaml"; // Replace with your actual file path
        if (!loadJsonFile(json_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load JSON file: %s", json_file.c_str());
            return;
        }

        // Timer to publish line markers every 10 seconds
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&JsonLineVisualizer::publishLineMarkers, this));

        RCLCPP_INFO(this->get_logger(), "Json Line Visualizer initialized. Publishing every 10 seconds.");
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<std::pair<double, double>> waypoints_; // Stores (x, y) pairs from JSON file

    bool loadJsonFile(const std::string &filename) {

        try {
            YAML::Node config = YAML::LoadFile(filename);
            if (!config["waypoint_groups"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML file does not contain 'waypoint_groups'.");
                return false;
            }

            for (const auto & group_node : config["waypoint_groups"]) {

                for (const auto & wp_node : group_node["waypoints"]) {
                    double x = wp_node["x"].as<double>();
                    double y = wp_node["y"].as<double>();
                    double yaw = wp_node["yaw"].as<double>();
                    // printf("%f,%f,%f\n",x,y,yaw);
                    waypoints_.emplace_back(x, y);
                }
            }

            printf("size: %d\n",waypoints_.size());

        return true;
        }
        catch (const YAML::Exception & e) {
        RCLCPP_ERROR(this->get_logger(), "YAML Exception: %s", e.what());
        return false;
        }
        catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        return false;
        }
    }

    void publishLineMarkers() {
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map"; // Change to your map frame
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "json_line";
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
        for (const auto &wp : waypoints_) {
            geometry_msgs::msg::Point point;
            point.x = wp.first;
            point.y = wp.second;
            point.z = 0.0; // Z-axis remains 0 for 2D visualization
            // printf("%f,%f\n",point.x,point.y);
            marker.points.push_back(point);
        }

        // Publish the line marker
        marker_pub_->publish(marker);
        RCLCPP_INFO(this->get_logger(), "Published line marker connecting %zu waypoints to RViz.", waypoints_.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JsonLineVisualizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
