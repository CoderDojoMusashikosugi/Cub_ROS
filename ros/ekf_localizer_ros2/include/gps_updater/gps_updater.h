#ifndef GPS_UPDATER_H_
#define GPS_UPDATER_H_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <memory>

class GpsUpdater : public rclcpp::Node
{
public:
    GpsUpdater();
    ~GpsUpdater();

private:
    void gps_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);
    bool check_gps_quality(const sensor_msgs::msg::NavSatFix& gps_msg);
    void convert_gps_to_local(double lat, double lon, double alt, 
                              double& x, double& y, double& z);
    void publish_gps_pose();

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr gps_pose_pub_;

    // GPS data
    sensor_msgs::msg::NavSatFix current_gps_;
    geometry_msgs::msg::PoseWithCovarianceStamped gps_pose_;

    // GPS coordinate converter
    std::unique_ptr<GeographicLib::LocalCartesian> gps_projector_;

    // Parameters
    std::string gps_topic_name_;
    std::string gps_pose_topic_name_;
    std::string map_frame_id_;
    
    double gps_origin_lat_;
    double gps_origin_lon_;
    double gps_origin_alt_;
    double min_satellites_;
    double max_hdop_;
    double max_covariance_threshold_;
    
    bool is_gps_initialized_;
    bool use_manual_origin_;
};

#endif // GPS_UPDATER_H_
