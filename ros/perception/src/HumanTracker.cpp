#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/common/centroid.h>
#include <mutex>

class HumanTracker : public rclcpp::Node
{
public:
    HumanTracker() : Node("human_tracker")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Human Tracker...");
        
        // Parameters
        this->declare_parameter("min_cluster_size", 30);
        this->declare_parameter("max_cluster_size", 2000);
        this->declare_parameter("cluster_tolerance", 0.5);
        this->declare_parameter("min_height", 1.0);
        this->declare_parameter("max_height", 2.2);
        this->declare_parameter("min_width", 0.3);
        this->declare_parameter("max_width", 1.0);
        this->declare_parameter("ground_threshold", 0.1);
        this->declare_parameter("voxel_leaf_size", 0.05);
        this->declare_parameter("use_camera", false);
        
        RCLCPP_INFO(this->get_logger(), "Parameters declared");
        
        // LiDAR subscriber
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&HumanTracker::cloudCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "LiDAR subscriber created");
        
        // Camera subscriber (optional)
        bool use_camera = this->get_parameter("use_camera").as_bool();
        if (use_camera) {
            image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/camera/color/image_raw", 10,
                std::bind(&HumanTracker::imageCallback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Camera subscriber created");
        }
        
        // Publishers
        human_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "human_position", 10);
        human_dir_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "human_direction", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "human_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "Publishers created");
        
        // TF Buffer
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        RCLCPP_INFO(this->get_logger(), "TF buffer and listener initialized");
        
        // Initialize tracking variables
        last_position_ = geometry_msgs::msg::Point();
        last_time_ = this->get_clock()->now();
        position_history_.clear();
        
        RCLCPP_INFO(this->get_logger(), "HSD32C Human Tracker initialized successfully");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr human_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr human_dir_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    geometry_msgs::msg::Point last_position_;
    rclcpp::Time last_time_;
    std::vector<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> position_history_;
    
    cv::Mat latest_image_;
    std::mutex image_mutex_;
    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received point cloud message");
        
        try {
            // Convert ROS PointCloud2 to PCL
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *cloud);
            
            RCLCPP_INFO(this->get_logger(), "Point cloud converted: %lu points", cloud->size());
            
            // Preprocess point cloud
            auto filtered_cloud = preprocessPointCloud(cloud);
            
            RCLCPP_INFO(this->get_logger(), "Point cloud filtered: %lu points", filtered_cloud->size());
            
            // Detect humans from LiDAR
            auto human_clusters = detectHumansFromLidar(filtered_cloud);
            
            RCLCPP_INFO(this->get_logger(), "Detected %lu potential human clusters", human_clusters.size());
            
            // Validate with camera if available
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> validated_humans;
            if (this->get_parameter("use_camera").as_bool()) {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (!latest_image_.empty()) {
                    validated_humans = validateWithCamera(human_clusters, latest_image_);
                    RCLCPP_INFO(this->get_logger(), "Validated %lu human detections with camera", validated_humans.size());
                } else {
                    validated_humans = human_clusters;
                    RCLCPP_INFO(this->get_logger(), "No camera image available, using LiDAR clusters directly");
                }
            } else {
                validated_humans = human_clusters;
            }
            
            if (!validated_humans.empty()) {
                RCLCPP_INFO(this->get_logger(), "Processing validated human clusters");
                
                // Select the closest human
                auto closest_human = findClosestHuman(validated_humans);
                
                if (closest_human && !closest_human->empty()) {
                    RCLCPP_INFO(this->get_logger(), "Found closest human with %lu points", closest_human->size());
                    
                    // Publish position
                    publishHumanPosition(closest_human, cloud_msg->header);
                    
                    // Estimate and publish direction
                    estimateAndPublishDirection(closest_human, cloud_msg->header);
                    
                    // Publish visualization markers
                    publishVisualizationMarkers(validated_humans, cloud_msg->header);
                } else {
                    RCLCPP_INFO(this->get_logger(), "No valid closest human found");
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "No validated human clusters to process");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in cloudCallback: %s", e.what());
        }
    }
    
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received camera image");
        
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = cv_ptr->image.clone();
            RCLCPP_INFO(this->get_logger(), "Camera image updated");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // Voxel grid downsampling
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(cloud);
        double leaf_size = this->get_parameter("voxel_leaf_size").as_double();
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*filtered);
        
        // Remove points outside region of interest
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.5, 10.0);  // 0.5m to 10m in front
        pass.filter(*filtered);
        
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(-5.0, 5.0);  // ±5m lateral
        pass.filter(*filtered);
        
        pass.setInputCloud(filtered);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(-0.5, 3.0);  // Ground to 3m height
        pass.filter(*filtered);
        
        return filtered;
    }
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> detectHumansFromLidar(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> human_clusters;
        
        try {
            if (!cloud || cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty point cloud received for human detection");
                return human_clusters;
            }
            
            RCLCPP_INFO(this->get_logger(), "Starting human detection from %lu points", cloud->size());
            
            // Remove ground plane
            auto cloud_no_ground = removeGroundPlane(cloud);
            
            if (!cloud_no_ground || cloud_no_ground->empty()) {
                RCLCPP_WARN(this->get_logger(), "No points remaining after ground removal");
                return human_clusters;
            }
            
            RCLCPP_INFO(this->get_logger(), "Points after ground removal: %lu", cloud_no_ground->size());
            
            // Euclidean clustering
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            
            double cluster_tolerance = this->get_parameter("cluster_tolerance").as_double();
            int min_cluster_size = this->get_parameter("min_cluster_size").as_int();
            int max_cluster_size = this->get_parameter("max_cluster_size").as_int();
            
            RCLCPP_INFO(this->get_logger(), "Clustering parameters: tolerance=%f, min_size=%d, max_size=%d",
                       cluster_tolerance, min_cluster_size, max_cluster_size);
            
            ec.setClusterTolerance(cluster_tolerance);
            ec.setMinClusterSize(min_cluster_size);
            ec.setMaxClusterSize(max_cluster_size);
            ec.setInputCloud(cloud_no_ground);
            ec.extract(cluster_indices);
            
            RCLCPP_INFO(this->get_logger(), "Found %lu clusters before filtering", cluster_indices.size());
            
            // Filter clusters by human-like characteristics
            for (const auto& indices : cluster_indices) {
                if (indices.indices.empty()) {
                    RCLCPP_WARN(this->get_logger(), "Empty cluster indices encountered");
                    continue;
                }
                
                pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::copyPointCloud(*cloud_no_ground, indices, *cluster);
                
                if (isHumanLikeCluster(cluster)) {
                    human_clusters.push_back(cluster);
                    RCLCPP_INFO(this->get_logger(), "Added human-like cluster with %lu points", cluster->size());
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "Final human clusters: %lu", human_clusters.size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in human detection: %s", e.what());
        }
        
        return human_clusters;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundPlane(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
        
        try {
            if (!cloud || cloud->empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty point cloud received for ground removal");
                return cloud_no_ground;
            }
            
            // RANSAC plane segmentation
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            
            double ground_threshold = this->get_parameter("ground_threshold").as_double();
            
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(ground_threshold);
            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);
            
            if (inliers->indices.empty()) {
                RCLCPP_WARN(this->get_logger(), "No ground plane found");
                return cloud;
            }
            
            // Extract non-ground points
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            extract.filter(*cloud_no_ground);
            
            RCLCPP_INFO(this->get_logger(), "Ground removal: %lu points remaining", cloud_no_ground->size());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in ground removal: %s", e.what());
            return cloud;  // Return original cloud if ground removal fails
        }
        
        return cloud_no_ground;
    }
    
    bool isHumanLikeCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster)
    {
        try {
            if (!cluster || cluster->empty()) {
                RCLCPP_WARN(this->get_logger(), "Invalid cluster for human-like check");
                return false;
            }
            
            RCLCPP_INFO(this->get_logger(), "Checking human-like characteristics for cluster with %lu points", cluster->size());
            
            // Calculate min and max points manually
            pcl::PointXYZ min_pt, max_pt;
            min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<float>::max();
            max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<float>::max();
            
            for (const auto& point : *cluster) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                    continue;  // Skip invalid points
                }
                min_pt.x = std::min(min_pt.x, point.x);
                min_pt.y = std::min(min_pt.y, point.y);
                min_pt.z = std::min(min_pt.z, point.z);
                max_pt.x = std::max(max_pt.x, point.x);
                max_pt.y = std::max(max_pt.y, point.y);
                max_pt.z = std::max(max_pt.z, point.z);
            }
            
            // Check if we have valid min/max points
            if (min_pt.x > max_pt.x || min_pt.y > max_pt.y || min_pt.z > max_pt.z) {
                RCLCPP_WARN(this->get_logger(), "Invalid min/max points calculated");
                return false;
            }
            
            double height = max_pt.z - min_pt.z;
            double width_x = max_pt.x - min_pt.x;
            double width_y = max_pt.y - min_pt.y;
            double width = std::max(width_x, width_y);
            
            double min_height = this->get_parameter("min_height").as_double();
            double max_height = this->get_parameter("max_height").as_double();
            double min_width = this->get_parameter("min_width").as_double();
            double max_width = this->get_parameter("max_width").as_double();
            
            RCLCPP_INFO(this->get_logger(), "Cluster dimensions: height=%f, width=%f", height, width);
            RCLCPP_INFO(this->get_logger(), "Thresholds: height=[%f, %f], width=[%f, %f]",
                       min_height, max_height, min_width, max_width);
            
            bool is_human = (height >= min_height && height <= max_height &&
                           width >= min_width && width <= max_width);
            
            if (is_human) {
                RCLCPP_INFO(this->get_logger(), "Cluster matches human-like characteristics");
            } else {
                RCLCPP_INFO(this->get_logger(), "Cluster does not match human-like characteristics");
            }
            
            return is_human;
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in human-like check: %s", e.what());
            return false;
        }
    }
    
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> validateWithCamera(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
        const cv::Mat& image)
    {
        // Simple validation - in real implementation, use YOLO or similar
        // For now, just return all clusters
        return clusters;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr findClosestHuman(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& humans)
    {
        if (humans.empty()) {
            RCLCPP_INFO(this->get_logger(), "No humans to find closest from");
            return nullptr;
        }
        
        double min_distance = std::numeric_limits<double>::max();
        pcl::PointCloud<pcl::PointXYZ>::Ptr closest = nullptr;
        
        for (const auto& human : humans) {
            if (!human || human->empty()) {
                RCLCPP_WARN(this->get_logger(), "Invalid human cluster encountered");
                continue;
            }
            
            try {
                // Calculate centroid
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*human, centroid);
                
                double distance = sqrt(centroid[0]*centroid[0] + centroid[1]*centroid[1]);
                
                if (distance < min_distance) {
                    min_distance = distance;
                    closest = human;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error computing centroid: %s", e.what());
                continue;
            }
        }
        
        if (closest) {
            RCLCPP_INFO(this->get_logger(), "Found closest human at distance: %f", min_distance);
        } else {
            RCLCPP_INFO(this->get_logger(), "No valid closest human found");
        }
        
        return closest;
    }
    
    void publishHumanPosition(const pcl::PointCloud<pcl::PointXYZ>::Ptr& human,
                             const std_msgs::msg::Header& header)
    {
        if (!human || human->empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid human cluster for position publishing");
            return;
        }
        
        try {
            // Calculate centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*human, centroid);
            
            geometry_msgs::msg::PointStamped pos_msg;
            pos_msg.header = header;
            pos_msg.point.x = centroid[0];
            pos_msg.point.y = centroid[1];
            pos_msg.point.z = centroid[2];
            
            human_pos_pub_->publish(pos_msg);
            RCLCPP_INFO(this->get_logger(), "Human position published: x=%f, y=%f, z=%f", 
                       pos_msg.point.x, pos_msg.point.y, pos_msg.point.z);
            
            // Update tracking history
            last_position_ = pos_msg.point;
            last_time_ = this->get_clock()->now();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error publishing human position: %s", e.what());
        }
    }
    
    void estimateAndPublishDirection(const pcl::PointCloud<pcl::PointXYZ>::Ptr& human,
                                   const std_msgs::msg::Header& header)
    {
        if (!human || human->empty()) {
            RCLCPP_WARN(this->get_logger(), "Invalid human cluster for direction estimation");
            return;
        }
        
        try {
            // Calculate centroid
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*human, centroid);
            
            geometry_msgs::msg::Point current_pos;
            current_pos.x = centroid[0];
            current_pos.y = centroid[1];
            current_pos.z = centroid[2];
            
            // Add to history
            auto current_time = this->get_clock()->now();
            position_history_.push_back({current_pos, current_time});
            
            // Keep only recent history (last 2 seconds)
            auto cutoff_time = current_time - rclcpp::Duration::from_seconds(2.0);
            position_history_.erase(
                std::remove_if(position_history_.begin(), position_history_.end(),
                              [cutoff_time](const auto& entry) {
                                  return entry.second < cutoff_time;
                              }),
                position_history_.end());
            
            RCLCPP_INFO(this->get_logger(), "Position history size: %lu", position_history_.size());
            
            // Estimate direction if we have enough history
            if (position_history_.size() >= 2) {
                auto& oldest = position_history_.front();
                auto& newest = position_history_.back();
                
                double dt = (newest.second - oldest.second).seconds();
                
                if (dt > 0.1) {  // At least 100ms difference
                    geometry_msgs::msg::Vector3Stamped dir_msg;
                    dir_msg.header = header;
                    dir_msg.vector.x = (newest.first.x - oldest.first.x) / dt;
                    dir_msg.vector.y = (newest.first.y - oldest.first.y) / dt;
                    dir_msg.vector.z = (newest.first.z - oldest.first.z) / dt;
                    
                    human_dir_pub_->publish(dir_msg);
                    RCLCPP_INFO(this->get_logger(), "Direction published: dx=%f, dy=%f, dz=%f",
                               dir_msg.vector.x, dir_msg.vector.y, dir_msg.vector.z);
                } else {
                    RCLCPP_INFO(this->get_logger(), "Time difference too small for direction estimation: %f", dt);
                }
            } else {
                RCLCPP_INFO(this->get_logger(), "Not enough position history for direction estimation");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in direction estimation: %s", e.what());
        }
    }
    
    void publishVisualizationMarkers(
        const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& humans,
        const std_msgs::msg::Header& header)
    {
        try {
            visualization_msgs::msg::MarkerArray marker_array;
            
            for (size_t i = 0; i < humans.size(); ++i) {
                if (!humans[i] || humans[i]->empty()) {
                    RCLCPP_WARN(this->get_logger(), "Invalid human cluster %lu for marker creation", i);
                    continue;
                }
                
                // Calculate centroid
                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*humans[i], centroid);
                
                visualization_msgs::msg::Marker marker;
                marker.header = header;
                marker.ns = "human_detection";
                marker.id = static_cast<int32_t>(i);
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.action = visualization_msgs::msg::Marker::ADD;
                
                marker.pose.position.x = centroid[0];
                marker.pose.position.y = centroid[1];
                marker.pose.position.z = centroid[2];
                marker.pose.orientation.w = 1.0;
                
                marker.scale.x = 0.5;
                marker.scale.y = 0.5;
                marker.scale.z = 1.8;
                
                marker.color.a = 0.7;
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                
                marker.lifetime = rclcpp::Duration::from_seconds(10);
                
                marker_array.markers.push_back(marker);
                RCLCPP_INFO(this->get_logger(), "Created marker for human %lu at position: x=%f, y=%f, z=%f",
                           i, centroid[0], centroid[1], centroid[2]);
            }
            
            if (!marker_array.markers.empty()) {
                marker_pub_->publish(marker_array);
                RCLCPP_INFO(this->get_logger(), "Published %lu markers", marker_array.markers.size());
            } else {
                RCLCPP_INFO(this->get_logger(), "No valid markers to publish");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in marker publishing: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HumanTracker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}