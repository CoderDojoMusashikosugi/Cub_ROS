#include "nav_manager/simple_localmap_creator.h"

namespace localmap_creator{
SimpleLocalmapCreator::SimpleLocalmapCreator(void)
: local_nh_("~"){
    localmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);
    localmap_expand_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map/expand", 1);
    cloud_sub_ = nh_.subscribe("/velodyne_obstacles", 1, &SimpleLocalmapCreator::cloud_callback, this, ros::TransportHints().reliable().tcpNoDelay());

    local_nh_.param("width", width_, 20.0);
    local_nh_.param("resolution", resolution_, 0.1);
    local_nh_.param("max_height", max_height_, 1.5);
    local_nh_.param("min_height", min_height_, 0.05);
    grid_width_ = std::round(width_ / resolution_);
    grid_width_2_ = grid_width_ / 2;
    range_ = width_ * 0.5;
    grid_size_ = grid_width_ * grid_width_;
    local_nh_.param("expand_radius", expand_radius_, 0.3);
    ROS_INFO_STREAM("width: " << width_);
    ROS_INFO_STREAM("max_height: " << max_height_);
    ROS_INFO_STREAM("min_height: " << min_height_);
    ROS_INFO_STREAM("resolution: " << resolution_);
    ROS_INFO_STREAM("grid_width_: " << grid_width_);
    ROS_INFO_STREAM("grid_width_2_: " << grid_width_2_);
    ROS_INFO_STREAM("range: " << range_);
    ROS_INFO_STREAM("grid_size: " << grid_size_);
    ROS_INFO_STREAM("expand_radius: " << expand_radius_);
}

void SimpleLocalmapCreator::process(void)
{
    ros::spin();
}

void SimpleLocalmapCreator::cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    PointCloudTypePtr cloud_ptr(new PointCloudType);
    pcl::fromROSMsg(*msg, *cloud_ptr);

    // initialize localmap
    nav_msgs::OccupancyGrid localmap;
    localmap.header = msg->header;
    localmap.info.height = grid_width_;
    localmap.info.width = grid_width_;
    localmap.info.resolution = resolution_;
    localmap.info.origin.position.x = -width_ * 0.5;
    localmap.info.origin.position.y = -width_ * 0.5;
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    tf2::convert(q, localmap.info.origin.orientation);
    localmap.data.resize(grid_size_, 0);

    // downsampling
    pcl::VoxelGrid<PointType> vg;
    vg.setInputCloud(cloud_ptr);
    vg.setLeafSize(resolution_, resolution_, resolution_);
    vg.filter(*cloud_ptr);

    // set cost
    const double range_squared = range_ * range_;
    for(const auto& p : cloud_ptr->points){
        const double distance_squared = p.x * p.x + p.y * p.y;
        if(distance_squared > range_squared){
            continue;
        }
        if(p.z < min_height_ || max_height_ < p.z){
            continue;
        }
        const int index = get_index_from_xy(p.x, p.y);
        if(0 <= index && index < grid_size_){
            localmap.data[index] = 100;
        }
    }
    localmap_pub_.publish(localmap);

    // expand
    nav_msgs::OccupancyGrid localmap_expand;
    localmap_expand = localmap;
    const int expand_radius_grid = std::round(expand_radius_ / resolution_);
    for(unsigned int i=0;i<grid_size_;++i){
        if(localmap.data[i] == 100) {
            int x = i%(int)grid_width_;
            int y = i/(int)grid_width_;
            for(int j=-expand_radius_grid; j<=expand_radius_grid; j++) {
                for(int k=-expand_radius_grid; k<=expand_radius_grid; k++) {
                    int index = (y+j)*grid_width_ + (x+k);
                    int index_x = index % (int)grid_width_;
                    int index_y = index / (int)grid_width_;
                    if(index >= 0 && index < grid_size_ && index_x >= x-expand_radius_grid &&
                       index_x <= x+expand_radius_grid && index_y >= y-expand_radius_grid &&
                       index_y <= y+expand_radius_grid) {
                        localmap_expand.data[index] = 100;
                    }

                }
            }
        }
    }
    localmap_expand_pub_.publish(localmap_expand);
}

int SimpleLocalmapCreator::get_index_from_xy(const double x, const double y)
{
    const int _x = floor(x / resolution_ + 0.5) + grid_width_2_;
    const int _y = floor(y / resolution_ + 0.5) + grid_width_2_;
    return _y * grid_width_ + _x;
}

int SimpleLocalmapCreator::get_x_index_from_index(const int index)
{
    return index % grid_width_;
}

int SimpleLocalmapCreator::get_y_index_from_index(const int index)
{
    return index / grid_width_;
}

double SimpleLocalmapCreator::get_x_from_index(const int index)
{
    return (get_x_index_from_index(index) - grid_width_2_) * resolution_;
}

double SimpleLocalmapCreator::get_y_from_index(const int index)
{
    return (get_y_index_from_index(index) - grid_width_2_) * resolution_;
}

bool SimpleLocalmapCreator::is_valid_point(int ix, int iy)
{
    if(ix < 0 || iy < 0){
        return false;
    }else if(ix >= grid_width_ || iy >= grid_width_){
        return false;
    }
    return true;
}

}// namespace localmap_creator

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_localmap_creator");
    localmap_creator::SimpleLocalmapCreator slc;
    slc.process();
    return 0;
}
