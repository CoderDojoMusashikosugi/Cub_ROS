#ifndef __SIMPLE_LOCALMAP_CREATOR_H
#define __SIMPLE_LOCALMAP_CREATOR_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <tf2/utils.h>

namespace localmap_creator
{
class SimpleLocalmapCreator
{
public:
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PointCloudType;
    typedef pcl::PointCloud<PointType>::Ptr PointCloudTypePtr;

    SimpleLocalmapCreator(void);
    void process(void);
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    int get_index_from_xy(const double x, const double y);
    int get_x_index_from_index(const int index);
    int get_y_index_from_index(const int index);
    double get_x_from_index(const int index);
    double get_y_from_index(const int index);
    bool is_valid_point(int ix, int iy);

protected:
    ros::NodeHandle nh_;
    ros::NodeHandle local_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher localmap_pub_;
    ros::Publisher localmap_expand_pub_;
    double width_;
    double max_height_;
    double min_height_;
    double resolution_;
    unsigned int grid_width_;
    unsigned int grid_width_2_;
    double range_;
    double grid_size_;
    double expand_radius_;
};
}
#endif// __SIMPLE_LOCALMAP_CREATOR_H
