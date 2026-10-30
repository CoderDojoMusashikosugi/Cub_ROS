#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <memory>

namespace ekf_localizer {
namespace cuda {

class PointCloudPreprocessorCUDA {
public:
    using PointType = pcl::PointXYZI;
    using PointCloudType = pcl::PointCloud<PointType>;

    PointCloudPreprocessorCUDA();
    ~PointCloudPreprocessorCUDA();

    /**
     * @brief GPU-accelerated range filter, coordinate transform, and voxel grid downsampling.
     *
     * @param input_cloud     Raw input point cloud from LiDAR
     * @param transform       4x4 transform matrix (e.g. sensor -> base_link)
     * @param limit_range_xy  Max distance in XY plane [m]
     * @param limit_range_z   Max distance in Z direction [m]
     * @param voxel_size      Voxel leaf size for downsampling [m]
     * @param output_cloud    Filtered, transformed, and downsampled point cloud
     * @param min_range       Min horizontal distance to remove ego vehicle points [m]
     * @param min_height      Min Z height in transformed frame (e.g. base_link) to remove ground [m]
     */
    void process(
        const PointCloudType& input_cloud,
        const Eigen::Matrix4f& transform,
        float limit_range_xy,
        float limit_range_z,
        float voxel_size,
        PointCloudType& output_cloud,
        float min_range = 0.0f,
        float min_height = -999.0f);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace cuda
} // namespace ekf_localizer

