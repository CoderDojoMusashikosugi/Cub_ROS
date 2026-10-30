#include "cuda/pointcloud_preprocessor.cuh"

#include <cuda_runtime.h>
#include <thrust/device_ptr.h>
#include <thrust/sort.h>
#include <thrust/unique.h>
#include <thrust/execution_policy.h>

#include <iostream>
#include <cstdint>
#include <cmath>

namespace ekf_localizer {
namespace cuda {

struct Transform4x4 {
    float m[16];
};

__global__ void filter_transform_voxel_kernel(
    const pcl::PointXYZI* __restrict__ in_pts,
    int n,
    Transform4x4 tf,
    float limit_xy,
    float limit_z,
    float min_range_sq,
    float min_height,
    float inv_voxel_size,
    uint64_t* __restrict__ out_keys,
    pcl::PointXYZI* __restrict__ out_pts,
    int* __restrict__ out_count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= n) return;

    const pcl::PointXYZI pt = in_pts[idx];
    float x = pt.x;
    float y = pt.y;
    float z = pt.z;

    if (!isfinite(x) || !isfinite(y) || !isfinite(z)) return;

    // Apply 4x4 matrix: [R|T]
    // Row-major: m[0..3] = row 0, m[4..7] = row 1, m[8..11] = row 2
    float tx = tf.m[0] * x + tf.m[1] * y + tf.m[2] * z + tf.m[3];
    float ty = tf.m[4] * x + tf.m[5] * y + tf.m[6] * z + tf.m[7];
    float tz = tf.m[8] * x + tf.m[9] * y + tf.m[10] * z + tf.m[11];

    if (tx * tx + ty * ty < min_range_sq) return;
    if (tz < min_height) return;
    if (fabsf(tx) > limit_xy || fabsf(ty) > limit_xy || fabsf(tz) > limit_z) return;

    int64_t vx = static_cast<int64_t>(floorf(tx * inv_voxel_size)) + 1048576;
    int64_t vy = static_cast<int64_t>(floorf(ty * inv_voxel_size)) + 1048576;
    int64_t vz = static_cast<int64_t>(floorf(tz * inv_voxel_size)) + 1048576;
    uint64_t key = (static_cast<uint64_t>(vx & 0x1FFFFF) << 42) |
                   (static_cast<uint64_t>(vy & 0x1FFFFF) << 21) |
                   static_cast<uint64_t>(vz & 0x1FFFFF);

    int dst = atomicAdd(out_count, 1);

    out_keys[dst] = key;
    pcl::PointXYZI p_out;
    p_out.x = tx;
    p_out.y = ty;
    p_out.z = tz;
    p_out.intensity = pt.intensity;
    out_pts[dst] = p_out;
}

struct PointCloudPreprocessorCUDA::Impl {
    pcl::PointXYZI* d_in_pts = nullptr;
    uint64_t* d_keys = nullptr;
    pcl::PointXYZI* d_out_pts = nullptr;
    int* d_count = nullptr;
    size_t capacity = 0;
    cudaStream_t stream = nullptr;

    Impl() {
        cudaStreamCreate(&stream);
        reallocate(65536);
    }

    ~Impl() {
        free_buffers();
        if (stream) {
            cudaStreamDestroy(stream);
        }
    }

    void free_buffers() {
        if (d_in_pts) { cudaFree(d_in_pts); d_in_pts = nullptr; }
        if (d_keys) { cudaFree(d_keys); d_keys = nullptr; }
        if (d_out_pts) { cudaFree(d_out_pts); d_out_pts = nullptr; }
        if (d_count) { cudaFree(d_count); d_count = nullptr; }
        capacity = 0;
    }

    void reallocate(size_t new_cap) {
        if (new_cap <= capacity) return;
        free_buffers();
        capacity = new_cap;
        cudaMalloc(&d_in_pts, capacity * sizeof(pcl::PointXYZI));
        cudaMalloc(&d_keys, capacity * sizeof(uint64_t));
        cudaMalloc(&d_out_pts, capacity * sizeof(pcl::PointXYZI));
        cudaMalloc(&d_count, sizeof(int));
    }
};

PointCloudPreprocessorCUDA::PointCloudPreprocessorCUDA()
    : impl_(std::make_unique<Impl>()) {}

PointCloudPreprocessorCUDA::~PointCloudPreprocessorCUDA() = default;

void PointCloudPreprocessorCUDA::process(
    const PointCloudType& input_cloud,
    const Eigen::Matrix4f& transform,
    float limit_range_xy,
    float limit_range_z,
    float voxel_size,
    PointCloudType& output_cloud,
    float min_range,
    float min_height)
{
    const size_t n = input_cloud.points.size();
    if (n == 0) {
        output_cloud.clear();
        return;
    }

    if (n > impl_->capacity) {
        impl_->reallocate(n * 2);
    }

    // Zero out count
    cudaMemsetAsync(impl_->d_count, 0, sizeof(int), impl_->stream);

    // Copy input points to device
    cudaMemcpyAsync(
        impl_->d_in_pts,
        input_cloud.points.data(),
        n * sizeof(pcl::PointXYZI),
        cudaMemcpyHostToDevice,
        impl_->stream);

    Transform4x4 tf;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            tf.m[r * 4 + c] = transform(r, c);
        }
    }

    float inv_voxel_size = (voxel_size > 1e-4f) ? (1.0f / voxel_size) : 1.0f;
    float min_range_sq = min_range * min_range;
    int threads = 256;
    int blocks = (n + threads - 1) / threads;

    filter_transform_voxel_kernel<<<blocks, threads, 0, impl_->stream>>>(
        impl_->d_in_pts,
        static_cast<int>(n),
        tf,
        limit_range_xy,
        limit_range_z,
        min_range_sq,
        min_height,
        inv_voxel_size,
        impl_->d_keys,
        impl_->d_out_pts,
        impl_->d_count);

    int valid_count = 0;
    cudaMemcpyAsync(
        &valid_count,
        impl_->d_count,
        sizeof(int),
        cudaMemcpyDeviceToHost,
        impl_->stream);
    cudaStreamSynchronize(impl_->stream);

    if (valid_count <= 0) {
        output_cloud.clear();
        return;
    }

    int final_count = valid_count;

    if (voxel_size > 1e-4f) {
        thrust::device_ptr<uint64_t> dev_keys(impl_->d_keys);
        thrust::device_ptr<pcl::PointXYZI> dev_pts(impl_->d_out_pts);

        thrust::sort_by_key(
            thrust::cuda::par.on(impl_->stream),
            dev_keys,
            dev_keys + valid_count,
            dev_pts);

        auto end_pair = thrust::unique_by_key(
            thrust::cuda::par.on(impl_->stream),
            dev_keys,
            dev_keys + valid_count,
            dev_pts);

        final_count = static_cast<int>(end_pair.second - dev_pts);
    }

    output_cloud.resize(final_count);
    cudaMemcpyAsync(
        output_cloud.points.data(),
        impl_->d_out_pts,
        final_count * sizeof(pcl::PointXYZI),
        cudaMemcpyDeviceToHost,
        impl_->stream);
    cudaStreamSynchronize(impl_->stream);

    output_cloud.width = final_count;
    output_cloud.height = 1;
    output_cloud.is_dense = true;
}

} // namespace cuda
} // namespace ekf_localizer

