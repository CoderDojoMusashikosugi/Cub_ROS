#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>
#include <Eigen/Geometry>

namespace cub_mapping {
inline Eigen::Isometry3d interpolate_pose(const Eigen::Isometry3d& a,
                                         const Eigen::Isometry3d& b, double alpha) {
  Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
  out.translation() = (1.0 - alpha) * a.translation() + alpha * b.translation();
  out.linear() = Eigen::Quaterniond(a.linear()).slerp(alpha, Eigen::Quaterniond(b.linear())).toRotationMatrix();
  return out;
}

struct Alignment {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double rmse = 0.0;
  size_t inliers = 0;
};

// Gravity is already observed by LIO. Fit yaw and translation only (no scale).
inline Alignment align_enu(const std::vector<Eigen::Vector3d>& local,
                           const std::vector<Eigen::Vector3d>& enu,
                           size_t min_pairs, double min_baseline, double max_rmse) {
  if (local.size() != enu.size() || local.size() < min_pairs)
    throw std::runtime_error("Insufficient RTK FIX / LIO pairs for ENU alignment");
  std::vector<size_t> indices(local.size());
  for (size_t i = 0; i < indices.size(); ++i) indices[i] = i;
  Alignment result;
  for (int iteration = 0; iteration < 4; ++iteration) {
    Eigen::Vector3d a = Eigen::Vector3d::Zero(), b = a;
    for (auto i : indices) { a += local[i]; b += enu[i]; }
    a /= indices.size(); b /= indices.size();
    double dot = 0, cross = 0, span_a = 0, span_b = 0;
    for (auto i : indices) {
      const Eigen::Vector2d x = (local[i] - a).head<2>(), y = (enu[i] - b).head<2>();
      dot += x.dot(y); cross += x.x() * y.y() - x.y() * y.x();
      span_a = std::max(span_a, x.norm()); span_b = std::max(span_b, y.norm());
    }
    if (2 * std::min(span_a, span_b) < min_baseline || std::hypot(dot, cross) < 1e-9)
      throw std::runtime_error("Insufficient horizontal motion for ENU yaw; increase calibration_duration");
    result.transform.linear() = Eigen::AngleAxisd(std::atan2(cross, dot), Eigen::Vector3d::UnitZ()).toRotationMatrix();
    result.transform.translation() = b - result.transform.linear() * a;
    std::vector<double> errors;
    for (auto i : indices) errors.push_back((result.transform * local[i] - enu[i]).norm());
    auto sorted = errors;
    std::sort(sorted.begin(), sorted.end());
    const double cutoff = std::max(0.5, 3.0 * sorted[sorted.size() / 2]);
    std::vector<size_t> next;
    double sum = 0;
    for (size_t k = 0; k < indices.size(); ++k) {
      sum += errors[k] * errors[k];
      if (errors[k] <= cutoff) next.push_back(indices[k]);
    }
    result.rmse = std::sqrt(sum / indices.size());
    result.inliers = indices.size();
    if (next.size() == indices.size() || iteration == 3) break;
    if (next.size() < min_pairs) throw std::runtime_error("Too few consistent GNSS alignment pairs");
    indices = next;
  }
  if (result.rmse > max_rmse) throw std::runtime_error("GNSS alignment RMSE exceeds configured limit");
  return result;
}
}
