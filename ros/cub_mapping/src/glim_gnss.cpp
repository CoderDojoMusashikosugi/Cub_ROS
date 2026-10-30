#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <set>
#include <nlohmann/json.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/mapping/callbacks.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include "cub_mapping/gnss_factor.hpp"
#include "cub_mapping/gnss_geometry.hpp"

namespace cub_mapping {
using Json = nlohmann::json;
using Fix = sensor_msgs::msg::NavSatFix;
using gtsam::symbol_shorthand::X;

struct Observation {
  double stamp;
  Eigen::Vector3d enu, sigma, arm;
  size_t submap = 0;
  bool matched = false, added = false, rejected = false;
};

class GlimGnss : public glim::ExtensionModuleROS2 {
 public:
  GlimGnss() {
    std::ifstream file(glim::GlobalConfig::get_config_path("config_cub_gnss"));
    if (!file) throw std::runtime_error("Missing config_cub_gnss.json");
    file >> config_;
    const auto origin = config_.at("origin_lla").get<std::vector<double>>();
    if (origin.size() != 3) throw std::runtime_error("origin_lla needs latitude, longitude, ellipsoid height");
    for (double v : origin) if (!std::isfinite(v)) throw std::runtime_error("Non-finite origin");
    earth_.Reset(origin[0], origin[1], origin[2]);
    calibration_ = config_.at("mode") == "calibrate";
    if (!calibration_ && config_.at("mode") != "mapping") throw std::runtime_error("Unknown GNSS mode");
    const std::string altitude = config_.at("altitude_reference");
    if (altitude != "msl" && altitude != "ellipsoid") throw std::runtime_error("altitude_reference must be msl or ellipsoid");
    height_offset_ = altitude == "msl" ? config_.at("geoid_height_m").get<double>() : 0.0;
    for (const auto* name : {"min_interval_s", "max_interpolation_gap_s", "sigma_horizontal_m", "sigma_vertical_m",
                             "max_sigma_m", "huber_k", "information_scale", "max_residual_m", "alignment_min_baseline_m", "alignment_max_rmse_m"}) {
      const double value = config_.at(name);
      if (!std::isfinite(value) || value <= 0) throw std::runtime_error(std::string("Invalid parameter: ") + name);
    }
    if (config_.at("settle_iterations").get<int>() < 0 || config_.at("settle_iterations").get<int>() > 100)
      throw std::runtime_error("settle_iterations must be between 0 and 100");
    if (config_.at("alignment_min_pairs").get<int>() < 3)
      throw std::runtime_error("alignment_min_pairs must be at least 3");
    if (!std::isfinite(height_offset_) || !std::isfinite(config_.at("time_offset_s").get<double>()))
      throw std::runtime_error("Non-finite height/time offset");
    const auto arm = config_.at("antenna_in_imu_m").get<std::vector<double>>();
    if (arm.size() != 3) throw std::runtime_error("antenna_in_imu_m needs xyz");
    arm_imu_ = Eigen::Vector3d(arm[0], arm[1], arm[2]);
    if (!arm_imu_.allFinite()) throw std::runtime_error("Non-finite lever arm");
    if (!calibration_) {
      std::ifstream alignment(config_.at("alignment_file").get<std::string>());
      Json data; alignment >> data;
      if (!data.at("success").get<bool>()) throw std::runtime_error("ENU alignment failed");
      if (data.at("origin_lla") != config_.at("origin_lla")) throw std::runtime_error("Alignment origin mismatch");
      const auto matrix = data.at("T_enu_local").get<std::vector<double>>();
      if (matrix.size() != 16) throw std::runtime_error("Invalid alignment matrix");
      for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) enu_local_.matrix()(r,c) = matrix[r*4+c];
      if (!enu_local_.matrix().allFinite() ||
          (enu_local_.linear().transpose()*enu_local_.linear()-Eigen::Matrix3d::Identity()).norm() > 1e-6 ||
          std::abs(enu_local_.linear().determinant()-1) > 1e-6)
        throw std::runtime_error("Invalid ENU rotation");
      glim::Config global(glim::GlobalConfig::get_config_path("config_global_mapping"));
      if (global.param<bool>("global_mapping", "enable_imu", true))
        throw std::runtime_error("GNSS mapping requires global enable_imu=false; local LIO IMU stays enabled");
    }
    insert_id_ = glim::GlobalMappingCallbacks::on_insert_submap.add([this](const glim::SubMap::ConstPtr& submap) {
      submaps_[submap->id] = submap;
    });
    update_id_ = glim::GlobalMappingCallbacks::on_smoother_update.add(
      [this](gtsam_points::ISAM2Ext& isam, gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
        update(isam, factors, values);
      });
    result_id_ = glim::GlobalMappingCallbacks::on_smoother_update_result.add(
      [this](gtsam_points::ISAM2Ext& isam, const auto&) {
        if (!settle_ || isam.empty()) return;
        for (int i = 0; i < config_.at("settle_iterations").get<int>(); ++i) isam.update();
      });
    spdlog::info("Cub GNSS: mode={} altitude={} offset={}m topic={}",
                 calibration_ ? "calibrate" : "mapping", altitude, height_offset_, config_.at("topic").get<std::string>());
  }
  ~GlimGnss() override {
    glim::GlobalMappingCallbacks::on_insert_submap.remove(insert_id_);
    glim::GlobalMappingCallbacks::on_smoother_update.remove(update_id_);
    glim::GlobalMappingCallbacks::on_smoother_update_result.remove(result_id_);
  }
  std::vector<glim::GenericTopicSubscription::Ptr> create_subscriptions() override {
    return {std::make_shared<glim::TopicSubscription<Fix>>(
      config_.at("topic").get<std::string>(), "sensor_msgs/msg/NavSatFix",
      [this](const Fix::ConstSharedPtr& fix) { receive(*fix); })};
  }

  void receive(const Fix& fix) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++counts_["received"];
    // This mapping is specific to cub_um982: GBAS_FIX is emitted only for quality=4.
    if (fix.status.status != sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX) { ++counts_["not_rtk_fix"]; return; }
    const auto frames = config_.at("accepted_frame_ids").get<std::vector<std::string>>();
    if (std::find(frames.begin(), frames.end(), fix.header.frame_id) == frames.end()) { ++counts_["wrong_frame"]; return; }
    if (!std::isfinite(fix.latitude) || !std::isfinite(fix.longitude) || !std::isfinite(fix.altitude) ||
        std::abs(fix.latitude) > 90 || std::abs(fix.longitude) > 180 || fix.header.stamp.nanosec >= 1000000000) {
      ++counts_["invalid"]; return;
    }
    const double stamp = fix.header.stamp.sec + fix.header.stamp.nanosec * 1e-9 + config_.at("time_offset_s").get<double>();
    if (stamp <= 0) { ++counts_["invalid"]; return; }
    if (stamp <= last_received_) { ++counts_["duplicate_or_out_of_order"]; return; }
    last_received_ = stamp;
    if (stamp - last_selected_ < config_.at("min_interval_s").get<double>()) { ++counts_["decimated"]; return; }
    Eigen::Vector3d sigma(config_.at("sigma_horizontal_m"), config_.at("sigma_horizontal_m"), config_.at("sigma_vertical_m"));
    if (fix.position_covariance_type != Fix::COVARIANCE_TYPE_UNKNOWN) {
      for (int i = 0; i < 3; ++i) {
        const double v = fix.position_covariance[i * 4];
        if (!std::isfinite(v) || v < 0) { ++counts_["invalid_covariance"]; return; }
        sigma[i] = std::max(sigma[i], std::sqrt(v));
      }
    }
    if (sigma.maxCoeff() > config_.at("max_sigma_m").get<double>()) { ++counts_["uncertain"]; return; }
    Observation obs;
    obs.stamp = stamp; obs.sigma = sigma;
    earth_.Forward(fix.latitude, fix.longitude, fix.altitude + height_offset_, obs.enu.x(), obs.enu.y(), obs.enu.z());
    observations_.push_back(obs);
    last_selected_ = stamp;
    ++counts_["selected"];
  }

  void match() {
    for (auto& obs : observations_) {
      if (obs.matched || obs.rejected) continue;
      for (const auto& entry : submaps_) {
        const auto& submap = entry.second;
        const auto& frames = submap->frames;
        if (frames.empty() || obs.stamp < frames.front()->stamp || obs.stamp > frames.back()->stamp) continue;
        auto right = std::lower_bound(frames.begin(), frames.end(), obs.stamp,
          [](const auto& frame, double stamp) { return frame->stamp < stamp; });
        Eigen::Isometry3d pose;
        if (right == frames.begin() || (*right)->stamp == obs.stamp) pose = (*right)->T_world_imu;
        else {
          const auto left = std::prev(right);
          const double gap = (*right)->stamp - (*left)->stamp;
          if (gap <= 0 || gap > config_.at("max_interpolation_gap_s").get<double>()) {
            obs.rejected = true; ++counts_["interpolation_gap"]; break;
          }
          pose = interpolate_pose((*left)->T_world_imu, (*right)->T_world_imu, (obs.stamp - (*left)->stamp) / gap);
        }
        // Submap origin is its centre sensor pose; never use the mutable global pose here.
        obs.arm = submap->origin_frame()->T_world_sensor().inverse() * (pose * arm_imu_);
        obs.submap = entry.first; obs.matched = true; ++counts_["matched"];
        break;
      }
    }
  }

  void update(gtsam_points::ISAM2Ext& isam, gtsam::NonlinearFactorGraph& factors, gtsam::Values& values) {
    std::lock_guard<std::mutex> lock(mutex_);
    settle_ = !calibration_ && values.empty();
    if (!calibration_ && !initialized_ && values.exists(X(0))) {
      const gtsam::Pose3 initial(enu_local_.matrix() * values.at<gtsam::Pose3>(X(0)).matrix());
      values.update(X(0), initial);
      // Global IMU edges are disabled. Local LIO still supplies gravity. Retain
      // a weak yaw/position prior and a roll/pitch prior to preserve its vertical.
      gtsam::Vector6 sigmas; sigmas << 0.02, 0.02, 10.0, 1000.0, 1000.0, 1000.0;
      factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(X(0), initial, gtsam::noiseModel::Diagonal::Sigmas(sigmas));
      initialized_ = true;
    }
    match();
    if (calibration_) return;
    for (auto& obs : observations_) {
      if (!obs.matched || obs.added || obs.rejected) continue;
      const auto key = X(obs.submap);
      if (!values.exists(key) && !isam.valueExists(key)) continue;
      const auto pose = values.exists(key) ? values.at<gtsam::Pose3>(key) : isam.calculateEstimate<gtsam::Pose3>(key);
      if ((pose.transformFrom(obs.arm) - obs.enu).norm() > config_.at("max_residual_m").get<double>()) {
        obs.rejected = true; ++counts_["residual_gate"]; continue;
      }
      // VGICP accumulates unnormalised point costs. Scale the entire GNSS
      // robust cost while preserving the Huber transition in physical units.
      const double gain = std::sqrt(config_.at("information_scale").get<double>());
      const auto noise = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config_.at("huber_k").get<double>() * gain),
        gtsam::noiseModel::Diagonal::Sigmas(obs.sigma / gain));
      factors.emplace_shared<GnssArmFactor>(key, obs.enu, obs.arm, noise);
      obs.added = true; ++counts_["factors"];
    }
  }

  void at_exit(const std::string& path) override {
    std::lock_guard<std::mutex> lock(mutex_);
    match();
    Json report = config_;
    report["counts"] = counts_;
    report["success"] = false;
    report["map_frame"] = calibration_ ? "local" : "ENU";
    if (calibration_) {
      std::vector<Eigen::Vector3d> local, enu;
      for (const auto& obs : observations_) if (obs.matched && !obs.rejected) {
        local.push_back(submaps_.at(obs.submap)->T_world_origin * obs.arm);
        enu.push_back(obs.enu);
      }
      try {
        const auto fit = align_enu(local, enu, config_.at("alignment_min_pairs"),
                                  config_.at("alignment_min_baseline_m"), config_.at("alignment_max_rmse_m"));
        std::vector<double> matrix;
        for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) matrix.push_back(fit.transform.matrix()(r,c));
        report["T_enu_local"] = matrix;
        report["alignment_rmse_m"] = fit.rmse;
        report["alignment_inliers"] = fit.inliers;
        report["success"] = true;
        spdlog::info("Cub GNSS alignment: {} pairs, RMSE={}m", fit.inliers, fit.rmse);
      } catch (const std::exception& e) {
        report["error"] = e.what(); spdlog::error("Cub GNSS alignment: {}", e.what());
      }
    } else {
      report["success"] = counts_["factors"] > 0;
      std::ofstream csv(path + "/gnss_residuals.csv");
      csv << "stamp,submap,east,north,up,error_e,error_n,error_u,used\n" << std::setprecision(15);
      double sum = 0; size_t count = 0;
      for (const auto& obs : observations_) if (obs.matched) {
        const Eigen::Vector3d error = submaps_.at(obs.submap)->T_world_origin * obs.arm - obs.enu;
        csv << obs.stamp << ',' << obs.submap << ',' << obs.enu.x() << ',' << obs.enu.y() << ',' << obs.enu.z()
            << ',' << error.x() << ',' << error.y() << ',' << error.z() << ',' << obs.added << '\n';
        if (obs.added) { sum += error.squaredNorm(); ++count; }
      }
      if (count) report["residual_rmse_m"] = std::sqrt(sum / count);
      spdlog::info("Cub GNSS: {} factors, residual RMSE={}m", count, count ? std::sqrt(sum/count) : -1.0);
    }
    size_t unmatched = 0;
    for (const auto& obs : observations_) if (!obs.matched && !obs.rejected) ++unmatched;
    report["counts"]["unmatched"] = unmatched;
    std::ofstream output(path + "/gnss_report.json");
    output << report.dump(2) << '\n';
    if (!output) throw std::runtime_error("Failed to save GNSS report");
  }
 private:
  Json config_;
  GeographicLib::LocalCartesian earth_;
  Eigen::Vector3d arm_imu_;
  Eigen::Isometry3d enu_local_ = Eigen::Isometry3d::Identity();
  bool calibration_ = false, initialized_ = false, settle_ = false;
  double height_offset_ = 0, last_selected_ = -1e100, last_received_ = -1e100;
  int insert_id_ = -1, update_id_ = -1, result_id_ = -1;
  std::mutex mutex_;
  std::map<std::string, size_t> counts_;
  std::vector<Observation> observations_;
  std::map<size_t, glim::SubMap::ConstPtr> submaps_;
};
}

extern "C" glim::ExtensionModule* create_extension_module() {
  return new cub_mapping::GlimGnss();
}
