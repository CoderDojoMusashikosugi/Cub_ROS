// Save GLIM's already deskewed, preprocessed scans in the LiDAR frame.
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <cmath>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/odometry/callbacks.hpp>

namespace fs = std::filesystem;
namespace cub_mapping {
class SaveFrames : public glim::ExtensionModule {
 public:
  SaveFrames() {
    glim::Config config(glim::GlobalConfig::get_config_path("config_save_frames"));
    directory_ = config.param_cast<std::string>("save_frames", "directory");
    sensor_ = config.param_cast<std::string>("save_frames", "sensor");
    if (fs::exists(directory_)) throw std::runtime_error("Frame output already exists: " + directory_.string());
    fs::create_directories(directory_ / "pending");
    callback_ = glim::OdometryEstimationCallbacks::on_new_frame.add(
      [this](const glim::EstimationFrame::ConstPtr& f) { save(f); });
  }
  ~SaveFrames() override { glim::OdometryEstimationCallbacks::on_new_frame.remove(callback_); }

  void at_exit(const std::string& dump) override {
    std::ifstream trajectory(fs::path(dump) / "traj_lidar.txt");
    if (!trajectory) throw std::runtime_error("No final traj_lidar.txt for frame indexing");
    std::ofstream manifest(directory_ / "manifest.csv");
    manifest << "row,timestamp,frame_id,points\n" << std::setprecision(17);
    std::string line; size_t row = 0, total = 0; double last_stamp = 0;
    while (std::getline(trajectory, line)) {
      std::istringstream in(line); double stamp;
      if (!(in >> stamp)) throw std::runtime_error("Invalid trajectory row");
      auto it = saved_.lower_bound(stamp - 1e-7);
      if (it == saved_.end() || std::abs(it->first - stamp) > 1e-7)
        throw std::runtime_error("Missing scan at final trajectory row " + std::to_string(row));
      const auto record = it->second;
      const auto target = directory_ / padded(row);
      fs::rename(directory_ / "pending" / std::to_string(record.id), target);
      manifest << row << ',' << stamp << ',' << record.id << ',' << record.count << '\n';
      total += record.count; ++row; last_stamp = stamp;
      saved_.erase(it);
    }
    if (!manifest || !row) throw std::runtime_error("Empty or unwritable frame manifest");
    manifest.close();
    if (!manifest) throw std::runtime_error("Cannot finish frame manifest");
    // The odometry callback can emit a final scan that never enters the final
    // mapping trajectory. Preserve trailing scans separately, never pair them
    // with an invented pose. Missing scans inside the trajectory remain errors.
    if (!saved_.empty() && saved_.begin()->first <= last_stamp + 1e-7)
      throw std::runtime_error("Saved scan inside final trajectory is unreferenced");
    if (!saved_.empty()) fs::rename(directory_ / "pending", directory_ / "unreferenced_tail");
    else fs::remove(directory_ / "pending");
    nlohmann::json report{{"success",true},{"frames",row},{"points",total},
      {"sensor",sensor_},{"coordinates","deskewed LiDAR-local metres"},{"source","GLIM preprocessed estimation frames"},
      {"trajectory","traj_lidar.txt"},{"unreferenced_tail_frames",saved_.size()}};
    std::ofstream output(directory_ / "report.json"); output << report.dump(2) << '\n';
    if (!output) throw std::runtime_error("Cannot save frame report");
    spdlog::info("Saved {} scans ({} points), indexed against final LiDAR trajectory", row, total);
  }
 private:
  static std::string padded(size_t n) { std::ostringstream s; s << std::setw(8) << std::setfill('0') << n; return s.str(); }
  struct Record {long id; size_t count;};
  void save(const glim::EstimationFrame::ConstPtr& f) {
    if (!f->frame || !f->frame->has_points() || !f->frame->size())
      throw std::runtime_error("Cannot save empty GLIM scan");
    if (saved_.count(f->stamp)) throw std::runtime_error("Duplicate GLIM scan timestamp");
    Eigen::Isometry3d to_lidar = Eigen::Isometry3d::Identity();
    if (f->frame_id == glim::FrameID::IMU) to_lidar = f->T_lidar_imu;
    else if (f->frame_id != glim::FrameID::LIDAR) throw std::runtime_error("Unsupported GLIM point frame");
    const auto dir = directory_ / "pending" / std::to_string(f->id);
    fs::create_directory(dir);
    std::ofstream xyz(dir / "points_compact.bin", std::ios::binary);
    std::ofstream intensity(dir / "intensities_compact.bin", std::ios::binary);
    for (size_t i = 0; i < f->frame->size(); ++i) {
      const Eigen::Vector3f p = (to_lidar * f->frame->points[i]).head<3>().cast<float>();
      const float value = f->frame->has_intensities() ? f->frame->intensities[i] : 0.0f;
      if (!p.allFinite() || !std::isfinite(value)) throw std::runtime_error("Non-finite saved scan");
      xyz.write(reinterpret_cast<const char*>(p.data()), 3 * sizeof(float));
      intensity.write(reinterpret_cast<const char*>(&value), sizeof(float));
    }
    xyz.close(); intensity.close();
    if (!xyz || !intensity) throw std::runtime_error("Failed writing scan (disk full?)");
    saved_[f->stamp] = {f->id, f->frame->size()};
  }
  fs::path directory_;
  std::string sensor_;
  int callback_;
  std::map<double, Record> saved_;
};
}
extern "C" glim::ExtensionModule* create_extension_module() { return new cub_mapping::SaveFrames(); }
