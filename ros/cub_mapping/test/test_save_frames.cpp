#include <gtest/gtest.h>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module.hpp>
#include <glim/odometry/callbacks.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

TEST(SaveFrames, ConvertsToLidarAndIndexesByFinalTimestamp) {
  namespace fs = std::filesystem;
  char pattern[] = "/tmp/cub_frame_test_XXXXXX";
  const fs::path root(mkdtemp(pattern));
  std::ofstream(root/"config.json") << R"({"global":{}})";
  std::ofstream(root/"config_save_frames.json") << nlohmann::json{
    {"save_frames",{{"directory",(root/"frames").string()},{"sensor","mid360"}}}};
  glim::GlobalConfig::instance(root.string(), true);
  auto plugin = glim::ExtensionModule::load_module("libcub_mapping_save_frames.so");
  ASSERT_TRUE(plugin);
  auto frame = std::make_shared<glim::EstimationFrame>();
  frame->id = 30; frame->stamp = 100; frame->frame_id = glim::FrameID::IMU;
  frame->T_lidar_imu = Eigen::Isometry3d::Identity();
  frame->T_lidar_imu.translation() = Eigen::Vector3d(.1,.2,.3);
  frame->T_lidar_imu.linear() = Eigen::AngleAxisd(1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  std::vector<Eigen::Vector4d> points{{1,2,3,1},{4,5,6,1}};
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  cloud->add_intensities(std::vector<double>{7,8}); frame->frame = cloud;
  glim::OdometryEstimationCallbacks::on_new_frame(frame);
  frame->id = 80; frame->stamp = 101;
  glim::OdometryEstimationCallbacks::on_new_frame(frame);
  frame->id = 81; frame->stamp = 102;
  glim::OdometryEstimationCallbacks::on_new_frame(frame);
  // Frame IDs intentionally differ from trajectory row indices.
  std::ofstream(root/"traj_lidar.txt") << "100 60 30 2 0 0 0 1\n101 61 30 2 0 0 0 1\n";
  plugin->at_exit(root.string());
  EXPECT_TRUE(fs::exists(root/"frames/00000000/points_compact.bin"));
  EXPECT_TRUE(fs::exists(root/"frames/00000001/points_compact.bin"));
  EXPECT_FALSE(fs::exists(root/"frames/pending"));
  EXPECT_TRUE(fs::exists(root/"frames/unreferenced_tail/81/points_compact.bin"));
  EXPECT_EQ(fs::file_size(root/"frames/00000000/points_compact.bin"), 24u);
  std::ifstream input(root/"frames/00000000/points_compact.bin", std::ios::binary);
  Eigen::Vector3f actual; input.read(reinterpret_cast<char*>(actual.data()), 12);
  const Eigen::Vector3f expected = (frame->T_lidar_imu * points[0]).head<3>().cast<float>();
  EXPECT_LT((actual-expected).norm(), 1e-6);
  nlohmann::json report; std::ifstream(root/"frames/report.json") >> report;
  EXPECT_EQ(report["frames"], 2); EXPECT_EQ(report["points"], 4);
  EXPECT_EQ(report["sensor"], "mid360");
  EXPECT_EQ(report["unreferenced_tail_frames"], 1);
  plugin.reset(); fs::remove_all(root);
}

TEST(SaveFrames, MissingFinalScanDoesNotProduceSuccessReport) {
  namespace fs = std::filesystem;
  char pattern[] = "/tmp/cub_frame_missing_XXXXXX";
  const fs::path root(mkdtemp(pattern));
  std::ofstream(root/"config.json") << R"({"global":{}})";
  std::ofstream(root/"config_save_frames.json") << nlohmann::json{
    {"save_frames",{{"directory",(root/"frames").string()},{"sensor","mid360"}}}};
  glim::GlobalConfig::instance(root.string(), true);
  auto plugin = glim::ExtensionModule::load_module("libcub_mapping_save_frames.so");
  ASSERT_TRUE(plugin);
  std::ofstream(root/"traj_lidar.txt") << "100 0 0 0 0 0 0 1\n";
  EXPECT_THROW(plugin->at_exit(root.string()), std::runtime_error);
  EXPECT_FALSE(fs::exists(root/"frames/report.json"));
  plugin.reset(); fs::remove_all(root);
}
