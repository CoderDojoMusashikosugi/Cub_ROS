#include <gtest/gtest.h>
#include <gtsam/base/serialization.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <GeographicLib/LocalCartesian.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <glim/util/config.hpp>
#include <glim/util/extension_module_ros2.hpp>
#include <glim/mapping/callbacks.hpp>
#include <gtsam_points/optimizers/isam2_ext.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "cub_mapping/gnss_factor.hpp"
#include "cub_mapping/gnss_geometry.hpp"

extern "C" glim::ExtensionModule* create_extension_module();

TEST(Gnss, GeoidAndOrigin) {
  GeographicLib::LocalCartesian earth(36.082628231, 140.076144739, 67.078);
  double e, n, u;
  earth.Forward(36.082628231, 140.076144739, 26.9307 + 40.1473, e, n, u);
  EXPECT_NEAR(e, 0, 1e-8); EXPECT_NEAR(n, 0, 1e-8); EXPECT_NEAR(u, 0, 1e-8);
  earth.Forward(36.082628231, 140.076144739, 26.9307, e, n, u);
  EXPECT_NEAR(u, -40.1473, 1e-6);
}

TEST(Gnss, ArmRotationJacobianAndSerialization) {
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.1, -0.2, 1.5), gtsam::Point3(2,3,4));
  const gtsam::Point3 arm(-0.09, 0.100608736, -0.382140249);
  auto factor = std::make_shared<cub_mapping::GnssArmFactor>(0, pose.transformFrom(arm), arm,
    gtsam::noiseModel::Isotropic::Sigma(3, 0.2));
  gtsam::Matrix H;
  EXPECT_LT(factor->evaluateError(pose, H).norm(), 1e-10);
  const auto numerical = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
    [&](const gtsam::Pose3& p) { return factor->evaluateError(p); }, pose);
  EXPECT_LT((H - numerical).norm(), 1e-6);
  gtsam::NonlinearFactorGraph graph; graph.add(factor);
  const auto serialized = gtsam::serializeBinary(graph);
  gtsam::NonlinearFactorGraph restored; gtsam::deserializeBinary(serialized, restored);
  auto loaded = std::dynamic_pointer_cast<cub_mapping::GnssArmFactor>(restored[0]);
  ASSERT_TRUE(loaded);
  const gtsam::Pose3 changed(gtsam::Rot3::Rz(0.7), gtsam::Point3(-1,2,1));
  EXPECT_LT((factor->evaluateError(changed) - loaded->evaluateError(changed)).norm(), 1e-12);
}

TEST(Gnss, InterpolationAndAlignment) {
  Eigen::Isometry3d a = Eigen::Isometry3d::Identity(), b = a;
  b.translation().x() = 2;
  b.linear() = Eigen::AngleAxisd(1, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto midpoint = cub_mapping::interpolate_pose(a, b, 0.5);
  EXPECT_NEAR(midpoint.translation().x(), 1, 1e-12);
  EXPECT_NEAR(Eigen::AngleAxisd(midpoint.linear()).angle(), 0.5, 1e-12);
  Eigen::Isometry3d transform = b; transform.translation() = Eigen::Vector3d(60,30,-1);
  std::vector<Eigen::Vector3d> local, enu;
  for (int i = 0; i < 60; ++i) {
    local.emplace_back(i * 0.5, 2 * std::sin(i * 0.1), 0.1 * i);
    enu.push_back(transform * local.back());
  }
  enu[10] += Eigen::Vector3d(12,-10,20);
  auto fit = cub_mapping::align_enu(local, enu, 20, 10, 1.5);
  EXPECT_LT((fit.transform.matrix() - transform.matrix()).norm(), 1e-8);
  EXPECT_EQ(fit.inliers, 59u);
  std::vector<Eigen::Vector3d> stationary(25, Eigen::Vector3d::Zero());
  EXPECT_THROW(cub_mapping::align_enu(stationary, stationary, 20, 10, 1.5), std::runtime_error);
  EXPECT_THROW(cub_mapping::align_enu({}, {}, 20, 10, 1.5), std::runtime_error);
}

TEST(Gnss, PluginSelectionLateDeliveryAndMapping) {
  using Json = nlohmann::json;
  using Fix = sensor_msgs::msg::NavSatFix;
  char pattern[] = "/tmp/cub_gnss_test_XXXXXX";
  const std::filesystem::path dir(mkdtemp(pattern));
  Json config; std::ifstream(GNSS_CONFIG_PATH) >> config;
  // Synthetic samples at 1 Hz require an interpolation interval >= 1 s.
  config["max_interpolation_gap_s"] = 1.1;
  std::ofstream(dir / "config.json") << Json({{"global", {{"config_path", dir.string()}}}});
  std::ofstream(dir / "config_global_mapping.json") << Json({{"global_mapping", {{"enable_imu", false}}}});
  std::ofstream(dir / "config_cub_gnss.json") << config;
  glim::GlobalConfig::instance(dir.string(), true);
  GeographicLib::LocalCartesian earth(36.082628231, 140.076144739, 67.078);
  Eigen::Isometry3d enu_local = Eigen::Isometry3d::Identity();
  enu_local.linear() = Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  enu_local.translation() = Eigen::Vector3d(60,30,2);
  auto submap = std::make_shared<glim::SubMap>(); submap->id = 0;
  for (int i = 0; i < 61; ++i) {
    auto frame = std::make_shared<glim::EstimationFrame>();
    frame->stamp = 100 + i;
    frame->frame_id = glim::FrameID::IMU;
    frame->T_world_imu = Eigen::Isometry3d::Identity();
    frame->T_world_imu.translation() = Eigen::Vector3d(i, std::sin(i*0.1), 0);
    frame->T_world_lidar = frame->T_world_imu;
    submap->frames.push_back(frame);
  }
  const auto local_origin = submap->origin_frame()->T_world_sensor();
  const auto arm_config = config["antenna_in_imu_m"].get<std::vector<double>>();
  const Eigen::Vector3d arm(arm_config[0], arm_config[1], arm_config[2]);
  for (bool mapping : {false, true}) {
    config["mode"] = mapping ? "mapping" : "calibrate";
    config["alignment_file"] = (dir / "gnss_report.json").string();
    std::ofstream(dir / "config_cub_gnss.json") << config;
    submap->T_world_origin = local_origin;
    std::unique_ptr<glim::ExtensionModule> module(create_extension_module());
    auto* ros = dynamic_cast<glim::ExtensionModuleROS2*>(module.get());
    ASSERT_NE(ros, nullptr);
    auto subscription = ros->create_subscriptions().at(0);
    rclcpp::Serialization<Fix> serialization;
    auto send = [&](const Fix& fix) {
      rclcpp::SerializedMessage serialized;
      serialization.serialize_message(&fix, &serialized);
      subscription->insert_message_instance(serialized, "sensor_msgs/msg/NavSatFix");
    };
    gtsam_points::ISAM2Ext isam;
    gtsam::NonlinearFactorGraph factors;
    gtsam::Values values;
    values.insert(gtsam::Symbol('x', 0), gtsam::Pose3(local_origin.matrix()));
    glim::GlobalMappingCallbacks::on_insert_submap(submap);
    // The submap arrives before any fixes. The next update must still attach them.
    glim::GlobalMappingCallbacks::on_smoother_update(isam, factors, values);
    for (int i = 0; i < 61; ++i) {
      Fix fix; fix.header.stamp.sec = 100 + i; fix.header.frame_id = "gnss_secondary_link";
      fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
      const Eigen::Vector3d p = enu_local * (submap->frames[i]->T_world_imu * arm);
      earth.Reverse(p.x(), p.y(), p.z(), fix.latitude, fix.longitude, fix.altitude);
      fix.altitude -= 40.1473;
      if (i == 4) fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
      if (i == 5) fix.header.frame_id = "gnss_main_link";
      if (i == 6) fix.altitude = std::numeric_limits<double>::quiet_NaN();
      send(fix);
      if (i == 10) send(fix);  // Never add a duplicate factor.
    }
    glim::GlobalMappingCallbacks::on_smoother_update(isam, factors, values);
    if (mapping) submap->T_world_origin = Eigen::Isometry3d(values.at<gtsam::Pose3>(gtsam::Symbol('x',0)).matrix());
    module->at_exit(dir.string());
    Json report; std::ifstream(dir / "gnss_report.json") >> report;
    EXPECT_TRUE(report["success"].get<bool>());
    EXPECT_EQ(report["counts"]["not_rtk_fix"], 1);
    EXPECT_EQ(report["counts"]["wrong_frame"], 1);
    EXPECT_EQ(report["counts"]["invalid"], 1);
    EXPECT_EQ(report["counts"]["duplicate_or_out_of_order"], 1);
    EXPECT_EQ(report["counts"]["matched"], 58);
    if (mapping) {
      EXPECT_EQ(report["counts"]["factors"], 58);
      EXPECT_LT(report["residual_rmse_m"].get<double>(), 1e-6);
      for (const auto& factor : factors) {
        auto gnss = std::dynamic_pointer_cast<cub_mapping::GnssArmFactor>(factor);
        if (!gnss) continue;
        const auto nominal = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(config["huber_k"]),
          gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.15,0.15,0.30)));
        auto plain = gnss->cloneWithNewNoiseModel(nominal);
        auto changed = values;
        auto pose = values.at<gtsam::Pose3>(gtsam::Symbol('x',0));
        changed.update(gtsam::Symbol('x',0), gtsam::Pose3(pose.rotation(), pose.translation()+gtsam::Point3(1,1,1)));
        EXPECT_NEAR(gnss->error(changed) / plain->error(changed), config["information_scale"].get<double>(), 1e-7);
        break;
      }
      const auto count = factors.size();
      glim::GlobalMappingCallbacks::on_smoother_update(isam, factors, values);
      EXPECT_EQ(count, factors.size());
    }
  }
  std::filesystem::remove_all(dir);
}
