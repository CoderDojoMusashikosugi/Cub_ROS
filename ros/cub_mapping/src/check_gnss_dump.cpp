#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <gtsam/base/serialization.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <glim/util/extension_module.hpp>
#include "cub_mapping/gnss_factor.hpp"

int main(int argc, char** argv) {
  if (argc != 2) { std::cerr << "Usage: check_gnss_dump DUMP_DIRECTORY\n"; return 2; }
  try {
    // Register GLIM's standard factor/noise types as well as our GNSS factor.
    glim::ExtensionModule::export_classes("libcub_mapping_gnss.so");
    const std::string path = argv[1];
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    gtsam::deserializeFromBinaryFile(path + "/graph.bin", graph);
    gtsam::deserializeFromBinaryFile(path + "/values.bin", values);
    size_t count = 0; double squared = 0;
    for (const auto& factor : graph) {
      auto gnss = std::dynamic_pointer_cast<cub_mapping::GnssArmFactor>(factor);
      if (!gnss) continue;
      const auto error = gnss->evaluateError(values.at<gtsam::Pose3>(gnss->keys()[0]));
      if (!error.allFinite()) throw std::runtime_error("Non-finite GNSS residual");
      squared += error.squaredNorm(); ++count;
    }
    nlohmann::json report;
    std::ifstream input(path + "/gnss_report.json"); input >> report;
    if (!report.at("success").get<bool>() || !count || count != report.at("counts").at("factors").get<size_t>())
      throw std::runtime_error("Restored GNSS factor count does not match report");
    std::cout << "Restored " << count << " GNSS factors; RMSE=" << std::sqrt(squared/count) << " m\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
