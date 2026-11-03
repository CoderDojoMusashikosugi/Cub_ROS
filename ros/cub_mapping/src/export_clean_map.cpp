#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <array>
#include <nlohmann/json.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;
int main(int argc, char** argv) {
  try {
    fs::path ground, stationary, prefix;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--help") { std::cout << "export_clean_map --ground GROUND.pcd --static STATIC.pcd --output PREFIX\n"; return 0; }
      if (i + 1 >= argc) throw std::runtime_error("Missing argument: " + arg);
      if (arg == "--ground") ground = argv[++i];
      else if (arg == "--static") stationary = argv[++i];
      else if (arg == "--output") prefix = argv[++i];
      else throw std::runtime_error("Unknown argument: " + arg);
    }
    if (ground.empty() || stationary.empty() || prefix.empty()) throw std::runtime_error("Specify --ground, --static and --output");
    prefix = fs::absolute(prefix);
    for (const auto* suffix : {".pcd", ".ply", ".xyz", ".json"})
      if (fs::exists(prefix.string() + suffix) || fs::exists(prefix.string() + suffix + ".partial"))
        throw std::runtime_error("Output exists: " + prefix.string() + suffix);
    pcl::PointCloud<pcl::PointXYZI> a, b;
    if (pcl::io::loadPCDFile(ground.string(), a) < 0 || pcl::io::loadPCDFile(stationary.string(), b) < 0)
      throw std::runtime_error("Unable to read input PCD");
    const size_t total = a.size() + b.size();
    if (!total) throw std::runtime_error("Both input point clouds are empty");
    // Validate before creating any export files. Preserve coordinates and intensity.
    for (auto* cloud : {&a, &b}) for (const auto& p : *cloud)
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) || !std::isfinite(p.intensity))
        throw std::runtime_error("Non-finite input points");
    fs::create_directories(prefix.parent_path());
    std::ofstream pcd(prefix.string()+".pcd.partial", std::ios::binary);
    std::ofstream ply(prefix.string()+".ply.partial", std::ios::binary);
    std::ofstream xyz(prefix.string()+".xyz.partial");
    pcd << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH "
        << total << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << total << "\nDATA binary\n";
    ply << "ply\nformat binary_little_endian 1.0\ncomment Ground plus static; original map coordinates in metres\nelement vertex "
        << total << "\nproperty float x\nproperty float y\nproperty float z\nproperty float intensity\nend_header\n";
    xyz << std::fixed << std::setprecision(9);
    Eigen::Vector3d low = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity()), high = -low;
    for (auto* cloud : {&a, &b}) for (const auto& p : *cloud) {
      const std::array<float,4> values{p.x,p.y,p.z,p.intensity};
      pcd.write(reinterpret_cast<const char*>(values.data()), sizeof(values));
      ply.write(reinterpret_cast<const char*>(values.data()), sizeof(values));
      xyz << p.x << ' ' << p.y << ' ' << p.z << ' ' << p.intensity << '\n';
      const Eigen::Vector3d v(p.x,p.y,p.z); low = low.cwiseMin(v); high = high.cwiseMax(v);
    }
    pcd.close(); ply.close(); xyz.close();
    if (!pcd || !ply || !xyz) throw std::runtime_error("Export failed (disk full?)");
    nlohmann::json report{{"success",true},{"ground_points",a.size()},{"static_points",b.size()},{"points",total},
      {"ground",fs::absolute(ground).string()},{"static",fs::absolute(stationary).string()},
      {"coordinate_operation","none; no centering, rotation, unit scaling or ground clamping"},
      {"xyz_columns",{"x","y","z","intensity"}},
      {"minimum",{low.x(),low.y(),low.z()}},{"maximum",{high.x(),high.y(),high.z()}}};
    std::ifstream provenance(ground.parent_path()/"run_report.json");
    if (provenance) { nlohmann::json source; provenance >> source; report["run_report"] = source; }
    std::ofstream info(prefix.string()+".json.partial"); info << report.dump(2) << '\n'; info.close();
    if (!info) throw std::runtime_error("Cannot write export metadata");
    for (const auto* suffix : {".pcd", ".ply", ".xyz", ".json"})
      fs::rename(prefix.string()+suffix+".partial", prefix.string()+suffix);
    std::cout << "Exported " << total << " points to " << prefix << ".{pcd,ply,xyz,json}\n";
    return 0;
  } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
