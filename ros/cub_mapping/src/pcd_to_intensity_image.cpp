// Copyright 2026 Cub Developer
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>

#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

namespace fs = std::filesystem;

void printUsage(const char* prog_name) {
  std::cout << "Usage: " << prog_name << " [options]\n"
            << "Options:\n"
            << "  --yaml <path>        Path to map.yaml (Required)\n"
            << "  --pcd <path>         Path to input PCD file\n"
            << "                       (default: /home/cub/maps/mid360_cleaned/clean_map.pcd)\n"
            << "  --output, -o <path>  Path to output PNG file (default: <map_dir>/map_intensity.png)\n"
            << "  --colormap <name>    turbo (default), viridis, jet, gray\n"
            << "  --min_z <val>        Minimum Z coordinate threshold (m)\n"
            << "  --max_z <val>        Maximum Z coordinate threshold (m)\n"
            << "  --min_intensity <v>  Force minimum intensity for scaling\n"
            << "  --max_intensity <v>  Force maximum intensity for scaling\n"
            << "  --percentile_clip <v> High percentile clip for intensity (default: 99.0)\n"
            << "  --help, -h           Show this help message\n";
}

int main(int argc, char** argv) {
  std::string pcd_path = "";
  std::string yaml_path = "";
  std::string output_path = "";
  std::string colormap_name = "turbo";

  bool use_z_min = false, use_z_max = false;
  double min_z = 0.0, max_z = 0.0;

  bool use_int_min = false, use_int_max = false;
  float force_int_min = 0.0f, force_int_max = 0.0f;
  double percentile_clip = 99.0;

  // Parse command line arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    } else if (arg == "--pcd" && i + 1 < argc) {
      pcd_path = argv[++i];
    } else if (arg == "--yaml" && i + 1 < argc) {
      yaml_path = argv[++i];
    } else if ((arg == "--output" || arg == "-o") && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "--colormap" && i + 1 < argc) {
      colormap_name = argv[++i];
    } else if (arg == "--min_z" && i + 1 < argc) {
      min_z = std::stod(argv[++i]);
      use_z_min = true;
    } else if (arg == "--max_z" && i + 1 < argc) {
      max_z = std::stod(argv[++i]);
      use_z_max = true;
    } else if (arg == "--min_intensity" && i + 1 < argc) {
      force_int_min = std::stof(argv[++i]);
      use_int_min = true;
    } else if (arg == "--max_intensity" && i + 1 < argc) {
      force_int_max = std::stof(argv[++i]);
      use_int_max = true;
    } else if (arg == "--percentile_clip" && i + 1 < argc) {
      percentile_clip = std::stod(argv[++i]);
    } else if (yaml_path.empty() && arg.find(".yaml") != std::string::npos) {
      yaml_path = arg; // Allow positional map.yaml
    } else {
      std::cerr << "Unknown argument: " << arg << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  // Default PCD path fallback
  if (pcd_path.empty()) {
    std::string default_pcd = "/home/cub/maps/mid360_cleaned/clean_map.pcd";
    if (fs::exists(default_pcd)) {
      pcd_path = default_pcd;
      std::cout << "Using default PCD: " << pcd_path << std::endl;
    } else {
      std::cerr << "Error: --pcd was not specified and default not found: " << default_pcd << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (yaml_path.empty()) {
    std::cerr << "Error: --yaml is required.\n";
    printUsage(argv[0]);
    return 1;
  }

  if (!fs::exists(pcd_path)) {
    std::cerr << "Error: PCD file not found: " << pcd_path << std::endl;
    return 1;
  }
  if (!fs::exists(yaml_path)) {
    std::cerr << "Error: YAML file not found: " << yaml_path << std::endl;
    return 1;
  }

  // 1. Load map.yaml
  std::cout << "[1/4] Loading YAML: " << yaml_path << std::endl;
  YAML::Node yaml_node;
  try {
    yaml_node = YAML::LoadFile(yaml_path);
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse YAML file: " << e.what() << std::endl;
    return 1;
  }

  double resolution = yaml_node["resolution"].as<double>();
  std::vector<double> origin = yaml_node["origin"].as<std::vector<double>>();
  std::string image_name = yaml_node["image"].as<std::string>();

  fs::path yaml_dir = fs::path(yaml_path).parent_path();
  fs::path image_path = yaml_dir / image_name;

  if (output_path.empty()) {
    output_path = (yaml_dir / "map_intensity.png").string();
  }

  // Read map image to get exact pixel dimensions
  cv::Mat ref_image = cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
  if (ref_image.empty()) {
    std::cerr << "Error: Failed to read reference image: " << image_path << std::endl;
    return 1;
  }

  int grid_width = ref_image.cols;
  int grid_height = ref_image.rows;
  double origin_x = origin[0];
  double origin_y = origin[1];

  std::cout << "  Resolution: " << resolution << " m/cell" << std::endl;
  std::cout << "  Origin: [" << origin_x << ", " << origin_y << "]" << std::endl;
  std::cout << "  Grid size: " << grid_width << " x " << grid_height << " pixels" << std::endl;
  std::cout << "  Output image: " << output_path << std::endl;

  // 2. Load PCD file
  std::cout << "[2/4] Loading PCD: " << pcd_path << " ..." << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *cloud) != 0) {
    std::cerr << "Failed to load PCD file: " << pcd_path << std::endl;
    return 1;
  }
  std::cout << "  Loaded " << cloud->size() << " points." << std::endl;

  // 3. Project points to grid
  std::cout << "[3/4] Projecting points to 2D grid..." << std::endl;
  std::vector<float> max_intensity_grid(grid_width * grid_height, -1.0f);
  size_t points_inside = 0;

  for (const auto& pt : cloud->points) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
      continue;
    }
    if (use_z_min && pt.z < min_z) continue;
    if (use_z_max && pt.z > max_z) continue;

    int gx = static_cast<int>(std::floor((pt.x - origin_x) / resolution));
    int gy = static_cast<int>(std::floor((pt.y - origin_y) / resolution));

    if (gx >= 0 && gx < grid_width && gy >= 0 && gy < grid_height) {
      int px = gx;
      int py = (grid_height - 1) - gy; // Vertical flip for image coordinate
      int idx = py * grid_width + px;

      if (pt.intensity > max_intensity_grid[idx]) {
        max_intensity_grid[idx] = pt.intensity;
      }
      points_inside++;
    }
  }
  std::cout << "  Points inside map bounds: " << points_inside << std::endl;

  // 4. Calculate Intensity bounds for normalization
  std::vector<float> valid_intensities;
  valid_intensities.reserve(grid_width * grid_height / 4);
  for (float val : max_intensity_grid) {
    if (val >= 0.0f) {
      valid_intensities.push_back(val);
    }
  }

  if (valid_intensities.empty()) {
    std::cerr << "Warning: No points projected onto grid!" << std::endl;
    return 1;
  }

  float norm_min = 0.0f;
  float norm_max = 255.0f;

  if (use_int_min) {
    norm_min = force_int_min;
  } else {
    // 1st percentile
    std::sort(valid_intensities.begin(), valid_intensities.end());
    size_t p1_idx = static_cast<size_t>(valid_intensities.size() * 0.01);
    norm_min = valid_intensities[p1_idx];
  }

  if (use_int_max) {
    norm_max = force_int_max;
  } else {
    // Upper percentile clip (e.g. 99th percentile) to prevent specular highlights from darkening roads
    size_t p_high_idx = static_cast<size_t>(valid_intensities.size() * (percentile_clip / 100.0));
    if (p_high_idx >= valid_intensities.size()) p_high_idx = valid_intensities.size() - 1;
    norm_max = valid_intensities[p_high_idx];
  }

  if (norm_max <= norm_min) {
    norm_max = norm_min + 1.0f;
  }

  std::cout << "  Intensity scaling range: [" << norm_min << ", " << norm_max << "]"
            << " (from " << valid_intensities.size() << " valid cells)" << std::endl;

  // 5. Generate RGBA Image
  std::cout << "[4/4] Generating colored overlay image with colormap '" << colormap_name << "'..." << std::endl;

  int cv_colormap = cv::COLORMAP_TURBO;
  bool is_gray = false;
  if (colormap_name == "viridis") {
    cv_colormap = cv::COLORMAP_VIRIDIS;
  } else if (colormap_name == "jet") {
    cv_colormap = cv::COLORMAP_JET;
  } else if (colormap_name == "gray") {
    is_gray = true;
  } else {
    cv_colormap = cv::COLORMAP_TURBO;
  }

  // Create BGRA Mat (transparent background)
  cv::Mat output_img(grid_height, grid_width, CV_8UC4, cv::Scalar(0, 0, 0, 0));

  for (int py = 0; py < grid_height; ++py) {
    for (int px = 0; px < grid_width; ++px) {
      int idx = py * grid_width + px;
      float val = max_intensity_grid[idx];

      if (val >= 0.0f) {
        // Normalize to 0-255
        float scaled = (val - norm_min) / (norm_max - norm_min) * 255.0f;
        uint8_t u_val = static_cast<uint8_t>(std::clamp(scaled, 0.0f, 255.0f));

        if (is_gray) {
          output_img.at<cv::Vec4b>(py, px) = cv::Vec4b(u_val, u_val, u_val, 255);
        } else {
          cv::Mat gray_pixel(1, 1, CV_8UC1, cv::Scalar(u_val));
          cv::Mat color_pixel;
          cv::applyColorMap(gray_pixel, color_pixel, cv_colormap);
          cv::Vec3b bgr = color_pixel.at<cv::Vec3b>(0, 0);
          output_img.at<cv::Vec4b>(py, px) = cv::Vec4b(bgr[0], bgr[1], bgr[2], 255);
        }
      }
    }
  }

  if (cv::imwrite(output_path, output_img)) {
    std::cout << "Successfully saved intensity overlay image to: " << output_path << std::endl;
  } else {
    std::cerr << "Failed to write image: " << output_path << std::endl;
    return 1;
  }

  return 0;
}

