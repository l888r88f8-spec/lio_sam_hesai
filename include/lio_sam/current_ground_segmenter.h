#pragma once
#include "common/slam_config.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdint>
#include <vector>

namespace lio_sam_hesai {
struct CurrentGroundResult {
  pcl::PointCloud<pcl::PointXYZI> ground;
  std::vector<std::int8_t> classification;
};
CurrentGroundResult SegmentCurrentGround(
    const GroundSegmentationConfig& config,
    const pcl::PointCloud<pcl::PointXYZI>& organized,
    const std::vector<std::uint8_t>& valid, int rows, int columns);
CurrentGroundResult SegmentProjectedGround(
    const GroundSegmentationConfig& config,
    const pcl::PointCloud<pcl::PointXYZI>& projected,
    const std::vector<std::uint32_t>& projection_indices,
    int rows, int columns);
}
