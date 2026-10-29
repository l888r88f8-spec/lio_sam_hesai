#include "common/dufomap_temporal_guard.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace lio_sam_hesai {

bool operator==(const WorldVoxel& lhs, const WorldVoxel& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z;
}

bool operator!=(const WorldVoxel& lhs, const WorldVoxel& rhs) {
  return !(lhs == rhs);
}

std::size_t WorldVoxelHash::operator()(const WorldVoxel& voxel) const {
  std::size_t seed = std::hash<std::int64_t>{}(voxel.x);
  seed ^= std::hash<std::int64_t>{}(voxel.y) + 0x9e3779b97f4a7c15ULL +
          (seed << 6U) + (seed >> 2U);
  seed ^= std::hash<std::int64_t>{}(voxel.z) + 0x9e3779b97f4a7c15ULL +
          (seed << 6U) + (seed >> 2U);
  return seed;
}

WorldVoxel WorldVoxelFor(const DufomapPoint& point, double resolution) {
  return WorldVoxel{
      static_cast<std::int64_t>(std::floor(point.x / resolution)),
      static_cast<std::int64_t>(std::floor(point.y / resolution)),
      static_cast<std::int64_t>(std::floor(point.z / resolution))};
}

KeyframeSupportModel::KeyframeSupportModel(double resolution,
                                           int saturation_support)
    : resolution_(resolution),
      saturation_support_(std::max(1, saturation_support)) {
  if (!std::isfinite(resolution_) || resolution_ <= 0.0) {
    throw std::invalid_argument(
        "temporal guard support resolution must be finite and positive");
  }
}

void KeyframeSupportModel::Clear() {
  voxels_.clear();
  has_keyframe_ = false;
  last_keyframe_ = 0;
}

bool KeyframeSupportModel::AddKeyframe(
    std::size_t keyframe_index,
    const std::vector<DufomapPoint>& world_points) {
  if (has_keyframe_ && keyframe_index <= last_keyframe_) return false;
  has_keyframe_ = true;
  last_keyframe_ = keyframe_index;
  for (const auto& point : world_points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    Entry& entry = voxels_[WorldVoxelFor(point, resolution_)];
    // One keyframe contributes at most once per voxel. Within a single call
    // this last_keyframe marker rejects duplicate points without a per-keyframe
    // voxel set.
    if (entry.count > 0 && entry.last_keyframe == keyframe_index) continue;
    entry.last_keyframe = keyframe_index;
    if (entry.count < saturation_support_) ++entry.count;
  }
  return true;
}

int KeyframeSupportModel::Support(const DufomapPoint& world_point) const {
  const auto found = voxels_.find(WorldVoxelFor(world_point, resolution_));
  return found == voxels_.end() ? 0 : found->second.count;
}

std::vector<std::uint8_t> ApplyTemporalGuard(
    const std::vector<std::uint8_t>& fine_labels,
    const std::vector<std::uint8_t>& coarse_labels,
    const std::vector<DufomapPoint>& world_points,
    const KeyframeSupportModel& support, int min_support) {
  if (fine_labels.size() != world_points.size() ||
      coarse_labels.size() != world_points.size()) {
    throw std::invalid_argument(
        "temporal guard label count does not match the point count");
  }
  if (min_support < 1) {
    throw std::invalid_argument(
        "dufomap.temporal_guard.min_keyframe_support must be at least 1");
  }
  std::vector<std::uint8_t> result = fine_labels;
  for (std::size_t index = 0; index < world_points.size(); ++index) {
    if (fine_labels[index] != 0U) {
      result[index] = 1U;
    } else if (coarse_labels[index] != 0U &&
               support.Support(world_points[index]) < min_support) {
      result[index] = 1U;
    } else {
      result[index] = 0U;
    }
  }
  return result;
}

bool ValidateTemporalGuardConfig(const DufomapTemporalGuardConfig& config,
                                 std::string* error) {
  if (error != nullptr) error->clear();
  if (!config.enable) return true;
  if (!std::isfinite(config.coarse_resolution) ||
      config.coarse_resolution <= 0.0) {
    if (error != nullptr) {
      *error =
          "dufomap.temporal_guard.coarse_resolution must be finite and "
          "positive while enabled";
    }
    return false;
  }
  if (config.min_keyframe_support < 1) {
    if (error != nullptr) {
      *error =
          "dufomap.temporal_guard.min_keyframe_support must be at least 1 "
          "while enabled";
    }
    return false;
  }
  return true;
}

}  // namespace lio_sam_hesai
