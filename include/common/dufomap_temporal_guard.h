#ifndef LIO_SAM_HESAI_COMMON_DUFOMAP_TEMPORAL_GUARD_H
#define LIO_SAM_HESAI_COMMON_DUFOMAP_TEMPORAL_GUARD_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/dufomap_config.h"
#include "lio_sam/dufomap_backend.h"

namespace lio_sam_hesai {

// One auxiliary coarse world voxel. Support is keyed by this voxel, not by the
// exact point, so duplicate points from one keyframe cannot raise support.
struct WorldVoxel {
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};
};

bool operator==(const WorldVoxel& lhs, const WorldVoxel& rhs);
bool operator!=(const WorldVoxel& lhs, const WorldVoxel& rhs);

struct WorldVoxelHash {
  std::size_t operator()(const WorldVoxel& voxel) const;
};

WorldVoxel WorldVoxelFor(const DufomapPoint& point, double resolution);

// Distinct-keyframe support for the coarse grid. The model is intentionally
// small and monotonic:
//   - AddKeyframe must be called with strictly increasing keyframe indices;
//     a repeated or older index is ignored, so one keyframe can never count
//     twice.
//   - One entry per observed world voxel stores a saturated count and the last
//     keyframe index. Once the count reaches the saturation threshold only
//     that saturated count remains; no per-keyframe voxel history is kept.
//   - Replacing a keyframe is done by Clear() followed by rebuilding all
//     keyframes in index order. There is no per-keyframe erase path.
class KeyframeSupportModel {
 public:
  explicit KeyframeSupportModel(
      double resolution,
      int saturation_support = std::numeric_limits<int>::max());

  // Forgets every voxel and the monotonic keyframe cursor.
  void Clear();

  // Adds one keyframe's world points. Returns false without changing anything
  // when the index is not strictly greater than the last accepted keyframe.
  bool AddKeyframe(std::size_t keyframe_index,
                   const std::vector<DufomapPoint>& world_points);

  // Saturated distinct-keyframe count for the voxel containing world_point.
  int Support(const DufomapPoint& world_point) const;

 private:
  struct Entry {
    int count{0};
    std::size_t last_keyframe{0};
  };

  double resolution_;
  int saturation_support_;
  bool has_keyframe_{false};
  std::size_t last_keyframe_{0};
  std::unordered_map<WorldVoxel, Entry, WorldVoxelHash> voxels_;
};

// Applies the final policy to one point set. fine_labels are authoritative for
// seen-free removal: a fine label of 1 is always kept as 1. Otherwise a coarse
// label of 1 removes the point only while distinct keyframe support is below
// min_support. Throws std::invalid_argument for mismatched inputs or
// min_support < 1.
std::vector<std::uint8_t> ApplyTemporalGuard(
    const std::vector<std::uint8_t>& fine_labels,
    const std::vector<std::uint8_t>& coarse_labels,
    const std::vector<DufomapPoint>& world_points,
    const KeyframeSupportModel& support, int min_support);

// Validates only when the guard is enabled. A disabled guard is always valid
// so existing configurations can keep unused placeholder values.
bool ValidateTemporalGuardConfig(const DufomapTemporalGuardConfig& config,
                                 std::string* error);

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_DUFOMAP_TEMPORAL_GUARD_H
