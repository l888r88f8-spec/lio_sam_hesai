#ifndef LIO_SAM_HESAI_LIO_SAM_DUFOMAP_BACKEND_H
#define LIO_SAM_HESAI_LIO_SAM_DUFOMAP_BACKEND_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

// Global configuration struct; only the C++20 implementation includes it,
// so this public header imposes no ROS dependency on callers.
struct DufomapConfig;

namespace lio_sam_hesai {

// One original lidar-frame point. Intensity is carried through unchanged for
// callers that write the filtered cloud; DUFOMap itself only uses XYZ.
struct DufomapPoint {
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

inline bool operator==(const DufomapPoint& lhs, const DufomapPoint& rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.z == rhs.z &&
         lhs.intensity == rhs.intensity;
}

// Direct C++ adapter over the pinned official DUFOMap/UFOMap core. One
// instance is one independent export state and calls must be serialized.
// UFOMap is C++20, so this header deliberately hides it behind PIMPL and stays
// usable from the C++17 mapping node.
class DufomapBackend {
 public:
  explicit DufomapBackend(const DufomapConfig& config);
  ~DufomapBackend();

  DufomapBackend(const DufomapBackend&) = delete;
  DufomapBackend& operator=(const DufomapBackend&) = delete;

  // Applies the configured range gate, transforms one local scan by the final
  // odometry_T_lidar pose, and integrates it with the lidar position as the
  // ray origin. Throws std::invalid_argument for malformed input and
  // std::runtime_error for an expired cooperative deadline or upstream error.
  void Integrate(const std::vector<DufomapPoint>& local_points,
                 const std::array<double, 16>& odometry_T_lidar);

  // Runs upstream propagateModified. Must be called after all frames and
  // before any classification.
  void Propagate();

  // Labels odometry-frame points with the upstream seenFree predicate: 1 means
  // dynamic (seen free), 0 means keep. Classification is chunked so the
  // cooperative deadline is observed between chunks. Input order and fields
  // are preserved by the caller; this method only returns labels.
  std::vector<std::uint8_t> Classify(
      const std::vector<DufomapPoint>& odometry_points);

  [[nodiscard]] std::size_t integrated_points() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Pinned provenance recorded in export summaries.
const char* DufomapBackendKind();
const char* DufomapBackendSource();
const char* DufomapUfomapRevision();

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_LIO_SAM_DUFOMAP_BACKEND_H
