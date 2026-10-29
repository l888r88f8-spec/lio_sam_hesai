#ifndef LIO_SAM_HESAI_COMMON_DUFOMAP_CONFIG_H
#define LIO_SAM_HESAI_COMMON_DUFOMAP_CONFIG_H

#include <string>

// Optional auxiliary coarse grid plus distinct-keyframe support. Disabled by
// default so callers that do not maintain complete keyframe scans keep the
// existing fine-only behavior.
struct DufomapTemporalGuardConfig {
  bool enable{false};
  double coarse_resolution{0.4};
  int min_keyframe_support{2};
};

// Online DUFOMap dynamic-point removal. The background worker owns the
// backend; live SLAM keyframes remain immutable. The parameters below configure
// the official backend (resolution / d_s / d_p); map_save still controls the
// exported point density.
struct DufomapConfig {
  bool enable{true};
  // Classification grid only; affects labels but never merges target points.
  // Output downsampling remains controlled by map_save resolutions.
  double resolution{0.1};
  double d_s{0.2};
  int d_p{2};  // supported upstream template range: [0, 4]
  int num_threads{0};  // 0 automatic; otherwise >=2 (serial misses TODO)
  // Evidence range gate applied to the cached local scan before integration.
  // A zero max_range disables the upper bound.
  double min_range{0.2};
  double max_range{100.0};
  // Cooperative timeout only: checked between frames and classification
  // batches, never inside one upstream core call.
  double timeout_seconds{3600.0};
  // Root of the persistent keyframe scan cache. Every mapping session creates
  // its own directory below it and keeps its scans after the node exits.
  // Empty selects a per-session directory below the system temp directory.
  std::string cache_directory;
  // Fine-grid decisions always win; the guard can only add coarse-only
  // removals that lack repeated complete-keyframe support.
  DufomapTemporalGuardConfig temporal_guard;
};

// Historical name used by the native backend and the standalone manual replay
// tool. Production mapping uses DufomapConfig.
using DufomapExportConfig = DufomapConfig;

#endif  // LIO_SAM_HESAI_COMMON_DUFOMAP_CONFIG_H
