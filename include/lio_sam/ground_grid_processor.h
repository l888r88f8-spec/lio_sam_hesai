#pragma once

#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace lio_sam_hesai {

using GroundGridPointType = pcl::PointXYZI;

// Final GroundMap export options. `resolution` is the resolved save-time cell
// size (map_save.ground_resolution or the service override), not the online
// ring-plane patch resolution.
struct GroundGridExportOptions {
  float resolution = 0.0f;
  float search_radius = 0.40f;
  float max_height_range = 0.12f;
  float max_roughness = 0.05f;
  float plane_distance_threshold = 0.12f;
  float max_slope_deg = 25.0f;
  float max_prediction_margin = 0.08f;
  float min_plane_inlier_ratio = 0.70f;
  int min_neighbor_cells = 6;
  int min_support_quadrants = 3;
  // Complete four-connected unobserved components up to this many cells are
  // filled together. Larger components are never filled in pieces.
  int max_hole_cells = 14;
  // Reuses ring_plane.grid_patch.enable. Only unobserved, well-supported
  // interior cells are filled; observed cells that were rejected stay empty.
  bool fill_unobserved_gaps = false;
};

// Exports one finite point per XY cell at the exact cell center. Each cell is
// reduced to a single robust height layer; equal competing layers are left
// empty instead of being averaged. Z comes from a local plane fitted to
// neighbouring cell representatives when supported, otherwise from the
// selected layer mean. Returns false for invalid options/output or coordinates.
bool BuildGroundGrid(const pcl::PointCloud<GroundGridPointType>& input,
    const GroundGridExportOptions& options,
    pcl::PointCloud<GroundGridPointType>* output,
    std::string* error = nullptr);

}  // namespace lio_sam_hesai
