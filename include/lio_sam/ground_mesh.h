#ifndef LIO_SAM_HESAI_GROUND_MESH_H_
#define LIO_SAM_HESAI_GROUND_MESH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "lio_sam/ground_ring_policy.h"

namespace lio_sam_hesai {

struct GroundMeshOptions {
  float resolution = 0.0f;
  float max_edge_length = 0.5f;
  float edge_length_per_meter = 0.25f;
  float max_range_ratio = 2.0f;
  float max_slope_deg = 25.0f;
  float max_height_range = 0.12f;
  std::size_t min_keyframe_support = 1;
};

struct GroundMeshStatistics {
  std::size_t frames = 0;
  std::size_t candidate_triangles = 0;
  std::size_t accepted_triangles = 0;
  std::size_t rejected_ground = 0;
  std::size_t rejected_edge = 0;
  std::size_t rejected_range = 0;
  std::size_t rejected_slope = 0;
  std::size_t rejected_degenerate = 0;
  std::size_t rasterized_cells = 0;
  std::size_t ambiguous_cells = 0;
};

class GroundMeshRasterizer {
 public:
  explicit GroundMeshRasterizer(const GroundMeshOptions &options);
  ~GroundMeshRasterizer();

  GroundMeshRasterizer(const GroundMeshRasterizer &) = delete;
  GroundMeshRasterizer &operator=(const GroundMeshRasterizer &) = delete;
  GroundMeshRasterizer(GroundMeshRasterizer &&) noexcept;
  GroundMeshRasterizer &operator=(GroundMeshRasterizer &&) noexcept;

  void AddFrame(const pcl::PointCloud<pcl::PointXYZI> &scan,
      const std::vector<std::uint32_t> &projection_indices,
      const std::vector<std::int8_t> &classification, int rows,
      int columns, GroundRingRange output_rings,
      const Eigen::Matrix4f &map_from_scan);

  std::size_t AppendMissingSamples(
    pcl::PointCloud<pcl::PointXYZI> *global_measured_ground);

  const GroundMeshStatistics &statistics() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_GROUND_MESH_H_
