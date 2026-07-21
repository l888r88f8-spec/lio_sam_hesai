#ifndef LIO_SAM_HESAI_RELOCALIZATION_SCAN_CONTEXT_H_
#define LIO_SAM_HESAI_RELOCALIZATION_SCAN_CONTEXT_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace relocalization {

// Scan Context stores the maximum height in each radial/azimuthal cell.
using ScanContextDescriptor = Eigen::MatrixXf;
using ScanContextRingKey = Eigen::VectorXf;

struct ScanContextConfig {
  int num_ring = 20;
  int num_sector = 60;
  double max_radius = 60.0;
  double min_radius = 1.0e-3;
  double height_offset = 2.0;
  int knn_candidates = 10;
  double ringkey_max_dist = 1.0;
  int min_valid_points = 20;
};

struct ScanContextCandidate {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  double yaw_shift = 0.0;
  double score = 0.0;
};

class ScanContext {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ScanContext() = default;
  explicit ScanContext(const ScanContextConfig& config) : config_(config) {}

  const ScanContextConfig& config() const { return config_; }

  ScanContextDescriptor MakeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>& cloud) const;
  static ScanContextRingKey MakeRingKey(const ScanContextDescriptor& desc);

  void AddKeyFrame(const ScanContextDescriptor& desc,
                   const Eigen::Matrix4d& pose);

  std::size_t Size() const { return descriptors_.size(); }
  bool Empty() const { return descriptors_.empty(); }

  bool SaveDB(const std::string& path, const std::string& map_tag) const;
  bool LoadDB(const std::string& path, std::string* map_tag_out = nullptr);

  std::vector<ScanContextCandidate> Query(
      const ScanContextDescriptor& query, int top_n) const;

  // The returned shift aligns database column (column + shift) to query
  // column. The corresponding query yaw correction is positive shift times
  // 2*pi/num_sector.
  static std::pair<float, int> DistanceBetween(
      const ScanContextDescriptor& query,
      const ScanContextDescriptor& database);

 private:
  ScanContextConfig config_{};
  std::vector<ScanContextDescriptor,
              Eigen::aligned_allocator<ScanContextDescriptor>> descriptors_;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>>
      poses_;
  std::vector<ScanContextRingKey,
              Eigen::aligned_allocator<ScanContextRingKey>> ring_keys_;
};

}  // namespace relocalization

#endif  // LIO_SAM_HESAI_RELOCALIZATION_SCAN_CONTEXT_H_
