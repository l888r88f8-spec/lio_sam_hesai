#ifndef LIO_SAM_HESAI_PATCHWORKPP_GROUND_SEGMENTER_H
#define LIO_SAM_HESAI_PATCHWORKPP_GROUND_SEGMENTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <vector>

struct PatchworkppConfig;

namespace lio_sam_hesai
{
// One instance per ordered scan stream: upstream adaptive thresholds persist
// between Segment calls. Use independent instances for online and export work.
// Calls on a given instance must be serialized.
class PatchworkppGroundSegmenter
{
 public:
  explicit PatchworkppGroundSegmenter(const PatchworkppConfig& config);
  ~PatchworkppGroundSegmenter();

  PatchworkppGroundSegmenter(const PatchworkppGroundSegmenter&) = delete;
  PatchworkppGroundSegmenter& operator=(const PatchworkppGroundSegmenter&) = delete;

  // Sorted, unique indices into the original lidar-frame cloud. No input point
  // or field is changed and coincident points retain separate indices. Invalid
  // XYZ points are ineligible; empty/all-invalid input returns no indices.
  // With RNR enabled, finite intensity is required for every valid XYZ point.
  std::vector<std::size_t> Segment(const pcl::PointCloud<pcl::PointXYZI>& cloud);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_PATCHWORKPP_GROUND_SEGMENTER_H
