#pragma once
#include "lio_sam/current_ground_segmenter.h"
#include "lio_sam/ground_segmentation_mode.h"
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

namespace lio_sam_hesai {
struct CompleteGroundScan {
  pcl::PointCloud<pcl::PointXYZI> points;
  std::vector<int> projection_cells;
};
// Call only after the ordinary projection has fixed its deskew reference.
// Extra returns never establish or update the LOAM reference themselves.
template <typename PointT, typename Deskew>
CompleteGroundScan MakeCompleteGroundScan(
    const pcl::PointCloud<PointT>& raw, const std::vector<int>& projection_cells,
    bool reference_established, Deskew&& deskew) {
  if(raw.size()!=projection_cells.size())
    throw std::invalid_argument("complete scan projection mapping size mismatch");
  CompleteGroundScan result;
  result.points.reserve(raw.size());result.projection_cells.reserve(raw.size());
  for(std::size_t index=0;index<raw.size();++index) {
    pcl::PointXYZI point;
    point.x=raw[index].x;point.y=raw[index].y;point.z=raw[index].z;
    point.intensity=raw[index].intensity;
    if(!std::isfinite(point.x)||!std::isfinite(point.y)||!std::isfinite(point.z))
      continue;
    if(reference_established) point=deskew(point,index);
    result.points.push_back(point);
    result.projection_cells.push_back(projection_cells[index]);
  }
  return result;
}
class OnlineGroundPipeline {
 public:
  using SegmentFunction=std::function<std::vector<std::size_t>(
      const pcl::PointCloud<pcl::PointXYZI>&)>;
  using PatchworkFactory=std::function<SegmentFunction()>;
  OnlineGroundPipeline(const GroundSegmentationConfig& current,
                       const PatchworkppConfig& patchwork,
                       PatchworkFactory factory={});
  GroundAlgorithm Algorithm() const { return algorithm_; }
  CurrentGroundResult Segment(
      const pcl::PointCloud<pcl::PointXYZI>& organized,
      const std::vector<std::uint8_t>& valid,int rows,int columns,
      const pcl::PointCloud<pcl::PointXYZI>& complete,
      const std::vector<int>& projection_cells);
 private:
  GroundSegmentationConfig current_;
  GroundAlgorithm algorithm_;
  SegmentFunction patchwork_;
};
}
