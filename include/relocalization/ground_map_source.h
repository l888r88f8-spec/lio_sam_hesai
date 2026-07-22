#ifndef LIO_SAM_HESAI_RELOCALIZATION_GROUND_MAP_SOURCE_H
#define LIO_SAM_HESAI_RELOCALIZATION_GROUND_MAP_SOURCE_H

#include "common/data_type.h"

#include <optional>

#include <pcl/kdtree/kdtree_flann.h>

struct GroundMapSource {
  PCLPointCloudXYZI::Ptr height_cloud;
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr xy_kdtree;
};

inline std::optional<GroundMapSource> SelectGroundMapSource(
    const PCLPointCloudXYZI::Ptr& global_cloud,
    const pcl::KdTreeFLANN<PCLPointXYZI>::Ptr& global_kdtree,
    const PCLPointCloudXYZI::Ptr& dedicated_ground_cloud,
    const pcl::KdTreeFLANN<PCLPointXYZI>::Ptr& dedicated_ground_kdtree) {
  if (dedicated_ground_cloud && !dedicated_ground_cloud->empty() &&
      dedicated_ground_kdtree) {
    return GroundMapSource{dedicated_ground_cloud, dedicated_ground_kdtree};
  }
  if (global_cloud && !global_cloud->empty() && global_kdtree) {
    return GroundMapSource{global_cloud, global_kdtree};
  }
  return std::nullopt;
}

#endif  // LIO_SAM_HESAI_RELOCALIZATION_GROUND_MAP_SOURCE_H
