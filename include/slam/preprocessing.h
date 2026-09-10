
#ifndef FUNNY_LIDAR_SLAM_PREPROCESSING_H
#define FUNNY_LIDAR_SLAM_PREPROCESSING_H

#include <memory>
#include <utility>

#include "lidar/lidar_distortion_corrector.h"
#include "lidar/lidar_model.h"
#include "lidar/lidar_point_type.h"
#include "loam/feature_extractor.h"
#include "loam/pointcloud_projector.h"
#include "runtime/frame_processing_watchdog.h"
#include "slam/system.h"

class PreProcessing {
 public:
  using DataSearcherIMU = DataSearcher<IMUData>;

 public:
  PreProcessing() = delete;

  PreProcessing(System* system_ptr, std::shared_ptr<const SlamConfig> config);

  ~PreProcessing() = default;

  void Run();

  [[nodiscard]] runtime::FrameProcessingSnapshot FrameProcessingSnapshot() const {
    return frame_processing_watchdog_.Snapshot();
  }

 private:
  void InitVoxelGridFilter();

  // Non-static: both scale point offset times by the injected configuration.
  pcl::PointCloud<relocalization::PointXYZIRT>::Ptr ConvertMessageToCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);

  static bool HasCloudField(const sensor_msgs::msg::PointCloud2& cloud,
                            const std::string& field_name);

  static const sensor_msgs::msg::PointField* FindCloudField(
      const sensor_msgs::msg::PointCloud2& cloud,
      const std::string& field_name);

  static double ReadCloudFieldAsDouble(
      const std::uint8_t* point_data,
      const sensor_msgs::msg::PointField& field);

  static pcl::PointCloud<relocalization::PointXYZIRT>::Ptr
  ConvertXYZIMessageToCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);

  pcl::PointCloud<relocalization::PointXYZIRT>::Ptr
  ConvertHesaiMessageToCloud(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);

  /*!
   * Compute lidar every point offset time
   * note: Only supports clockwise rotation lidar scan
   * @param cloud
   * @param lidar_rate
   */
  static void ComputePointOffsetTime(
      const pcl::PointCloud<relocalization::PointXYZIRT>::Ptr& cloud,
      double lidar_rate = 10.0);

  static std::pair<float, float> GetLidarPointMinMaxOffsetTime(
      const pcl::PointCloud<relocalization::PointXYZIRT>::Ptr& cloud);

 private:
  std::shared_ptr<const SlamConfig> config_;

  // voxel grid filter
  pcl::VoxelGrid<PCLPointXYZI> corner_voxel_filter_;
  pcl::VoxelGrid<PCLPointXYZI> planer_voxel_filter_;

  PCLPointCloudXYZIRT::Ptr temp_cloud_ptr_ = nullptr;

  std::shared_ptr<loam::FeatureExtractor> feature_extractor_ptr_ = nullptr;
  std::shared_ptr<loam::PointcloudProjector> cloud_projector_ptr_ = nullptr;
  std::shared_ptr<LidarDistortionCorrector> lidar_distortion_corrector_ptr_ =
      nullptr;

  System* system_ptr_ = nullptr;
  runtime::FrameProcessingWatchdog frame_processing_watchdog_;
};

#endif  // FUNNY_LIDAR_SLAM_PREPROCESSING_H
