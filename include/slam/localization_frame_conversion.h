#ifndef LIO_SAM_HESAI_SLAM_LOCALIZATION_FRAME_CONVERSION_H
#define LIO_SAM_HESAI_SLAM_LOCALIZATION_FRAME_CONVERSION_H

#include <cmath>

#include "common/rigid_transform.h"
#include "common/pointcloud_utility.h"

// The localization matcher consumes IMU-frame geometry; map providers and
// Scan Context keyframes consume lidar-frame poses/geometry. Keep these
// boundary conversions shared by initialization, tracking and relocalization.
namespace localization_frames {

inline Mat4d MapLidarPoseToMapImuPose(
    const MapTLidar& map_T_lidar, const LidarTImu& lidar_T_imu) {
  return (map_T_lidar * lidar_T_imu).Matrix();
}

inline Mat4d MapImuPoseToMapLidarPose(
    const Mat4d& map_imu_pose, const ImuTLidar& imu_T_lidar) {
  return (MapTImu::FromMatrix(map_imu_pose) * imu_T_lidar).Matrix();
}

inline PCLPointCloudXYZI ScanContextLidarCloud(
    const PCLPointCloudXYZI& imu_cloud,
    const LidarTImu& lidar_T_imu) {
  return TransformPointCloud(imu_cloud, lidar_T_imu.Matrix());
}

inline Mat4d ScanContextCandidateLidarPose(
    const Mat4d& keyframe_map_lidar_pose, double yaw_shift,
    double imu_roll, double imu_pitch,
    const ImuTLidar& imu_T_lidar) {
  const double lidar_yaw = std::atan2(keyframe_map_lidar_pose(1, 0),
                                     keyframe_map_lidar_pose(0, 0)) + yaw_shift;
  const Mat3d nominal_map_lidar_rotation =
      Eigen::AngleAxisd(lidar_yaw, Vec3d::UnitZ()).toRotationMatrix();
  const Mat3d nominal_map_imu_rotation =
      nominal_map_lidar_rotation * imu_T_lidar.Inverse().Rotation();
  const double imu_yaw = std::atan2(
      nominal_map_imu_rotation(1, 0), nominal_map_imu_rotation(0, 0));
  const Mat3d map_imu_rotation =
      (Eigen::AngleAxisd(imu_yaw, Vec3d::UnitZ()) *
       Eigen::AngleAxisd(imu_pitch, Vec3d::UnitY()) *
       Eigen::AngleAxisd(imu_roll, Vec3d::UnitX())).toRotationMatrix();
  Mat4d candidate_lidar_pose = keyframe_map_lidar_pose;
  candidate_lidar_pose.block<3, 3>(0, 0) =
      map_imu_rotation * imu_T_lidar.Rotation();
  return candidate_lidar_pose;
}

}  // namespace localization_frames

#endif  // LIO_SAM_HESAI_SLAM_LOCALIZATION_FRAME_CONVERSION_H
