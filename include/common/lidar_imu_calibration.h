#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <stdexcept>
#include <vector>

#include "common/rigid_transform.h"

// Canonical calibration semantics are imu_T_lidar: the configured rotation and
// translation give the lidar pose in the IMU frame. The mapping-specific
// rotation, lever arm and RPY values are derived from it and are never
// configured independently.
struct LidarImuCalibration {
  ImuTLidar imu_T_lidar{ImuTLidar::Identity()};
  LidarTImu lidar_T_imu{LidarTImu::Identity()};
  Eigen::Matrix3d mapping_imu_to_lidar_rotation =
      Eigen::Matrix3d::Identity();
  // IMU-to-lidar translation expressed in lidar axes for mapping poses.
  Eigen::Vector3d mapping_imu_to_lidar_lever_arm =
      Eigen::Vector3d::Zero();
  Eigen::Matrix3d mapping_rpy_rotation = Eigen::Matrix3d::Identity();
};

inline LidarImuCalibration ParseLidarImuCalibration(
    const std::vector<double>& translation,
    const std::vector<double>& rotation,
    const std::vector<double>& rpy)
{
  if (translation.size() != 3) {
    throw std::invalid_argument(
        "Lidar-to-IMU translation must contain exactly 3 values");
  }
  if (rotation.size() != 9) {
    throw std::invalid_argument(
        "Lidar-to-IMU rotation must contain exactly 9 values");
  }
  if (rpy.size() != 9) {
    throw std::invalid_argument(
        "Lidar-to-IMU RPY rotation must contain exactly 9 values");
  }

  using RowMajorMatrix3d =
      Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
  const Eigen::Map<const Eigen::Vector3d> translation_vector(
      translation.data());
  const Eigen::Map<const RowMajorMatrix3d> rotation_matrix(rotation.data());
  const Eigen::Map<const RowMajorMatrix3d> rpy_matrix(rpy.data());

  // The configured rotation places the lidar in the IMU frame, so the
  // validated canonical transform is imu_T_lidar and its inverse follows.
  Eigen::Isometry3d imu_T_lidar = Eigen::Isometry3d::Identity();
  imu_T_lidar.linear() = rotation_matrix;
  imu_T_lidar.translation() = translation_vector;

  LidarImuCalibration calibration;
  calibration.imu_T_lidar = ImuTLidar(imu_T_lidar);
  calibration.lidar_T_imu = calibration.imu_T_lidar.Inverse();
  calibration.mapping_imu_to_lidar_rotation = rotation_matrix.transpose();
  calibration.mapping_imu_to_lidar_lever_arm =
      calibration.mapping_imu_to_lidar_rotation * translation_vector;
  calibration.mapping_rpy_rotation = rpy_matrix;
  return calibration;
}
