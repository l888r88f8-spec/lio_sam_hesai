#pragma once

#include <Eigen/Dense>

#include <stdexcept>
#include <vector>

struct LidarImuCalibration {
  Eigen::Matrix4d lidar_to_imu = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d mapping_imu_to_lidar_rotation =
      Eigen::Matrix3d::Identity();
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

  LidarImuCalibration calibration;
  calibration.lidar_to_imu.block<3, 3>(0, 0) = rotation_matrix;
  calibration.lidar_to_imu.block<3, 1>(0, 3) = translation_vector;
  calibration.mapping_imu_to_lidar_rotation = rotation_matrix.transpose();
  calibration.mapping_rpy_rotation = rpy_matrix;
  return calibration;
}
