#include "common/lidar_imu_calibration.h"

#include <gtest/gtest.h>

#include <stdexcept>
#include <vector>

TEST(LidarImuCalibration, PreservesCanonicalLidarToImuDirection)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> rotation = {
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0};
  const std::vector<double> rpy = {
      1.0, 0.0,  0.0,
      0.0, 0.0, -1.0,
      0.0, 1.0,  0.0};

  Eigen::Matrix3d expected_rotation;
  expected_rotation << 0.0, -1.0, 0.0,
                       1.0,  0.0, 0.0,
                       0.0,  0.0, 1.0;
  Eigen::Matrix3d expected_rpy;
  expected_rpy << 1.0, 0.0,  0.0,
                  0.0, 0.0, -1.0,
                  0.0, 1.0,  0.0;

  const auto calibration =
      ParseLidarImuCalibration(translation, rotation, rpy);

  EXPECT_TRUE((calibration.lidar_to_imu.block<3, 3>(0, 0).isApprox(
      expected_rotation)));
  EXPECT_TRUE((calibration.lidar_to_imu.block<3, 1>(0, 3).isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.10))));
  EXPECT_TRUE(calibration.mapping_imu_to_lidar_rotation.isApprox(
      expected_rotation.transpose()));
  EXPECT_TRUE(calibration.mapping_rpy_rotation.isApprox(expected_rpy));
  EXPECT_TRUE(calibration.lidar_to_imu.row(3).isApprox(
      Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)));
}

TEST(LidarImuCalibration, RejectsInvalidTranslationLength)
{
  const std::vector<double> short_translation(2, 0.0);
  const std::vector<double> long_translation(4, 0.0);
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

  EXPECT_THROW(
      ParseLidarImuCalibration(short_translation, identity, identity),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(long_translation, identity, identity),
      std::invalid_argument);
}

TEST(LidarImuCalibration, RejectsInvalidRotationLength)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> short_rotation(8, 0.0);
  const std::vector<double> long_rotation(10, 0.0);
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

  EXPECT_THROW(
      ParseLidarImuCalibration(translation, short_rotation, identity),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(translation, long_rotation, identity),
      std::invalid_argument);
}

TEST(LidarImuCalibration, RejectsInvalidRpyLength)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  const std::vector<double> short_rpy(8, 0.0);
  const std::vector<double> long_rpy(10, 0.0);

  EXPECT_THROW(
      ParseLidarImuCalibration(translation, identity, short_rpy),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(translation, identity, long_rpy),
      std::invalid_argument);
}
