#include "lidar/lidar_distortion_corrector.h"
#include "slam/localization_frame_conversion.h"
#include "common/lidar_imu_calibration.h"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <type_traits>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;

static_assert(std::is_constructible_v<LidarDistortionCorrector, ImuTLidar>);
static_assert(!std::is_constructible_v<LidarDistortionCorrector, LidarTImu>);

Mat3d Rpy(double roll, double pitch, double yaw) {
  return (Eigen::AngleAxisd(yaw, Vec3d::UnitZ()) *
          Eigen::AngleAxisd(pitch, Vec3d::UnitY()) *
          Eigen::AngleAxisd(roll, Vec3d::UnitX())).toRotationMatrix();
}

LidarImuCalibration Calibration(const Vec3d& translation,
                                const Mat3d& rotation) {
  std::vector<double> row_major_rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      row_major_rotation.push_back(rotation(row, column));
    }
  }
  return ParseLidarImuCalibration(
      {translation.x(), translation.y(), translation.z()},
      row_major_rotation, {1., 0., 0., 0., 1., 0., 0., 0., 1.});
}

LidarImuCalibration TiltedCalibration() {
  return Calibration(Vec3d(0.4, -0.2, 0.1),
                     Rpy(20. * kPi / 180., -10. * kPi / 180., kPi / 6.));
}

std::shared_ptr<DataSearcher<IMUData>> ImuTrajectory() {
  IMUData first;
  first.timestamp_ = 1000000;
  first.orientation_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(20. * kPi / 180., Vec3d::UnitZ()));
  IMUData last;
  last.timestamp_ = 1500000;
  last.orientation_ = Eigen::Quaterniond(
      Eigen::AngleAxisd(80. * kPi / 180., Vec3d::UnitZ()));
  return std::make_shared<DataSearcher<IMUData>>(
      std::vector<IMUData>{first, last});
}

Vec3d Deskew(LidarDistortionCorrector& corrector,
            const Vec3d& point, float relative_time) {
  float x = 0., y = 0., z = 0.;
  EXPECT_TRUE(corrector.ProcessPoint(point.x(), point.y(), point.z(),
                                    x, y, z, relative_time));
  return Vec3d(x, y, z);
}

void ExpectVectorNear(const Vec3d& actual, const Vec3d& expected) {
  EXPECT_NEAR(actual.x(), expected.x(), 1e-6);
  EXPECT_NEAR(actual.y(), expected.y(), 1e-6);
  EXPECT_NEAR(actual.z(), expected.z(), 1e-6);
}

}  // namespace

TEST(LocalizationExtrinsicDirection, DefaultDeskewOriginIsAboveImu) {
  LidarDistortionCorrector corrector(
      Calibration(Vec3d(0., 0., 0.1), Mat3d::Identity()).imu_T_lidar);
  corrector.SetDataSearcher(ImuTrajectory());
  ASSERT_TRUE(corrector.SetRefTime(1000000));
  ExpectVectorNear(Deskew(corrector, Vec3d::Zero(), 0.), Vec3d(0., 0., 0.1));
}

TEST(LocalizationExtrinsicDirection, DeskewTransformsLidarPointIntoImu) {
  const auto calibration = TiltedCalibration();
  LidarDistortionCorrector corrector(calibration.imu_T_lidar);
  corrector.SetDataSearcher(ImuTrajectory());
  ASSERT_TRUE(corrector.SetRefTime(1000000));
  const Vec3d point(1.2, -0.8, 2.3);
  const Vec3d expected = calibration.imu_T_lidar.Rotation() * point +
                        Vec3d(0.4, -0.2, 0.1);
  ExpectVectorNear(Deskew(corrector, point, 0.), expected);
}

TEST(LocalizationExtrinsicDirection, DeskewInterpolatesMotionAfterCalibration) {
  const auto calibration = TiltedCalibration();
  LidarDistortionCorrector corrector(calibration.imu_T_lidar);
  corrector.SetDataSearcher(ImuTrajectory());
  ASSERT_TRUE(corrector.SetRefTime(1000000));
  const Vec3d point(1.2, -0.8, 2.3);
  // The actual search/interpolator produces 50 deg at 1.25 seconds, relative
  // to the 20 deg reference. Rotate the calibrated point, including its arm.
  const Vec3d expected = Eigen::AngleAxisd(kPi / 6., Vec3d::UnitZ()) *
      (calibration.imu_T_lidar.Rotation() * point + Vec3d(0.4, -0.2, 0.1));
  ExpectVectorNear(Deskew(corrector, point, 0.25f), expected);
}

TEST(LocalizationExtrinsicDirection, DefaultProductionPoseRoundTripHasNoZDrift) {
  const auto calibration = Calibration(Vec3d(0., 0., 0.1), Mat3d::Identity());
  const Mat4d map_T_imu = localization_frames::MapLidarPoseToMapImuPose(
      MapTLidar::Identity(), calibration.lidar_T_imu);
  EXPECT_NEAR(map_T_imu(2, 3), -0.1, 1e-12);
  const Mat4d returned = localization_frames::MapImuPoseToMapLidarPose(
      map_T_imu, calibration.imu_T_lidar);
  EXPECT_NEAR(returned(2, 3), 0., 1e-12);
}

TEST(LocalizationExtrinsicDirection, ProductionPoseRoundTripHasNoYawDrift) {
  const auto calibration = Calibration(Vec3d(0.4, -0.2, 0.1),
                                       Rpy(0., 0., kPi / 6.));
  const Mat4d map_T_imu = localization_frames::MapLidarPoseToMapImuPose(
      MapTLidar::Identity(), calibration.lidar_T_imu);
  EXPECT_NEAR(std::atan2(map_T_imu(1, 0), map_T_imu(0, 0)), -kPi / 6., 1e-12);
  const Mat4d returned = localization_frames::MapImuPoseToMapLidarPose(
      map_T_imu, calibration.imu_T_lidar);
  EXPECT_NEAR(std::atan2(returned(1, 0), returned(0, 0)), 0., 1e-12);
  ExpectVectorNear(returned.block<3, 1>(0, 3), Vec3d::Zero());
}

TEST(LocalizationExtrinsicDirection, ProductionPoseConvertsRotatedLeverArm) {
  const auto calibration = TiltedCalibration();
  Mat4d map_lidar_pose = Mat4d::Identity();
  const Mat3d map_lidar_rotation = Rpy(-0.1, 0.3, -0.7);
  const Vec3d map_lidar_translation(10., -4., 3.);
  map_lidar_pose.block<3, 3>(0, 0) = map_lidar_rotation;
  map_lidar_pose.block<3, 1>(0, 3) = map_lidar_translation;
  const Mat3d lidar_imu_rotation = calibration.imu_T_lidar.Rotation().transpose();
  const Vec3d lidar_imu_translation =
      -lidar_imu_rotation * Vec3d(0.4, -0.2, 0.1);
  const Mat4d map_imu_pose = localization_frames::MapLidarPoseToMapImuPose(
      MapTLidar::FromMatrix(map_lidar_pose), calibration.lidar_T_imu);
  EXPECT_TRUE((map_imu_pose.block<3, 3>(0, 0).isApprox(
      map_lidar_rotation * lidar_imu_rotation, 1e-12)));
  ExpectVectorNear(map_imu_pose.block<3, 1>(0, 3),
      map_lidar_translation + map_lidar_rotation * lidar_imu_translation);
  const Mat4d returned = localization_frames::MapImuPoseToMapLidarPose(
      map_imu_pose, calibration.imu_T_lidar);
  EXPECT_TRUE(returned.isApprox(map_lidar_pose, 1e-12));
}

TEST(LocalizationExtrinsicDirection, ScanContextDescriptorGetsLidarGeometry) {
  const auto calibration = TiltedCalibration();
  PCLPointCloudXYZI imu_cloud;
  const std::vector<Vec3d> lidar_points = {
      Vec3d::Zero(), Vec3d(1.2, -0.8, 2.3), Vec3d(-3., 4., 1.)};
  for (const auto& lidar_point : lidar_points) {
    const Vec3d imu_point = calibration.imu_T_lidar.Rotation() * lidar_point +
                           Vec3d(0.4, -0.2, 0.1);
    PCLPointXYZI point;
    point.getVector3fMap() = imu_point.cast<float>();
    point.intensity = 42.;
    imu_cloud.push_back(point);
  }
  const auto query = localization_frames::ScanContextLidarCloud(
      imu_cloud, calibration.lidar_T_imu);
  ASSERT_EQ(query.size(), lidar_points.size());
  for (std::size_t index = 0; index < query.size(); ++index) {
    ExpectVectorNear(query[index].getVector3fMap().cast<double>(),
                     lidar_points[index]);
    EXPECT_FLOAT_EQ(query[index].intensity, 42.);
  }
}

TEST(LocalizationExtrinsicDirection, ScanContextCombinesImuTiltWithLidarYaw) {
  const auto calibration = TiltedCalibration();
  Mat4d keyframe_pose = Mat4d::Identity();
  keyframe_pose.block<3, 3>(0, 0) = Rpy(0., 0., 0.8);
  keyframe_pose.block<3, 1>(0, 3) = Vec3d(7., -2., 1.);
  const double yaw_shift = -0.2;
  const double roll = -0.15;
  const double pitch = 0.25;
  const Mat3d expected_nominal_imu_rotation = Rpy(0., 0., 0.6) *
      calibration.imu_T_lidar.Rotation().transpose();
  const double expected_imu_yaw = std::atan2(
      expected_nominal_imu_rotation(1, 0), expected_nominal_imu_rotation(0, 0));
  const Mat3d expected_map_imu_rotation = Rpy(roll, pitch, expected_imu_yaw);
  const Mat4d candidate = localization_frames::ScanContextCandidateLidarPose(
      keyframe_pose, yaw_shift, roll, pitch, calibration.imu_T_lidar);
  EXPECT_TRUE((candidate.block<3, 3>(0, 0).isApprox(
      expected_map_imu_rotation * calibration.imu_T_lidar.Rotation(), 1e-12)));
  ExpectVectorNear(candidate.block<3, 1>(0, 3), Vec3d(7., -2., 1.));
}

TEST(LocalizationExtrinsicDirection, ScanContextMatcherSeedIsMapImuPose) {
  const auto calibration = TiltedCalibration();
  Mat4d candidate = Mat4d::Identity();
  const Mat3d candidate_rotation = Rpy(-0.15, 0.25, 0.6);
  candidate.block<3, 3>(0, 0) = candidate_rotation;
  candidate.block<3, 1>(0, 3) = Vec3d(7., -2., 1.);
  const Mat4d seed = localization_frames::MapLidarPoseToMapImuPose(
      MapTLidar::FromMatrix(candidate), calibration.lidar_T_imu);
  const Mat3d lidar_imu_rotation = calibration.imu_T_lidar.Rotation().transpose();
  EXPECT_TRUE((seed.block<3, 3>(0, 0).isApprox(
      candidate_rotation * lidar_imu_rotation, 1e-12)));
  ExpectVectorNear(seed.block<3, 1>(0, 3), Vec3d(7., -2., 1.) -
      candidate_rotation * lidar_imu_rotation * Vec3d(0.4, -0.2, 0.1));
}
