#include "common/rigid_transform.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <type_traits>

TEST(RigidTransform, CompositionPreservesFrameTypes) {
  const BaseTLidar base_T_lidar = BaseTLidar::Identity();
  const LidarTImu lidar_T_imu = LidarTImu::Identity();
  const BaseTImu base_T_imu = base_T_lidar * lidar_T_imu;
  static_assert(std::is_same_v<decltype(base_T_imu), const BaseTImu>);
  EXPECT_TRUE(base_T_imu.Matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(RigidTransform, InverseSwapsFrameTypes) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const LidarTBase lidar_T_base = BaseTLidar(value).Inverse();
  static_assert(std::is_same_v<decltype(lidar_T_base), const LidarTBase>);
  EXPECT_TRUE((lidar_T_base * lidar_T_base.Inverse()).Matrix().isApprox(
      Eigen::Matrix4d::Identity()));
  EXPECT_TRUE(lidar_T_base.Translation().isApprox(
      Eigen::Vector3d(-1.0, -2.0, -3.0)));
}

TEST(RigidTransform, CompositionAccumulatesRotationAndTranslation) {
  Eigen::Isometry3d base_lidar = Eigen::Isometry3d::Identity();
  base_lidar.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  base_lidar.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);

  Eigen::Isometry3d lidar_imu = Eigen::Isometry3d::Identity();
  lidar_imu.translation() = Eigen::Vector3d(0.0, 2.0, 0.0);

  const BaseTImu base_T_imu =
      BaseTLidar(base_lidar) * LidarTImu(lidar_imu);

  // The lidar-frame lever arm is rotated into the base frame before it is
  // added to the base-frame translation: Rz(90 deg) * (0,2,0) + (1,0,0).
  EXPECT_TRUE(base_T_imu.Translation().isApprox(
      Eigen::Vector3d(-1.0, 0.0, 0.0)));
  EXPECT_TRUE(base_T_imu.Rotation().isApprox(base_lidar.linear()));
}

TEST(RigidTransform, IdentityRoundTripsThroughEigen) {
  const MapTLidar identity = MapTLidar::Identity();
  EXPECT_TRUE(identity.Matrix().isApprox(Eigen::Matrix4d::Identity()));
  EXPECT_TRUE(identity.Rotation().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_TRUE(identity.Translation().isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      MapTLidar::FromMatrix(identity.Matrix()).Matrix().isApprox(
          identity.Matrix()));
}

TEST(RigidTransform, RejectsInvalidHomogeneousMatrix) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(3, 0) = 1.0;
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}

TEST(RigidTransform, RejectsNonOrthogonalRotation) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 0) = 2.0;
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}

TEST(RigidTransform, RejectsReflectionRotation) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 0) = -1.0;
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}

TEST(RigidTransform, RejectsNaNValue) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}

TEST(RigidTransform, RejectsInfiniteValue) {
  Eigen::Matrix4d invalid = Eigen::Matrix4d::Identity();
  invalid(0, 3) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(MapTLidar::FromMatrix(invalid), std::invalid_argument);
}

TEST(RigidTransform, RejectsNonFiniteIsometryConstruction) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() =
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  // Brace initialization is required here: `LidarTBase(value)` would parse as
  // a declaration of a variable named `value` rather than a conversion, so the
  // validating constructor would never run.
  EXPECT_THROW(LidarTBase{value}, std::invalid_argument);
}

TEST(RigidTransform, AcceptsValidIsometryConstruction) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  const LidarTBase lidar_T_base{value};
  EXPECT_TRUE(lidar_T_base.Translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(RigidTransform, RejectsFloatPrecisionRotation)
{
  // The optimizer carries float RPY angles. Composing them into a float
  // rotation and widening it to double leaves ~1e-7 of rounding error, which
  // is far above the validator's tolerance -- so a publication boundary must
  // rebuild the rotation in double (as mapping's publishOdometry does) rather
  // than round-tripping the float Affine3f. This test pins that behaviour.
  const float roll = 0.031F, pitch = -0.017F, yaw = 1.234F;
  const Eigen::Matrix3f float_rotation =
      (Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()))
          .toRotationMatrix();

  Eigen::Isometry3d from_float = Eigen::Isometry3d::Identity();
  from_float.linear() = float_rotation.cast<double>();
  EXPECT_THROW(OdomTLidar{from_float}, std::invalid_argument);

  // The same angles composed in double are accepted.
  Eigen::Isometry3d from_double = Eigen::Isometry3d::Identity();
  from_double.linear() =
      (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  EXPECT_NO_THROW(OdomTLidar{from_double});
}
