#include "common/ros_pose_adapter.h"

#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>
#include <string>

namespace {

geometry_msgs::msg::PoseWithCovarianceStamped MakeInitialPose(
    const std::string& frame_id, double x, double y, double z,
    double qw = 1.0, double qx = 0.0, double qy = 0.0, double qz = 0.0) {
  geometry_msgs::msg::PoseWithCovarianceStamped message;
  message.header.frame_id = frame_id;
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.position.z = z;
  message.pose.pose.orientation.w = qw;
  message.pose.pose.orientation.x = qx;
  message.pose.pose.orientation.y = qy;
  message.pose.pose.orientation.z = qz;
  return message;
}

}  // namespace

TEST(RosPoseAdapter, ToPoseCarriesTranslationAndRotation) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.linear() =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  value.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const geometry_msgs::msg::Pose pose = RosPoseAdapter::ToPose(MapTBase(value));

  EXPECT_DOUBLE_EQ(pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(pose.position.y, 2.0);
  EXPECT_DOUBLE_EQ(pose.position.z, 3.0);
  EXPECT_DOUBLE_EQ(pose.orientation.x, 0.0);
  EXPECT_DOUBLE_EQ(pose.orientation.y, 0.0);
  EXPECT_NEAR(pose.orientation.z, std::sqrt(0.5), 1e-12);
  EXPECT_NEAR(pose.orientation.w, std::sqrt(0.5), 1e-12);
}

TEST(RosPoseAdapter, ToTransformStampedFillsFrames) {
  FrameIds frames{"map", "odom", "base_link", "imu_link", "hesai_lidar"};
  const auto tf = RosPoseAdapter::ToTransformStamped(
      MapTBase::Identity(), frames.map, frames.base_link, rclcpp::Time(123));

  EXPECT_EQ(tf.header.frame_id, "map");
  EXPECT_EQ(tf.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(tf.transform.rotation.w, 1.0);
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, 0.0);
  EXPECT_EQ(rclcpp::Time(tf.header.stamp).nanoseconds(), 123);
}

TEST(RosPoseAdapter, ToTransformStampedRejectsEmptyFrames) {
  EXPECT_THROW(RosPoseAdapter::ToTransformStamped(
                   MapTBase::Identity(), "", "base_link", rclcpp::Time(123)),
               std::invalid_argument);
  EXPECT_THROW(RosPoseAdapter::ToTransformStamped(
                   MapTBase::Identity(), "map", "", rclcpp::Time(123)),
               std::invalid_argument);
}

TEST(RosPoseAdapter, ToOdometryFillsFramesAndPose) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(4.0, 5.0, 6.0);

  const auto odometry = RosPoseAdapter::ToOdometry(
      MapTBase(value), "map", "base_link", rclcpp::Time(456));

  EXPECT_EQ(odometry.header.frame_id, "map");
  EXPECT_EQ(odometry.child_frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(odometry.header.stamp).nanoseconds(), 456);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.x, 4.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.y, 5.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.position.z, 6.0);
  EXPECT_DOUBLE_EQ(odometry.pose.pose.orientation.w, 1.0);
  // Covariance stays the caller's responsibility.
  EXPECT_DOUBLE_EQ(odometry.pose.covariance[0], 0.0);
}

TEST(RosPoseAdapter, ParseMapTBaseAcceptsConfiguredFrame) {
  std::string error;
  const auto parsed = RosPoseAdapter::ParseMapTBase(
      MakeInitialPose("map", 1.0, 2.0, 3.0), "map", &error);

  ASSERT_TRUE(parsed.has_value()) << error;
  EXPECT_TRUE(parsed->Translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(error.empty());
}

TEST(RosPoseAdapter, ParseMapTBaseAcceptsEmptyFrame) {
  std::string error;
  const auto parsed = RosPoseAdapter::ParseMapTBase(
      MakeInitialPose("", 1.0, 2.0, 3.0), "map", &error);

  ASSERT_TRUE(parsed.has_value()) << error;
  EXPECT_TRUE(parsed->Translation().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(RosPoseAdapter, ParseMapTBaseRejectsForeignFrame) {
  std::string error;
  auto message = MakeInitialPose("map", 1.0, 2.0, 3.0);
  EXPECT_TRUE(RosPoseAdapter::ParseMapTBase(message, "map", &error).has_value());

  message.header.frame_id = "odom";
  EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(message, "map", &error).has_value());
  EXPECT_NE(error.find("odom"), std::string::npos);
}

TEST(RosPoseAdapter, ParseMapTBaseRejectsEmptyMapFrame) {
  std::string error;
  EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(
                   MakeInitialPose("map", 0.0, 0.0, 0.0), "", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());
}

TEST(RosPoseAdapter, ParseMapTBaseRejectsNonFinitePosition) {
  for (const double bad :
       {std::numeric_limits<double>::quiet_NaN(),
        std::numeric_limits<double>::infinity()}) {
    std::string error;
    EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(
                     MakeInitialPose("map", bad, 0.0, 0.0), "map", &error)
                     .has_value())
      << "position value " << bad << " must be rejected";
    EXPECT_FALSE(error.empty());
  }
}

TEST(RosPoseAdapter, ParseMapTBaseRejectsZeroQuaternion) {
  std::string error;
  EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(
                   MakeInitialPose("map", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                   "map", &error)
                   .has_value());
  EXPECT_FALSE(error.empty());
}

TEST(RosPoseAdapter, ParseMapTBaseRejectsNonFiniteQuaternion) {
  std::string error;
  EXPECT_FALSE(
      RosPoseAdapter::ParseMapTBase(
          MakeInitialPose("map", 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                          std::numeric_limits<double>::quiet_NaN()),
          "map", &error)
          .has_value());
  EXPECT_FALSE(error.empty());
}

TEST(RosPoseAdapter, ParseMapTBaseNormalizesQuaternion) {
  std::string error;
  const auto parsed = RosPoseAdapter::ParseMapTBase(
      MakeInitialPose("map", 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0), "map",
      &error);

  ASSERT_TRUE(parsed.has_value()) << error;
  EXPECT_TRUE(
      parsed->Rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(RosPoseAdapter, ParseMapTBaseAcceptsNullErrorPointer) {
  EXPECT_TRUE(RosPoseAdapter::ParseMapTBase(
                  MakeInitialPose("map", 0.0, 0.0, 0.0), "map", nullptr)
                  .has_value());
  EXPECT_FALSE(RosPoseAdapter::ParseMapTBase(
                   MakeInitialPose("odom", 0.0, 0.0, 0.0), "map", nullptr)
                   .has_value());
}
