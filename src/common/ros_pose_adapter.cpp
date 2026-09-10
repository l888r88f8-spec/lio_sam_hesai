#include "common/ros_pose_adapter.h"

#include <Eigen/Geometry>

#include <cmath>

std::optional<MapTBase> RosPoseAdapter::ParseMapTBase(
    const geometry_msgs::msg::PoseWithCovarianceStamped& message,
    const std::string& map_frame, std::string* error) {
  const auto fail = [error](const std::string& reason) {
    if (error != nullptr) {
      *error = reason;
    }
    return std::optional<MapTBase>{};
  };

  if (map_frame.empty()) {
    return fail("configured map frame must not be empty");
  }

  const std::string& frame_id = message.header.frame_id;
  if (!frame_id.empty() && frame_id != map_frame) {
    return fail("initial pose frame_id must be empty or '" + map_frame +
                "', got '" + frame_id + "'");
  }

  const geometry_msgs::msg::Point& position = message.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) {
    return fail("initial pose position must be finite");
  }

  const geometry_msgs::msg::Quaternion& orientation =
      message.pose.pose.orientation;
  Eigen::Quaterniond normalized(orientation.w, orientation.x, orientation.y,
                                orientation.z);
  if (!normalized.coeffs().allFinite()) {
    return fail("initial pose orientation must be finite");
  }
  if (normalized.norm() < 1e-9) {
    return fail("initial pose orientation must be a non-zero quaternion");
  }
  normalized.normalize();

  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.linear() = normalized.toRotationMatrix();
  value.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  return MapTBase::FromMatrix(value.matrix());
}
