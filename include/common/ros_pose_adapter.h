#ifndef LIO_SAM_HESAI_COMMON_ROS_POSE_ADAPTER_H
#define LIO_SAM_HESAI_COMMON_ROS_POSE_ADAPTER_H

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <optional>
#include <stdexcept>
#include <string>

#include "common/rigid_transform.h"

// Frame identifiers shared by mapping and localization. Mode-specific frame
// defaults live with their own configuration; these are the common defaults.
struct FrameIds {
  std::string map{"map"};
  std::string odometry{"odom"};
  std::string base_link{"base_link"};
  std::string imu{"imu_link"};
  std::string lidar{"hesai_lidar"};
};

// The single place that turns direction-safe transforms into ROS messages and
// validates incoming pose messages. Algorithm code must not assemble message
// frames or quaternions by hand.
class RosPoseAdapter {
 public:
  template <typename Parent, typename Child>
  static geometry_msgs::msg::Pose ToPose(
      const RigidTransform3d<Parent, Child>& transform) {
    const Eigen::Vector3d translation = transform.Translation();
    Eigen::Quaterniond rotation(transform.Rotation());
    rotation.normalize();

    geometry_msgs::msg::Pose pose;
    pose.position.x = translation.x();
    pose.position.y = translation.y();
    pose.position.z = translation.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
  }

  template <typename Parent, typename Child>
  static geometry_msgs::msg::TransformStamped ToTransformStamped(
      const RigidTransform3d<Parent, Child>& transform,
      const std::string& parent, const std::string& child,
      const rclcpp::Time& stamp) {
    ValidateFrameNames(parent, child);

    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    const geometry_msgs::msg::Pose pose = ToPose(transform);
    message.transform.translation.x = pose.position.x;
    message.transform.translation.y = pose.position.y;
    message.transform.translation.z = pose.position.z;
    message.transform.rotation = pose.orientation;
    return message;
  }

  template <typename Parent, typename Child>
  static nav_msgs::msg::Odometry ToOdometry(
      const RigidTransform3d<Parent, Child>& transform,
      const std::string& parent, const std::string& child,
      const rclcpp::Time& stamp) {
    ValidateFrameNames(parent, child);

    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    message.pose.pose = ToPose(transform);
    return message;
  }

  // Interprets an incoming initial-pose message as map_T_base. An empty
  // header.frame_id is accepted as the configured map frame for ROS
  // compatibility; any other frame is rejected. On failure the returned
  // optional is empty and `error` (when provided) explains why.
  static std::optional<MapTBase> ParseMapTBase(
      const geometry_msgs::msg::PoseWithCovarianceStamped& message,
      const std::string& map_frame, std::string* error);

 private:
  static void ValidateFrameNames(const std::string& parent,
                                 const std::string& child) {
    if (parent.empty() || child.empty()) {
      throw std::invalid_argument(
          "ROS pose adapter requires non-empty parent and child frames");
    }
  }
};

#endif  // LIO_SAM_HESAI_COMMON_ROS_POSE_ADAPTER_H
