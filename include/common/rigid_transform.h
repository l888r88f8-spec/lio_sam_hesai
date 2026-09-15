#ifndef LIO_SAM_HESAI_COMMON_RIGID_TRANSFORM_H
#define LIO_SAM_HESAI_COMMON_RIGID_TRANSFORM_H

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

// Empty compile-time frame tags. A transform RigidTransform3d<Parent, Child>
// carries the semantic that it maps Child-frame coordinates into the Parent
// frame (the `parent_T_child` convention). These tags add no runtime state.
struct MapFrame {};
struct OdomFrame {};
struct BaseFrame {};
struct ImuFrame {};
struct LidarFrame {};
// LocalIntegrationFrame is reserved for the localization-internal relative
// preintegration state. It is intentionally distinct from OdomFrame so the
// internal state cannot be mistaken for a ROS odom transform.
struct LocalIntegrationFrame {};

// Direction-safe rigid transform. Composition and inversion are the only
// allowed algebra and both preserve the frame types, so an invalid frame
// combination (for example multiplying Map_T_Lidar by Odom_T_Lidar) fails to
// compile. The wrapped value is an Eigen::Isometry3d.
template <typename Parent, typename Child>
class RigidTransform3d {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RigidTransform3d() : value_(Eigen::Isometry3d::Identity()) {}

  explicit RigidTransform3d(const Eigen::Isometry3d& value) : value_(value) {
    Validate(value_.matrix());
  }

  static RigidTransform3d Identity() { return {}; }

  static RigidTransform3d FromMatrix(const Eigen::Matrix4d& matrix) {
    Validate(matrix);
    // Assign the validated matrix directly: constructing an Isometry3d from a
    // raw Matrix4d would re-assert the orthonormality we already checked.
    Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
    value.matrix() = matrix;
    return RigidTransform3d(value);
  }

  template <typename Descendant>
  RigidTransform3d<Parent, Descendant> operator*(
      const RigidTransform3d<Child, Descendant>& rhs) const {
    return RigidTransform3d<Parent, Descendant>::FromMatrix(
        value_.matrix() * rhs.Matrix());
  }

  RigidTransform3d<Child, Parent> Inverse() const {
    return RigidTransform3d<Child, Parent>(value_.inverse());
  }

  const Eigen::Matrix4d& Matrix() const { return value_.matrix(); }
  // Reads the rotation block directly. Transform::rotation() would return a
  // quaternion and force a round trip back through a rotation matrix.
  Eigen::Matrix3d Rotation() const { return value_.linear(); }
  Eigen::Vector3d Translation() const { return value_.translation(); }

 private:
  static void Validate(const Eigen::Matrix4d& matrix) {
    if (!matrix.allFinite()) {
      throw std::invalid_argument(
          "rigid transform must contain only finite values");
    }
    const Eigen::RowVector4d homogeneous(0.0, 0.0, 0.0, 1.0);
    if (!matrix.row(3).isApprox(homogeneous)) {
      throw std::invalid_argument(
          "rigid transform must have a homogeneous last row");
    }
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
    if (!(rotation.transpose() * rotation).isApprox(
            Eigen::Matrix3d::Identity())) {
      throw std::invalid_argument(
          "rigid transform rotation must be orthonormal");
    }
    if (std::fabs(rotation.determinant() - 1.0) > 1e-6) {
      throw std::invalid_argument(
          "rigid transform rotation determinant must be +1");
    }
  }

  Eigen::Isometry3d value_;
};

using MapTLidar = RigidTransform3d<MapFrame, LidarFrame>;
using MapTImu = RigidTransform3d<MapFrame, ImuFrame>;
using MapTBase = RigidTransform3d<MapFrame, BaseFrame>;
using OdomTLidar = RigidTransform3d<OdomFrame, LidarFrame>;
using OdomTBase = RigidTransform3d<OdomFrame, BaseFrame>;
using BaseTLidar = RigidTransform3d<BaseFrame, LidarFrame>;
using LidarTBase = RigidTransform3d<LidarFrame, BaseFrame>;
using ImuTLidar = RigidTransform3d<ImuFrame, LidarFrame>;
using LidarTImu = RigidTransform3d<LidarFrame, ImuFrame>;
using BaseTImu = RigidTransform3d<BaseFrame, ImuFrame>;
using ImuTBase = RigidTransform3d<ImuFrame, BaseFrame>;
using LocalTImu = RigidTransform3d<LocalIntegrationFrame, ImuFrame>;

#endif  // LIO_SAM_HESAI_COMMON_RIGID_TRANSFORM_H
