
#ifndef FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H
#define FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H

#include "common/data_type.h"
#include "common/rigid_transform.h"

#include <limits>
#include <memory>

struct DataBase {
    TimeStampUs timestamp_ = 0u; // us
};

struct IMUData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Vec3d angular_velocity_ = Vec3d::Zero();
    Vec3d linear_acceleration_ = Vec3d::Zero();
    Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
};

struct GPSData : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    double longitude_{0.0};
    double latitude_{0.0};
    double altitude_{0.0};

    Vec3d local_xyz_ = Vec3d::Zero();
    Vec3d local_xyz_velocity_ = Vec3d::Zero();
    Mat3d local_orientation = Mat3d::Identity();
};

struct NavStateData : DataBase {
    using Ptr = std::shared_ptr<NavStateData>;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void SetPose(const Mat4d& pose) {
        R_ = pose.block<3, 3>(0, 0);
        P_ = pose.block<3, 1>(0, 3);
    }

    [[nodiscard]] Mat4d Pose() const {
        Mat4d T = Mat4d::Identity();
        T.block<3, 3>(0, 0) = R_;
        T.block<3, 1>(0, 3) = P_;
        return T;
    }

    Vec3d P_ = Vec3d::Zero();
    Vec3d V_ = Vec3d::Zero();
    Mat3d R_ = Mat3d::Identity();

    Vec3d bg_ = Vec3d::Zero(); // gyro bias
    Vec3d ba_ = Vec3d::Zero(); // accel bias

    Mat15d info_ = Mat15d::Identity();
};

struct LocalizationResult : DataBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // map_T_lidar is the matcher result; System converts it to map_T_base for
    // output. local_T_imu is the localization-internal relative preintegration
    // state: it is deliberately not an OdomFrame and must never reach a ROS
    // frame ID or TF.
    MapTLidar map_T_lidar{MapTLidar::Identity()};
    LocalTImu local_T_imu{LocalTImu::Identity()};
    bool has_map_ground_z = false;
    double map_ground_z = 0.0;

    // Health observability defaults protect legacy result paths that do not
    // populate these fields.
    float reg_fitness = std::numeric_limits<float>::infinity();
    int consistency_hit = 0;
    int consistency_required = 0;
    bool is_tracking = false;
    TimeStampUs health_arrival_steady_us = 0U;
    std::uint64_t tf_continuity_epoch_{0U};
};

#endif //FUNNY_LIDAR_SLAM_SENSOR_DATA_TYPE_H
