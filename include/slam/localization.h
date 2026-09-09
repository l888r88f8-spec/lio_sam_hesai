
#ifndef FUNNY_LIDAR_SLAM_LOCALIZATION_H
#define FUNNY_LIDAR_SLAM_LOCALIZATION_H

#include "slam/system.h"
#include "slam/config_parameters.h"
#include "common/data_type.h"
#include "imu/pre_integration.h"
#include "localization_health/reinitialization_control.h"
#include "localization_map/map_provider.h"
#include "registration/registration_interface.h"
#include "registration/icp_optimized.h"
#include "relocalization/scan_context.h"
#include "runtime/frame_processing_watchdog.h"

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

class Localization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Localization(System* system_ptr);

    PCLPointCloudXYZI GetGlobalCloudMap();

    PCLPointCloudXYZI GetLocalCloudMap();

    PCLPointCloudXYZI GetCurrentLidarCloudMap();

    void SetInitPose(const Mat4d& pose);

    void RequestAutomaticRelocalization();

    bool PrepareMap();

    void Run();

    [[nodiscard]] bool IsMapReady() const { return map_ready_.load(); }

    [[nodiscard]] double MapPreparationDurationMs() const {
        return map_preparation_duration_ms_.load();
    }

    [[nodiscard]] runtime::FrameProcessingSnapshot FrameProcessingSnapshot() const {
        return frame_processing_watchdog_.Snapshot();
    }

    struct MapLidarPrediction {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Mat4d map_lidar_pose{Mat4d::Identity()};
        TimeStampUs timestamp_{0U};
    };

    [[nodiscard]] std::optional<MapLidarPrediction> PredictMapLidarPose(
        const std::vector<IMUData>& imu_segment) const;

private:
    enum class InitializationResult {
        kInitialized,
        kAutomaticConsistencyPending,
        kFailed,
    };

    InitializationResult Init(std::uint64_t tf_continuity_epoch);

    void LoadScanContextDb();

    [[nodiscard]] std::optional<Mat4d> TryAutoRelocalize(
        float* best_fitness_out = nullptr);

    void InitPreintegration();

    [[nodiscard]] PreIntegration::ConfigPara MakePreIntegrationConfig() const;

    void UpdateTfPredictionAnchor(const NavStateData& nav_state);

    void InitMatcher();

    bool DiscardTimedOutFrameIfNeeded(std::uint64_t now_us);

    bool DiscardTimedOutFrameIfNeeded();

    // Builds a fresh matcher without modifying matcher_. Automatic
    // relocalization uses a fresh instance for each ordered candidate.
    [[nodiscard]] std::shared_ptr<RegistrationInterface> CreateMatcher() const;

    void SetLidarPoseInformation();

    [[nodiscard]] Mat4d MapLidarPoseToMapImuPose(
        const Mat4d& map_lidar_pose) const;

    [[nodiscard]] Mat4d MapImuPoseToMapLidarPose(
        const Mat4d& map_imu_pose) const;

    void CacheResultToSystem(
        const LocalizationResult& result,
        std::uint64_t tf_continuity_epoch) const;

    void UpdateCurrentLidarCloud(const PCLPointCloudXYZI& lidar_cloud);

    /*!
     * @brief Centered on pose, load nearby maps
     * @param pose
     * @return local map cloud
     */
    PCLPointCloudXYZI::ConstPtr LoadLocalMap(
        const Mat4d& pose, bool force_rebuild = false);

    void UpdateActiveMapSnapshot(
        std::shared_ptr<const localization_map::MapSnapshot> snapshot);

    [[nodiscard]] NavStateData IntegrateImuMeasures(const NavStateData& last_nav_state) const;
    [[nodiscard]] NavStateData IntegrateImuMeasuresOdom(
        const NavStateData& last_odom_state) const;

    void Optimize(NavStateData& curr_nav_state, const Mat4d& lidar_pose);

    [[nodiscard]] std::optional<double> EstimateMapGroundZ(
        const Mat4d& map_lidar_pose,
        const std::shared_ptr<const localization_map::MapSnapshot>& snapshot =
            nullptr);

private:
    struct TfPredictionAnchor {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        NavStateData nav_state;
        Mat4d map_lidar_pose{Mat4d::Identity()};
        TimeStampUs timestamp{0U};
    };

    std::shared_ptr<PreIntegration> pre_integration_ptr_ = nullptr;
    std::shared_ptr<PreIntegration> odom_pre_integration_ptr_ = nullptr;
    std::shared_ptr<RegistrationInterface> matcher_ = nullptr;
    PointcloudClusterPtr curr_cloud_cluster_ptr_ = nullptr;

    System* system_ptr_ = nullptr;
    runtime::FrameProcessingWatchdog frame_processing_watchdog_;

    mutable std::mutex mutex_tf_prediction_anchor_;
    std::optional<TfPredictionAnchor> tf_prediction_anchor_;

    // navigation state and bias
    NavStateData last_nav_state_;
    NavStateData last_odom_state_;

    Mat4d delta_pose_ = Mat4d::Identity();
    Mat4d curr_pose_ = Mat4d::Identity();
    Mat4d last_pose_ = Mat4d::Identity();

    // information
    Mat6d lidar_pose_info_ = Mat6d::Identity();

    TimeStampUs curr_time_us_ = 0u;

    std::atomic_bool has_init_{false};
    localization_health::ReinitializationControl reinitialization_control_;
    std::atomic_bool map_ready_{false};
    std::atomic<double> map_preparation_duration_ms_{0.0};

    std::mutex mutex_lidar_cloud_;
    std::mutex mutex_map_snapshot_;
    PCLPointCloudXYZI lidar_cloud_;
    std::unique_ptr<localization_map::MapProvider> map_provider_;
    localization_map::MapMode selected_map_mode_{
        localization_map::MapMode::kGlobal};
    std::filesystem::path resolved_map_path_;
    std::shared_ptr<const localization_map::MapSnapshot> active_map_snapshot_;
    PCLPointCloudXYZI::Ptr active_ground_xy_cloud_{nullptr};
    pcl::KdTreeFLANN<PCLPointXYZI>::Ptr active_ground_xy_kdtree_{nullptr};

    bool need_update_local_map_{};
    bool force_local_map_rebuild_{false};

    std::optional<Mat4d> init_pose_optional_;
    std::mutex mutex_init_pose_optional_;

    relocalization::ScanContext scan_context_;
    bool scan_context_db_loaded_{false};
    std::optional<Mat4d> consistency_ref_pose_;
    int consistency_hit_count_{0};
};

#endif //FUNNY_LIDAR_SLAM_LOCALIZATION_H
