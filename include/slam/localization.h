
#ifndef FUNNY_LIDAR_SLAM_LOCALIZATION_H
#define FUNNY_LIDAR_SLAM_LOCALIZATION_H

#include "slam/system.h"
#include "slam/config_parameters.h"
#include "common/data_type.h"
#include "common/compare_function.h"
#include "imu/pre_integration.h"
#include "registration/registration_interface.h"
#include "registration/icp_optimized.h"
#include "relocalization/scan_context.h"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <unordered_set>
#include <vector>

#include <pcl/kdtree/kdtree_flann.h>

class Localization {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit Localization(System* system_ptr);

    PCLPointCloudXYZI GetGlobalCloudMap();

    PCLPointCloudXYZI GetCurrentLidarCloudMap();

    void SetInitPose(const Mat4d& pose);

    void Run();

    [[nodiscard]] bool IsMapReady() const { return map_ready_.load(); }

private:
    bool Init();

    void LoadScanContextDb();

    [[nodiscard]] std::optional<Mat4d> TryAutoRelocalize(
        float* best_fitness_out = nullptr);

    void InitPreintegration();

    void InitMatcher();

    // Builds a fresh matcher without modifying matcher_. Automatic
    // relocalization uses a fresh instance for each ordered candidate.
    [[nodiscard]] std::shared_ptr<RegistrationInterface> CreateMatcher() const;

    void SetLidarPoseInformation();

    [[nodiscard]] Mat4d MapLidarPoseToMapImuPose(
        const Mat4d& map_lidar_pose) const;

    [[nodiscard]] Mat4d MapImuPoseToMapLidarPose(
        const Mat4d& map_imu_pose) const;

    void CacheResultToSystem(const LocalizationResult& result) const;

    void UpdateCurrentLidarCloud(const PCLPointCloudXYZI& lidar_cloud);

    /*!
     * @brief Centered on pose, load nearby maps
     * @param pose
     * @return local map cloud
     */
    PCLPointCloudXYZI::Ptr LoadLocalMap(const Mat4d& pose);

    PCLPointCloudXYZI::Ptr LoadCandidateLocalMap(const Mat4d& pose);

    void LoadTileMapIndices();

    static PCLPointCloudXYZI::Ptr LoadGlobalMap(const std::string& path);

    static PCLPointCloudXYZI::Ptr LoadTileMap(const Vec2i& tile_map_index);

    /*!
     * @brief Centered on index, load nearby tile map indices
     * @param index
     * @param step_size
     * @return
     */
    static std::vector<Vec2i> GenerateTileMapIndices(const Vec2i& index, int step_size);

    [[nodiscard]] NavStateData IntegrateImuMeasures(const NavStateData& last_nav_state) const;
    [[nodiscard]] NavStateData IntegrateImuMeasuresOdom(
        const NavStateData& last_odom_state) const;

    void Optimize(NavStateData& curr_nav_state, const Mat4d& lidar_pose);

    [[nodiscard]] std::optional<double> EstimateMapGroundZ(
        const Mat4d& map_lidar_pose);

private:
    std::shared_ptr<PreIntegration> pre_integration_ptr_ = nullptr;
    std::shared_ptr<PreIntegration> odom_pre_integration_ptr_ = nullptr;
    std::shared_ptr<RegistrationInterface> matcher_ = nullptr;
    PointcloudClusterPtr curr_cloud_cluster_ptr_ = nullptr;

    System* system_ptr_ = nullptr;

    std::map<Vec2i, PCLPointCloudXYZI::Ptr, LessVec2i> local_map_data_;
    std::set<Vec2i, LessVec2i> map_data_indices_;

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
    std::atomic_bool map_ready_{false};

    std::mutex mutex_lidar_cloud_;
    std::mutex mutex_global_map_cloud_;
    PCLPointCloudXYZI lidar_cloud_;
    PCLPointCloudXYZI::Ptr global_map_cloud_ptr_{nullptr};
    PCLPointCloudXYZI::Ptr global_map_xy_cloud_ptr_{nullptr};
    pcl::KdTreeFLANN<PCLPointXYZI>::Ptr global_map_xy_kdtree_{nullptr};
    PCLPointCloudXYZI::Ptr ground_map_cloud_ptr_{nullptr};
    PCLPointCloudXYZI::Ptr ground_map_xy_cloud_ptr_{nullptr};
    pcl::KdTreeFLANN<PCLPointXYZI>::Ptr ground_map_xy_kdtree_{nullptr};

    bool use_tile_map_{false}; // Not recommended
    bool need_update_local_map_{};
    double tile_map_grid_size_;
    double tile_map_grid_size_half_;

    std::vector<double> local_map_edge_;

    std::optional<Mat4d> init_pose_optional_;
    std::mutex mutex_init_pose_optional_;

    relocalization::ScanContext scan_context_;
    bool scan_context_db_loaded_{false};
    std::optional<Mat4d> consistency_ref_pose_;
    int consistency_hit_count_{0};
};

#endif //FUNNY_LIDAR_SLAM_LOCALIZATION_H
