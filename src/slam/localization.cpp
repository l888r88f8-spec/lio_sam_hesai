#include "slam/localization.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <filesystem>
#include <iomanip>
#include <limits>

#include <g2o/core/sparse_optimizer.h>

#include "common/map_artifact_layout.h"
#include "localization_map/global_map_provider.h"
#include "localization_map/tiled_map_provider.h"
#include "optimization/g2o/accel_bias_rw_edge.h"
#include "optimization/g2o/gyro_bias_rw_edge.h"
#include "optimization/g2o/g2o_optimizer_header.h"
#include "optimization/g2o/position_edge.h"
#include "optimization/g2o/pre_integration_edge.h"
#include "optimization/g2o/prior_nav_state_edge.h"
#include "optimization/g2o/rotation_edge.h"
#include "registration/icp_optimized.h"
#include "registration/incremental_ndt.h"
#include "registration/loam_point_to_plane_ivox.h"
#include "registration/loam_point_to_plane_kdtree.h"
#include "relocalization/acceptance_policy.h"
#include "slam/localization_frame_conversion.h"

namespace {

std::uint64_t SteadyNowMicroseconds() {
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

// Manifest path for a configured tiled root, or nullopt when the root does
// not name a directory and therefore cannot form a tiled layout.
std::optional<std::filesystem::path> TiledManifestFor(
    const std::filesystem::path& tiled_root) {
  if (tiled_root.filename().empty() && tiled_root.parent_path().empty()) {
    return std::nullopt;
  }
  try {
    return MapArtifactLayout::FromTiledRoot(tiled_root).TiledManifest();
  } catch (const std::invalid_argument&) {
    return std::nullopt;
  }
}

std::optional<localization_map::MapMode> ResolveMapMode(
    const std::string& configured_mode, const std::filesystem::path& map_path,
    std::string* error) {
  namespace fs = std::filesystem;
  std::error_code status_error;
  const bool is_file = fs::is_regular_file(map_path, status_error);
  status_error.clear();
  const bool is_directory = fs::is_directory(map_path, status_error);
  const auto fail = [&](const std::string& message) {
    if (error != nullptr) {
      *error = message;
    }
    return std::optional<localization_map::MapMode>{};
  };

  if (configured_mode == "global") {
    return is_file ? std::optional{localization_map::MapMode::kGlobal}
                   : fail("global map path must be a regular file");
  }
  if (configured_mode == "tiled") {
    status_error.clear();
    const std::optional<std::filesystem::path> manifest =
        TiledManifestFor(map_path);
    if (!is_directory || !manifest.has_value() ||
        !fs::is_regular_file(*manifest, status_error)) {
      return fail("tiled map path must contain manifest.yaml");
    }
    return localization_map::MapMode::kTiled;
  }
  if (configured_mode != "auto") {
    return fail("map mode must be auto, global, or tiled");
  }
  if (is_file) {
    return localization_map::MapMode::kGlobal;
  }
  if (is_directory) {
    return localization_map::MapMode::kTiled;
  }
  return fail("map path is neither a regular file nor a directory");
}

}  // namespace

Localization::Localization(System* system_ptr,
                           std::shared_ptr<const SlamConfig> config)
    : system_ptr_(system_ptr),
      config_(std::move(config)),
      frame_processing_watchdog_(system_ptr->FrameProcessingTimeoutUs()) {
  SetLidarPoseInformation();
  InitMatcher();
}

void Localization::SetInitPose(const MapTLidar& map_T_lidar) {
  [[maybe_unused]] auto manual_initialization =
      reinitialization_control_.BeginManualInitialization();
  std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
  // RViz/System provides map_T_lidar, while registration consumes the
  // deskewed IMU-frame cloud and therefore uses map_T_imu internally.
  init_pose_optional_ = MapLidarPoseToMapImuPose(map_T_lidar);
  has_init_ = false;
}

void Localization::RequestAutomaticRelocalization() {
  reinitialization_control_.RequestAutomaticRelocalization();
}

// map_T_imu = map_T_lidar * lidar_T_imu
Mat4d Localization::MapLidarPoseToMapImuPose(
    const MapTLidar& map_T_lidar) const {
  return localization_frames::MapLidarPoseToMapImuPose(
      map_T_lidar, config_->calibration.lidar_T_imu);
}

// map_T_lidar = map_T_imu * imu_T_lidar
Mat4d Localization::MapImuPoseToMapLidarPose(
    const Mat4d& map_imu_pose) const {
  return localization_frames::MapImuPoseToMapLidarPose(
      map_imu_pose, config_->calibration.imu_T_lidar);
}

std::optional<Localization::MapLidarPrediction>
Localization::PredictMapLidarPose(
    const std::vector<IMUData>& imu_segment) const {
  std::optional<TfPredictionAnchor> anchor;
  {
    std::lock_guard<std::mutex> lock(mutex_tf_prediction_anchor_);
    anchor = tf_prediction_anchor_;
  }

  if (!anchor.has_value() || imu_segment.size() < 2) {
    return std::nullopt;
  }

  for (std::size_t index = 1; index < imu_segment.size(); ++index) {
    if (imu_segment[index].timestamp_ <= imu_segment[index - 1].timestamp_) {
      return std::nullopt;
    }
  }

  if (imu_segment.front().timestamp_ != anchor->timestamp ||
      imu_segment.back().timestamp_ <= anchor->timestamp) {
    return std::nullopt;
  }

  PreIntegration predictor(MakePreIntegrationConfig());
  predictor.SetGyroAccBias(anchor->nav_state.bg_, anchor->nav_state.ba_);
  predictor.IntegrateDataSegment(imu_segment);
  if (!std::isfinite(predictor.GetTotalIntegrationTime()) ||
      predictor.GetTotalIntegrationTime() <= 0.0) {
    return std::nullopt;
  }

  NavStateData predicted = predictor.Predict(anchor->nav_state);
  predicted.timestamp_ = imu_segment.back().timestamp_;
  if (predicted.timestamp_ <= anchor->timestamp ||
      !predicted.P_.allFinite() || !predicted.V_.allFinite() ||
      !predicted.R_.allFinite()) {
    return std::nullopt;
  }

  return MapLidarPrediction{
      MapTLidar::FromMatrix(MapImuPoseToMapLidarPose(predicted.Pose())),
      predicted.timestamp_};
}

void Localization::SetLidarPoseInformation() {
  const auto& lidar_rot_std =
      config_->localization.lidar.rotation_noise_std;
  const auto& lidar_posi_std =
      config_->localization.lidar.position_noise_std;

  lidar_pose_info_.setZero();
  lidar_pose_info_.block<3, 3>(0, 0) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_rot_std, 2.0);
  lidar_pose_info_.block<3, 3>(3, 3) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_posi_std, 2.0);
}

std::shared_ptr<RegistrationInterface> Localization::CreateMatcher() const {
  if (config_->localization.frontend.registration_and_searcher_mode ==
      kIcpOptimized) {
    return std::make_shared<IcpOptimized<double>>(
        config_->localization.frontend.registration.optimization_iter_num,
        config_->localization.frontend.registration.local_map_size,
        static_cast<float>(config_->localization.frontend.registration
                               .local_map_cloud_filter_size),
        static_cast<float>(config_->localization.frontend.registration
                               .source_cloud_filter_size),
        static_cast<double>(
            config_->localization.frontend.registration.point_search_thres),
        static_cast<double>(
            config_->localization.frontend.registration.position_converge_thres),
        static_cast<double>(
            config_->localization.frontend.registration.rotation_converge_thres),
        static_cast<double>(
            config_->localization.frontend.registration.keyframe_delta_rotation),
        static_cast<double>(
            config_->localization.frontend.registration.keyframe_delta_distance),
        true);
  } else if (config_->localization.frontend.registration_and_searcher_mode ==
             kIncrementalNDT) {
    return std::make_shared<IncrementalNDT>(
        config_->localization.frontend.registration.ndt_voxel_size,
        config_->localization.frontend.registration.ndt_outlier_threshold,
        config_->localization.frontend.registration.source_cloud_filter_size,
        static_cast<double>(
            config_->localization.frontend.registration.rotation_converge_thres),
        static_cast<double>(
            config_->localization.frontend.registration.position_converge_thres),
        config_->localization.frontend.registration.ndt_min_points_in_voxel,
        config_->localization.frontend.registration.ndt_max_points_in_voxel,
        config_->localization.frontend.registration.ndt_min_effective_pts,
        config_->localization.frontend.registration.ndt_capacity,
        config_->localization.frontend.registration.optimization_iter_num, true);
  } else if (config_->localization.frontend.registration_and_searcher_mode ==
             kPointToPlane_IVOX) {
    return std::make_shared<LoamPointToPlaneIVOX<double>>(
        config_->localization.frontend.registration.point_to_planar_thres,
        config_->localization.frontend.registration.position_converge_thres,
        config_->localization.frontend.registration.rotation_converge_thres,
        config_->localization.frontend.registration.optimization_iter_num, true);
  } else if (config_->localization.frontend.registration_and_searcher_mode ==
             kPointToPlane_KdTree) {
    return std::make_shared<LoamPointToPlaneKdtree<double>>(
        config_->localization.frontend.registration.point_to_planar_thres,
        config_->localization.frontend.registration.position_converge_thres,
        config_->localization.frontend.registration.rotation_converge_thres,
        config_->localization.frontend.registration.keyframe_delta_rotation,
        config_->localization.frontend.registration.keyframe_delta_distance,
        config_->localization.frontend.registration.local_map_size,
        config_->localization.frontend.registration.local_map_cloud_filter_size,
        config_->localization.frontend.registration.optimization_iter_num, true);
  } else {
    LOG(FATAL) << "Unsupported registration type! "
                  "You can choose one of the following: "
                  "IcpOptimized, kIncrementalNDT";
  }
  return nullptr;
}

void Localization::InitMatcher() { matcher_ = CreateMatcher(); }

bool Localization::DiscardTimedOutFrameIfNeeded(std::uint64_t now_us) {
  if (!frame_processing_watchdog_.Exceeded(now_us)) {
    return false;
  }

  frame_processing_watchdog_.Complete(now_us, true);
  InitMatcher();
  InitPreintegration();
  if (pre_integration_ptr_) {
    pre_integration_ptr_->SetGyroAccBias(last_nav_state_.bg_,
                                         last_nav_state_.ba_);
  }
  if (local_pre_integration_ptr_) {
    local_pre_integration_ptr_->SetGyroAccBias(last_local_state_.bg_,
                                               last_local_state_.ba_);
  }
  need_update_local_map_ = true;
  force_local_map_rebuild_ = true;
  delta_pose_ = Mat4d::Identity();
  LOG(WARNING) << "Drop overdue localization frame.";
  return true;
}

bool Localization::DiscardTimedOutFrameIfNeeded() {
  return DiscardTimedOutFrameIfNeeded(
      runtime::FrameProcessingWatchdog::SteadyNowMicroseconds());
}

PreIntegration::ConfigPara Localization::MakePreIntegrationConfig() const {
  PreIntegration::ConfigPara config_para;
  config_para.init_acc_bias_ =
      Vec3d(config_->localization.imu.init_acc_bias,
            config_->localization.imu.init_acc_bias,
            config_->localization.imu.init_acc_bias);
  config_para.init_gyro_bias_ =
      Vec3d(config_->localization.imu.init_gyro_bias,
            config_->localization.imu.init_gyro_bias,
            config_->localization.imu.init_gyro_bias);
  config_para.gravity_ = system_ptr_->gravity_vector_;
  config_para.acc_noise_std_ =
      Vec3d(config_->localization.imu.acc_noise_std,
            config_->localization.imu.acc_noise_std,
            config_->localization.imu.acc_noise_std);
  config_para.gyro_noise_std_ =
      Vec3d(config_->localization.imu.gyro_noise_std,
            config_->localization.imu.gyro_noise_std,
            config_->localization.imu.gyro_noise_std);
  config_para.integration_noise_cov_ = Vec3d(1.0e-8, 1.0e-8, 1.0e-8);

  return config_para;
}

void Localization::InitPreintegration() {
  const auto config_para = MakePreIntegrationConfig();
  pre_integration_ptr_ = std::make_shared<PreIntegration>(config_para);
  local_pre_integration_ptr_ = std::make_shared<PreIntegration>(config_para);
}

void Localization::UpdateTfPredictionAnchor(const NavStateData& nav_state) {
  TfPredictionAnchor anchor;
  anchor.nav_state = nav_state;
  anchor.map_T_lidar =
      MapTLidar::FromMatrix(MapImuPoseToMapLidarPose(nav_state.Pose()));
  anchor.timestamp = nav_state.timestamp_;

  std::lock_guard<std::mutex> lock(mutex_tf_prediction_anchor_);
  tf_prediction_anchor_ = std::move(anchor);
}

Localization::InitializationResult Localization::Init(
    std::uint64_t tf_continuity_epoch) {
  Mat4d init_pose;
  bool has_manual_init_pose = false;
  int result_consistency_hit = 0;
  int result_consistency_required = 0;
  {
    std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
    if (init_pose_optional_.has_value()) {
      init_pose = init_pose_optional_.value();
      init_pose_optional_.reset();
      has_manual_init_pose = true;
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
    }
  }

  if (!has_manual_init_pose) {
    if (!config_->localization.localization.auto_relocalization ||
        !scan_context_db_loaded_) {
      LOG_EVERY_N(INFO, 10) << "Please set initialization pose for "
                               "localization by RViz -> 2D Pose Estimate";
      return InitializationResult::kFailed;
    }

    float auto_fitness = std::numeric_limits<float>::infinity();
    const auto auto_pose = TryAutoRelocalize(&auto_fitness);
    if (!auto_pose.has_value()) {
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
      return InitializationResult::kFailed;
    }

    {
      std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
      if (init_pose_optional_.has_value()) {
        init_pose = init_pose_optional_.value();
        init_pose_optional_.reset();
        has_manual_init_pose = true;
        consistency_ref_pose_.reset();
        consistency_hit_count_ = 0;
      }
    }

    if (!has_manual_init_pose &&
        ShouldEarlyAcceptRelocalization(
            auto_fitness,
            config_->localization.localization.sc_accept_fitness)) {
      LOG(INFO) << "Auto relocalization accepted early with fitness "
                << auto_fitness << " (< "
                << config_->localization.localization.sc_accept_fitness
                << ").";
      init_pose = auto_pose.value();
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
    } else if (!has_manual_init_pose) {
      const int required_frames = std::max(
          1, config_->localization.localization.sc_consistency_frames);
      const double translation_threshold = std::max(
          0.0,
          config_->localization.localization.sc_consistency_trans_thres);
      const double yaw_threshold = std::max(
          0.0,
          config_->localization.localization.sc_consistency_yaw_thres);
      if (consistency_ref_pose_.has_value()) {
        const Mat4d delta = consistency_ref_pose_.value().inverse() *
                            auto_pose.value();
        const double translation_delta = delta.block<3, 1>(0, 3).norm();
        const double yaw_delta =
            std::fabs(std::atan2(delta(1, 0), delta(0, 0)));
        if (translation_delta <= translation_threshold &&
            yaw_delta <= yaw_threshold) {
          ++consistency_hit_count_;
        } else {
          consistency_ref_pose_ = auto_pose;
          consistency_hit_count_ = 1;
        }
      } else {
        consistency_ref_pose_ = auto_pose;
        consistency_hit_count_ = 1;
      }

      if (consistency_hit_count_ < required_frames) {
        LOG(INFO) << "Auto relocalization consistency "
                  << consistency_hit_count_ << "/" << required_frames;
        return InitializationResult::kAutomaticConsistencyPending;
      }

      init_pose = auto_pose.value();
      result_consistency_hit = consistency_hit_count_;
      result_consistency_required = required_frames;
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
    }
  }

  // Auto candidates each use a temporary matcher; start final initialization
  // with a fresh local-map matcher for both automatic and manual poses.
  InitMatcher();
  const auto initial_map = map_provider_->LoadCandidate(
      MapImuPoseToMapLidarPose(init_pose),
      std::chrono::milliseconds(
          config_->localization.localization.tiled_map.load_timeout_ms));
  const auto local_map =
      initial_map.state == localization_map::MapRequestState::kReady &&
              initial_map.snapshot
          ? initial_map.snapshot->registration_cloud
          : PCLPointCloudXYZI::ConstPtr{};

  if (!local_map || local_map->empty()) {
    {
      std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
      last_map_request_error_ = initial_map.error;
      map_request_unavailable_without_coverage_ = true;
    }
    LOG(WARNING) << "Initial local map is empty, please check localization map "
                    "path and initialization pose. "
                 << initial_map.error;
    return InitializationResult::kFailed;
  }

  matcher_->AddCloudToLocalMap({*local_map});
  const bool success = matcher_->Match(curr_cloud_cluster_ptr_, init_pose);

  const float fitness_score = matcher_->GetFitnessScore(2.0);

  if (!success || fitness_score >= 1.0) {
    LOG(WARNING) << "Initial registration failed. "
                 << "Please try to continue initialization and give a better "
                    "initial pose.";
    return InitializationResult::kFailed;
  }

  Mat15d cov = Mat15d::Zero();
  cov.block<3, 3>(0, 0) =
      Mat3d::Identity() * 1.0e-6 * 1.0e-6;  // rotation information matrix
  cov.block<3, 3>(3, 3) =
      Mat3d::Identity() * 1.0e-2 * 1.0e-2;  // velocity information matrix
  cov.block<3, 3>(6, 6) =
      Mat3d::Identity() * 1.0e-6 * 1.0e-6;  // position information matrix
  cov.block<3, 3>(9, 9) =
      Mat3d::Identity() *
      std::pow(0.1 * kDegree2Radian, 2);  // bias gyro information matrix
  cov.block<3, 3>(12, 12) =
      Mat3d::Identity() * 0.1 * 0.1;  // bias accel information matrix

  NavStateData initialized_nav_state = last_nav_state_;
  initialized_nav_state.SetPose(init_pose);
  initialized_nav_state.V_ = Vec3d::Zero();
  initialized_nav_state.info_ = cov.inverse();
  initialized_nav_state.timestamp_ = curr_time_us_;

  NavStateData initialized_local_state = last_local_state_;
  initialized_local_state.SetPose(Mat4d::Identity());
  initialized_local_state.V_ = Vec3d::Zero();
  initialized_local_state.timestamp_ = curr_time_us_;

  LocalizationResult init_result;
  init_result.timestamp_ = curr_time_us_;
  init_result.map_T_lidar =
      MapTLidar::FromMatrix(MapImuPoseToMapLidarPose(init_pose));
  init_result.local_T_imu =
      LocalTImu::FromMatrix(initialized_local_state.Pose());
  init_result.reg_fitness = fitness_score;
  init_result.consistency_hit = result_consistency_hit;
  init_result.consistency_required = result_consistency_required;
  init_result.is_tracking = false;
  if (config_->localization.localization.enable_ground_height_constraint) {
    if (const auto map_ground_z = EstimateMapGroundZ(
            init_result.map_T_lidar.Matrix(), initial_map.snapshot);
        map_ground_z.has_value()) {
      init_result.has_map_ground_z = true;
      init_result.map_ground_z = map_ground_z.value();
    }
  }
  const auto initialized_lidar_cloud = TransformPointCloud(
      VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5), init_pose);
  const auto initialization_commit_now_us =
      runtime::FrameProcessingWatchdog::SteadyNowMicroseconds();
  if (DiscardTimedOutFrameIfNeeded(initialization_commit_now_us)) {
    return InitializationResult::kFailed;
  }
  frame_processing_watchdog_.Complete(initialization_commit_now_us, false);
  UpdateActiveMapSnapshot(initial_map.snapshot);
  UpdateTfPredictionAnchor(initialized_nav_state);
  curr_pose_ = init_pose;
  last_nav_state_ = initialized_nav_state;
  delta_pose_ = Mat4d::Identity();
  last_pose_ = init_pose;
  last_local_state_ = initialized_local_state;
  CacheResultToSystem(init_result, tf_continuity_epoch);
  UpdateCurrentLidarCloud(*initialized_lidar_cloud);

  return InitializationResult::kInitialized;
}

bool Localization::PrepareMap() {
  const auto preparation_start = std::chrono::steady_clock::now();
  const auto record_preparation_duration = [&]() {
    map_preparation_duration_ms_.store(
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - preparation_start)
            .count());
  };

  map_ready_.store(false);
  const LocalizationMapConfig& map_config = config_->localization.localization;
  resolved_map_path_ = map_config.map_path.empty()
                           ? MapArtifactLayout(kDataPath).GlobalMap()
                           : std::filesystem::path(map_config.map_path);
  std::string mode_error;
  const auto mode = ResolveMapMode(map_config.map_mode,
                                   resolved_map_path_, &mode_error);
  if (!mode.has_value()) {
    {
      std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
      map_provider_.reset();
      active_map_snapshot_.reset();
      last_map_request_error_ = mode_error;
      map_request_unavailable_without_coverage_ = true;
    }
    record_preparation_duration();
    LOG(ERROR) << "Localization map configuration is invalid: " << mode_error;
    return false;
  }
  selected_map_mode_ = *mode;
  std::unique_ptr<localization_map::MapProvider> new_provider;

  if (selected_map_mode_ == localization_map::MapMode::kGlobal) {
    std::filesystem::path ground_path;
    if (map_config.ground_map_path.empty()) {
      // The resolved path is a GlobalMap.pcd, so its directory is the root.
      ground_path = MapArtifactLayout(MapArtifactLayout::RootDirectoryOf(
                                        resolved_map_path_))
                        .GroundMap();
    } else {
      ground_path = map_config.ground_map_path;
    }
    localization_map::GlobalMapProviderConfig provider_config;
    provider_config.global_map_path = resolved_map_path_;
    provider_config.ground_map_path = ground_path;
    provider_config.global_voxel_size = 0.3F;
    new_provider = std::make_unique<localization_map::GlobalMapProvider>(
        std::move(provider_config));
  } else {
    localization_map::TiledMapProviderConfig provider_config;
    provider_config.root = resolved_map_path_;
    provider_config.active_radius = map_config.tiled_map.active_radius_tiles;
    provider_config.prefetch_radius =
        map_config.tiled_map.prefetch_radius_tiles;
    provider_config.max_cached_tiles = static_cast<std::size_t>(
        map_config.tiled_map.max_cached_tiles);
    provider_config.max_cache_bytes = static_cast<std::size_t>(
        map_config.tiled_map.max_cache_mib) * 1024U * 1024U;
    // Always load the overview so tiled mode publishes a global-map overview;
    // it is a small 5 m resolution cloud.
    provider_config.load_overview = true;
    // Match the legacy GlobalMapProvider density. Some registration backends
    // leave local_map_cloud_filter_size unset at the numeric sentinel.
    provider_config.registration_voxel_size = 0.3F;
    new_provider = std::make_unique<localization_map::TiledMapProvider>(
        std::move(provider_config));
  }

  std::string preparation_error;
  if (!new_provider->Prepare(&preparation_error)) {
    {
      std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
      map_provider_ = std::move(new_provider);
      active_map_snapshot_.reset();
      active_ground_xy_cloud_.reset();
      active_ground_xy_kdtree_.reset();
      last_map_request_error_ = preparation_error;
      map_request_unavailable_without_coverage_ = true;
    }
    record_preparation_duration();
    LOG(ERROR) << "Localization map preparation failed: "
               << preparation_error;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
    map_provider_ = std::move(new_provider);
    active_map_snapshot_.reset();
    active_ground_xy_cloud_.reset();
    active_ground_xy_kdtree_.reset();
    last_map_request_error_.clear();
    map_request_unavailable_without_coverage_ = false;
  }

  if (map_config.auto_relocalization) {
    LoadScanContextDb();
  } else {
    scan_context_db_loaded_ = false;
  }

  record_preparation_duration();
  map_ready_.store(true);
  return true;
}

void Localization::Run() {
  if (!IsMapReady()) {
    LOG(ERROR) << "Localization map preparation has not completed.";
    return;
  }

  LOG(INFO) << "\033[1;32m----> Localization Started.\033[0m";

  while (rclcpp::ok()) {
    System::CloudClusterWorkItem cloud_cluster_work_item;
    {
      std::unique_lock<std::mutex> lock(
          system_ptr_->mutex_cloud_cluster_deque_);
      while (system_ptr_->cloud_cluster_deque_.empty()) {
        if (!rclcpp::ok()) {
          return;
        }
        system_ptr_->cv_localization_.wait(lock);
      }

      cloud_cluster_work_item =
          system_ptr_->cloud_cluster_deque_.front();
      system_ptr_->cloud_cluster_deque_.pop_front();
    }

    curr_cloud_cluster_ptr_ = cloud_cluster_work_item.cloud_cluster_ptr;
    const std::uint64_t tf_continuity_epoch =
        cloud_cluster_work_item.tf_continuity_epoch_;
    frame_processing_watchdog_.Begin(
        runtime::FrameProcessingWatchdog::SteadyNowMicroseconds());
    const auto complete_frame_without_timeout = [this] {
      if (!frame_processing_watchdog_.Snapshot().active) {
        return;
      }
      frame_processing_watchdog_.Complete(
          runtime::FrameProcessingWatchdog::SteadyNowMicroseconds(), false);
    };

    curr_time_us_ = curr_cloud_cluster_ptr_->timestamp_;
    {
      auto reinitialization_attempt =
          reinitialization_control_.BeginWorkerAttempt();
      if (reinitialization_attempt.automatic_relocalization_requested()) {
        has_init_.store(false);
        consistency_ref_pose_.reset();
        consistency_hit_count_ = 0;
        InitPreintegration();
      }

      if (!has_init_) {
        if (reinitialization_attempt.waiting_for_automatic_request()) {
          complete_frame_without_timeout();
          continue;
        }

        const InitializationResult initialization_result =
            Init(tf_continuity_epoch);
        has_init_ = initialization_result == InitializationResult::kInitialized;
        if (initialization_result ==
            InitializationResult::kAutomaticConsistencyPending) {
          reinitialization_attempt.RecordAutomaticConsistencyPending();
        } else {
          reinitialization_attempt.RecordInitializationResult(has_init_);
        }

        if (has_init_) {
          InitPreintegration();
        } else {
          complete_frame_without_timeout();
        }
        continue;
      }
    }

    if (curr_cloud_cluster_ptr_->imu_data_.size() < 2) {
      LOG(WARNING) << "Skip localization frame due to insufficient IMU data: "
                   << curr_cloud_cluster_ptr_->imu_data_.size();
      complete_frame_without_timeout();
      continue;
    }

    const bool force_local_map_rebuild = force_local_map_rebuild_;
    Timer timer_load_map;
    auto local_map = LoadLocalMap(
        MapImuPoseToMapLidarPose(last_pose_), force_local_map_rebuild);
    if (DiscardTimedOutFrameIfNeeded()) {
      continue;
    }
    if (need_update_local_map_) {
      if (!local_map || local_map->empty()) {
        LOG(WARNING) << "Skip localization frame because local map is empty.";
        complete_frame_without_timeout();
        continue;
      }
      DLOG(INFO) << "Load local map use time(ms): " << timer_load_map.End();
      Timer timer_add_cloud;
      matcher_->AddCloudToLocalMap({*local_map});
      if (force_local_map_rebuild) {
        force_local_map_rebuild_ = false;
      }
      DLOG(INFO) << "Add local map to matcher use time(ms): "
                 << timer_add_cloud.End();
    }

    const bool reset_imu_integration =
        curr_cloud_cluster_ptr_->reset_imu_integration_;
    if (reset_imu_integration) {
      LOG(WARNING) << std::setprecision(15)
                   << "Reset localization IMU integration at lidar time "
                   << static_cast<double>(curr_time_us_) * kMicroseconds2Seconds;
    }

    NavStateData predict_local_state = last_local_state_;
    if (!reset_imu_integration && local_pre_integration_ptr_) {
      predict_local_state = IntegrateImuMeasuresLocal(last_local_state_);
    }
    predict_local_state.timestamp_ = curr_time_us_;

    NavStateData predict_nav_state;

    if (reset_imu_integration) {
      predict_nav_state = last_nav_state_;
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (config_->localization.frontend.fusion_method ==
        kFusionTightCouplingOptimization) {
      // use imu integrals as predicted values for registration
      if (!pre_integration_ptr_) {
        InitPreintegration();
      }
      predict_nav_state = IntegrateImuMeasures(last_nav_state_);
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (config_->localization.frontend.fusion_method ==
               kFusionLooseCoupling) {
      const Mat4d odom_delta =
          last_local_state_.Pose().inverse() * predict_local_state.Pose();
      predict_nav_state.SetPose(last_pose_ * odom_delta);
      predict_nav_state.V_ = predict_local_state.V_;
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (config_->localization.frontend.fusion_method ==
               kFusionTightCouplingKF) {
      LOG(FATAL) << "Kalman filter support will be coming soon!";
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << config_->localization.frontend.fusion_method;
    }

    // Performing pointcloud registration.
    // Use the predicted pose as the initial pose.
    Mat4d match_pose = predict_nav_state.Pose();
    const bool match_success =
        matcher_->Match(curr_cloud_cluster_ptr_, match_pose);
    if (DiscardTimedOutFrameIfNeeded()) {
      continue;
    }
    if (!match_success) {
      complete_frame_without_timeout();
      continue;
    }

    NavStateData curr_nav_state;
    if (reset_imu_integration) {
      curr_nav_state = predict_nav_state;
      curr_nav_state.SetPose(match_pose);
      curr_nav_state.timestamp_ = curr_time_us_;
      predict_local_state.SetPose(match_pose);
    } else if (config_->localization.frontend.fusion_method ==
        kFusionTightCouplingOptimization) {
      curr_nav_state.SetPose(predict_nav_state.Pose());
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;

      /// TODO: Dynamically adjust lidar information matrix

      // Performing pre-integration optimization
      Optimize(curr_nav_state, match_pose);
    } else if (config_->localization.frontend.fusion_method ==
               kFusionLooseCoupling) {
      curr_nav_state.SetPose(match_pose);
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << config_->localization.frontend.fusion_method;
    }

    if (DiscardTimedOutFrameIfNeeded()) {
      continue;
    }

    Mat4d curr_pose = curr_nav_state.Pose();
    const Mat4d delta_pose = last_pose_.inverse() * curr_pose;

    // Cache result to System for visualization
    LocalizationResult result;
    result.timestamp_ = curr_time_us_;
    result.map_T_lidar = MapTLidar::FromMatrix(MapImuPoseToMapLidarPose(curr_pose));
    result.local_T_imu =
        LocalTImu::FromMatrix(predict_local_state.Pose());
    result.reg_fitness = matcher_->GetFitnessScore(2.0F);
    result.consistency_hit = 0;
    result.consistency_required = 0;
    result.is_tracking = true;
    if (config_->localization.localization.enable_ground_height_constraint) {
      if (const auto map_ground_z =
              EstimateMapGroundZ(result.map_T_lidar.Matrix());
          map_ground_z.has_value()) {
        result.has_map_ground_z = true;
        result.map_ground_z = map_ground_z.value();
      }
    }
    const auto current_lidar_cloud = TransformPointCloud(
        VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5),
        curr_pose);
    const auto tracking_commit_now_us =
        runtime::FrameProcessingWatchdog::SteadyNowMicroseconds();
    if (DiscardTimedOutFrameIfNeeded(tracking_commit_now_us)) {
      continue;
    }
    frame_processing_watchdog_.Complete(tracking_commit_now_us, false);
    UpdateTfPredictionAnchor(curr_nav_state);
    CacheResultToSystem(result, tf_continuity_epoch);
    UpdateCurrentLidarCloud(*current_lidar_cloud);

    delta_pose_ = delta_pose;
    last_pose_ = curr_pose;
    last_nav_state_ = curr_nav_state;
    last_local_state_ = predict_local_state;
    last_local_state_.bg_ = curr_nav_state.bg_;
    last_local_state_.ba_ = curr_nav_state.ba_;

    // reset pre-integration and reset bias
    if (config_->localization.frontend.fusion_method ==
        kFusionTightCouplingOptimization) {
      if (pre_integration_ptr_) {
        pre_integration_ptr_->Reset();
        pre_integration_ptr_->SetGyroAccBias(curr_nav_state.bg_,
                                             curr_nav_state.ba_);
      }
    }
    if (local_pre_integration_ptr_) {
      local_pre_integration_ptr_->Reset();
      local_pre_integration_ptr_->SetGyroAccBias(curr_nav_state.bg_,
                                                 curr_nav_state.ba_);
    }
  }
}

void Localization::LoadScanContextDb() {
  std::string db_path = config_->localization.localization.sc_db_path;
  if (db_path.empty()) {
    // Whichever mode resolved, the map root is the resolved path's directory.
    db_path = MapArtifactLayout(
                  MapArtifactLayout::RootDirectoryOf(resolved_map_path_))
                  .ScanContextDb()
                  .string();
  }

  std::string map_tag;
  scan_context_db_loaded_ = scan_context_.LoadDB(db_path, &map_tag);
  if (scan_context_db_loaded_) {
    // LoadDB overwrites config from the file; re-apply the retrieval-only
    // ring-key gate from the runtime parameters (the fine registration below
    // is the real validator, so a generous gate is preferred).
    scan_context_.SetRingKeyMaxDist(
        config_->localization.localization.sc_ringkey_max_dist);
    LOG(INFO) << "Loaded Scan Context DB with " << scan_context_.Size()
              << " keyframes from " << db_path << " (map_tag=" << map_tag
              << ").";
  } else {
    LOG(WARNING) << "Failed to load Scan Context DB from " << db_path
                 << "; automatic relocalization is unavailable.";
  }
}

std::optional<Mat4d> Localization::TryAutoRelocalize(
    float* best_fitness_out) {
  if (!scan_context_db_loaded_ || scan_context_.Empty() ||
      !curr_cloud_cluster_ptr_ || curr_cloud_cluster_ptr_->ordered_cloud_.empty()) {
    return std::nullopt;
  }
  if (!config_->localization.imu.has_orientation ||
      curr_cloud_cluster_ptr_->imu_data_.empty()) {
    LOG_EVERY_N(WARNING, 10)
        << "Scan Context auto relocalization requires an IMU orientation.";
    return std::nullopt;
  }

  const LidarTImu& lidar_T_imu = config_->calibration.lidar_T_imu;
  const double calibration_determinant = lidar_T_imu.Rotation().determinant();
  if (!lidar_T_imu.Matrix().allFinite() ||
      !std::isfinite(calibration_determinant) ||
      std::fabs(calibration_determinant) <
          std::numeric_limits<double>::epsilon()) {
    LOG(WARNING) << "Skip Scan Context relocalization due to invalid "
                    "lidar-IMU calibration.";
    return std::nullopt;
  }
  // Mapping keyframe features are in the lidar frame, whereas localization
  // ordered_cloud_ is deskewed in the IMU frame. Convert only the descriptor
  // input so Scan Context compares like-for-like local geometry.
  const PCLPointCloudXYZI query_cloud =
      localization_frames::ScanContextLidarCloud(
          curr_cloud_cluster_ptr_->ordered_cloud_, lidar_T_imu);
  const auto query_descriptor = scan_context_.MakeDescriptor(query_cloud);
  const int top_n = std::max(
      1, config_->localization.localization.sc_top_n);
  const auto candidates = scan_context_.Query(query_descriptor, top_n);
  if (candidates.empty()) {
    LOG_EVERY_N(INFO, 10) << "Scan Context returned no relocalization candidate.";
    return std::nullopt;
  }

  Eigen::Quaterniond imu_orientation =
      curr_cloud_cluster_ptr_->imu_data_.back().orientation_;
  if (!imu_orientation.coeffs().allFinite() ||
      imu_orientation.squaredNorm() < std::numeric_limits<double>::epsilon()) {
    LOG(WARNING) << "Skip Scan Context relocalization due to invalid IMU orientation.";
    return std::nullopt;
  }
  imu_orientation.normalize();
  const Mat3d imu_rotation = imu_orientation.toRotationMatrix();
  const double roll = std::atan2(imu_rotation(2, 1), imu_rotation(2, 2));
  const double pitch = std::atan2(
      -imu_rotation(2, 0),
      std::hypot(imu_rotation(2, 1), imu_rotation(2, 2)));
  const float fitness_threshold = static_cast<float>(std::max(
      0.0, config_->localization.localization.sc_fitness_thres));

  std::optional<LidarTBase> lidar_T_base;
  if (config_->localization.localization.enable_ground_height_constraint) {
    lidar_T_base = system_ptr_->GetLidarTBase();
    if (!lidar_T_base.has_value()) {
      LOG_EVERY_N(WARNING, 10)
          << "Scan Context relocalization is waiting for lidar_T_base TF.";
      return std::nullopt;
    }
  }

  const double accept_fitness = std::max(
      0.0, config_->localization.localization.sc_accept_fitness);
  const PointcloudClusterPtr source_cloud = curr_cloud_cluster_ptr_;
  std::optional<Mat4d> best_pose;
  float best_fitness = std::numeric_limits<float>::infinity();
  double total_preparation_ms = 0.0;
  double total_registration_ms = 0.0;
  std::size_t prepared_count = 0U;
  std::size_t evaluated_count = 0U;
  const char* stop_reason = "exhausted";

  for (std::size_t candidate_index = 0;
       candidate_index < candidates.size(); ++candidate_index) {
    if (candidate_index > 0U) {
      std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
      if (init_pose_optional_.has_value()) {
        LOG(INFO) << "Stop automatic relocalization candidate evaluation "
                     "because a manual initial pose is pending.";
        stop_reason = "manual_pose";
        break;
      }
    }

    const auto& candidate = candidates[candidate_index];
    const char* stage = "preparation";
    try {
      Timer preparation_timer;
      Mat4d candidate_lidar_pose =
          localization_frames::ScanContextCandidateLidarPose(
              candidate.pose, candidate.yaw_shift, roll, pitch,
              config_->calibration.imu_T_lidar);

      const auto candidate_map = map_provider_->LoadCandidate(
          candidate_lidar_pose,
          std::chrono::milliseconds(
              config_->localization.localization.tiled_map
                  .load_timeout_ms));
      if (candidate_map.state != localization_map::MapRequestState::kReady ||
          !candidate_map.snapshot ||
          !candidate_map.snapshot->registration_cloud ||
          candidate_map.snapshot->registration_cloud->empty()) {
        const double preparation_ms = preparation_timer.End();
        total_preparation_ms += preparation_ms;
        LOG(INFO) << "Auto relocalization candidate " << candidate_index + 1U
                  << "/" << candidates.size() << " (score="
                  << candidate.score << ") prepared in " << preparation_ms
                  << " ms, registered in 0 ms, fitness="
                  << std::numeric_limits<float>::infinity()
                  << " (map unavailable: " << candidate_map.error << ").";
        continue;
      }

      if (config_->localization.localization
              .enable_ground_height_constraint) {
        if (const auto ground_z = EstimateMapGroundZ(
                candidate_lidar_pose, candidate_map.snapshot);
            ground_z.has_value()) {
          // The cached transform is lidar_T_base, so map_T_base is
          // map_T_lidar * lidar_T_base.
          const Vec3d lidar_T_base_translation =
              lidar_T_base.value().Translation();
          candidate_lidar_pose(2, 3) =
              ground_z.value() +
                  config_->localization.localization.base_link_ground_height -
              (candidate_lidar_pose.block<3, 3>(0, 0) *
               lidar_T_base_translation)
                  .z();
        }
      }

      const double preparation_ms = preparation_timer.End();
      total_preparation_ms += preparation_ms;
      ++prepared_count;

      stage = "fine registration";
      float fitness = std::numeric_limits<float>::infinity();
      Mat4d match_pose = MapLidarPoseToMapImuPose(
          MapTLidar::FromMatrix(candidate_lidar_pose));
      Timer registration_timer;
      ++evaluated_count;
      auto matcher = CreateMatcher();
      matcher->AddCloudToLocalMap(
          {*candidate_map.snapshot->registration_cloud});
      const bool match_ok = matcher->Match(source_cloud, match_pose);
      if (match_ok) {
        fitness = matcher->GetFitnessScore(2.0F);
      }
      const double registration_ms = registration_timer.End();
      total_registration_ms += registration_ms;

      LOG(INFO) << "Auto relocalization candidate " << candidate_index + 1U
                << "/" << candidates.size() << " (score=" << candidate.score
                << ") prepared in " << preparation_ms
                << " ms, registered in " << registration_ms
                << " ms, fitness=" << fitness << ".";

      const bool qualifies =
          match_ok && std::isfinite(fitness) && fitness < fitness_threshold;
      if (qualifies && fitness < best_fitness) {
        best_fitness = fitness;
        best_pose = match_pose;
      }
      if (qualifies &&
          ShouldEarlyAcceptRelocalization(fitness, accept_fitness)) {
        stop_reason = "fitness";
        break;
      }
    } catch (const std::exception& error) {
      LOG(WARNING) << "Auto relocalization candidate "
                   << candidate_index + 1U << " threw during " << stage
                   << ": " << error.what();
    } catch (...) {
      LOG(WARNING) << "Auto relocalization candidate "
                   << candidate_index + 1U << " threw an unknown exception "
                   << "during " << stage << ".";
    }
  }

  LOG(INFO) << "Auto relocalization prepared " << prepared_count
            << " candidates in " << total_preparation_ms
            << " ms and evaluated " << evaluated_count << " serially in "
            << total_registration_ms << " ms (stop_reason=" << stop_reason
            << ").";

  if (best_pose.has_value()) {
    LOG(INFO) << "Auto relocalization candidate accepted with fitness "
              << best_fitness << ".";
  }
  if (best_fitness_out != nullptr) {
    *best_fitness_out = best_fitness;
  }
  return best_pose;
}

PCLPointCloudXYZI::ConstPtr Localization::LoadLocalMap(
    const Mat4d& pose, bool force_rebuild) {
  need_update_local_map_ = false;
  if (!map_provider_) {
    need_update_local_map_ = true;
    return nullptr;
  }

  std::optional<Vec2d> velocity_xy;
  if (has_init_.load() && last_nav_state_.V_.allFinite()) {
    velocity_xy = last_nav_state_.V_.head<2>();
  }
  const auto request =
      map_provider_->RequestTracking(pose, velocity_xy, force_rebuild);
  switch (request.state) {
    case localization_map::MapRequestState::kReady:
      if (!request.snapshot || !request.snapshot->registration_cloud ||
          request.snapshot->registration_cloud->empty()) {
        need_update_local_map_ = true;
        return nullptr;
      }
      UpdateActiveMapSnapshot(request.snapshot);
      need_update_local_map_ = true;
      return request.snapshot->registration_cloud;
    case localization_map::MapRequestState::kUnchanged: {
      std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
      map_request_unavailable_without_coverage_ = false;
      return active_map_snapshot_
                 ? active_map_snapshot_->registration_cloud
                 : PCLPointCloudXYZI::ConstPtr{};
    }
    case localization_map::MapRequestState::kPending:
      if (request.previous_snapshot_covers_center) {
        std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
        map_request_unavailable_without_coverage_ = false;
        return active_map_snapshot_
                   ? active_map_snapshot_->registration_cloud
                   : PCLPointCloudXYZI::ConstPtr{};
      }
      {
        std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
        map_request_unavailable_without_coverage_ = false;
      }
      need_update_local_map_ = true;
      return nullptr;
    case localization_map::MapRequestState::kUnavailable:
      {
        std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
        last_map_request_error_ = request.error;
        map_request_unavailable_without_coverage_ =
            !request.previous_snapshot_covers_center;
      }
      LOG_EVERY_N(WARNING, 50)
          << "Localization map request unavailable: " << request.error;
      need_update_local_map_ = true;
      return nullptr;
  }
  need_update_local_map_ = true;
  return nullptr;
}

void Localization::UpdateActiveMapSnapshot(
    std::shared_ptr<const localization_map::MapSnapshot> snapshot) {
  PCLPointCloudXYZI::ConstPtr ground_source;
  if (snapshot && snapshot->ground_cloud && !snapshot->ground_cloud->empty()) {
    ground_source = snapshot->ground_cloud;
  } else if (snapshot) {
    ground_source = snapshot->registration_cloud;
  }

  PCLPointCloudXYZI::Ptr flattened;
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr tree;
  if (ground_source && !ground_source->empty()) {
    flattened.reset(new PCLPointCloudXYZI(*ground_source));
    for (auto& point : *flattened) {
      point.z = 0.0F;
    }
    tree.reset(new pcl::KdTreeFLANN<PCLPointXYZI>());
    tree->setInputCloud(flattened);
  }

  std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
  active_map_snapshot_ = std::move(snapshot);
  active_ground_xy_cloud_ = std::move(flattened);
  active_ground_xy_kdtree_ = std::move(tree);
  last_map_request_error_.clear();
  map_request_unavailable_without_coverage_ = false;
}

NavStateData Localization::IntegrateImuMeasures(
    const NavStateData& last_nav_state) const {
  pre_integration_ptr_->IntegrateDataSegment(
      curr_cloud_cluster_ptr_->imu_data_);
  return pre_integration_ptr_->Predict(last_nav_state);
}

NavStateData Localization::IntegrateImuMeasuresLocal(
    const NavStateData& last_local_state) const {
  local_pre_integration_ptr_->IntegrateDataSegment(
      curr_cloud_cluster_ptr_->imu_data_);
  return local_pre_integration_ptr_->Predict(last_local_state);
}

void Localization::Optimize(NavStateData& curr_nav_state,
                            const Mat4d& lidar_pose) {
  using BlockSolverType = g2o::BlockSolverX;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;
  auto* solver = new g2o::OptimizationAlgorithmLevenberg(
      std::make_unique<BlockSolverType>(std::make_unique<LinearSolverType>()));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);

  // === pre-integration error optimization ===
  // last rotation vertex
  auto vertex_rotation_last = new VertexRotation;
  vertex_rotation_last->setId(0);
  vertex_rotation_last->setEstimate(last_nav_state_.R_);
  optimizer.addVertex(vertex_rotation_last);

  // last velocity vertex
  auto vertex_velocity_last = new VertexVelocity;
  vertex_velocity_last->setId(1);
  vertex_velocity_last->setEstimate(last_nav_state_.V_);
  optimizer.addVertex(vertex_velocity_last);

  // last position vertex
  auto vertex_position_last = new VertexPosition;
  vertex_position_last->setId(2);
  vertex_position_last->setEstimate(last_nav_state_.P_);
  optimizer.addVertex(vertex_position_last);

  // current rotation vertex
  auto vertex_rotation_curr = new VertexRotation;
  vertex_rotation_curr->setId(3);
  vertex_rotation_curr->setEstimate(curr_nav_state.R_);
  optimizer.addVertex(vertex_rotation_curr);

  // current velocity vertex
  auto vertex_velocity_curr = new VertexVelocity;
  vertex_velocity_curr->setId(4);
  vertex_velocity_curr->setEstimate(curr_nav_state.V_);
  optimizer.addVertex(vertex_velocity_curr);

  // current position vertex
  auto vertex_position_curr = new VertexPosition;
  vertex_position_curr->setId(5);
  vertex_position_curr->setEstimate(curr_nav_state.P_);
  optimizer.addVertex(vertex_position_curr);

  // last gyro bias vertex
  auto vertex_bg_last = new VertexBiasGyro;
  vertex_bg_last->setId(6);
  vertex_bg_last->setEstimate(last_nav_state_.bg_);
  optimizer.addVertex(vertex_bg_last);

  // last accel bias vertex
  auto vertex_ba_last = new VertexBiasAccel;
  vertex_ba_last->setId(7);
  vertex_ba_last->setEstimate(last_nav_state_.ba_);
  optimizer.addVertex(vertex_ba_last);

  // current gyro bias vertex
  auto vertex_bg_curr = new VertexBiasGyro;
  vertex_bg_curr->setId(8);
  vertex_bg_curr->setEstimate(last_nav_state_.bg_);
  optimizer.addVertex(vertex_bg_curr);

  // current accel bias vertex
  auto vertex_ba_curr = new VertexBiasAccel;
  vertex_ba_curr->setId(9);
  vertex_ba_curr->setEstimate(last_nav_state_.ba_);
  optimizer.addVertex(vertex_ba_curr);

  // === prior last nav state edge ===
  auto* edge_prior_last_nav_state =
      new EdgePriorNavState(last_nav_state_, last_nav_state_.info_);
  edge_prior_last_nav_state->setVertex(0, vertex_rotation_last);
  edge_prior_last_nav_state->setVertex(1, vertex_velocity_last);
  edge_prior_last_nav_state->setVertex(2, vertex_position_last);
  edge_prior_last_nav_state->setVertex(3, vertex_bg_last);
  edge_prior_last_nav_state->setVertex(4, vertex_ba_last);
  optimizer.addEdge(edge_prior_last_nav_state);

  // === current lidar rotation edge ===
  auto* edge_lidar_rotation_edge = new EdgeRotation();
  edge_lidar_rotation_edge->setVertex(0, vertex_rotation_curr);
  edge_lidar_rotation_edge->setInformation(lidar_pose_info_.block<3, 3>(0, 0));
  edge_lidar_rotation_edge->setMeasurement(lidar_pose.block<3, 3>(0, 0));
  optimizer.addEdge(edge_lidar_rotation_edge);

  // === current lidar position edge ===
  auto* edge_lidar_position_edge = new EdgePosition();
  edge_lidar_position_edge->setVertex(0, vertex_position_curr);
  edge_lidar_position_edge->setInformation(lidar_pose_info_.block<3, 3>(3, 3));
  edge_lidar_position_edge->setMeasurement(lidar_pose.block<3, 1>(0, 3));
  optimizer.addEdge(edge_lidar_position_edge);

  // === pre-integration edge ===
  auto* edge_preintegration = new EdgePreIntegration(pre_integration_ptr_);
  edge_preintegration->setVertex(0, vertex_rotation_last);
  edge_preintegration->setVertex(1, vertex_velocity_last);
  edge_preintegration->setVertex(2, vertex_position_last);
  edge_preintegration->setVertex(3, vertex_bg_last);
  edge_preintegration->setVertex(4, vertex_ba_last);
  edge_preintegration->setVertex(5, vertex_rotation_curr);
  edge_preintegration->setVertex(6, vertex_velocity_curr);
  edge_preintegration->setVertex(7, vertex_position_curr);
  optimizer.addEdge(edge_preintegration);

  // === imu acc bias random walk edge ===
  Mat3d acc_bias_rw_info = Mat3d::Identity();
  acc_bias_rw_info =
      acc_bias_rw_info *
      (1.0 / std::pow(config_->localization.imu.acc_bias_rw_noise_std, 2.0));
  auto* edge_acc_bias_rw = new EdgeAccelBiasRW();
  edge_acc_bias_rw->setVertex(0, vertex_ba_last);
  edge_acc_bias_rw->setVertex(1, vertex_ba_curr);
  edge_acc_bias_rw->setInformation(acc_bias_rw_info);
  optimizer.addEdge(edge_acc_bias_rw);

  // === imu gyro bias random walk edge ===
  Mat3d gyro_bias_rw_info = Mat3d::Identity();
  gyro_bias_rw_info =
      gyro_bias_rw_info *
      (1.0 /
       std::pow(config_->localization.imu.gyro_bias_rw_noise_std, 2.0));
  auto* edge_gyro_bias_rw = new EdgeGyroBiasRW();
  edge_gyro_bias_rw->setVertex(0, vertex_bg_last);
  edge_gyro_bias_rw->setVertex(1, vertex_bg_curr);
  edge_gyro_bias_rw->setInformation(gyro_bias_rw_info);
  optimizer.addEdge(edge_gyro_bias_rw);

  // start to optimize
  optimizer.initializeOptimization();
  optimizer.optimize(config_->localization.frontend.fusion_opti_iters);

  // get the results after optimization
  const Mat3d& curr_R_opti = vertex_rotation_curr->estimate();
  const Vec3d& curr_V_opti = vertex_velocity_curr->estimate();
  const Vec3d& curr_P_opti = vertex_position_curr->estimate();
  const Vec3d& curr_bg_opti = vertex_bg_curr->estimate();
  const Vec3d& curr_ba_opti = vertex_ba_curr->estimate();

  // Get all state posterior information
  // R_i, V_i, P_i, bg_i, ba_i, R_j, V_j, P_j, bg_j, ba_j
  //  0    3    6    9    12    15    18   21   24    27
  Eigen::Matrix<double, 30, 30> posterior_info =
      Eigen::Matrix<double, 30, 30>::Zero();

  // gyro bias random walk posterior info
  const Mat6d gyro_bias_posterior_info =
      edge_gyro_bias_rw->GetPosteriorInformation();
  posterior_info.block<3, 3>(9, 9) +=
      gyro_bias_posterior_info.block<3, 3>(0, 0);
  posterior_info.block<3, 3>(24, 24) +=
      gyro_bias_posterior_info.block<3, 3>(3, 3);
  posterior_info.block<3, 3>(9, 24) +=
      gyro_bias_posterior_info.block<3, 3>(0, 3);
  posterior_info.block<3, 3>(24, 9) +=
      gyro_bias_posterior_info.block<3, 3>(3, 0);

  // acc bias random walk posterior info
  const Mat6d acc_bias_posterior_info =
      edge_acc_bias_rw->GetPosteriorInformation();
  posterior_info.block<3, 3>(12, 12) +=
      acc_bias_posterior_info.block<3, 3>(0, 0);
  posterior_info.block<3, 3>(27, 27) +=
      acc_bias_posterior_info.block<3, 3>(3, 3);
  posterior_info.block<3, 3>(12, 27) +=
      acc_bias_posterior_info.block<3, 3>(0, 3);
  posterior_info.block<3, 3>(27, 12) +=
      acc_bias_posterior_info.block<3, 3>(3, 0);

  // pre-integration posterior info
  const Mat24d edge_preintegration_posterior_info =
      edge_preintegration->GetPosteriorInformation();
  posterior_info.block<24, 24>(0, 0) += edge_preintegration_posterior_info;

  // gps rotation posterior info
  const Mat3d edge_lidar_rotation_posterior_info =
      edge_lidar_rotation_edge->GetPosteriorInformation();
  posterior_info.block<3, 3>(15, 15) += edge_lidar_rotation_posterior_info;

  // gps position posterior info
  const Mat3d edge_lidar_position_posterior_info =
      edge_lidar_position_edge->GetPosteriorInformation();
  posterior_info.block<3, 3>(21, 21) += edge_lidar_position_posterior_info;

  // last nav state prior posterior info
  const Mat15d edge_last_nav_state_prior_posterior_info =
      edge_prior_last_nav_state->GetPosteriorInformation();
  posterior_info.block<15, 15>(0, 0) +=
      edge_last_nav_state_prior_posterior_info;

  curr_nav_state.R_ = curr_R_opti;
  curr_nav_state.V_ = curr_V_opti;
  curr_nav_state.P_ = curr_P_opti;
  curr_nav_state.bg_ = curr_bg_opti;
  curr_nav_state.ba_ = curr_ba_opti;
  curr_nav_state.info_ =
      Marginalize(posterior_info, 0, 14).block<15, 15>(15, 15);

  LOG_IF(WARNING, curr_ba_opti.x() > 1.0 || curr_ba_opti.y() > 1.0 ||
                      curr_ba_opti.z() > 1.0)
      << "The value of acc bias is too large, and the optimization may "
         "diverge.";
  LOG_IF(WARNING, curr_bg_opti.x() > 1.0 || curr_bg_opti.y() > 1.0 ||
                      curr_bg_opti.z() > 1.0)
      << "The value of gyro bias is too large, and the optimization may "
         "diverge.";
}

std::optional<double> Localization::EstimateMapGroundZ(
    const Mat4d& map_T_lidar_matrix,
    const std::shared_ptr<const localization_map::MapSnapshot>& snapshot) {
  const double radius = std::max(
      0.0, config_->localization.localization.ground_search_radius);
  const double percentile = std::clamp(
      config_->localization.localization.ground_z_percentile, 0.0, 1.0);
  const int min_points = std::max(
      0, config_->localization.localization.ground_min_points);
  const Vec3d position = map_T_lidar_matrix.block<3, 1>(0, 3);

  PCLPointCloudXYZI::ConstPtr height_cloud;
  pcl::KdTreeFLANN<PCLPointXYZI>::Ptr xy_kdtree;
  PCLPointCloudXYZI::Ptr temporary_xy_cloud;
  if (snapshot) {
    height_cloud = snapshot->ground_cloud && !snapshot->ground_cloud->empty()
                       ? snapshot->ground_cloud
                       : snapshot->registration_cloud;
    if (height_cloud && !height_cloud->empty()) {
      temporary_xy_cloud.reset(new PCLPointCloudXYZI(*height_cloud));
      for (auto& point : *temporary_xy_cloud) {
        point.z = 0.0F;
      }
      xy_kdtree.reset(new pcl::KdTreeFLANN<PCLPointXYZI>());
      xy_kdtree->setInputCloud(temporary_xy_cloud);
    }
  } else {
    std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
    if (active_map_snapshot_) {
      height_cloud = active_map_snapshot_->ground_cloud &&
                             !active_map_snapshot_->ground_cloud->empty()
                         ? active_map_snapshot_->ground_cloud
                         : active_map_snapshot_->registration_cloud;
    }
    xy_kdtree = active_ground_xy_kdtree_;
  }
  if (!height_cloud || !xy_kdtree) {
    return std::nullopt;
  }

  std::vector<double> nearby_z;
  PCLPointXYZI query_point;
  query_point.x = static_cast<float>(position.x());
  query_point.y = static_cast<float>(position.y());
  query_point.z = 0.0F;
  std::vector<int> point_indices;
  std::vector<float> point_squared_distances;
  xy_kdtree->radiusSearch(query_point, radius, point_indices,
                          point_squared_distances);
  nearby_z.reserve(point_indices.size());
  for (const int point_index : point_indices) {
    if (point_index < 0 ||
        point_index >= static_cast<int>(height_cloud->size())) {
      continue;
    }
    const auto& point = height_cloud->points[point_index];
    if (!std::isfinite(point.z)) {
      continue;
    }
    nearby_z.emplace_back(static_cast<double>(point.z));
  }

  if (nearby_z.size() < static_cast<std::size_t>(min_points)) {
    LOG(WARNING) << "Skip map ground height constraint: only "
                 << nearby_z.size() << " map points within " << radius
                 << " m.";
    return std::nullopt;
  }

  const auto index = static_cast<std::size_t>(
      std::round(percentile * static_cast<double>(nearby_z.size() - 1)));
  std::nth_element(nearby_z.begin(), nearby_z.begin() + index, nearby_z.end());
  return nearby_z[index];
}

void Localization::CacheResultToSystem(
    const LocalizationResult& result,
    std::uint64_t tf_continuity_epoch) const {
  LocalizationResult queued_result = result;
  queued_result.health_arrival_steady_us = SteadyNowMicroseconds();
  queued_result.tf_continuity_epoch_ = tf_continuity_epoch;

  std::lock_guard<std::mutex> lg(
      system_ptr_->mutex_localization_results_deque_);
  while (system_ptr_->localization_results_deque_.size() >=
         System::kMaxLocalizationResultQueueSize) {
    system_ptr_->localization_results_deque_.pop_front();
  }
  system_ptr_->localization_results_deque_.emplace_back(
      std::move(queued_result));
}

void Localization::UpdateCurrentLidarCloud(
    const PCLPointCloudXYZI& lidar_cloud) {
  std::lock_guard<std::mutex> lg(mutex_lidar_cloud_);
  lidar_cloud_ = lidar_cloud;
}

PCLPointCloudXYZI::ConstPtr Localization::GetGlobalCloudMap() {
  std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
  if (!map_provider_) {
    return nullptr;
  }
  return map_provider_->Overview();
}

PCLPointCloudXYZI::ConstPtr Localization::GetLocalCloudMap() {
  std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
  if (!active_map_snapshot_ ||
      !active_map_snapshot_->registration_cloud) {
    return nullptr;
  }
  return active_map_snapshot_->registration_cloud;
}

localization_map::MapDiagnostics Localization::MapDiagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_map_snapshot_);
  localization_map::MapDiagnostics diagnostics;
  if (map_provider_) {
    diagnostics = map_provider_->Diagnostics();
  }
  diagnostics.snapshot_available =
      active_map_snapshot_ && active_map_snapshot_->registration_cloud &&
      !active_map_snapshot_->registration_cloud->empty();
  diagnostics.snapshot_degraded =
      active_map_snapshot_ && active_map_snapshot_->degraded;
  diagnostics.request_unavailable_without_coverage =
      map_request_unavailable_without_coverage_;
  if (active_map_snapshot_ && !active_map_snapshot_->missing_indices.empty()) {
    if (!diagnostics.last_error.empty()) {
      diagnostics.last_error += "; ";
    }
    diagnostics.last_error += "missing tiles:";
    for (const auto& index : active_map_snapshot_->missing_indices) {
      diagnostics.last_error += " [" + std::to_string(index.x) + "," +
                                std::to_string(index.y) + "]";
    }
  }
  if (diagnostics.last_error.empty()) {
    diagnostics.last_error = last_map_request_error_;
  }
  const std::string root = resolved_map_path_.string();
  const std::size_t root_position = diagnostics.last_error.find(root);
  if (!root.empty() && root_position != std::string::npos) {
    diagnostics.last_error.erase(root_position, root.size());
    if (root_position < diagnostics.last_error.size() &&
        diagnostics.last_error[root_position] == '/') {
      diagnostics.last_error.erase(root_position, 1U);
    }
  }
  return diagnostics;
}

PCLPointCloudXYZI Localization::GetCurrentLidarCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_lidar_cloud_);
  return lidar_cloud_;
}
