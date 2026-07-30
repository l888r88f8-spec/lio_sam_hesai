#include "slam/localization.h"

#include <pcl/conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>

#include <g2o/core/sparse_optimizer.h>

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
#include "relocalization/ground_map_source.h"
#include "slam/split_map.h"

namespace {

std::uint64_t SteadyNowMicroseconds() {
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

bool HasPclField(const pcl::PCLPointCloud2& cloud,
                 const std::string& field_name) {
  return std::find_if(cloud.fields.begin(), cloud.fields.end(),
                      [&](const pcl::PCLPointField& field) {
                        return field.name == field_name;
                      }) != cloud.fields.end();
}

PCLPointCloudXYZI::Ptr LoadPcdAsXYZI(const std::string& path) {
  pcl::PCLPointCloud2 cloud_blob;
  if (pcl::io::loadPCDFile(path, cloud_blob) != 0) {
    return nullptr;
  }

  PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
  if (HasPclField(cloud_blob, "intensity")) {
    pcl::fromPCLPointCloud2(cloud_blob, *cloud);
    return cloud;
  }

  PCLPointCloudXYZ cloud_xyz;
  pcl::fromPCLPointCloud2(cloud_blob, cloud_xyz);
  cloud->reserve(cloud_xyz.size());
  for (const auto& point_xyz : cloud_xyz) {
    PCLPointXYZI point_xyzi;
    point_xyzi.x = point_xyz.x;
    point_xyzi.y = point_xyz.y;
    point_xyzi.z = point_xyz.z;
    point_xyzi.intensity = 0.0f;
    cloud->push_back(point_xyzi);
  }
  cloud->header = cloud_xyz.header;
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = cloud_xyz.is_dense;
  return cloud;
}

}  // namespace

Localization::Localization(System* system_ptr)
    : system_ptr_(system_ptr),
      frame_processing_watchdog_(system_ptr->FrameProcessingTimeoutUs()) {
  if (use_tile_map_) {
    tile_map_grid_size_ = ConfigParameters::Instance().tile_map_grid_size_;
    tile_map_grid_size_half_ = tile_map_grid_size_ * 0.5;
  }
  SetLidarPoseInformation();
  InitMatcher();
}

void Localization::SetInitPose(const Mat4d& pose) {
  [[maybe_unused]] auto manual_initialization =
      reinitialization_control_.BeginManualInitialization();
  std::lock_guard<std::mutex> lg(mutex_init_pose_optional_);
  // RViz/System provides map->lidar, while registration consumes the
  // deskewed IMU-frame cloud and therefore uses map->imu internally.
  init_pose_optional_ = MapLidarPoseToMapImuPose(pose);
  has_init_ = false;
}

void Localization::RequestAutomaticRelocalization() {
  reinitialization_control_.RequestAutomaticRelocalization();
}

Mat4d Localization::MapLidarPoseToMapImuPose(
    const Mat4d& map_lidar_pose) const {
  return map_lidar_pose *
         ConfigParameters::Instance().calibration_lidar_to_imu_.inverse();
}

Mat4d Localization::MapImuPoseToMapLidarPose(
    const Mat4d& map_imu_pose) const {
  return map_imu_pose * ConfigParameters::Instance().calibration_lidar_to_imu_;
}

void Localization::SetLidarPoseInformation() {
  const auto& lidar_rot_std =
      ConfigParameters::Instance().lidar_rotation_noise_std_;
  const auto& lidar_posi_std =
      ConfigParameters::Instance().lidar_position_noise_std_;

  lidar_pose_info_.setZero();
  lidar_pose_info_.block<3, 3>(0, 0) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_rot_std, 2.0);
  lidar_pose_info_.block<3, 3>(3, 3) =
      Mat3d::Identity() * 1.0 / std::pow(lidar_posi_std, 2.0);
}

std::shared_ptr<RegistrationInterface> Localization::CreateMatcher() const {
  if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
      kIcpOptimized) {
    return std::make_shared<IcpOptimized<double>>(
        ConfigParameters::Instance().registration_opti_iter_num_,
        ConfigParameters::Instance().registration_local_map_size_,
        static_cast<float>(ConfigParameters::Instance()
                               .registration_local_map_cloud_filter_size_),
        static_cast<float>(ConfigParameters::Instance()
                               .registration_source_cloud_filter_size_),
        static_cast<double>(
            ConfigParameters::Instance().registration_point_search_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_position_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_rotation_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_keyframe_delta_rotation_),
        static_cast<double>(
            ConfigParameters::Instance().registration_keyframe_delta_dist_),
        true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kIncrementalNDT) {
    return std::make_shared<IncrementalNDT>(
        ConfigParameters::Instance().registration_ndt_voxel_size_,
        ConfigParameters::Instance().registration_ndt_outlier_threshold_,
        ConfigParameters::Instance().registration_source_cloud_filter_size_,
        static_cast<double>(
            ConfigParameters::Instance().registration_rotation_converge_thres_),
        static_cast<double>(
            ConfigParameters::Instance().registration_position_converge_thres_),
        ConfigParameters::Instance().registration_ndt_min_points_in_voxel_,
        ConfigParameters::Instance().registration_ndt_max_points_in_voxel_,
        ConfigParameters::Instance().registration_ndt_min_effective_pts_,
        ConfigParameters::Instance().registration_ndt_capacity_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kPointToPlane_IVOX) {
    return std::make_shared<LoamPointToPlaneIVOX<double>>(
        ConfigParameters::Instance().registration_point_to_planar_thres_,
        ConfigParameters::Instance().registration_position_converge_thres_,
        ConfigParameters::Instance().registration_rotation_converge_thres_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else if (ConfigParameters::Instance().registration_and_searcher_mode_ ==
             kPointToPlane_KdTree) {
    return std::make_shared<LoamPointToPlaneKdtree<double>>(
        ConfigParameters::Instance().registration_point_to_planar_thres_,
        ConfigParameters::Instance().registration_position_converge_thres_,
        ConfigParameters::Instance().registration_rotation_converge_thres_,
        ConfigParameters::Instance().registration_keyframe_delta_rotation_,
        ConfigParameters::Instance().registration_keyframe_delta_dist_,
        ConfigParameters::Instance().registration_local_map_size_,
        ConfigParameters::Instance().registration_local_map_cloud_filter_size_,
        ConfigParameters::Instance().registration_opti_iter_num_, true);
  } else {
    LOG(FATAL) << "Unsupported registration type! "
                  "You can choose one of the following: "
                  "IcpOptimized, kIncrementalNDT";
  }
  return nullptr;
}

void Localization::InitMatcher() { matcher_ = CreateMatcher(); }

bool Localization::DiscardTimedOutFrameIfNeeded() {
  const auto now_us =
      runtime::FrameProcessingWatchdog::SteadyNowMicroseconds();
  if (!frame_processing_watchdog_.Exceeded(now_us)) {
    return false;
  }

  frame_processing_watchdog_.Complete(now_us, true);
  InitMatcher();
  InitPreintegration();
  need_update_local_map_ = true;
  delta_pose_ = Mat4d::Identity();
  LOG(WARNING) << "Drop overdue localization frame.";
  return true;
}

void Localization::InitPreintegration() {
  PreIntegration::ConfigPara config_para;
  config_para.init_acc_bias_ =
      Vec3d(ConfigParameters::Instance().imu_init_acc_bias_,
            ConfigParameters::Instance().imu_init_acc_bias_,
            ConfigParameters::Instance().imu_init_acc_bias_);
  config_para.init_gyro_bias_ =
      Vec3d(ConfigParameters::Instance().imu_init_gyro_bias_,
            ConfigParameters::Instance().imu_init_gyro_bias_,
            ConfigParameters::Instance().imu_init_gyro_bias_);
  config_para.gravity_ = ConfigParameters::Instance().gravity_vector_;
  config_para.acc_noise_std_ =
      Vec3d(ConfigParameters::Instance().imu_acc_noise_std_,
            ConfigParameters::Instance().imu_acc_noise_std_,
            ConfigParameters::Instance().imu_acc_noise_std_);
  config_para.gyro_noise_std_ =
      Vec3d(ConfigParameters::Instance().imu_gyro_noise_std_,
            ConfigParameters::Instance().imu_gyro_noise_std_,
            ConfigParameters::Instance().imu_gyro_noise_std_);
  config_para.integration_noise_cov_ = Vec3d(1.0e-8, 1.0e-8, 1.0e-8);

  pre_integration_ptr_ = std::make_shared<PreIntegration>(config_para);
  odom_pre_integration_ptr_ = std::make_shared<PreIntegration>(config_para);
}

Localization::InitializationResult Localization::Init() {
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
    if (!ConfigParameters::Instance().localization_auto_relocalization_ ||
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
            ConfigParameters::Instance().localization_sc_accept_fitness_)) {
      LOG(INFO) << "Auto relocalization accepted early with fitness "
                << auto_fitness << " (< "
                << ConfigParameters::Instance().localization_sc_accept_fitness_
                << ").";
      init_pose = auto_pose.value();
      consistency_ref_pose_.reset();
      consistency_hit_count_ = 0;
    } else if (!has_manual_init_pose) {
      const int required_frames = std::max(
          1, ConfigParameters::Instance().localization_sc_consistency_frames_);
      const double translation_threshold = std::max(
          0.0,
          ConfigParameters::Instance().localization_sc_consistency_trans_thres_);
      const double yaw_threshold = std::max(
          0.0,
          ConfigParameters::Instance().localization_sc_consistency_yaw_thres_);
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
  const auto local_map =
      LoadLocalMap(MapImuPoseToMapLidarPose(init_pose));

  if (!local_map || local_map->empty()) {
    LOG(WARNING) << "Initial local map is empty, please check localization map "
                    "path and initialization pose.";
    return InitializationResult::kFailed;
  }

  matcher_->AddCloudToLocalMap({*local_map});
  const bool success = matcher_->Match(curr_cloud_cluster_ptr_, init_pose);

  const float fitness_score = matcher_->GetFitnessScore(2.0);

  if (success && fitness_score < 1.0) {
    curr_pose_ = init_pose;
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
    last_nav_state_.SetPose(init_pose);
    last_nav_state_.V_ = Vec3d::Zero();
    last_nav_state_.info_ = cov.inverse();
    last_nav_state_.timestamp_ = curr_time_us_;

    delta_pose_.setIdentity();
    last_pose_ = init_pose;
  } else {
    LOG(WARNING) << "Initial registration failed. "
                 << "Please try to continue initialization and give a better "
                    "initial pose.";
    return InitializationResult::kFailed;
  }

  last_odom_state_.SetPose(Mat4d::Identity());
  last_odom_state_.V_ = Vec3d::Zero();
  last_odom_state_.bg_ = last_nav_state_.bg_;
  last_odom_state_.ba_ = last_nav_state_.ba_;
  last_odom_state_.timestamp_ = curr_time_us_;

  LocalizationResult init_result;
  init_result.timestamp_ = curr_time_us_;
  init_result.map_pose = MapImuPoseToMapLidarPose(init_pose);
  init_result.odom_pose = last_odom_state_.Pose();
  init_result.reg_fitness = fitness_score;
  init_result.consistency_hit = result_consistency_hit;
  init_result.consistency_required = result_consistency_required;
  init_result.is_tracking = false;
  if (ConfigParameters::Instance().localization_enable_ground_height_constraint_) {
    if (const auto map_ground_z = EstimateMapGroundZ(init_result.map_pose);
        map_ground_z.has_value()) {
      init_result.has_map_ground_z = true;
      init_result.map_ground_z = map_ground_z.value();
    }
  }
  if (DiscardTimedOutFrameIfNeeded()) {
    return InitializationResult::kFailed;
  }
  CacheResultToSystem(init_result);
  UpdateCurrentLidarCloud(*TransformPointCloud(
      VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5), init_pose));

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
  {
    std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
    std::string global_map_path =
        ConfigParameters::Instance().localization_map_path_;
    if (global_map_path.empty()) {
      global_map_path = kDataPath + kGlobalMapFileName;
    }
    global_map_cloud_ptr_ = LoadGlobalMap(global_map_path);

    if (global_map_cloud_ptr_) {
      constexpr auto filter_size = 0.3f;
      global_map_cloud_ptr_ =
          VoxelGridCloud(global_map_cloud_ptr_, filter_size);
    }

    if (global_map_cloud_ptr_ && !global_map_cloud_ptr_->empty()) {
      global_map_xy_cloud_ptr_.reset(new PCLPointCloudXYZI(
          *global_map_cloud_ptr_));
      for (auto& point : global_map_xy_cloud_ptr_->points) {
        point.z = 0.0F;
      }
      global_map_xy_kdtree_.reset(new pcl::KdTreeFLANN<PCLPointXYZI>());
      global_map_xy_kdtree_->setInputCloud(global_map_xy_cloud_ptr_);
    } else {
      global_map_cloud_ptr_.reset();
      global_map_xy_cloud_ptr_.reset();
      global_map_xy_kdtree_.reset();
      ground_map_cloud_ptr_.reset();
      ground_map_xy_cloud_ptr_.reset();
      ground_map_xy_kdtree_.reset();
      record_preparation_duration();
      LOG(ERROR) << "Global map is missing or empty at " << global_map_path
                 << ".";
      return false;
    }

    std::string ground_map_path =
        ConfigParameters::Instance().localization_ground_map_path_;
    if (ground_map_path.empty()) {
      const auto slash = global_map_path.find_last_of('/');
      ground_map_path = slash == std::string::npos
                            ? "GroundMap.pcd"
                            : global_map_path.substr(0, slash + 1U) +
                                  "GroundMap.pcd";
    }
    PCLPointCloudXYZI::Ptr ground_map = LoadGlobalMap(ground_map_path);
    if (ground_map && !ground_map->empty()) {
      ground_map_cloud_ptr_ = ground_map;
      ground_map_xy_cloud_ptr_.reset(new PCLPointCloudXYZI(*ground_map));
      for (auto& point : ground_map_xy_cloud_ptr_->points) {
        point.z = 0.0F;
      }
      ground_map_xy_kdtree_.reset(new pcl::KdTreeFLANN<PCLPointXYZI>());
      ground_map_xy_kdtree_->setInputCloud(ground_map_xy_cloud_ptr_);
      LOG(INFO) << "Loaded dedicated ground map with " << ground_map->size()
                << " points from " << ground_map_path << ".";
    } else {
      ground_map_cloud_ptr_.reset();
      ground_map_xy_cloud_ptr_.reset();
      ground_map_xy_kdtree_.reset();
      LOG(WARNING) << "Ground map unavailable at " << ground_map_path
                   << "; EstimateMapGroundZ falls back to the global map.";
    }
  }

  if (ConfigParameters::Instance().localization_auto_relocalization_) {
    LoadScanContextDb();
  } else {
    scan_context_db_loaded_ = false;
  }

  // If use tile map, load tile map indices file
  if (use_tile_map_) {
    LoadTileMapIndices();
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
    {
      std::unique_lock<std::mutex> lock(
          system_ptr_->mutex_cloud_cluster_deque_);
      while (system_ptr_->cloud_cluster_deque_.empty()) {
        if (!rclcpp::ok()) {
          return;
        }
        system_ptr_->cv_localization_.wait(lock);
      }

      curr_cloud_cluster_ptr_ = system_ptr_->cloud_cluster_deque_.front();
      system_ptr_->cloud_cluster_deque_.pop_front();
    }

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

        const InitializationResult initialization_result = Init();
        if (DiscardTimedOutFrameIfNeeded()) {
          has_init_.store(false);
          reinitialization_attempt.RecordInitializationResult(false);
          continue;
        }
        has_init_ = initialization_result == InitializationResult::kInitialized;
        if (initialization_result ==
            InitializationResult::kAutomaticConsistencyPending) {
          reinitialization_attempt.RecordAutomaticConsistencyPending();
        } else {
          reinitialization_attempt.RecordInitializationResult(has_init_);
        }

        if (has_init_) {
          InitPreintegration();
        }
        complete_frame_without_timeout();
        continue;
      }
    }

    if (curr_cloud_cluster_ptr_->imu_data_.size() < 2) {
      LOG(WARNING) << "Skip localization frame due to insufficient IMU data: "
                   << curr_cloud_cluster_ptr_->imu_data_.size();
      complete_frame_without_timeout();
      continue;
    }

    Timer timer_load_map;
    auto local_map = LoadLocalMap(MapImuPoseToMapLidarPose(last_pose_));
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

    NavStateData predict_odom_state = last_odom_state_;
    if (!reset_imu_integration && odom_pre_integration_ptr_) {
      predict_odom_state = IntegrateImuMeasuresOdom(last_odom_state_);
    }
    predict_odom_state.timestamp_ = curr_time_us_;

    NavStateData predict_nav_state;

    if (reset_imu_integration) {
      predict_nav_state = last_nav_state_;
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      // use imu integrals as predicted values for registration
      if (!pre_integration_ptr_) {
        InitPreintegration();
      }
      predict_nav_state = IntegrateImuMeasures(last_nav_state_);
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionLooseCoupling) {
      const Mat4d odom_delta =
          last_odom_state_.Pose().inverse() * predict_odom_state.Pose();
      predict_nav_state.SetPose(last_pose_ * odom_delta);
      predict_nav_state.V_ = predict_odom_state.V_;
      predict_nav_state.timestamp_ = curr_time_us_;
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionTightCouplingKF) {
      LOG(FATAL) << "Kalman filter support will be coming soon!";
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << ConfigParameters::Instance().fusion_method_;
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
      predict_odom_state.SetPose(match_pose);
    } else if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      curr_nav_state.SetPose(predict_nav_state.Pose());
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;

      /// TODO: Dynamically adjust lidar information matrix

      // Performing pre-integration optimization
      Optimize(curr_nav_state, match_pose);
    } else if (ConfigParameters::Instance().fusion_method_ ==
               kFusionLooseCoupling) {
      curr_nav_state.SetPose(match_pose);
      curr_nav_state.V_ = predict_nav_state.V_;
      curr_nav_state.timestamp_ = curr_time_us_;
    } else {
      LOG(FATAL) << "Fusion method doesn't support: "
                 << ConfigParameters::Instance().fusion_method_;
    }

    if (DiscardTimedOutFrameIfNeeded()) {
      continue;
    }

    Mat4d curr_pose = curr_nav_state.Pose();
    const Mat4d delta_pose = last_pose_.inverse() * curr_pose;

    // Cache result to System for visualization
    LocalizationResult result;
    result.timestamp_ = curr_time_us_;
    result.map_pose = MapImuPoseToMapLidarPose(curr_pose);
    result.odom_pose = predict_odom_state.Pose();
    result.reg_fitness = matcher_->GetFitnessScore(2.0F);
    result.consistency_hit = 0;
    result.consistency_required = 0;
    result.is_tracking = true;
    if (ConfigParameters::Instance().localization_enable_ground_height_constraint_) {
      if (const auto map_ground_z = EstimateMapGroundZ(result.map_pose);
          map_ground_z.has_value()) {
        result.has_map_ground_z = true;
        result.map_ground_z = map_ground_z.value();
      }
    }
    if (DiscardTimedOutFrameIfNeeded()) {
      continue;
    }
    CacheResultToSystem(result);
    UpdateCurrentLidarCloud(*TransformPointCloud(
        VoxelGridCloud(curr_cloud_cluster_ptr_->ordered_cloud_, 0.5),
        curr_pose));

    delta_pose_ = delta_pose;
    last_pose_ = curr_pose;
    last_nav_state_ = curr_nav_state;
    last_odom_state_ = predict_odom_state;
    last_odom_state_.bg_ = curr_nav_state.bg_;
    last_odom_state_.ba_ = curr_nav_state.ba_;

    // reset pre-integration and reset bias
    if (ConfigParameters::Instance().fusion_method_ ==
        kFusionTightCouplingOptimization) {
      if (pre_integration_ptr_) {
        pre_integration_ptr_->Reset();
        pre_integration_ptr_->SetGyroAccBias(curr_nav_state.bg_,
                                             curr_nav_state.ba_);
      }
    }
    if (odom_pre_integration_ptr_) {
      odom_pre_integration_ptr_->Reset();
      odom_pre_integration_ptr_->SetGyroAccBias(curr_nav_state.bg_,
                                                curr_nav_state.ba_);
    }
    complete_frame_without_timeout();
  }
}

PCLPointCloudXYZI::Ptr Localization::LoadGlobalMap(const std::string& path) {
  if (path.empty()) {
    return nullptr;
  }

  return LoadPcdAsXYZI(path);
}

void Localization::LoadScanContextDb() {
  if (use_tile_map_) {
    LOG(WARNING) << "Scan Context auto relocalization is disabled for tile maps.";
    scan_context_db_loaded_ = false;
    return;
  }
  std::string db_path = ConfigParameters::Instance().localization_sc_db_path_;
  if (db_path.empty()) {
    std::string map_path = ConfigParameters::Instance().localization_map_path_;
    if (map_path.empty()) {
      map_path = kDataPath + kGlobalMapFileName;
    }
    const auto slash = map_path.find_last_of('/');
    db_path = slash == std::string::npos
                  ? "sc_db.bin"
                  : map_path.substr(0, slash + 1U) + "sc_db.bin";
  }

  std::string map_tag;
  scan_context_db_loaded_ = scan_context_.LoadDB(db_path, &map_tag);
  if (scan_context_db_loaded_) {
    // LoadDB overwrites config from the file; re-apply the retrieval-only
    // ring-key gate from the runtime parameters (the fine registration below
    // is the real validator, so a generous gate is preferred).
    scan_context_.SetRingKeyMaxDist(
        ConfigParameters::Instance().localization_sc_ringkey_max_dist_);
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
  if (!ConfigParameters::Instance().imu_has_orientation_ ||
      curr_cloud_cluster_ptr_->imu_data_.empty()) {
    LOG_EVERY_N(WARNING, 10)
        << "Scan Context auto relocalization requires an IMU orientation.";
    return std::nullopt;
  }

  const Mat4d& lidar_to_imu =
      ConfigParameters::Instance().calibration_lidar_to_imu_;
  const double calibration_determinant =
      lidar_to_imu.block<3, 3>(0, 0).determinant();
  if (!lidar_to_imu.allFinite() ||
      !std::isfinite(calibration_determinant) ||
      std::fabs(calibration_determinant) <
          std::numeric_limits<double>::epsilon()) {
    LOG(WARNING) << "Skip Scan Context relocalization due to invalid "
                    "lidar-to-IMU calibration.";
    return std::nullopt;
  }
  const Mat4d imu_to_lidar = lidar_to_imu.inverse();

  // Mapping keyframe features are in the lidar frame, whereas localization
  // ordered_cloud_ is deskewed in the IMU frame. Convert only the descriptor
  // input so Scan Context compares like-for-like local geometry.
  const PCLPointCloudXYZI query_cloud = TransformPointCloud(
      curr_cloud_cluster_ptr_->ordered_cloud_, imu_to_lidar);
  const auto query_descriptor = scan_context_.MakeDescriptor(query_cloud);
  const int top_n = std::max(
      1, ConfigParameters::Instance().localization_sc_top_n_);
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
      0.0, ConfigParameters::Instance().localization_sc_fitness_thres_));
  const Mat3d lidar_to_imu_rotation = lidar_to_imu.block<3, 3>(0, 0);
  const Mat3d imu_to_lidar_rotation = imu_to_lidar.block<3, 3>(0, 0);

  std::optional<Mat4d> base_to_lidar;
  if (ConfigParameters::Instance().localization_enable_ground_height_constraint_) {
    base_to_lidar = system_ptr_->GetBaseToLidarTransform();
    if (!base_to_lidar.has_value()) {
      LOG_EVERY_N(WARNING, 10)
          << "Scan Context relocalization is waiting for base-to-lidar TF.";
      return std::nullopt;
    }
  }

  const double accept_fitness = std::max(
      0.0, ConfigParameters::Instance().localization_sc_accept_fitness_);
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
      const double lidar_yaw =
          std::atan2(candidate.pose(1, 0), candidate.pose(0, 0)) +
          candidate.yaw_shift;
      const Mat3d nominal_map_lidar_rotation =
          Eigen::AngleAxisd(lidar_yaw, Vec3d::UnitZ()).toRotationMatrix();
      const Mat3d nominal_map_imu_rotation =
          nominal_map_lidar_rotation * imu_to_lidar_rotation;
      const double imu_yaw = std::atan2(
          nominal_map_imu_rotation(1, 0), nominal_map_imu_rotation(0, 0));
      const Mat3d map_imu_rotation =
          (Eigen::AngleAxisd(imu_yaw, Vec3d::UnitZ()) *
           Eigen::AngleAxisd(pitch, Vec3d::UnitY()) *
           Eigen::AngleAxisd(roll, Vec3d::UnitX()))
              .toRotationMatrix();

      Mat4d candidate_lidar_pose = candidate.pose;
      candidate_lidar_pose.block<3, 3>(0, 0) =
          map_imu_rotation * lidar_to_imu_rotation;

      if (ConfigParameters::Instance()
              .localization_enable_ground_height_constraint_) {
        if (const auto ground_z = EstimateMapGroundZ(candidate_lidar_pose);
            ground_z.has_value()) {
          // System's cached transform maps base_link coordinates into the
          // lidar frame, so map->base is map->lidar * base->lidar.
          const Vec3d base_to_lidar_translation =
              base_to_lidar.value().block<3, 1>(0, 3);
          candidate_lidar_pose(2, 3) =
              ground_z.value() + ConfigParameters::Instance()
                                     .localization_base_link_ground_height_ -
              (candidate_lidar_pose.block<3, 3>(0, 0) *
               base_to_lidar_translation)
                  .z();
        }
      }

      auto local_map = LoadCandidateLocalMap(candidate_lidar_pose);
      const double preparation_ms = preparation_timer.End();
      total_preparation_ms += preparation_ms;
      if (!local_map || local_map->empty()) {
        LOG(INFO) << "Auto relocalization candidate " << candidate_index + 1U
                  << "/" << candidates.size() << " (score="
                  << candidate.score << ") prepared in " << preparation_ms
                  << " ms, registered in 0 ms, fitness="
                  << std::numeric_limits<float>::infinity()
                  << " (empty local map).";
        continue;
      }
      ++prepared_count;

      stage = "fine registration";
      float fitness = std::numeric_limits<float>::infinity();
      Mat4d match_pose = candidate_lidar_pose * imu_to_lidar;
      Timer registration_timer;
      ++evaluated_count;
      auto matcher = CreateMatcher();
      matcher->AddCloudToLocalMap({*local_map});
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

PCLPointCloudXYZI::Ptr Localization::LoadLocalMap(const Mat4d& pose) {
  need_update_local_map_ = false;

  if (use_tile_map_) {
    const Vec2d position_xy = pose.block<2, 1>(0, 3);
    Vec2i curr_grid_index;
    curr_grid_index.x() = std::floor(
        (position_xy.x() - tile_map_grid_size_half_) / tile_map_grid_size_);
    curr_grid_index.y() = std::floor(
        (position_xy.y() - tile_map_grid_size_half_) / tile_map_grid_size_);

    // generate neighbor tile map indices
    const auto tile_map_indices = GenerateTileMapIndices(curr_grid_index, 1);

    // load new tile map cloud
    for (const Vec2i& tile_map_index : tile_map_indices) {
      if (map_data_indices_.find(tile_map_index) == map_data_indices_.end()) {
        continue;
      }

      if (local_map_data_.find(tile_map_index) == local_map_data_.end()) {
        auto tile_map_cloud = LoadTileMap(tile_map_index);

        if (!tile_map_cloud) {
          continue;
        }

        local_map_data_[tile_map_index] = VoxelGridCloud(
            tile_map_cloud, ConfigParameters::Instance()
                                .registration_local_map_cloud_filter_size_);
        need_update_local_map_ = true;
      }
    }

    // delete distant tile map cloud
    for (auto it = local_map_data_.begin(); it != local_map_data_.end();) {
      if ((it->first - curr_grid_index).cast<float>().norm() > 2) {
        local_map_data_.erase(it++);
        need_update_local_map_ = true;
      } else {
        ++it;
      }
    }

    static PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);

    // No need to update the map
    if (!need_update_local_map_) {
      return map_cloud;
    }

    map_cloud->clear();

    // Stitching tile map cloud
    for (const auto& data : local_map_data_) {
      if (!data.second) {
        continue;
      }
      *map_cloud += *data.second;
    }

    return map_cloud;
  } else {
    if (!global_map_cloud_ptr_ || global_map_cloud_ptr_->empty()) {
      LOG_EVERY_N(WARNING, 50)
          << "Global map is unavailable, localization cannot load local map.";
      return nullptr;
    }

    PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);

    static bool first = true;
    static pcl::CropBox<PCLPointXYZI> crop_box;

    if (first || !has_init_) {
      crop_box.setInputCloud(global_map_cloud_ptr_);
      local_map_edge_.clear();
      first = false;
    }

    if (local_map_edge_.empty()) {
      need_update_local_map_ = true;
    } else {
      for (int i = 0; i < 3; ++i) {
        if (std::fabs(pose(i, 3) - local_map_edge_[i]) > 50.0 &&
            std::fabs(pose(i, 3) - local_map_edge_[i + 3]) > 50.0) {
          continue;
        }
        need_update_local_map_ = true;
        break;
      }
    }

    if (!need_update_local_map_) {
      return nullptr;
    }

    local_map_edge_.resize(6);
    local_map_edge_[0] = pose(0, 3) - 100.0;
    local_map_edge_[1] = pose(1, 3) - 100.0;
    local_map_edge_[2] = pose(2, 3) - 100.0;

    local_map_edge_[3] = pose(0, 3) + 100.0;
    local_map_edge_[4] = pose(1, 3) + 100.0;
    local_map_edge_[5] = pose(2, 3) + 100.0;

    crop_box.setMin(
        Vec4d(local_map_edge_[0], local_map_edge_[1], local_map_edge_[2], 1.0)
            .cast<float>());
    crop_box.setMax(
        Vec4d(local_map_edge_[3], local_map_edge_[4], local_map_edge_[5], 1.0)
            .cast<float>());
    crop_box.filter(*map_cloud);

    return map_cloud;
  }
}

PCLPointCloudXYZI::Ptr Localization::LoadCandidateLocalMap(
    const Mat4d& pose) {
  if (use_tile_map_) {
    LOG(WARNING) << "Scan Context auto relocalization does not support tile maps.";
    return nullptr;
  }

  PCLPointCloudXYZI::Ptr map_cloud(new PCLPointCloudXYZI);
  std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
  if (!global_map_cloud_ptr_ || global_map_cloud_ptr_->empty() ||
      !global_map_xy_kdtree_) {
    return nullptr;
  }

  constexpr double kLocalMapHalfExtent = 50.0;
  const Vec3d position = pose.block<3, 1>(0, 3);
  PCLPointXYZI query_point;
  query_point.x = static_cast<float>(position.x());
  query_point.y = static_cast<float>(position.y());
  query_point.z = 0.0F;
  std::vector<int> point_indices;
  std::vector<float> point_squared_distances;
  global_map_xy_kdtree_->radiusSearch(
      query_point, std::sqrt(2.0) * kLocalMapHalfExtent, point_indices,
      point_squared_distances);
  map_cloud->reserve(point_indices.size());
  for (const int point_index : point_indices) {
    if (point_index < 0 ||
        point_index >= static_cast<int>(global_map_cloud_ptr_->size())) {
      continue;
    }
    const auto& point = global_map_cloud_ptr_->points[point_index];
    if (std::fabs(static_cast<double>(point.x) - position.x()) >
            kLocalMapHalfExtent ||
        std::fabs(static_cast<double>(point.y) - position.y()) >
            kLocalMapHalfExtent ||
        std::fabs(static_cast<double>(point.z) - position.z()) >
            kLocalMapHalfExtent) {
      continue;
    }
    map_cloud->push_back(point);
  }
  return map_cloud;
}

PCLPointCloudXYZI::Ptr Localization::LoadTileMap(const Vec2i& tile_map_index) {
  std::string tile_map_path = kTileMapFolder + TILE_MAP_NAME(tile_map_index);
  return LoadPcdAsXYZI(tile_map_path);
}

std::vector<Vec2i> Localization::GenerateTileMapIndices(const Vec2i& index,
                                                        int step_size) {
  std::vector<Vec2i> neighbor_indices;
  for (int i = -step_size; i <= step_size; ++i) {
    for (int j = -step_size; j <= step_size; ++j) {
      Vec2i step(i, j);
      neighbor_indices.emplace_back(index + step);
    }
  }

  return neighbor_indices;
}

NavStateData Localization::IntegrateImuMeasures(
    const NavStateData& last_nav_state) const {
  pre_integration_ptr_->IntegrateDataSegment(
      curr_cloud_cluster_ptr_->imu_data_);
  return pre_integration_ptr_->Predict(last_nav_state);
}

NavStateData Localization::IntegrateImuMeasuresOdom(
    const NavStateData& last_odom_state) const {
  odom_pre_integration_ptr_->IntegrateDataSegment(
      curr_cloud_cluster_ptr_->imu_data_);
  return odom_pre_integration_ptr_->Predict(last_odom_state);
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
      (1.0 / std::pow(ConfigParameters::Instance().imu_acc_rw_noise_std_, 2.0));
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
       std::pow(ConfigParameters::Instance().imu_gyro_rw_noise_std_, 2.0));
  auto* edge_gyro_bias_rw = new EdgeGyroBiasRW();
  edge_gyro_bias_rw->setVertex(0, vertex_bg_last);
  edge_gyro_bias_rw->setVertex(1, vertex_bg_curr);
  edge_gyro_bias_rw->setInformation(gyro_bias_rw_info);
  optimizer.addEdge(edge_gyro_bias_rw);

  // start to optimize
  optimizer.initializeOptimization();
  optimizer.optimize(ConfigParameters::Instance().frontend_fusion_opti_iters_);

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
    const Mat4d& map_lidar_pose) {
  const double radius = std::max(
      0.0, ConfigParameters::Instance().localization_ground_search_radius_);
  const double percentile = std::clamp(
      ConfigParameters::Instance().localization_ground_z_percentile_, 0.0, 1.0);
  const int min_points = std::max(
      0, ConfigParameters::Instance().localization_ground_min_points_);
  const Vec3d position = map_lidar_pose.block<3, 1>(0, 3);

  std::vector<double> nearby_z;
  {
    std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
    const auto source = SelectGroundMapSource(
        global_map_cloud_ptr_, global_map_xy_kdtree_, ground_map_cloud_ptr_,
        ground_map_xy_kdtree_);
    if (!source.has_value()) {
      return std::nullopt;
    }

    PCLPointXYZI query_point;
    query_point.x = static_cast<float>(position.x());
    query_point.y = static_cast<float>(position.y());
    query_point.z = 0.0F;
    std::vector<int> point_indices;
    std::vector<float> point_squared_distances;
    source->xy_kdtree->radiusSearch(
        query_point, radius, point_indices, point_squared_distances);
    nearby_z.reserve(point_indices.size());
    for (const int point_index : point_indices) {
      if (point_index < 0 ||
          point_index >= static_cast<int>(source->height_cloud->size())) {
        continue;
      }
      const auto& point = source->height_cloud->points[point_index];
      if (!std::isfinite(point.z)) {
        continue;
      }
      nearby_z.emplace_back(static_cast<double>(point.z));
    }
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

void Localization::CacheResultToSystem(const LocalizationResult& result) const {
  LocalizationResult queued_result = result;
  queued_result.health_arrival_steady_us = SteadyNowMicroseconds();

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

PCLPointCloudXYZI Localization::GetGlobalCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_global_map_cloud_);
  if (!global_map_cloud_ptr_) {
    return {};
  }

  return *global_map_cloud_ptr_;
}

PCLPointCloudXYZI Localization::GetCurrentLidarCloudMap() {
  std::lock_guard<std::mutex> lg(mutex_lidar_cloud_);
  return lidar_cloud_;
}

void Localization::LoadTileMapIndices() {
  const std::string file_name = kTileMapFolder + kTileMapIndicesFileName;

  std::ifstream file(file_name);

  CHECK(file.is_open()) << "Load tile map indices file failed!";

  while (!file.eof()) {
    int x, y;
    file >> x >> y;
    map_data_indices_.emplace(x, y);
  }

  file.close();
}
