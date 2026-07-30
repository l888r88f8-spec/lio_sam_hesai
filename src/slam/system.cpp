#include "slam/system.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "common/constant_variable.h"
#include "common/lidar_imu_calibration.h"
#include "common/math_function.h"
#include "common/ros_utility.h"
#include "lidar/lidar_model.h"
#include "slam/config_parameters.h"
#include "slam/localization.h"
#include "slam/preprocessing.h"

namespace {

constexpr long double kMicrosecondsPerSecond = 1000000.0L;
constexpr std::uint64_t kMicrosecondsPerMillisecond = 1000U;

std::uint64_t SecondsToMicroseconds(double seconds,
                                    const std::string& parameter_name) {
  if (!std::isfinite(seconds) || seconds <= 0.0) {
    throw std::invalid_argument(parameter_name +
                                " must be finite and greater than zero");
  }

  const long double microseconds =
      static_cast<long double>(seconds) * kMicrosecondsPerSecond;
  if (microseconds < 1.0L ||
      microseconds >
          static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    throw std::invalid_argument(parameter_name +
                                " cannot be represented in microseconds");
  }
  return static_cast<std::uint64_t>(microseconds);
}

std::uint64_t RateToPeriodMicroseconds(double rate_hz,
                                       const std::string& parameter_name) {
  if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
    throw std::invalid_argument(parameter_name +
                                " must be finite and greater than zero");
  }

  const long double period_us =
      kMicrosecondsPerSecond / static_cast<long double>(rate_hz);
  if (period_us < 1.0L ||
      period_us >
          static_cast<long double>(std::numeric_limits<std::uint64_t>::max())) {
    throw std::invalid_argument(parameter_name +
                                " produces an unrepresentable period");
  }
  return static_cast<std::uint64_t>(period_us);
}

std::uint64_t MillisecondsToMicroseconds(
    int milliseconds, const std::string& parameter_name) {
  if (milliseconds <= 0) {
    throw std::invalid_argument(parameter_name + " must be greater than zero");
  }

  const auto value = static_cast<std::uint64_t>(milliseconds);
  if (value >
      std::numeric_limits<std::uint64_t>::max() /
          kMicrosecondsPerMillisecond) {
    throw std::invalid_argument(parameter_name +
                                " cannot be represented in microseconds");
  }
  return value * kMicrosecondsPerMillisecond;
}

const char* BoolString(bool value) { return value ? "true" : "false"; }

std::uint64_t SteadyNowMicroseconds() {
  const auto elapsed = std::chrono::steady_clock::now().time_since_epoch();
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count());
}

}  // namespace

System::System(const rclcpp::NodeOptions& options)
    : Node("lio_sam_hesai_relocalization", options) {
  InitConfigParameters();

  const auto& config = ConfigParameters::Instance();
  const runtime::ThreadRuntimeConfig preprocessing_thread_config{
      config.runtime_enable_realtime_scheduling_,
      config.runtime_preprocessing_fifo_priority_,
      config.runtime_preprocessing_cpu_core_};
  const runtime::ThreadRuntimeConfig localization_thread_config{
      config.runtime_enable_realtime_scheduling_,
      config.runtime_localization_fifo_priority_,
      config.runtime_localization_cpu_core_};
  runtime::ThreadRuntimeController::Validate(preprocessing_thread_config);
  runtime::ThreadRuntimeController::Validate(localization_thread_config);
  runtime_frame_processing_timeout_us_ = MillisecondsToMicroseconds(
      config.runtime_frame_processing_timeout_ms_,
      "runtime.frame_processing_timeout_ms");
  preprocessing_thread_runtime_ =
      std::make_unique<runtime::ThreadRuntimeController>(
          preprocessing_thread_config);
  localization_thread_runtime_ =
      std::make_unique<runtime::ThreadRuntimeController>(
          localization_thread_config);

  localization_health::DataStreamWatchdog::Config watchdog_config;
  watchdog_config.lidar_stall_timeout_us = SecondsToMicroseconds(
      config.health_lidar_stall_timeout_,
      "localization_health.lidar_stall_timeout");
  watchdog_config.imu_stall_timeout_us = SecondsToMicroseconds(
      config.health_imu_stall_timeout_,
      "localization_health.imu_stall_timeout");
  watchdog_config.nominal_lidar_period_us = RateToPeriodMicroseconds(
      config.health_nominal_lidar_rate_hz_,
      "localization_health.nominal_lidar_rate_hz");
  watchdog_config.nominal_imu_period_us = RateToPeriodMicroseconds(
      config.health_nominal_imu_rate_hz_,
      "localization_health.nominal_imu_rate_hz");
  data_stream_watchdog_ =
      std::make_unique<localization_health::DataStreamWatchdog>(
          watchdog_config);

  localization_health::LocalizationStateMachine::Config state_config;
  state_config.degrade_enter_frames = config.health_degrade_enter_frames_;
  state_config.recover_frames = config.health_recover_frames_;
  state_config.lost_enter_frames = config.health_lost_enter_frames_;
  state_config.degraded_fitness_thres =
      config.health_degraded_fitness_thres_;
  state_config.lost_fitness_thres = config.health_lost_fitness_thres_;
  state_config.nominal_lidar_rate_hz = config.health_nominal_lidar_rate_hz_;
  state_config.nominal_imu_rate_hz = config.health_nominal_imu_rate_hz_;
  state_config.min_stream_rate_ratio = config.health_min_rate_ratio_;
  state_config.result_timeout_us = SecondsToMicroseconds(
      config.health_result_timeout_, "localization_health.result_timeout");
  state_config.base_covariance = config.health_base_covariance_;
  state_config.max_covariance_scale = config.health_max_covariance_scale_;

  localization_health::RecoveryAttemptGate::Config recovery_gate_config;
  recovery_gate_config.nominal_lidar_rate_hz =
      config.health_nominal_lidar_rate_hz_;
  recovery_gate_config.nominal_imu_rate_hz = config.health_nominal_imu_rate_hz_;
  recovery_gate_config.min_stream_rate_ratio = config.health_min_rate_ratio_;
  localization_recovery_coordinator_ =
      std::make_unique<localization_health::LocalizationRecoveryCoordinator>(
          state_config, recovery_gate_config);

  InitLidarModel();

  localization_ptr_ = std::make_shared<Localization>(this);
  if (!localization_ptr_->PrepareMap()) {
    throw std::runtime_error("Failed to prepare localization map.");
  }

  InitPublisher();
  InitSubscriber();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(
      ConfigParameters::Instance().imu_data_searcher_buffer_size_);

  localization_thread_ptr_ = std::make_shared<std::thread>([this] {
    localization_thread_runtime_->ApplyToCurrentThread();
    localization_ptr_->Run();
  });

  pre_processing_ptr_ = std::make_shared<PreProcessing>(this);
  pre_processing_thread_ptr_ = std::make_shared<std::thread>([this] {
    preprocessing_thread_runtime_->ApplyToCurrentThread();
    pre_processing_ptr_->Run();
  });

  timer_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  system_timer_ =
      create_wall_timer(std::chrono::milliseconds(10),
                        std::bind(&System::Run, this), timer_cb_group_);
}

System::~System() {
  cv_localization_.notify_one();
  if (localization_thread_ptr_ && localization_thread_ptr_->joinable()) {
    localization_thread_ptr_->join();
  }
  cv_preprocessing_.notify_one();
  if (pre_processing_thread_ptr_ && pre_processing_thread_ptr_->joinable()) {
    pre_processing_thread_ptr_->join();
  }
}

void System::InitLidarModel() {
  const auto& lidar_type = ConfigParameters::Instance().lidar_sensor_type_;
  if (lidar_type == "None" || lidar_type == "Hesai") {
    LidarModel::Instance(lidar_type);
    int lidar_horizon_scan = ConfigParameters::Instance().lidar_horizon_scan_;
    LidarModel::Instance()->horizon_scan_num_ = lidar_horizon_scan;
    LidarModel::Instance()->vertical_scan_num_ =
        ConfigParameters::Instance().lidar_scan_;
    LidarModel::Instance()->h_res_ =
        Degree2Radian(360.0f / static_cast<float>(lidar_horizon_scan));
    LidarModel::Instance()->v_res_ = static_cast<float>(
        Degree2Radian(ConfigParameters::Instance().lidar_vertical_resolution_));
    LidarModel::Instance()->lower_angle_ = static_cast<float>(
        Degree2Radian(ConfigParameters::Instance().lidar_lower_angle_));
  } else {
    LidarModel::Instance(lidar_type);
  }
}

void System::InitConfigParameters() {
  ConfigParameters& config = ConfigParameters::Instance();

  // sensor topic name
  util::param(this, "sensor_topic.lidar_topic", config.lidar_topic_,
              config.lidar_topic_);
  util::param(this, "sensor_topic.imu_topic", config.imu_topic_,
              config.imu_topic_);

  // frame ids
  util::param(this, "frames.map", config.map_frame_, config.map_frame_);
  util::param(this, "frames.base_link", config.base_link_frame_,
              config.base_link_frame_);
  util::param(this, "frames.imu", config.imu_frame_, config.imu_frame_);
  util::param(this, "frames.lidar", config.lidar_frame_, config.lidar_frame_);

  // output topics
  util::param(this, "output_topic.mapping_odom_topic", config.mapping_odom_topic_,
              config.mapping_odom_topic_);

  // localization map path
  util::param(this, "localization.map_path", config.localization_map_path_,
              config.localization_map_path_);
  util::param(this, "localization.enable_ground_height_constraint",
              config.localization_enable_ground_height_constraint_, false);
  util::param(this, "localization.base_link_ground_height",
              config.localization_base_link_ground_height_, 0.275);
  util::param(this, "localization.ground_search_radius",
              config.localization_ground_search_radius_, 5.0);
  util::param(this, "localization.ground_z_percentile",
              config.localization_ground_z_percentile_, 0.20);
  util::param(this, "localization.ground_min_points",
              config.localization_ground_min_points_, 100);
  util::param(this, "localization.auto_relocalization",
              config.localization_auto_relocalization_, false);
  util::param(this, "localization.sc_db_path",
              config.localization_sc_db_path_, config.localization_sc_db_path_);
  util::param(this, "localization.sc_top_n", config.localization_sc_top_n_,
              8);
  util::param(this, "localization.sc_fitness_thres",
              config.localization_sc_fitness_thres_, 1.0);
  util::param(this, "localization.sc_consistency_frames",
              config.localization_sc_consistency_frames_, 3);
  util::param(this, "localization.sc_consistency_trans_thres",
              config.localization_sc_consistency_trans_thres_, 0.5);
  util::param(this, "localization.sc_consistency_yaw_thres",
              config.localization_sc_consistency_yaw_thres_, 0.1);
  util::param(this, "localization.sc_ringkey_max_dist",
              config.localization_sc_ringkey_max_dist_, 5.0);
  util::param(this, "localization.sc_accept_fitness",
              config.localization_sc_accept_fitness_, 0.3);
  util::param(this, "localization.ground_map_path",
              config.localization_ground_map_path_,
              config.localization_ground_map_path_);

  // localization health parameters
  util::param(this, "localization_health.enable", config.health_enable_, true);
  util::param(this, "localization_health.degrade_enter_frames",
              config.health_degrade_enter_frames_, 3);
  util::param(this, "localization_health.recover_frames",
              config.health_recover_frames_, 5);
  util::param(this, "localization_health.lost_enter_frames",
              config.health_lost_enter_frames_, 10);
  util::param(this, "localization_health.degraded_fitness_thres",
              config.health_degraded_fitness_thres_, 0.5);
  util::param(this, "localization_health.lost_fitness_thres",
              config.health_lost_fitness_thres_, 1.0);
  util::param(this, "localization_health.nominal_lidar_rate_hz",
              config.health_nominal_lidar_rate_hz_, 10.0);
  util::param(this, "localization_health.nominal_imu_rate_hz",
              config.health_nominal_imu_rate_hz_, 100.0);
  util::param(this, "localization_health.min_rate_ratio",
              config.health_min_rate_ratio_, 0.5);
  util::param(this, "localization_health.lidar_stall_timeout",
              config.health_lidar_stall_timeout_, 0.5);
  util::param(this, "localization_health.imu_stall_timeout",
              config.health_imu_stall_timeout_, 0.1);
  util::param(this, "localization_health.result_timeout",
              config.health_result_timeout_, 1.0);
  util::param(this, "localization_health.base_covariance",
              config.health_base_covariance_, 0.05);
  util::param(this, "localization_health.max_covariance_scale",
              config.health_max_covariance_scale_, 100.0);

  // runtime resource governance parameters
  util::param(this, "runtime.enable_realtime_scheduling",
              config.runtime_enable_realtime_scheduling_, true);
  util::param(this, "runtime.preprocessing_fifo_priority",
              config.runtime_preprocessing_fifo_priority_, 70);
  util::param(this, "runtime.localization_fifo_priority",
              config.runtime_localization_fifo_priority_, 80);
  util::param(this, "runtime.preprocessing_cpu_core",
              config.runtime_preprocessing_cpu_core_, 4);
  util::param(this, "runtime.localization_cpu_core",
              config.runtime_localization_cpu_core_, 5);
  util::param(this, "runtime.frame_processing_timeout_ms",
              config.runtime_frame_processing_timeout_ms_, 500);

  // lidar config parameters
  util::param(this, "lidar.sensor_type", config.lidar_sensor_type_,
              StringEmpty);
  util::param(this, "lidar.point_jump_span",
              config.lidar_point_jump_span_, IntNaN);
  util::param(this, "lidar.scan_lines", config.lidar_scan_, IntNaN);
  util::param(this, "lidar.lower_angle_deg", config.lidar_lower_angle_,
              DoubleNaN);
  util::param(this, "lidar.horizon_scan", config.lidar_horizon_scan_,
              IntNaN);
  util::param(this, "lidar.vertical_resolution_deg",
              config.lidar_vertical_resolution_, DoubleNaN);
  util::param(this, "lidar.min_range", config.lidar_use_min_dist_,
              FloatNaN);
  util::param(this, "lidar.max_range", config.lidar_use_max_dist_,
              FloatNaN);
  util::param(this, "lidar.point_time_scale",
              config.lidar_point_time_scale_, DoubleNaN);
  util::param(this, "lidar.rotation_noise_std",
              config.lidar_rotation_noise_std_, DoubleNaN);
  util::param(this, "lidar.position_noise_std",
              config.lidar_position_noise_std_, DoubleNaN);

  // imu config parameters
  util::param(this, "imu.has_orientation", config.imu_has_orientation_, false);
  util::param(this, "imu.init_acc_bias", config.imu_init_acc_bias_, DoubleNaN);
  util::param(this, "imu.init_gyro_bias", config.imu_init_gyro_bias_,
              DoubleNaN);
  util::param(this, "imu.acc_noise_std", config.imu_acc_noise_std_, DoubleNaN);
  util::param(this, "imu.gyro_noise_std", config.imu_gyro_noise_std_,
              DoubleNaN);
  util::param(this, "imu.acc_bias_rw_noise_std", config.imu_acc_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.gyro_bias_rw_noise_std", config.imu_gyro_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.data_searcher_buffer_size",
              config.imu_data_searcher_buffer_size_, IntNaN);

  // gravity
  util::param(this, "imu.gravity", config.gravity_norm, DoubleNaN);

  // calibration parameters
  std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  std::vector<double> rotation = identity;
  std::vector<double> rpy = identity;
  util::param_vector(this, "calibration.translation", translation,
                     translation);
  util::param_vector(this, "calibration.rotation", rotation, rotation);
  util::param_vector(this, "calibration.rpy", rpy, rpy);

  try {
    const LidarImuCalibration calibration =
        ParseLidarImuCalibration(translation, rotation, rpy);
    config.calibration_lidar_to_imu_ = calibration.lidar_to_imu;
  } catch (const std::exception& error) {
    RCLCPP_FATAL(get_logger(),
                 "Invalid lidar-IMU calibration parameters: %s",
                 error.what());
    throw;
  }

  // frontend config parameters
  util::param(this, "frontend.registration_and_searcher_mode",
              config.registration_and_searcher_mode_, StringEmpty);
  util::param(this, "frontend.feature.corner_thres",
              config.loam_feature_corner_thres_, FloatNaN);
  util::param(this, "frontend.feature.planar_thres",
              config.loam_feature_planar_thres_, FloatNaN);
  util::param(this, "frontend.feature.planar_voxel_filter_size",
              config.loam_feature_planar_voxel_filter_size_, FloatNaN);
  util::param(this, "frontend.feature.corner_voxel_filter_size",
              config.loam_feature_corner_voxel_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.line_ratio_thres",
              config.registration_line_ratio_thres_, FloatNaN);
  util::param(this, "frontend.registration.point_search_thres",
              config.registration_point_search_thres_, FloatNaN);
  util::param(this, "frontend.registration.point_to_planar_thres",
              config.registration_point_to_planar_thres_, FloatNaN);
  util::param(this, "frontend.registration.local_planar_map_size",
              config.registration_local_planar_map_size_, IntNaN);
  util::param(this, "frontend.registration.local_corner_map_size",
              config.registration_local_corner_map_size_, IntNaN);
  util::param(this, "frontend.registration.keyframe_delta_distance",
              config.registration_keyframe_delta_dist_, DoubleNaN);
  util::param(this, "frontend.registration.keyframe_delta_rotation",
              config.registration_keyframe_delta_rotation_, DoubleNaN);
  util::param(this, "frontend.registration.rotation_converge_thres",
              config.registration_rotation_converge_thres_, FloatNaN);
  util::param(this, "frontend.registration.position_converge_thres",
              config.registration_position_converge_thres_, FloatNaN);
  util::param(this, "frontend.registration.local_corner_voxel_filter_size",
              config.registration_local_corner_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.local_planar_voxel_filter_size",
              config.registration_local_planar_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.local_map_size",
              config.registration_local_map_size_, IntNaN);
  util::param(this, "frontend.registration.local_map_cloud_filter_size",
              config.registration_local_map_cloud_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.source_cloud_filter_size",
              config.registration_source_cloud_filter_size_, FloatNaN);
  util::param(this, "frontend.registration.optimization_iter_num",
              config.registration_opti_iter_num_, IntNaN);
  util::param(this, "frontend.registration.ndt_voxel_size",
              config.registration_ndt_voxel_size_, DoubleNaN);
  util::param(this, "frontend.registration.ndt_outlier_threshold",
              config.registration_ndt_outlier_threshold_, DoubleNaN);
  util::param(this, "frontend.registration.ndt_min_points_in_voxel",
              config.registration_ndt_min_points_in_voxel_, IntNaN);
  util::param(this, "frontend.registration.ndt_max_points_in_voxel",
              config.registration_ndt_max_points_in_voxel_, IntNaN);
  util::param(this, "frontend.registration.ndt_min_effective_pts",
              config.registration_ndt_min_effective_pts_, IntNaN);
  util::param(this, "frontend.registration.ndt_capacity",
              config.registration_ndt_capacity_, IntNaN);
  util::param(this, "frontend.fusion_method", config.fusion_method_,
              StringEmpty);
  util::param(this, "frontend.fusion_opti_iters",
              config.frontend_fusion_opti_iters_, IntNaN);

  // system config parameters
  util::param(this, "system.keyframe_delta_distance",
              config.system_keyframe_delta_dist_, DoubleNaN);
  util::param(this, "system.keyframe_delta_rotation",
              config.system_keyframe_delta_rotation_, DoubleNaN);
  util::param(this, "loop_closure.enable",
              config.system_enable_loopclosure_, false);
  util::param(this, "visualization.enable_global_map",
              config.system_enable_visualize_global_map_, false);
  util::param(this, "visualization.global_map_resolution",
              config.system_global_map_visualization_resolution_, FloatNaN);

  // split map
  util::param(this, "system.tile_map_grid_size", config.tile_map_grid_size_,
              DoubleNaN);

  // loopclosure config parameters
  util::param(this, "loop_closure.skip_near_loopclosure_threshold",
              config.lc_skip_near_loopclosure_threshold_, IntNaN);
  util::param(this, "loop_closure.skip_near_keyframe_threshold",
              config.lc_skip_near_keyframe_threshold_, IntNaN);
  util::param(this, "loop_closure.candidate_local_map_left_range",
              config.lc_candidate_local_map_left_range_, IntNaN);
  util::param(this, "loop_closure.candidate_local_map_right_range",
              config.lc_candidate_local_map_right_range_, IntNaN);
  util::param(this, "loop_closure.loopclosure_local_map_left_range",
              config.lc_loopclosure_local_map_left_range_, IntNaN);
  util::param(this, "loop_closure.near_neighbor_distance_threshold",
              config.lc_near_neighbor_distance_threshold_, DoubleNaN);
  util::param(this, "loop_closure.registration_converge_threshold",
              config.lc_registration_converge_threshold_, FloatNaN);
}

void System::InitPublisher() {
  localization_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("lio_sam/localization/path", 5);
  localization_global_cloud_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "lio_sam/localization/global_map",
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  localization_current_lidar_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          "lio_sam/localization/current_lidar", 5);

  localization_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      ConfigParameters::Instance().mapping_odom_topic_, 5);
  diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
                                                              10);
}

void System::InitSubscriber() {
  lidar_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = lidar_cb_group_;
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      ConfigParameters::Instance().lidar_topic_,
      rclcpp::QoS(10).best_effort().keep_last(1),
      std::bind(&System::LidarMsgCallBack, this, std::placeholders::_1),
      lidar_sub_opt);

  imu_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = imu_cb_group_;
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      ConfigParameters::Instance().imu_topic_,
      rclcpp::QoS(400).best_effort().keep_last(1),
      std::bind(&System::ImuMsgCallBack, this, std::placeholders::_1),
      imu_sub_opt);

  init_pose_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto init_sub_opt = rclcpp::SubscriptionOptions();
  init_sub_opt.callback_group = init_pose_cb_group_;
  localization_init_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", rclcpp::QoS(1).reliable().keep_last(1),
          std::bind(&System::LocalizationInitPoseMsgCallBack, this,
                    std::placeholders::_1),
          init_sub_opt);
}

void System::LidarMsgCallBack(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr) {
  data_stream_watchdog_->RecordLidar(RosTimeToUs(cloud_ros_ptr->header),
                                     SteadyNowMicroseconds());

  if (!has_imu_init_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);
  while (raw_cloud_deque_.size() >= kMaxRawCloudQueueSize) {
    raw_cloud_deque_.pop_front();
  }
  raw_cloud_deque_.push_back(cloud_ros_ptr);
  cv_preprocessing_.notify_one();
}

void System::LocalizationInitPoseMsgCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  Eigen::Quaterniond q = RosQuaternionToEigen(msg->pose.pose.orientation);
  Vec3d t = RosPoint3dToEigen(msg->pose.pose.position);
  Mat4d map_base_pose = Mat4d::Identity();
  map_base_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  map_base_pose.block<3, 1>(0, 3) = t;

  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;
  const auto& lidar_frame = ConfigParameters::Instance().lidar_frame_;

  try {
    auto tf_msg =
        tf_buffer_->lookupTransform(base_frame, lidar_frame, rclcpp::Time(0));
    const Mat4d T_base_to_lidar = tf2::transformToEigen(tf_msg).matrix();
    localization_ptr_->SetInitPose(map_base_pose * T_base_to_lidar);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(),
                "Ignore initial pose because TF lookup failed (%s -> %s): %s",
                base_frame.c_str(), lidar_frame.c_str(), ex.what());
  }
}

void System::ImuMsgCallBack(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr) {
  static Vec3d init_mean_acc = Vec3d::Zero();
  static Vec3d last_angular_velocity;
  static Eigen::Quaterniond last_orientation;
  static TimeStampUs last_timestamp;

  IMUData imu_data;
  imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);
  data_stream_watchdog_->RecordImu(imu_data.timestamp_,
                                   SteadyNowMicroseconds());
  imu_data.angular_velocity_ = RosVec3dToEigen(imu_ptr->angular_velocity);
  imu_data.linear_acceleration_ = RosVec3dToEigen(imu_ptr->linear_acceleration);

  if (ConfigParameters::Instance().fusion_method_ == kFusionLooseCoupling) {
    if (!has_imu_init_.load()) {
      if (ConfigParameters::Instance().imu_has_orientation_) {
        imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
      } else {
        imu_data.orientation_ = Eigen::Quaterniond::Identity();
        last_angular_velocity = imu_data.angular_velocity_;
        last_orientation = Eigen::Quaterniond::Identity();
        last_timestamp = imu_data.timestamp_;
      }
      has_imu_init_.store(true);
      imu_data_searcher_ptr_->CacheData(imu_data);
      return;
    }
  } else {
    if (!has_imu_init_.load()) {
      has_imu_init_.store(InitIMU(imu_data, init_mean_acc));

      if (has_imu_init_.load()) {
        if (ConfigParameters::Instance().imu_has_orientation_) {
          imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
        } else {
          imu_data.orientation_ = Eigen::Quaterniond::Identity();
          last_angular_velocity = imu_data.angular_velocity_;
          last_orientation = Eigen::Quaterniond::Identity();
          last_timestamp = imu_data.timestamp_;
        }

        imu_data.linear_acceleration_ =
            imu_data.linear_acceleration_ *
            ConfigParameters::Instance().gravity_norm / init_mean_acc.norm();

        imu_data_searcher_ptr_->CacheData(imu_data);
      }
      return;
    }

    imu_data.linear_acceleration_ = imu_data.linear_acceleration_ *
                                    ConfigParameters::Instance().gravity_norm /
                                    init_mean_acc.norm();
  }

  if (ConfigParameters::Instance().imu_has_orientation_) {
    imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
  } else {
    const Eigen::Quaterniond delta_q(
        SO3Exp((last_angular_velocity + imu_data.angular_velocity_) * 0.5 *
               (imu_data.timestamp_ - last_timestamp) * kMicroseconds2Seconds));
    imu_data.orientation_ = last_orientation * delta_q;

    last_timestamp = imu_data.timestamp_;
    last_orientation = imu_data.orientation_;
    last_angular_velocity = imu_data.angular_velocity_;
  }

  imu_data_searcher_ptr_->CacheData(imu_data);
}

bool System::InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc) {
  static Vec3d mean_acc = Vec3d::Zero();
  static Vec3d mean_gyro = Vec3d::Zero();
  static Vec3d cov_acc = Vec3d::Zero();
  static Vec3d cov_gyro = Vec3d::Zero();
  static int N = 0;

  if (N == 0) {
    mean_acc = imu_data.linear_acceleration_;
    mean_gyro = imu_data.angular_velocity_;
    N = 1;
  } else {
    const auto& acc = imu_data.linear_acceleration_;
    const auto& gyro = imu_data.angular_velocity_;

    mean_acc += (acc - mean_acc) / N;
    mean_gyro += (gyro - mean_gyro) / N;

    cov_acc =
        cov_acc * (N - 1.0) / N +
        (acc - mean_acc).cwiseProduct(acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyro =
        cov_gyro * (N - 1.0) / N +
        (gyro - mean_gyro).cwiseProduct(gyro - mean_gyro) * (N - 1.0) / (N * N);

    N++;
  }

  init_mean_acc = mean_acc;

  if (N > 300) {
    N = 0;
    mean_acc = Vec3d::Zero();
    mean_gyro = Vec3d::Zero();
    cov_acc = Vec3d::Zero();
    cov_gyro = Vec3d::Zero();

    LOG(WARNING) << "IMU movement acceleration is too large, Reinitialize!";
    return false;
  }

  if (N > 200 && cov_acc.norm() < 0.05 && cov_gyro.norm() < 0.01) {
    ConfigParameters::Instance().gravity_vector_ =
        -mean_acc / mean_acc.norm() * ConfigParameters::Instance().gravity_norm;
    return true;
  }

  return false;
}

bool System::UpdateBaseToLidarTransform() {
  {
    std::lock_guard<std::mutex> lg(mutex_base_to_lidar_transform_);
    if (has_base_to_lidar_) {
      return true;
    }
  }

  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;
  const auto& lidar_frame = ConfigParameters::Instance().lidar_frame_;

  try {
    auto tf_msg = tf_buffer_->lookupTransform(
        lidar_frame, base_frame, rclcpp::Time(0));
    Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_msg);
    std::lock_guard<std::mutex> lg(mutex_base_to_lidar_transform_);
    if (!has_base_to_lidar_) {
      T_base_to_lidar_ = tf_eigen.matrix();
      has_base_to_lidar_ = true;
    }
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s", lidar_frame.c_str(),
                         base_frame.c_str(), ex.what());
  }

  std::lock_guard<std::mutex> lg(mutex_base_to_lidar_transform_);
  return has_base_to_lidar_;
}

std::optional<Mat4d> System::GetBaseToLidarTransform() {
  if (!UpdateBaseToLidarTransform()) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lg(mutex_base_to_lidar_transform_);
  return T_base_to_lidar_;
}

void System::Run() {
  // publish localization path and odometry
  bool had_new_result = false;
  if (ProcessLocalizationResultCache(had_new_result)) {
    PublishLocalizationPath();

    auto current_lidar_cloud = localization_ptr_->GetCurrentLidarCloudMap();
    ApplyLocalizationZOffset(current_lidar_cloud);
    PublishRosCloud(localization_current_lidar_cloud_pub_,
                    current_lidar_cloud.makeShared(),
                    ConfigParameters::Instance().map_frame_);
  }
  if (!had_new_result) {
    PublishLocalizationHealth();
  }

  // publish global cloud map
  static bool first_global_map_visualization = true;
  if (first_global_map_visualization &&
      localization_global_cloud_map_pub_->get_subscription_count() > 0) {
    const auto global_cloud_map = localization_ptr_->GetGlobalCloudMap();

    if (!global_cloud_map.empty()) {
      PublishRosCloud(localization_global_cloud_map_pub_,
                      global_cloud_map.makeShared(),
                      ConfigParameters::Instance().map_frame_);
      first_global_map_visualization = false;
    }
  }
}

bool System::ProcessLocalizationResultCache(bool& had_new_result) {
  had_new_result = false;
  decltype(localization_results_deque_) localization_results;

  {
    std::lock_guard<std::mutex> lg(mutex_localization_results_deque_);

    if (localization_results_deque_.empty()) {
      return false;
    }

    localization_results.swap(localization_results_deque_);
  }
  had_new_result = true;

  for (const auto& health_result : localization_results) {
    {
      std::lock_guard<std::mutex> lock(mutex_health_result_);
      last_health_result_stamp_ = health_result.timestamp_;
      last_health_result_arrival_steady_us_ =
          health_result.health_arrival_steady_us == 0U
              ? SteadyNowMicroseconds()
              : health_result.health_arrival_steady_us;
      last_health_result_fitness_ = health_result.reg_fitness;
      last_health_consistency_hit_ = health_result.consistency_hit;
      last_health_consistency_required_ = health_result.consistency_required;
      last_health_result_is_tracking_ = health_result.is_tracking;
      has_health_result_ = true;
      has_new_health_result_ = true;
    }
    PublishLocalizationHealth();
  }

  const LocalizationResult& result = localization_results.back();

  const rclcpp::Time stamp(result.timestamp_ * 1000ULL);

  const auto base_to_lidar = GetBaseToLidarTransform();
  if (!base_to_lidar.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip localization result because %s -> %s TF is unavailable.",
        ConfigParameters::Instance().lidar_frame_.c_str(),
        ConfigParameters::Instance().base_link_frame_.c_str());
    return false;
  }

  Mat4d map_base_pose = result.map_pose * base_to_lidar.value();
  if (ConfigParameters::Instance().localization_enable_ground_height_constraint_ &&
      result.has_map_ground_z) {
    const double desired_base_z =
        result.map_ground_z +
        ConfigParameters::Instance().localization_base_link_ground_height_;
    last_localization_z_offset_ = desired_base_z - map_base_pose(2, 3);
    has_localization_z_offset_ = true;
    map_base_pose(2, 3) = desired_base_z;

    const double base_ground_height = map_base_pose(2, 3) - result.map_ground_z;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Current base_link ground height: %.3f m (base_link_z=%.3f, "
        "map_ground_z=%.3f)",
        base_ground_height, map_base_pose(2, 3), result.map_ground_z);
  } else {
    last_localization_z_offset_ = 0.0;
    has_localization_z_offset_ = false;
  }
  PublishTF(map_base_pose, result.timestamp_);

  const Eigen::Quaterniond q(map_base_pose.block<3, 3>(0, 0));
  const Vec3d& t = map_base_pose.block<3, 1>(0, 3);
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose.orientation.x = q.x();
  pose_stamped.pose.orientation.y = q.y();
  pose_stamped.pose.orientation.z = q.z();
  pose_stamped.pose.orientation.w = q.w();
  pose_stamped.pose.position.x = t.x();
  pose_stamped.pose.position.y = t.y();
  pose_stamped.pose.position.z = t.z();
  pose_stamped.header.frame_id = ConfigParameters::Instance().map_frame_;
  pose_stamped.header.stamp = stamp;
  localization_path_.poses.emplace_back(std::move(pose_stamped));
  if (localization_path_.poses.size() > kMaxLocalizationPathSize) {
    localization_path_.poses.erase(localization_path_.poses.begin());
  }

  nav_msgs::msg::Odometry mapping_odom;
  mapping_odom.header.stamp = stamp;
  mapping_odom.header.frame_id = ConfigParameters::Instance().map_frame_;
  mapping_odom.child_frame_id = ConfigParameters::Instance().base_link_frame_;
  mapping_odom.pose.pose.position.x = map_base_pose(0, 3);
  mapping_odom.pose.pose.position.y = map_base_pose(1, 3);
  mapping_odom.pose.pose.position.z = map_base_pose(2, 3);
  mapping_odom.pose.pose.orientation.x = q.x();
  mapping_odom.pose.pose.orientation.y = q.y();
  mapping_odom.pose.pose.orientation.z = q.z();
  mapping_odom.pose.pose.orientation.w = q.w();
  if (ConfigParameters::Instance().health_enable_) {
    mapping_odom.pose.covariance.fill(0.0);
    const double covariance =
        ConfigParameters::Instance().health_base_covariance_ *
        current_health_covariance_scale_;
    mapping_odom.pose.covariance[0] = covariance;
    mapping_odom.pose.covariance[7] = covariance;
    mapping_odom.pose.covariance[14] = covariance;
    mapping_odom.pose.covariance[21] = covariance;
    mapping_odom.pose.covariance[28] = covariance;
    mapping_odom.pose.covariance[35] = covariance;
  }
  localization_odom_pub_->publish(mapping_odom);

  return true;
}

void System::PublishLocalizationHealth() {
  const auto& config = ConfigParameters::Instance();
  if (!diagnostics_pub_) {
    return;
  }
  if (!config.health_enable_) {
    PublishRuntimeDiagnostics();
    return;
  }
  if (!data_stream_watchdog_ || !localization_recovery_coordinator_) {
    return;
  }

  const std::uint64_t steady_now_us = SteadyNowMicroseconds();

  localization_health::HealthInputs inputs;
  inputs.has_imu_init = has_imu_init_.load();
  inputs.map_ready = localization_ptr_ && localization_ptr_->IsMapReady();
  const double map_preparation_duration_ms =
      localization_ptr_ ? localization_ptr_->MapPreparationDurationMs() : 0.0;
  inputs.now_us = steady_now_us;
  {
    std::lock_guard<std::mutex> lock(mutex_health_result_);
    inputs.has_init = has_health_result_;
    inputs.is_tracking = last_health_result_is_tracking_;
    inputs.has_new_result = has_new_health_result_;
    inputs.reg_fitness = last_health_result_fitness_;
    inputs.consistency_hit = last_health_consistency_hit_;
    inputs.consistency_required = last_health_consistency_required_;
    if (has_health_result_ &&
        steady_now_us > last_health_result_arrival_steady_us_) {
      inputs.last_result_age_us =
          steady_now_us - last_health_result_arrival_steady_us_;
    }
    has_new_health_result_ = false;
  }
  inputs.stream = data_stream_watchdog_->Evaluate(steady_now_us);
  inputs.has_new_stream_observation = inputs.stream.has_new_observation;

  std::function<void()> request_automatic_relocalization;
  if (localization_ptr_) {
    request_automatic_relocalization = [this] {
      localization_ptr_->RequestAutomaticRelocalization();
    };
  }
  const auto recovery_update = localization_recovery_coordinator_->Update(
      inputs, config.localization_auto_relocalization_,
      request_automatic_relocalization);
  const auto& report = recovery_update.report;
  current_health_covariance_scale_ = report.covariance_scale;
  if (recovery_update.request_sent) {
    last_auto_relocalization_trigger_ =
        localization_health::ToString(recovery_update.reason);
    LOG(INFO) << "Requesting automatic relocalization due to "
              << last_auto_relocalization_trigger_;
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lio_sam_hesai: localization";
  status.hardware_id = "lio_sam_hesai_relocalization";
  switch (report.level) {
    case localization_health::HealthLevel::kOk:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      break;
    case localization_health::HealthLevel::kWarn:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      break;
    case localization_health::HealthLevel::kError:
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      break;
  }
  status.message = report.reason;

  const auto add_value = [&status](const std::string& key,
                                   const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.emplace_back(std::move(item));
  };
  add_value("health_score", std::to_string(report.health_score));
  add_value("s_fitness", std::to_string(report.s_fitness));
  add_value("s_stream", std::to_string(report.s_stream));
  add_value("s_consistency", std::to_string(report.s_consistency));
  add_value("lidar_rate_hz", std::to_string(inputs.stream.lidar_rate_hz));
  add_value("imu_rate_hz", std::to_string(inputs.stream.imu_rate_hz));
  add_value("covariance_scale", std::to_string(report.covariance_scale));
  add_value("auto_relocalization_trigger",
            last_auto_relocalization_trigger_);
  add_value("reg_fitness",
            std::to_string(localization_health::DiagnosticFitnessForPublication(
                inputs.reg_fitness, config.health_lost_fitness_thres_)));
  add_value("last_result_age_us", std::to_string(inputs.last_result_age_us));
  add_value("has_imu_init", BoolString(inputs.has_imu_init));
  add_value("has_init", BoolString(inputs.has_init));
  add_value("consistency_hit", std::to_string(inputs.consistency_hit));
  add_value("consistency_required",
            std::to_string(inputs.consistency_required));
  add_value("reason", report.reason);
  add_value("is_tracking", BoolString(inputs.is_tracking));
  add_value("map_ready", BoolString(inputs.map_ready));
  add_value("map_preparation_duration_ms",
            std::to_string(map_preparation_duration_ms));
  AppendRuntimeDiagnostics(status);
  add_value("rates_known", BoolString(inputs.stream.rates_known));
  add_value("lidar_stalled", BoolString(inputs.stream.lidar_stalled));
  add_value("imu_stalled", BoolString(inputs.stream.imu_stalled));
  add_value("timestamp_monotonic",
            BoolString(inputs.stream.timestamp_monotonic));
  add_value("timestamp_jump", BoolString(inputs.stream.timestamp_jump));
  diagnostics.status.emplace_back(std::move(status));
  diagnostics_pub_->publish(diagnostics);
}

void System::PublishRuntimeDiagnostics() {
  if (!diagnostics_pub_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = get_clock()->now();
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "lio_sam_hesai: runtime";
  status.hardware_id = "lio_sam_hesai_relocalization";
  status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  status.message = "runtime resource diagnostics";
  AppendRuntimeDiagnostics(status);
  diagnostics.status.emplace_back(std::move(status));
  diagnostics_pub_->publish(diagnostics);
}

void System::AppendRuntimeDiagnostics(
    diagnostic_msgs::msg::DiagnosticStatus& status) const {
  const auto& config = ConfigParameters::Instance();
  const runtime::ThreadRuntimeStatus preprocessing_runtime_status =
      preprocessing_thread_runtime_
          ? preprocessing_thread_runtime_->Snapshot()
          : runtime::ThreadRuntimeStatus{};
  const runtime::ThreadRuntimeStatus localization_runtime_status =
      localization_thread_runtime_
          ? localization_thread_runtime_->Snapshot()
          : runtime::ThreadRuntimeStatus{};

  const auto add_value = [&status](const std::string& key,
                                   const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.emplace_back(std::move(item));
  };
  add_value("preprocessing_fifo_priority",
            std::to_string(config.runtime_preprocessing_fifo_priority_));
  add_value("preprocessing_cpu_core",
            std::to_string(config.runtime_preprocessing_cpu_core_));
  add_value("preprocessing_fifo_applied",
            BoolString(preprocessing_runtime_status.scheduling_applied));
  add_value("preprocessing_fifo_errno",
            std::to_string(preprocessing_runtime_status.scheduling_errno));
  add_value("preprocessing_affinity_applied",
            BoolString(preprocessing_runtime_status.affinity_applied));
  add_value("preprocessing_affinity_errno",
            std::to_string(preprocessing_runtime_status.affinity_errno));
  add_value("localization_fifo_priority",
            std::to_string(config.runtime_localization_fifo_priority_));
  add_value("localization_cpu_core",
            std::to_string(config.runtime_localization_cpu_core_));
  add_value("localization_fifo_applied",
            BoolString(localization_runtime_status.scheduling_applied));
  add_value("localization_fifo_errno",
            std::to_string(localization_runtime_status.scheduling_errno));
  add_value("localization_affinity_applied",
            BoolString(localization_runtime_status.affinity_applied));
  add_value("localization_affinity_errno",
            std::to_string(localization_runtime_status.affinity_errno));
}

void System::ApplyLocalizationZOffset(PCLPointCloudXYZI& cloud) const {
  if (!has_localization_z_offset_ ||
      !ConfigParameters::Instance().localization_enable_ground_height_constraint_) {
    return;
  }

  for (auto& point : cloud.points) {
    point.z += static_cast<float>(last_localization_z_offset_);
  }
}

void System::PublishLocalizationPath() {
  if (localization_path_pub_->get_subscription_count() > 0) {
    localization_path_.header.frame_id = ConfigParameters::Instance().map_frame_;
    localization_path_.header.stamp = this->get_clock()->now();
    localization_path_pub_->publish(localization_path_);
  }
}

void System::PublishTF(const Mat4d& map_base_pose, TimeStampUs timestamp) {
  const rclcpp::Time stamp(timestamp * 1000ULL);
  const auto& map_frame = ConfigParameters::Instance().map_frame_;
  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;

  tf_broadcaster_->sendTransform(eigen2Transform(
      map_base_pose.block<3, 3>(0, 0),
      map_base_pose.block<3, 1>(0, 3),
      map_frame,
      base_frame,
      static_cast<double>(stamp.nanoseconds())));
}
