#include "slam/system.h"

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "common/constant_variable.h"
#include "common/lidar_imu_calibration.h"
#include "common/math_function.h"
#include "common/ros_pose_adapter.h"
#include "common/ros_utility.h"
#include "lidar/lidar_model.h"
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

TimeStampUs RosTimeToMicroseconds(const rclcpp::Time& time) {
  const auto nanoseconds = time.nanoseconds();
  if (nanoseconds <= 0) {
    return 0U;
  }
  return static_cast<TimeStampUs>(nanoseconds / 1000LL);
}

const char* TfModeString(localization_health::TfPublicationMode mode) {
  switch (mode) {
    case localization_health::TfPublicationMode::kMeasured:
      return "measured";
    case localization_health::TfPublicationMode::kExtrapolated:
      return "extrapolated";
    case localization_health::TfPublicationMode::kHeld:
      return "held";
    case localization_health::TfPublicationMode::kNoAnchor:
      return "no_anchor";
  }
  return "no_anchor";
}

const char* TfHoldReasonString(localization_health::TfHoldReason reason) {
  switch (reason) {
    case localization_health::TfHoldReason::kNone:
      return "none";
    case localization_health::TfHoldReason::kImuUnavailable:
      return "imu_unavailable";
    case localization_health::TfHoldReason::kExtrapolationLimit:
      return "extrapolation_limit";
    case localization_health::TfHoldReason::kNoAnchor:
      return "no_anchor";
  }
  return "no_anchor";
}

}  // namespace

System::System(const rclcpp::NodeOptions& options)
    : Node("lio_sam_hesai_relocalization", options),
      config_(std::make_shared<const SlamConfig>(
          LoadSlamConfig(*this, SlamMode::kLocalization))) {
  const runtime::ThreadRuntimeConfig preprocessing_thread_config{
      config_->localization.runtime.enable_realtime_scheduling,
      config_->localization.runtime.preprocessing_fifo_priority,
      config_->localization.runtime.preprocessing_cpu_core};
  const runtime::ThreadRuntimeConfig localization_thread_config{
      config_->localization.runtime.enable_realtime_scheduling,
      config_->localization.runtime.localization_fifo_priority,
      config_->localization.runtime.localization_cpu_core};
  runtime::ThreadRuntimeController::Validate(preprocessing_thread_config);
  runtime::ThreadRuntimeController::Validate(localization_thread_config);
  runtime_frame_processing_timeout_us_ = MillisecondsToMicroseconds(
      config_->localization.runtime.frame_processing_timeout_ms,
      "runtime.frame_processing_timeout_ms");
  preprocessing_thread_runtime_ =
      std::make_unique<runtime::ThreadRuntimeController>(
          preprocessing_thread_config);
  localization_thread_runtime_ =
      std::make_unique<runtime::ThreadRuntimeController>(
          localization_thread_config);

  localization_health::DataStreamWatchdog::Config watchdog_config;
  watchdog_config.lidar_stall_timeout_us = SecondsToMicroseconds(
      config_->localization.health.lidar_stall_timeout,
      "localization_health.lidar_stall_timeout");
  watchdog_config.imu_stall_timeout_us = SecondsToMicroseconds(
      config_->localization.health.imu_stall_timeout,
      "localization_health.imu_stall_timeout");
  watchdog_config.nominal_lidar_period_us = RateToPeriodMicroseconds(
      config_->localization.health.nominal_lidar_rate_hz,
      "localization_health.nominal_lidar_rate_hz");
  watchdog_config.nominal_imu_period_us = RateToPeriodMicroseconds(
      config_->localization.health.nominal_imu_rate_hz,
      "localization_health.nominal_imu_rate_hz");
  data_stream_watchdog_ =
      std::make_unique<localization_health::DataStreamWatchdog>(
          watchdog_config);

  localization_health::LocalizationStateMachine::Config state_config;
  state_config.degrade_enter_frames =
      config_->localization.health.degrade_enter_frames;
  state_config.recover_frames = config_->localization.health.recover_frames;
  state_config.lost_enter_frames =
      config_->localization.health.lost_enter_frames;
  state_config.degraded_fitness_thres =
      config_->localization.health.degraded_fitness_thres;
  state_config.lost_fitness_thres =
      config_->localization.health.lost_fitness_thres;
  state_config.nominal_lidar_rate_hz =
      config_->localization.health.nominal_lidar_rate_hz;
  state_config.nominal_imu_rate_hz =
      config_->localization.health.nominal_imu_rate_hz;
  state_config.min_stream_rate_ratio =
      config_->localization.health.min_rate_ratio;
  const TimeStampUs health_result_timeout_us = SecondsToMicroseconds(
      config_->localization.health.result_timeout,
      "localization_health.result_timeout");
  state_config.result_timeout_us = health_result_timeout_us;
  state_config.base_covariance = config_->localization.health.base_covariance;
  state_config.max_covariance_scale =
      config_->localization.health.max_covariance_scale;

  localization_health::RecoveryAttemptGate::Config recovery_gate_config;
  recovery_gate_config.nominal_lidar_rate_hz =
      config_->localization.health.nominal_lidar_rate_hz;
  recovery_gate_config.nominal_imu_rate_hz =
      config_->localization.health.nominal_imu_rate_hz;
  recovery_gate_config.min_stream_rate_ratio =
      config_->localization.health.min_rate_ratio;
  localization_recovery_coordinator_ =
      std::make_unique<localization_health::LocalizationRecoveryCoordinator>(
          state_config, recovery_gate_config);

  localization_health::TfContinuityController::Config tf_config;
  tf_config.publish_period_us = RateToPeriodMicroseconds(
      config_->localization.tf.publish_rate_hz,
      "localization_tf.publish_rate_hz");
  tf_config.max_extrapolation_us = SecondsToMicroseconds(
      config_->localization.tf.max_extrapolation_seconds,
      "localization_tf.max_extrapolation_seconds");
  if (tf_config.max_extrapolation_us > health_result_timeout_us) {
    throw std::invalid_argument(
        "localization_tf.max_extrapolation_seconds must not exceed "
        "localization_health.result_timeout");
  }
  localization_health::TfContinuityController::Validate(tf_config);
  tf_continuity_controller_ =
      std::make_unique<localization_health::TfContinuityController>(tf_config);

  InitLidarModel();

  localization_ptr_ = std::make_shared<Localization>(this, config_);
  if (!localization_ptr_->PrepareMap()) {
    throw std::runtime_error("Failed to prepare localization map.");
  }

  InitPublisher();
  InitSubscriber();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(
      config_->localization.imu.data_searcher_buffer_size);

  localization_thread_ptr_ = std::make_shared<std::thread>([this] {
    localization_thread_runtime_->ApplyToCurrentThread();
    localization_ptr_->Run();
  });

  pre_processing_ptr_ = std::make_shared<PreProcessing>(this, config_);
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

std::uint64_t System::FrameProcessingTimeoutUs() const {
  return runtime_frame_processing_timeout_us_;
}

void System::InitLidarModel() {
  const LocalizationLidarConfig& lidar = config_->localization.lidar;
  if (lidar.sensor_type == "None" || lidar.sensor_type == "Hesai") {
    LidarModel::Instance(lidar.sensor_type);
    const int lidar_horizon_scan = lidar.horizon_scan;
    LidarModel::Instance()->horizon_scan_num_ = lidar_horizon_scan;
    LidarModel::Instance()->vertical_scan_num_ = lidar.scan_lines;
    LidarModel::Instance()->h_res_ =
        Degree2Radian(360.0f / static_cast<float>(lidar_horizon_scan));
    LidarModel::Instance()->v_res_ =
        static_cast<float>(Degree2Radian(lidar.vertical_resolution_deg));
    LidarModel::Instance()->lower_angle_ =
        static_cast<float>(Degree2Radian(lidar.lower_angle_deg));
  } else {
    LidarModel::Instance(lidar.sensor_type);
  }
}

void System::InitPublisher() {
  const OutputTopics& outputs = config_->outputs;
  localization_path_pub_ =
      create_publisher<nav_msgs::msg::Path>(outputs.path, 5);
  localization_global_cloud_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          outputs.global_map,
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  localization_local_cloud_map_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
          outputs.local_map,
          rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  localization_current_lidar_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          outputs.current_lidar, 5);

  localization_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      outputs.odometry, 5);
  diagnostics_pub_ =
      create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          outputs.diagnostics, 10);
}

void System::InitSubscriber() {
  lidar_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lidar_sub_opt = rclcpp::SubscriptionOptions();
  lidar_sub_opt.callback_group = lidar_cb_group_;
  lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      config_->sensors.lidar,
      rclcpp::QoS(10).best_effort().keep_last(1),
      std::bind(&System::LidarMsgCallBack, this, std::placeholders::_1),
      lidar_sub_opt);

  imu_cb_group_ =
      create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto imu_sub_opt = rclcpp::SubscriptionOptions();
  imu_sub_opt.callback_group = imu_cb_group_;
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      config_->sensors.imu,
      rclcpp::QoS(400).best_effort().keep_last(1),
      std::bind(&System::ImuMsgCallBack, this, std::placeholders::_1),
      imu_sub_opt);

  init_pose_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto init_sub_opt = rclcpp::SubscriptionOptions();
  init_sub_opt.callback_group = init_pose_cb_group_;
  localization_init_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          config_->command_topics.initial_pose,
          rclcpp::QoS(1).reliable().keep_last(1),
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
  const std::uint64_t tf_continuity_epoch =
      tf_continuity_input_epoch_.load();
  raw_cloud_deque_.emplace_back(
      RawCloudWorkItem{cloud_ros_ptr, tf_continuity_epoch});
  cv_preprocessing_.notify_one();
}

void System::LocalizationInitPoseMsgCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  const std::string& map_frame = config_->frames.map;
  const std::string& base_frame = config_->frames.base_link;
  const std::string& lidar_frame = config_->frames.lidar;

  // An initial pose is unambiguously map_T_base. Invalid poses must not
  // partially update localization state, so parse before touching the TF tree.
  std::string parse_error;
  const std::optional<MapTBase> map_T_base =
      RosPoseAdapter::ParseMapTBase(*msg, map_frame, &parse_error);
  if (!map_T_base.has_value()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignore initial pose: %s", parse_error.c_str());
    return;
  }

  try {
    // lookupTransform(base, lidar) yields base_T_lidar, which is the inverse
    // of the lidar_T_base used by the output path.
    const auto tf_msg =
        tf_buffer_->lookupTransform(base_frame, lidar_frame, rclcpp::Time(0));
    const BaseTLidar base_T_lidar(tf2::transformToEigen(tf_msg));
    localization_ptr_->SetInitPose(map_T_base.value() * base_T_lidar);
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(get_logger(),
                "Ignore initial pose because TF lookup failed (%s -> %s): %s",
                base_frame.c_str(), lidar_frame.c_str(), ex.what());
  }
}

void System::ImuMsgCallBack(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr) {
  static Vec3d init_mean_acc = Vec3d::Zero();

  IMUData imu_data;
  imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);
  data_stream_watchdog_->RecordImu(imu_data.timestamp_,
                                   SteadyNowMicroseconds());
  imu_data.angular_velocity_ = RosVec3dToEigen(imu_ptr->angular_velocity);
  imu_data.linear_acceleration_ = RosVec3dToEigen(imu_ptr->linear_acceleration);

  const LocalizationImuConfig& imu_config = config_->localization.imu;
  if (config_->localization.frontend.fusion_method == kFusionLooseCoupling) {
    if (!has_imu_init_.load()) {
      if (imu_config.has_orientation) {
        imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
      } else {
        UpdateImuOrientationFromAngularVelocity(imu_data);
      }
      has_imu_init_.store(true);
      imu_data_searcher_ptr_->CacheData(imu_data);
      return;
    }
  } else {
    if (!has_imu_init_.load()) {
      has_imu_init_.store(InitIMU(imu_data, init_mean_acc));

      if (has_imu_init_.load()) {
        if (imu_config.has_orientation) {
          imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
        } else {
          UpdateImuOrientationFromAngularVelocity(imu_data);
        }

        imu_data.linear_acceleration_ =
            imu_data.linear_acceleration_ * imu_config.gravity /
            init_mean_acc.norm();

        imu_data_searcher_ptr_->CacheData(imu_data);
      }
      return;
    }

    imu_data.linear_acceleration_ = imu_data.linear_acceleration_ *
                                    imu_config.gravity / init_mean_acc.norm();
  }

  if (imu_config.has_orientation) {
    imu_data.orientation_ = RosQuaternionToEigen(imu_ptr->orientation);
  } else {
    UpdateImuOrientationFromAngularVelocity(imu_data);
  }

  imu_data_searcher_ptr_->CacheData(imu_data);
}

void System::ResetImuOrientationState() {
  std::lock_guard<std::mutex> lock(mutex_imu_orientation_state_);
  last_imu_angular_velocity_ = Vec3d::Zero();
  last_imu_orientation_ = Eigen::Quaterniond::Identity();
  last_imu_timestamp_.reset();
}

void System::UpdateImuOrientationFromAngularVelocity(IMUData& imu_data) {
  std::lock_guard<std::mutex> lock(mutex_imu_orientation_state_);
  if (last_imu_timestamp_.has_value() &&
      imu_data.timestamp_ >= last_imu_timestamp_.value()) {
    const Eigen::Quaterniond delta_q(
        SO3Exp((last_imu_angular_velocity_ + imu_data.angular_velocity_) * 0.5 *
               (imu_data.timestamp_ - last_imu_timestamp_.value()) *
               kMicroseconds2Seconds));
    imu_data.orientation_ = last_imu_orientation_ * delta_q;
  } else {
    imu_data.orientation_ = Eigen::Quaterniond::Identity();
  }
  last_imu_timestamp_ = imu_data.timestamp_;
  last_imu_orientation_ = imu_data.orientation_;
  last_imu_angular_velocity_ = imu_data.angular_velocity_;
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
    gravity_vector_ =
        -mean_acc / mean_acc.norm() * config_->localization.imu.gravity;
    return true;
  }

  return false;
}

bool System::UpdateLidarTBase() {
  {
    std::lock_guard<std::mutex> lg(mutex_lidar_t_base_);
    if (has_lidar_t_base_) {
      return true;
    }
  }

  const std::string& base_frame = config_->frames.base_link;
  const std::string& lidar_frame = config_->frames.lidar;

  try {
    auto tf_msg = tf_buffer_->lookupTransform(
        lidar_frame, base_frame, rclcpp::Time(0));
    const Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_msg);
    std::lock_guard<std::mutex> lg(mutex_lidar_t_base_);
    if (!has_lidar_t_base_) {
      lidar_T_base_ = LidarTBase(tf_eigen);
      has_lidar_t_base_ = true;
    }
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s", lidar_frame.c_str(),
                         base_frame.c_str(), ex.what());
  }

  std::lock_guard<std::mutex> lg(mutex_lidar_t_base_);
  return has_lidar_t_base_;
}

std::optional<LidarTBase> System::GetLidarTBase() {
  if (!UpdateLidarTBase()) {
    return std::nullopt;
  }

  std::lock_guard<std::mutex> lg(mutex_lidar_t_base_);
  return lidar_T_base_;
}

void System::ResetTfContinuityOnTimeJump(TimeStampUs now_us) {
  if (last_ros_time_us_.has_value() &&
      now_us < last_ros_time_us_.value()) {
    tf_continuity_expected_epoch_ =
        tf_continuity_input_epoch_.fetch_add(1U) + 1U;
    if (tf_continuity_controller_) {
      tf_continuity_controller_->Reset();
      last_tf_extrapolation_duration_us_ = 0U;
    }
    if (imu_data_searcher_ptr_) {
      imu_data_searcher_ptr_->Clear();
    }
    ResetImuOrientationState();
    {
      std::lock_guard<std::mutex> lock(mutex_raw_cloud_deque_);
      raw_cloud_deque_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_cloud_cluster_deque_);
      cloud_cluster_deque_.clear();
    }
    {
      std::lock_guard<std::mutex> lock(mutex_localization_results_deque_);
      localization_results_deque_.clear();
    }
    cv_preprocessing_.notify_all();
    cv_localization_.notify_all();
    tf_continuity_time_jump_recovery_active_ = true;
    RCLCPP_WARN(get_logger(),
                "ROS time moved backward; resetting localization TF continuity.");
  }
  last_ros_time_us_ = now_us;
}

void System::Run() {
  // publish localization path and odometry
  ResetTfContinuityOnTimeJump(RosTimeToMicroseconds(get_clock()->now()));
  bool had_new_result = false;
  if (ProcessLocalizationResultCache(had_new_result)) {
    PublishLocalizationPath();

    auto current_lidar_cloud = localization_ptr_->GetCurrentLidarCloudMap();
    ApplyLocalizationZOffset(current_lidar_cloud);
    PublishRosCloud(localization_current_lidar_cloud_pub_,
                    current_lidar_cloud.makeShared(),
                    config_->frames.map);
  }
  if (!had_new_result) {
    PublishLocalizationHealth();
    PublishTfFallback();
  }

  // publish global cloud map
  static bool first_global_map_visualization = true;
  if (first_global_map_visualization &&
      localization_global_cloud_map_pub_->get_subscription_count() > 0) {
    const auto global_cloud_map = localization_ptr_->GetGlobalCloudMap();

    if (global_cloud_map && !global_cloud_map->empty()) {
      PublishRosCloud(localization_global_cloud_map_pub_,
                      global_cloud_map,
                      config_->frames.map);
      first_global_map_visualization = false;
    }
  }

  if (localization_local_cloud_map_pub_->get_subscription_count() > 0) {
    const auto local_cloud_map = localization_ptr_->GetLocalCloudMap();
    if (local_cloud_map && !local_cloud_map->empty() &&
        local_cloud_map.get() != last_published_local_map_.get()) {
      PublishRosCloud(localization_local_cloud_map_pub_, local_cloud_map,
                      config_->frames.map);
      last_published_local_map_ = local_cloud_map;
    }
  }
}

void System::PublishTfFallback() {
  if (!tf_continuity_controller_) {
    return;
  }

  std::optional<localization_health::TfPoseSample> extrapolated;
  std::optional<TimeStampUs> sensor_now_us;
  const auto anchor_sensor_timestamp_us =
      tf_continuity_controller_->anchor_sensor_timestamp_us();
  const bool imu_is_fresh = data_stream_watchdog_ &&
      !data_stream_watchdog_->IsImuStalled(SteadyNowMicroseconds());
  if (imu_is_fresh &&
      config_->localization.tf.enable_imu_extrapolation &&
      anchor_sensor_timestamp_us.has_value() && imu_data_searcher_ptr_ &&
      localization_ptr_) {
    const auto latest_imu = imu_data_searcher_ptr_->LatestData();
    if (latest_imu.has_value()) {
      sensor_now_us = latest_imu->timestamp_;
    }
    if (latest_imu.has_value() &&
        latest_imu->timestamp_ > anchor_sensor_timestamp_us.value()) {
      const auto imu_segment = imu_data_searcher_ptr_->GetDataSegment(
          anchor_sensor_timestamp_us.value(), latest_imu->timestamp_);
      const auto map_lidar_prediction =
          localization_ptr_->PredictMapLidarPose(imu_segment);
      const auto lidar_T_base = GetLidarTBase();
      if (map_lidar_prediction.has_value() && lidar_T_base.has_value()) {
        Eigen::Isometry3d map_T_base = Eigen::Isometry3d(
            (map_lidar_prediction->map_T_lidar * lidar_T_base.value())
                .Matrix());
        if (config_->localization.localization.enable_ground_height_constraint &&
            has_localization_z_offset_) {
          map_T_base.translation().z() += last_localization_z_offset_;
        }
        extrapolated = localization_health::TfPoseSample{
            MapTBase(map_T_base), map_lidar_prediction->timestamp_};
      }
    }
  }

  const TimeStampUs publication_now_us =
      RosTimeToMicroseconds(get_clock()->now());
  const auto decision = tf_continuity_controller_->Select(
      publication_now_us, sensor_now_us, extrapolated);
  if (!decision.has_value()) {
    return;
  }

  last_tf_extrapolation_duration_us_ = decision->extrapolation_duration_us;
  PublishTF(decision->sample.map_T_base,
            decision->publication_timestamp_us);
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
  if (tf_continuity_time_jump_recovery_active_) {
    const auto first_mismatched_epoch_result = std::remove_if(
        localization_results.begin(), localization_results.end(),
        [this](const LocalizationResult& result) {
          return result.tf_continuity_epoch_ != tf_continuity_expected_epoch_;
        });
    if (first_mismatched_epoch_result != localization_results.end()) {
      RCLCPP_WARN(get_logger(),
                  "Discarding mismatched-epoch localization results during TF recovery.");
      localization_results.erase(first_mismatched_epoch_result,
                                 localization_results.end());
    }
  }

  if (localization_results.empty()) {
    return false;
  }
  had_new_result = true;

  const LocalizationResult& result = localization_results.back();

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

  const rclcpp::Time stamp(result.timestamp_ * 1000ULL);

  const auto lidar_T_base = GetLidarTBase();
  if (!lidar_T_base.has_value()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip localization result because %s -> %s TF is unavailable.",
        config_->frames.lidar.c_str(),
        config_->frames.base_link.c_str());
    return false;
  }

  // map_T_base = map_T_lidar * lidar_T_base
  Eigen::Isometry3d map_T_base =
      Eigen::Isometry3d((result.map_T_lidar * lidar_T_base.value()).Matrix());
  if (config_->localization.localization.enable_ground_height_constraint &&
      result.has_map_ground_z) {
    const double desired_base_z =
        result.map_ground_z +
        config_->localization.localization.base_link_ground_height;
    last_localization_z_offset_ = desired_base_z - map_T_base.translation().z();
    has_localization_z_offset_ = true;
    map_T_base.translation().z() = desired_base_z;

    const double base_ground_height =
        map_T_base.translation().z() - result.map_ground_z;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Current base_link ground height: %.3f m (base_link_z=%.3f, "
        "map_ground_z=%.3f)",
        base_ground_height, map_T_base.translation().z(), result.map_ground_z);
  } else {
    last_localization_z_offset_ = 0.0;
    has_localization_z_offset_ = false;
  }
  const MapTBase sampled_map_T_base(map_T_base);
  const TimeStampUs publication_now_us =
      RosTimeToMicroseconds(get_clock()->now());
  const auto measured_decision = tf_continuity_controller_->RecordMeasured(
      localization_health::TfPoseSample{sampled_map_T_base, result.timestamp_},
      publication_now_us);
  last_tf_extrapolation_duration_us_ =
      measured_decision.extrapolation_duration_us;
  PublishTF(measured_decision.sample.map_T_base,
            measured_decision.publication_timestamp_us);
  tf_continuity_time_jump_recovery_active_ = false;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose =
      RosPoseAdapter::ToPose(measured_decision.sample.map_T_base);
  pose_stamped.header.frame_id = config_->frames.map;
  pose_stamped.header.stamp = stamp;
  localization_path_.poses.emplace_back(std::move(pose_stamped));
  if (localization_path_.poses.size() > kMaxLocalizationPathSize) {
    localization_path_.poses.erase(localization_path_.poses.begin());
  }

  nav_msgs::msg::Odometry localization_odometry = RosPoseAdapter::ToOdometry(
      measured_decision.sample.map_T_base, config_->frames.map,
      config_->frames.base_link, stamp);
  if (config_->localization.health.enable) {
    localization_odometry.pose.covariance.fill(0.0);
    const double covariance = config_->localization.health.base_covariance *
                              current_health_covariance_scale_;
    localization_odometry.pose.covariance[0] = covariance;
    localization_odometry.pose.covariance[7] = covariance;
    localization_odometry.pose.covariance[14] = covariance;
    localization_odometry.pose.covariance[21] = covariance;
    localization_odometry.pose.covariance[28] = covariance;
    localization_odometry.pose.covariance[35] = covariance;
  }
  localization_odom_pub_->publish(localization_odometry);

  return true;
}

void System::PublishLocalizationHealth() {

  if (!diagnostics_pub_) {
    return;
  }
  if (!config_->localization.health.enable) {
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
      inputs, config_->localization.localization.auto_relocalization,
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
                inputs.reg_fitness, config_->localization.health.lost_fitness_thres)));
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

  localization_health::TfPublicationMode tf_mode =
      localization_health::TfPublicationMode::kNoAnchor;
  localization_health::TfHoldReason tf_hold_reason =
      localization_health::TfHoldReason::kNoAnchor;
  TimeStampUs tf_anchor_age_us = 0U;
  TimeStampUs tf_extrapolation_age_us = 0U;
  if (tf_continuity_controller_) {
    tf_mode = tf_continuity_controller_->mode();
    tf_hold_reason = tf_continuity_controller_->hold_reason();
    const auto anchor_sensor_timestamp_us =
        tf_continuity_controller_->anchor_sensor_timestamp_us();
    const auto latest_imu = imu_data_searcher_ptr_
        ? imu_data_searcher_ptr_->LatestData()
        : std::nullopt;
    if (anchor_sensor_timestamp_us.has_value() && latest_imu.has_value() &&
        latest_imu->timestamp_ >= anchor_sensor_timestamp_us.value()) {
      tf_anchor_age_us =
          latest_imu->timestamp_ - anchor_sensor_timestamp_us.value();
    }
    if (tf_mode == localization_health::TfPublicationMode::kExtrapolated) {
      tf_extrapolation_age_us = last_tf_extrapolation_duration_us_;
    }
  }
  add_value("tf_mode", TfModeString(tf_mode));
  add_value("tf_anchor_age_ms", std::to_string(
      static_cast<double>(tf_anchor_age_us) /
      static_cast<double>(kMicrosecondsPerMillisecond)));
  add_value("tf_extrapolation_age_ms", std::to_string(
      static_cast<double>(tf_extrapolation_age_us) /
      static_cast<double>(kMicrosecondsPerMillisecond)));
  add_value("tf_hold_reason", TfHoldReasonString(tf_hold_reason));

  AppendRuntimeDiagnostics(status);
  AppendMapDiagnostics(status);
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
  AppendMapDiagnostics(status);
  diagnostics.status.emplace_back(std::move(status));
  diagnostics_pub_->publish(diagnostics);
}

void System::AppendMapDiagnostics(
    diagnostic_msgs::msg::DiagnosticStatus& status) const {
  const localization_map::MapDiagnostics map =
      localization_ptr_ ? localization_ptr_->MapDiagnostics()
                        : localization_map::MapDiagnostics{};
  const auto add_value = [&status](const std::string& key,
                                   const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.emplace_back(std::move(item));
  };
  add_value("map_mode",
            map.mode == localization_map::MapMode::kTiled ? "tiled"
                                                          : "global");
  add_value("map_manifest_version", std::to_string(map.manifest_version));
  add_value("map_active_tile_count", std::to_string(map.active_tiles));
  add_value("map_cached_tile_count", std::to_string(map.cached_tiles));
  add_value("map_cache_mib", std::to_string(map.cache_mib));
  add_value("map_cache_hits", std::to_string(map.cache_hits));
  add_value("map_cache_misses", std::to_string(map.cache_misses));
  add_value("map_tile_load_ms", std::to_string(map.last_load_ms));
  add_value("map_tile_load_failures", std::to_string(map.load_failures));
  add_value("map_tile_load_state", map.load_state);
  add_value("map_tile_load_error", map.last_error);

  const bool unavailable = map.request_unavailable_without_coverage;
  const bool degraded = map.snapshot_degraded || map.nonfatal_fault_active;
  const auto previous_level = status.level;
  if (unavailable) {
    status.level = std::max(
        status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
    if (previous_level < diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      status.message = "localization map unavailable";
    }
  } else if (degraded) {
    status.level = std::max(
        status.level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
    if (previous_level < diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      status.message = "localization map degraded";
    }
  }
}

void System::AppendRuntimeDiagnostics(
    diagnostic_msgs::msg::DiagnosticStatus& status) const {

  const runtime::ThreadRuntimeStatus preprocessing_runtime_status =
      preprocessing_thread_runtime_
          ? preprocessing_thread_runtime_->Snapshot()
          : runtime::ThreadRuntimeStatus{};
  const runtime::ThreadRuntimeStatus localization_runtime_status =
      localization_thread_runtime_
          ? localization_thread_runtime_->Snapshot()
          : runtime::ThreadRuntimeStatus{};
  const runtime::FrameProcessingSnapshot preprocessing_frame_snapshot =
      pre_processing_ptr_
          ? pre_processing_ptr_->FrameProcessingSnapshot()
          : runtime::FrameProcessingSnapshot{};
  const runtime::FrameProcessingSnapshot localization_frame_snapshot =
      localization_ptr_
          ? localization_ptr_->FrameProcessingSnapshot()
          : runtime::FrameProcessingSnapshot{};

  const auto add_value = [&status](const std::string& key,
                                   const std::string& value) {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = value;
    status.values.emplace_back(std::move(item));
  };
  add_value("preprocessing_fifo_priority",
            std::to_string(config_->localization.runtime.preprocessing_fifo_priority));
  add_value("preprocessing_cpu_core",
            std::to_string(config_->localization.runtime.preprocessing_cpu_core));
  add_value("preprocessing_fifo_applied",
            BoolString(preprocessing_runtime_status.scheduling_applied));
  add_value("preprocessing_fifo_errno",
            std::to_string(preprocessing_runtime_status.scheduling_errno));
  add_value("preprocessing_affinity_applied",
            BoolString(preprocessing_runtime_status.affinity_applied));
  add_value("preprocessing_affinity_errno",
            std::to_string(preprocessing_runtime_status.affinity_errno));
  add_value("localization_fifo_priority",
            std::to_string(config_->localization.runtime.localization_fifo_priority));
  add_value("localization_cpu_core",
            std::to_string(config_->localization.runtime.localization_cpu_core));
  add_value("localization_fifo_applied",
            BoolString(localization_runtime_status.scheduling_applied));
  add_value("localization_fifo_errno",
            std::to_string(localization_runtime_status.scheduling_errno));
  add_value("localization_affinity_applied",
            BoolString(localization_runtime_status.affinity_applied));
  add_value("localization_affinity_errno",
            std::to_string(localization_runtime_status.affinity_errno));
  add_value("preprocessing_last_frame_duration_ms",
            std::to_string(
                static_cast<double>(
                    preprocessing_frame_snapshot.last_duration_us) /
                static_cast<double>(kMicrosecondsPerMillisecond)));
  add_value("preprocessing_last_frame_timed_out",
            BoolString(preprocessing_frame_snapshot.last_frame_timed_out));
  add_value("preprocessing_timeout_count",
            std::to_string(preprocessing_frame_snapshot.timeout_count));
  add_value("preprocessing_dropped_count",
            std::to_string(preprocessing_frame_snapshot.dropped_count));
  add_value("localization_last_frame_duration_ms",
            std::to_string(
                static_cast<double>(
                    localization_frame_snapshot.last_duration_us) /
                static_cast<double>(kMicrosecondsPerMillisecond)));
  add_value("localization_last_frame_timed_out",
            BoolString(localization_frame_snapshot.last_frame_timed_out));
  add_value("localization_timeout_count",
            std::to_string(localization_frame_snapshot.timeout_count));
  add_value("localization_dropped_count",
            std::to_string(localization_frame_snapshot.dropped_count));
}

void System::ApplyLocalizationZOffset(PCLPointCloudXYZI& cloud) const {
  if (!has_localization_z_offset_ ||
      !config_->localization.localization.enable_ground_height_constraint) {
    return;
  }

  for (auto& point : cloud.points) {
    point.z += static_cast<float>(last_localization_z_offset_);
  }
}

void System::PublishLocalizationPath() {
  if (localization_path_pub_->get_subscription_count() > 0) {
    localization_path_.header.frame_id = config_->frames.map;
    localization_path_.header.stamp = this->get_clock()->now();
    localization_path_pub_->publish(localization_path_);
  }
}

void System::PublishTF(const MapTBase& map_T_base, TimeStampUs timestamp) {
  const rclcpp::Time stamp(timestamp * 1000ULL);
  tf_broadcaster_->sendTransform(RosPoseAdapter::ToTransformStamped(
      map_T_base, config_->frames.map, config_->frames.base_link, stamp));
}
