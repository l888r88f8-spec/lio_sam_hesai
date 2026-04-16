#include "slam/system.h"

#include <algorithm>
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <thread>
#include <utility>

#include "common/constant_variable.h"
#include "common/math_function.h"
#include "common/ros_utility.h"
#include "lidar/lidar_model.h"
#include "slam/config_parameters.h"
#include "slam/localization.h"
#include "slam/preprocessing.h"

namespace {

constexpr double kMapToOdomTranslationAlpha = 0.18;
constexpr double kMapToOdomZAlpha = 0.08;
constexpr double kMapToOdomYawAlpha = 0.12;
constexpr double kMapToOdomMaxTranslationCorrectionSpeed = 0.35;  // m/s
constexpr double kMapToOdomMaxZCorrectionSpeed = 0.05;            // m/s
constexpr double kMapToOdomMaxYawCorrectionSpeed = 0.30;          // rad/s
constexpr double kMapToOdomRejectTranslationJump = 2.0;           // m
constexpr double kMapToOdomRejectYawJump = 45.0 * kDegree2Radian; // rad

double NormalizeAngle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double ClampAbs(double value, double limit) {
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

Mat4d MakePlanarTransform(double x, double y, double z, double yaw) {
  Mat4d pose = Mat4d::Identity();
  pose.block<3, 3>(0, 0) =
      Eigen::AngleAxisd(yaw, Vec3d::UnitZ()).toRotationMatrix();
  pose(0, 3) = x;
  pose(1, 3) = y;
  pose(2, 3) = z;
  return pose;
}

}  // namespace

System::System(const rclcpp::NodeOptions& options)
    : Node("lio_sam_hesai_relocalization", options) {
  InitConfigParameters();
  InitLidarModel();
  InitPublisher();
  InitSubscriber();

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  imu_data_searcher_ptr_ = std::make_shared<IMUDataSearcher>(
      ConfigParameters::Instance().imu_data_searcher_buffer_size_);

  localization_ptr_ = std::make_shared<Localization>(this);
  localization_thread_ptr_ = std::make_shared<std::thread>(
      &Localization::Run, localization_ptr_.get());

  pre_processing_ptr_ = std::make_shared<PreProcessing>(this);
  pre_processing_thread_ptr_ =
      std::make_shared<std::thread>(&PreProcessing::Run, pre_processing_ptr_);

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
  util::param(this, "frames.odom", config.odom_frame_, config.odom_frame_);
  util::param(this, "frames.base_link", config.base_link_frame_,
              config.base_link_frame_);
  util::param(this, "frames.imu", config.imu_frame_, config.imu_frame_);
  util::param(this, "frames.lidar", config.lidar_frame_, config.lidar_frame_);

  // output topics
  util::param(this, "output.odom_topic", config.odom_topic_,
              config.odom_topic_);
  util::param(this, "output.mapping_odom_topic", config.mapping_odom_topic_,
              config.mapping_odom_topic_);

  // localization map path
  util::param(this, "localization.map_path", config.localization_map_path_,
              config.localization_map_path_);

  // lidar config parameters
  util::param(this, "lidar.lidar_sensor_type", config.lidar_sensor_type_,
              StringEmpty);
  util::param(this, "lidar.lidar_point_jump_span",
              config.lidar_point_jump_span_, IntNaN);
  util::param(this, "lidar.lidar_scan", config.lidar_scan_, IntNaN);
  util::param(this, "lidar.lidar_lower_angle", config.lidar_lower_angle_,
              DoubleNaN);
  util::param(this, "lidar.lidar_horizon_scan", config.lidar_horizon_scan_,
              IntNaN);
  util::param(this, "lidar.lidar_vertical_resolution",
              config.lidar_vertical_resolution_, DoubleNaN);
  util::param(this, "lidar.lidar_use_min_distance", config.lidar_use_min_dist_,
              FloatNaN);
  util::param(this, "lidar.lidar_use_max_distance", config.lidar_use_max_dist_,
              FloatNaN);
  util::param(this, "lidar.lidar_point_time_scale",
              config.lidar_point_time_scale_, DoubleNaN);
  util::param(this, "lidar.lidar_rotation_noise_std",
              config.lidar_rotation_noise_std_, DoubleNaN);
  util::param(this, "lidar.lidar_position_noise_std",
              config.lidar_position_noise_std_, DoubleNaN);

  // imu config parameters
  util::param(this, "imu.has_orientation", config.imu_has_orientation_, false);
  util::param(this, "imu.init_acc_bias", config.imu_init_acc_bias_, DoubleNaN);
  util::param(this, "imu.init_gyro_bias", config.imu_init_gyro_bias_,
              DoubleNaN);
  util::param(this, "imu.acc_noise_std", config.imu_acc_noise_std_, DoubleNaN);
  util::param(this, "imu.gyro_noise_std", config.imu_gyro_noise_std_,
              DoubleNaN);
  util::param(this, "imu.acc_rw_noise_std", config.imu_acc_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.gyro_rw_noise_std", config.imu_gyro_rw_noise_std_,
              DoubleNaN);
  util::param(this, "imu.data_searcher_buffer_size",
              config.imu_data_searcher_buffer_size_, IntNaN);

  // gravity
  util::param(this, "gravity", config.gravity_norm, DoubleNaN);

  // calibration parameters
  std::vector<double> lidar_to_imu = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  util::param_vector(this, "calibration.lidar_to_imu", lidar_to_imu,
                     lidar_to_imu);
  config.calibration_lidar_to_imu_ =
      Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::RowMajor>>(
          lidar_to_imu.data());

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
  util::param(this, "system.enable_loopclosure",
              config.system_enable_loopclosure_, false);
  util::param(this, "system.enable_visualize_global_map",
              config.system_enable_visualize_global_map_, false);
  util::param(this, "system.global_map_visualization_resolution",
              config.system_global_map_visualization_resolution_, FloatNaN);

  // split map
  util::param(this, "system.tile_map_grid_size", config.tile_map_grid_size_,
              DoubleNaN);

  // loopclosure config parameters
  util::param(this, "loopclosure.skip_near_loopclosure_threshold",
              config.lc_skip_near_loopclosure_threshold_, IntNaN);
  util::param(this, "loopclosure.skip_near_keyframe_threshold",
              config.lc_skip_near_keyframe_threshold_, IntNaN);
  util::param(this, "loopclosure.candidate_local_map_left_range",
              config.lc_candidate_local_map_left_range_, IntNaN);
  util::param(this, "loopclosure.candidate_local_map_right_range",
              config.lc_candidate_local_map_right_range_, IntNaN);
  util::param(this, "loopclosure.loopclosure_local_map_left_range",
              config.lc_loopclosure_local_map_left_range_, IntNaN);
  util::param(this, "loopclosure.near_neighbor_distance_threshold",
              config.lc_near_neighbor_distance_threshold_, DoubleNaN);
  util::param(this, "loopclosure.registration_converge_threshold",
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
  localization_imu_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      ConfigParameters::Instance().odom_topic_, 50);
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
  if (!has_imu_init_.load()) {
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_raw_cloud_deque_);
  raw_cloud_deque_.push_back(cloud_ros_ptr);
  cv_preprocessing_.notify_one();
}

void System::LocalizationInitPoseMsgCallBack(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  Eigen::Quaterniond q = RosQuaternionToEigen(msg->pose.pose.orientation);
  Vec3d t = RosPoint3dToEigen(msg->pose.pose.position);
  Mat4d pose = Mat4d::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = t;

  localization_ptr_->SetInitPose(pose);
}

void System::ImuMsgCallBack(
    const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr) {
  static Vec3d init_mean_acc = Vec3d::Zero();
  static Vec3d last_angular_velocity;
  static Eigen::Quaterniond last_orientation;
  static TimeStampUs last_timestamp;

  IMUData imu_data;
  imu_data.timestamp_ = RosTimeToUs(imu_ptr->header);
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

bool System::UpdateImuToBaseTransform() {
  if (has_base_from_imu_) {
    return true;
  }

  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;
  const auto& imu_frame = ConfigParameters::Instance().imu_frame_;

  try {
    auto tf_msg = tf_buffer_->lookupTransform(
        base_frame, imu_frame, rclcpp::Time(0));
    Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_msg);
    T_base_from_imu_ = tf_eigen.matrix();
    has_base_from_imu_ = true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s", base_frame.c_str(),
                         imu_frame.c_str(), ex.what());
  }

  return has_base_from_imu_;
}

bool System::LookupExternalOdomBasePose(Mat4d& odom_base_pose) {
  const auto& odom_frame = ConfigParameters::Instance().odom_frame_;
  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;

  try {
    auto tf_msg =
        tf_buffer_->lookupTransform(odom_frame, base_frame, rclcpp::Time(0));
    odom_base_pose = tf2::transformToEigen(tf_msg).matrix();
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s", odom_frame.c_str(),
                         base_frame.c_str(), ex.what());
  }

  return false;
}

bool System::LookupExternalOdomImuPose(Mat4d& odom_pose) {
  if (!UpdateImuToBaseTransform()) {
    return false;
  }

  Mat4d odom_base_pose = Mat4d::Identity();
  if (!LookupExternalOdomBasePose(odom_base_pose)) {
    return false;
  }

  odom_pose = odom_base_pose * T_base_from_imu_;
  return true;
}

void System::Run() {
  // publish localization path and odometry
  if (ProcessLocalizationResultCache()) {
    PublishLocalizationPath();

    const auto current_lidar_cloud =
        localization_ptr_->GetCurrentLidarCloudMap();
    PublishRosCloud(localization_current_lidar_cloud_pub_,
                    current_lidar_cloud.makeShared(),
                    ConfigParameters::Instance().map_frame_);
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

bool System::ProcessLocalizationResultCache() {
  LocalizationResult result;

  {
    std::lock_guard<std::mutex> lg(mutex_localization_results_deque_);

    if (localization_results_deque_.empty()) {
      return false;
    }

    result = localization_results_deque_.front();
    localization_results_deque_.pop_front();
  }

  const Mat4d map_pose = result.map_pose;
  const rclcpp::Time stamp(result.timestamp_ * 1000);
  PublishTF(map_pose, result.timestamp_);

  Mat4d odom_base_pose = Mat4d::Identity();
  const bool has_external_odom = LookupExternalOdomBasePose(odom_base_pose);

  Mat4d map_base_pose = map_pose;
  if (has_external_odom && has_map_to_odom_) {
    map_base_pose = last_map_to_odom_ * odom_base_pose;
  } else {
    Mat4d T_imu_to_base = Mat4d::Identity();
    if (UpdateImuToBaseTransform()) {
      T_imu_to_base = T_base_from_imu_.inverse();
    }
    map_base_pose = map_pose * T_imu_to_base;
  }

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

  if (!has_external_odom) {
    return true;
  }

  nav_msgs::msg::Odometry mapping_odom;
  mapping_odom.header.stamp = stamp;
  mapping_odom.header.frame_id = ConfigParameters::Instance().odom_frame_;
  mapping_odom.child_frame_id = "odom_mapping";
  mapping_odom.pose.pose.position.x = odom_base_pose(0, 3);
  mapping_odom.pose.pose.position.y = odom_base_pose(1, 3);
  mapping_odom.pose.pose.position.z = odom_base_pose(2, 3);
  Eigen::Quaterniond q_odom(odom_base_pose.block<3, 3>(0, 0));
  mapping_odom.pose.pose.orientation.x = q_odom.x();
  mapping_odom.pose.pose.orientation.y = q_odom.y();
  mapping_odom.pose.pose.orientation.z = q_odom.z();
  mapping_odom.pose.pose.orientation.w = q_odom.w();
  localization_odom_pub_->publish(mapping_odom);

  nav_msgs::msg::Odometry imu_odom = mapping_odom;
  imu_odom.child_frame_id = "odom_imu";
  localization_imu_odom_pub_->publish(imu_odom);

  return true;
}

void System::PublishLocalizationPath() {
  if (localization_path_pub_->get_subscription_count() > 0) {
    localization_path_.header.frame_id = ConfigParameters::Instance().map_frame_;
    localization_path_.header.stamp = this->get_clock()->now();
    localization_path_pub_->publish(localization_path_);
  }
}

void System::PublishTF(const Mat4d& map_pose, TimeStampUs timestamp) {
  Mat4d odom_pose = Mat4d::Identity();
  if (!LookupExternalOdomImuPose(odom_pose)) {
    return;
  }

  const Mat4d raw_map_to_odom = map_pose * odom_pose.inverse();
  Mat4d map_to_odom = StabilizeMapToOdom(raw_map_to_odom, timestamp);
  const auto& map_frame = ConfigParameters::Instance().map_frame_;
  const auto& odom_frame = ConfigParameters::Instance().odom_frame_;

  const double stamp = static_cast<double>(timestamp) * 1e3;
  tf_broadcaster_->sendTransform(eigen2Transform(
      map_to_odom.block<3, 3>(0, 0), map_to_odom.block<3, 1>(0, 3), map_frame,
      odom_frame, stamp));
}

Mat4d System::StabilizeMapToOdom(const Mat4d& raw_map_to_odom,
                                 TimeStampUs timestamp) {
  const Vec3d raw_translation = raw_map_to_odom.block<3, 1>(0, 3);
  const Vec3d raw_rpy = RotationMatrixToRPY(raw_map_to_odom.block<3, 3>(0, 0));
  const double raw_yaw = raw_rpy.z();

  const Mat4d planar_raw =
      MakePlanarTransform(raw_translation.x(), raw_translation.y(),
                          raw_translation.z(), raw_yaw);

  if (!has_map_to_odom_) {
    last_map_to_odom_ = planar_raw;
    has_map_to_odom_ = true;
    last_map_to_odom_timestamp_ = timestamp;
    return last_map_to_odom_;
  }

  const Vec3d prev_translation = last_map_to_odom_.block<3, 1>(0, 3);
  const Vec3d prev_rpy =
      RotationMatrixToRPY(last_map_to_odom_.block<3, 3>(0, 0));
  const double prev_yaw = prev_rpy.z();

  const Eigen::Vector2d raw_xy_delta =
      raw_translation.head<2>() - prev_translation.head<2>();
  const double raw_yaw_delta = NormalizeAngle(raw_yaw - prev_yaw);

  if (raw_xy_delta.norm() > kMapToOdomRejectTranslationJump ||
      std::abs(raw_yaw_delta) > kMapToOdomRejectYawJump) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Reject unstable map->odom update: delta_xy=%.3f m delta_yaw=%.2f deg",
        raw_xy_delta.norm(), raw_yaw_delta * kRadian2Degree);
    return last_map_to_odom_;
  }

  const double dt = std::clamp(
      static_cast<double>(timestamp - last_map_to_odom_timestamp_) * 1.0e-6,
      0.02, 0.20);

  Eigen::Vector2d target_xy =
      prev_translation.head<2>() +
      raw_xy_delta * kMapToOdomTranslationAlpha;
  const double target_z =
      prev_translation.z() +
      (raw_translation.z() - prev_translation.z()) * kMapToOdomZAlpha;
  const double target_yaw =
      NormalizeAngle(prev_yaw + raw_yaw_delta * kMapToOdomYawAlpha);

  Eigen::Vector2d filtered_xy_delta = target_xy - prev_translation.head<2>();
  const double max_xy_step = kMapToOdomMaxTranslationCorrectionSpeed * dt;
  const double filtered_xy_norm = filtered_xy_delta.norm();
  if (filtered_xy_norm > max_xy_step && filtered_xy_norm > 1.0e-6) {
    filtered_xy_delta *= max_xy_step / filtered_xy_norm;
  }

  const double filtered_z_delta = ClampAbs(
      target_z - prev_translation.z(), kMapToOdomMaxZCorrectionSpeed * dt);
  const double filtered_yaw_delta = ClampAbs(
      NormalizeAngle(target_yaw - prev_yaw),
      kMapToOdomMaxYawCorrectionSpeed * dt);

  const Eigen::Vector2d filtered_xy =
      prev_translation.head<2>() + filtered_xy_delta;
  const double filtered_z = prev_translation.z() + filtered_z_delta;
  const double filtered_yaw = NormalizeAngle(prev_yaw + filtered_yaw_delta);

  last_map_to_odom_ = MakePlanarTransform(filtered_xy.x(), filtered_xy.y(),
                                          filtered_z, filtered_yaw);
  last_map_to_odom_timestamp_ = timestamp;
  return last_map_to_odom_;
}
