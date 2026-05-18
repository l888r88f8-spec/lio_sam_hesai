#include "slam/system.h"

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
  util::param(this, "frames.base_link", config.base_link_frame_,
              config.base_link_frame_);
  util::param(this, "frames.imu", config.imu_frame_, config.imu_frame_);
  util::param(this, "frames.lidar", config.lidar_frame_, config.lidar_frame_);

  // output topics
  util::param(this, "output.mapping_odom_topic", config.mapping_odom_topic_,
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

bool System::UpdateLidarToBaseTransform() {
  if (has_lidar_to_base_) {
    return true;
  }

  const auto& base_frame = ConfigParameters::Instance().base_link_frame_;
  const auto& lidar_frame = ConfigParameters::Instance().lidar_frame_;

  try {
    auto tf_msg = tf_buffer_->lookupTransform(
        lidar_frame, base_frame, rclcpp::Time(0));
    Eigen::Isometry3d tf_eigen = tf2::transformToEigen(tf_msg);
    T_lidar_to_base_ = tf_eigen.matrix();
    has_lidar_to_base_ = true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "TF lookup failed (%s -> %s): %s", lidar_frame.c_str(),
                         base_frame.c_str(), ex.what());
  }

  return has_lidar_to_base_;
}

void System::Run() {
  // publish localization path and odometry
  if (ProcessLocalizationResultCache()) {
    PublishLocalizationPath();

    auto current_lidar_cloud = localization_ptr_->GetCurrentLidarCloudMap();
    ApplyLocalizationZOffset(current_lidar_cloud);
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

    result = localization_results_deque_.back();
    localization_results_deque_.clear();
  }

  const rclcpp::Time stamp(result.timestamp_ * 1000ULL);

  if (!UpdateLidarToBaseTransform()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skip localization result because %s -> %s TF is unavailable.",
        ConfigParameters::Instance().lidar_frame_.c_str(),
        ConfigParameters::Instance().base_link_frame_.c_str());
    return false;
  }

  Mat4d map_base_pose = result.map_pose * T_lidar_to_base_;
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
  localization_odom_pub_->publish(mapping_odom);

  return true;
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
