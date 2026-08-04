
#ifndef FUNNY_LIDAR_SLAM_SYSTEM_H
#define FUNNY_LIDAR_SLAM_SYSTEM_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <thread>

#include "common/sensor_data_type.h"
#include "imu/imu_data_searcher.h"
#include "lidar/pointcloud_cluster.h"
#include "localization_health/data_stream_watchdog.h"
#include "localization_health/localization_recovery_coordinator.h"
#include "localization_health/tf_continuity_controller.h"
#include "runtime/thread_runtime_controller.h"

class PreProcessing;
class Localization;

class System : public rclcpp::Node {
 public:
  explicit System(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  System() = delete;

  ~System();

  void Run();

  // lookupTransform(lidar, base) returns the transform from base to lidar.
  std::optional<Mat4d> GetBaseToLidarTransform();

  [[nodiscard]] std::uint64_t FrameProcessingTimeoutUs() const;

  static constexpr std::size_t kMaxRawCloudQueueSize = 3;
  static constexpr std::size_t kMaxCloudClusterQueueSize = 2;
  static constexpr std::size_t kMaxLocalizationResultQueueSize = 5;
  static constexpr std::size_t kMaxLocalizationPathSize = 2000;

  struct RawCloudWorkItem {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_ros_ptr;
    std::uint64_t tf_continuity_epoch_{0U};
  };

  struct CloudClusterWorkItem {
    PointcloudClusterPtr cloud_cluster_ptr;
    std::uint64_t tf_continuity_epoch_{0U};
  };

  [[nodiscard]] std::uint64_t TfContinuityInputEpoch() const {
    return tf_continuity_input_epoch_.load();
  }

 private:
  static void InitLidarModel();

  void InitSubscriber();
  void InitPublisher();
  void InitConfigParameters();

  void PublishLocalizationPath();
  void PublishLocalizationHealth();
  void PublishRuntimeDiagnostics();
  void AppendRuntimeDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus& status) const;
  void ResetTfContinuityOnTimeJump(TimeStampUs now_us);
  void PublishTfFallback();
  void PublishTF(const Mat4d& map_base_pose, TimeStampUs timestamp);

  void LidarMsgCallBack(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);
  void LocalizationInitPoseMsgCallBack(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void ImuMsgCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr);

  bool ProcessLocalizationResultCache(bool& had_new_result);
  void ApplyLocalizationZOffset(PCLPointCloudXYZI& cloud) const;

  static bool InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc);
  void ResetImuOrientationState();
  void UpdateImuOrientationFromAngularVelocity(IMUData& imu_data);

  bool UpdateBaseToLidarTransform();

 private:
  // ros2 subscribers
  rclcpp::CallbackGroup::SharedPtr imu_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::CallbackGroup::SharedPtr lidar_cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::CallbackGroup::SharedPtr init_pose_cb_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      localization_init_pose_sub_;

  // ros2 publishers
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr localization_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      localization_global_cloud_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      localization_current_lidar_cloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr system_timer_;

  // pre-processing
  std::shared_ptr<PreProcessing> pre_processing_ptr_ = nullptr;
  std::unique_ptr<runtime::ThreadRuntimeController>
      preprocessing_thread_runtime_;

  // localization
  std::shared_ptr<Localization> localization_ptr_ = nullptr;
  std::unique_ptr<runtime::ThreadRuntimeController>
      localization_thread_runtime_;
  std::uint64_t runtime_frame_processing_timeout_us_{0U};
  std::unique_ptr<localization_health::DataStreamWatchdog>
      data_stream_watchdog_;
  std::unique_ptr<localization_health::LocalizationRecoveryCoordinator>
      localization_recovery_coordinator_;
  std::unique_ptr<localization_health::TfContinuityController>
      tf_continuity_controller_;

  std::shared_ptr<std::thread> pre_processing_thread_ptr_ = nullptr;
  std::shared_ptr<std::thread> localization_thread_ptr_ = nullptr;

 public:
  // data searcher use to search imu data
  std::shared_ptr<IMUDataSearcher> imu_data_searcher_ptr_ = nullptr;

  // condition variables
  std::condition_variable cv_localization_;
  std::condition_variable cv_preprocessing_;

  std::atomic_bool has_imu_init_{false};

  // mutex
  std::mutex mutex_raw_cloud_deque_;
  std::mutex mutex_cloud_cluster_deque_;
  std::mutex mutex_localization_results_deque_;
  std::mutex mutex_health_result_;
  std::mutex mutex_base_to_lidar_transform_;
  std::mutex mutex_imu_orientation_state_;

  // deque
  std::deque<LocalizationResult, Eigen::aligned_allocator<LocalizationResult>>
      localization_results_deque_;
  std::deque<CloudClusterWorkItem> cloud_cluster_deque_;
  std::deque<RawCloudWorkItem> raw_cloud_deque_;

  // localization ros path
  nav_msgs::msg::Path localization_path_;

  Mat4d T_base_to_lidar_ = Mat4d::Identity();
  bool has_base_to_lidar_ = false;
  Vec3d last_imu_angular_velocity_ = Vec3d::Zero();
  Eigen::Quaterniond last_imu_orientation_ = Eigen::Quaterniond::Identity();
  std::optional<TimeStampUs> last_imu_timestamp_;
  double last_localization_z_offset_ = 0.0;
  bool has_localization_z_offset_ = false;
  TimeStampUs last_tf_extrapolation_duration_us_ = 0U;
  std::optional<TimeStampUs> last_ros_time_us_;
  bool tf_continuity_time_jump_recovery_active_{false};
  std::atomic<std::uint64_t> tf_continuity_input_epoch_{0U};
  std::uint64_t tf_continuity_expected_epoch_{0U};

  TimeStampUs last_health_result_stamp_ = 0U;
  TimeStampUs last_health_result_arrival_steady_us_ = 0U;
  float last_health_result_fitness_ = std::numeric_limits<float>::infinity();
  int last_health_consistency_hit_ = 0;
  int last_health_consistency_required_ = 0;
  bool last_health_result_is_tracking_ = false;
  bool has_health_result_ = false;
  bool has_new_health_result_ = false;
  double current_health_covariance_scale_ = 1.0;
  std::string last_auto_relocalization_trigger_{"none"};
};

#endif  // FUNNY_LIDAR_SLAM_SYSTEM_H
