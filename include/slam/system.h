
#ifndef FUNNY_LIDAR_SLAM_SYSTEM_H
#define FUNNY_LIDAR_SLAM_SYSTEM_H

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>

#include "common/sensor_data_type.h"
#include "imu/imu_data_searcher.h"
#include "lidar/pointcloud_cluster.h"

class PreProcessing;
class Localization;

class System : public rclcpp::Node {
 public:
  explicit System(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  System() = delete;

  ~System();

  void Run();
  bool LookupExternalOdomImuPose(Mat4d& odom_pose);

 private:
  static void InitLidarModel();

  void InitSubscriber();
  void InitPublisher();
  void InitConfigParameters();

  void PublishLocalizationPath();
  void PublishTF(const Mat4d& map_pose, TimeStampUs timestamp);

  void LidarMsgCallBack(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr& cloud_ros_ptr);
  void LocalizationInitPoseMsgCallBack(
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void ImuMsgCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_ptr);

  bool ProcessLocalizationResultCache();

  static bool InitIMU(const IMUData& imu_data, Vec3d& init_mean_acc);

  bool UpdateImuToBaseTransform();
  bool LookupExternalOdomBasePose(Mat4d& odom_base_pose);

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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr localization_imu_odom_pub_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::TimerBase::SharedPtr system_timer_;

  // pre-processing
  std::shared_ptr<PreProcessing> pre_processing_ptr_ = nullptr;

  // localization
  std::shared_ptr<Localization> localization_ptr_ = nullptr;

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

  // deque
  std::deque<LocalizationResult, Eigen::aligned_allocator<LocalizationResult>>
      localization_results_deque_;
  std::deque<PointcloudClusterPtr> cloud_cluster_deque_;
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> raw_cloud_deque_;

  // localization ros path
  nav_msgs::msg::Path localization_path_;

  Mat4d T_base_from_imu_ = Mat4d::Identity();
  bool has_base_from_imu_ = false;
};

#endif  // FUNNY_LIDAR_SLAM_SYSTEM_H
