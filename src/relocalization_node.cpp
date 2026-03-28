//
// Relocalization node for lio_sam_hesai.
//
#include <glog/logging.h>

#include <rclcpp/rclcpp.hpp>

#include "slam/system.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  rclcpp::init(argc, argv);
  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<System>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  google::ShutdownGoogleLogging();

  return 0;
}
