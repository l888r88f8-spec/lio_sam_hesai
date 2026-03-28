cmake_minimum_required(VERSION 3.5)
project(lio_sam_hesai)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-DPROJECT_SOURCE_DIR=\"${PROJECT_SOURCE_DIR}\")

include(cmake/glog.cmake)
include(cmake/g2o.cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
set(OpenCV_STATIC ON)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
  include/lio_sam
  ${GLOG_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMap.srv" DEPENDENCIES std_msgs sensor_msgs)



add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM Eigen)
target_link_libraries(${PROJECT_NAME}_imuPreintegration gtsam "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptimization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL GTSAM)

if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}" OpenMP::OpenMP_CXX)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization gtsam "${cpp_typesupport_target}")
endif()

add_library(${PROJECT_NAME}_relocalization_lib
  src/imu/data_synchronizer.cpp
  src/imu/pre_integration.cpp
  src/lidar/lidar_model.cpp
  src/lidar/lidar_distortion_corrector.cpp
  src/loam/feature_extractor.cpp
  src/loam/pointcloud_projector.cpp
  src/ivox_map/ivox_map.cpp
  src/ivox_map/voxel_grid_node.cpp
  src/optimization/g2o/accel_bias_prior_edge.cpp
  src/optimization/g2o/accel_bias_rw_edge.cpp
  src/optimization/g2o/gyro_bias_prior_edge.cpp
  src/optimization/g2o/gyro_bias_rw_edge.cpp
  src/optimization/g2o/position_edge.cpp
  src/optimization/g2o/pre_integration_edge.cpp
  src/optimization/g2o/prior_nav_state_edge.cpp
  src/optimization/g2o/relative_pose_edge.cpp
  src/optimization/g2o/rotation_edge.cpp
  src/optimization/g2o/velocity_edge.cpp
  src/slam/config_parameters.cpp
  src/slam/preprocessing.cpp
  src/slam/localization.cpp
  src/slam/split_map.cpp
  src/slam/system.cpp
)
ament_target_dependencies(${PROJECT_NAME}_relocalization_lib
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions
  pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs
  tf2_geometry_msgs OpenCV PCL
)
target_link_libraries(${PROJECT_NAME}_relocalization_lib
  ${GLOG_LIBRARIES}
  ${G2O_LIBRARIES}
)

add_executable(${PROJECT_NAME}_relocalization src/relocalization_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_relocalization
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions
  pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs
  tf2_geometry_msgs OpenCV PCL
)
target_link_libraries(${PROJECT_NAME}_relocalization
  ${PROJECT_NAME}_relocalization_lib
  ${GLOG_LIBRARIES}
  ${G2O_LIBRARIES}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ${PROJECT_NAME}_imageProjection
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_imuPreintegration
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_featureExtraction
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_mapOptimization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_relocalization
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
