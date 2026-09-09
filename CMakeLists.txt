cmake_minimum_required(VERSION 3.5)
project(lio_sam_hesai)
include(GNUInstallDirs)

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
find_package(diagnostic_msgs REQUIRED)
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
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 QUIET)
if(TARGET Eigen3::Eigen)
  set(LIO_SAM_EIGEN_TARGET Eigen3::Eigen)
else()
  find_package(Eigen REQUIRED)
  set(LIO_SAM_EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
endif()
find_package(OpenMP REQUIRED)
find_package(Threads REQUIRED)

set(GTSAM_LIBRARY_PATH_SUFFIXES
  ${CMAKE_INSTALL_LIBDIR}
  lib
  lib64)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(APPEND GTSAM_LIBRARY_PATH_SUFFIXES
    lib/${CMAKE_LIBRARY_ARCHITECTURE}
    ${CMAKE_INSTALL_LIBDIR}/${CMAKE_LIBRARY_ARCHITECTURE})
endif()
list(REMOVE_DUPLICATES GTSAM_LIBRARY_PATH_SUFFIXES)

find_path(LIO_SAM_GTSAM_INCLUDE_DIR
  NAMES gtsam/geometry/Rot3.h
  PATHS ${CMAKE_PREFIX_PATH} /usr/local /usr
  PATH_SUFFIXES include
  REQUIRED)
find_path(LIO_SAM_GTSAM_UNSTABLE_INCLUDE_DIR
  NAMES gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h
  PATHS ${CMAKE_PREFIX_PATH} /usr/local /usr
  PATH_SUFFIXES include
  REQUIRED)
find_library(LIO_SAM_GTSAM_LIBRARY
  NAMES gtsam
  PATHS ${CMAKE_PREFIX_PATH} /usr/local /usr
  PATH_SUFFIXES ${GTSAM_LIBRARY_PATH_SUFFIXES}
  REQUIRED)
find_library(LIO_SAM_GTSAM_UNSTABLE_LIBRARY
  NAMES gtsam_unstable
  PATHS ${CMAKE_PREFIX_PATH} /usr/local /usr
  PATH_SUFFIXES ${GTSAM_LIBRARY_PATH_SUFFIXES})
set(GTSAM_LIBRARIES ${LIO_SAM_GTSAM_LIBRARY})
if(LIO_SAM_GTSAM_UNSTABLE_LIBRARY)
  list(APPEND GTSAM_LIBRARIES ${LIO_SAM_GTSAM_UNSTABLE_LIBRARY})
endif()
message(STATUS "Found GTSAM include directory: ${LIO_SAM_GTSAM_INCLUDE_DIR}")
message(STATUS "Found GTSAM unstable include directory: ${LIO_SAM_GTSAM_UNSTABLE_INCLUDE_DIR}")
message(STATUS "Found GTSAM libraries: ${GTSAM_LIBRARIES}")

include_directories(
  include
  include/lio_sam
  ${LIO_SAM_GTSAM_INCLUDE_DIR}
  ${LIO_SAM_GTSAM_UNSTABLE_INCLUDE_DIR}
  ${LIO_SAM_EIGEN_INCLUDE_DIRS}
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
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${GTSAM_LIBRARIES} ${LIO_SAM_EIGEN_TARGET} "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptimization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_sources(${PROJECT_NAME}_mapOptimization PRIVATE
  src/ivox_map/ivox_map.cpp
  src/ivox_map/voxel_grid_node.cpp
  src/relocalization/scan_context.cpp
)

if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization ${GTSAM_LIBRARIES} "${cpp_typesupport_target}" OpenMP::OpenMP_CXX ${GLOG_LIBRARIES})
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization ${GTSAM_LIBRARIES} "${cpp_typesupport_target}" ${GLOG_LIBRARIES})
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
  src/relocalization/scan_context.cpp
  src/localization_health/data_stream_watchdog.cpp
  src/localization_health/localization_state_machine.cpp
  src/localization_health/recovery_attempt_gate.cpp
  src/localization_health/localization_recovery_coordinator.cpp
  src/localization_health/reinitialization_control.cpp
  src/localization_health/tf_continuity_controller.cpp
  src/runtime/thread_runtime_controller.cpp
  src/runtime/frame_processing_watchdog.cpp
  src/slam/config_parameters.cpp
  src/slam/preprocessing.cpp
  src/slam/localization.cpp
  src/slam/split_map.cpp
  src/slam/system.cpp
)
ament_target_dependencies(${PROJECT_NAME}_relocalization_lib
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs diagnostic_msgs pcl_conversions
  pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs
  tf2_geometry_msgs OpenCV PCL
)
target_link_libraries(${PROJECT_NAME}_relocalization_lib
  ${GLOG_LIBRARIES}
  ${G2O_LIBRARIES}
  Threads::Threads
)
target_sources(${PROJECT_NAME}_relocalization_lib PRIVATE
  src/localization_map/global_map_provider.cpp
  src/localization_map/tiled_map_provider.cpp
  src/localization_map/tile_cache.cpp
  src/localization_map/tile_manifest.cpp
  src/localization_map/tile_map_generator.cpp
)
target_link_libraries(${PROJECT_NAME}_relocalization_lib yaml-cpp)
set_target_properties(${PROJECT_NAME}_relocalization_lib PROPERTIES
  INSTALL_RPATH "$ORIGIN")

add_executable(${PROJECT_NAME}_relocalization src/relocalization_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_relocalization
  rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs diagnostic_msgs pcl_conversions
  pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs
  tf2_geometry_msgs OpenCV PCL
)
target_link_libraries(${PROJECT_NAME}_relocalization
  ${PROJECT_NAME}_relocalization_lib
  ${GLOG_LIBRARIES}
  ${G2O_LIBRARIES}
)

add_executable(${PROJECT_NAME}_generate_tiled_map src/generate_tiled_map.cpp)
target_link_libraries(${PROJECT_NAME}_generate_tiled_map
  ${PROJECT_NAME}_relocalization_lib)
set_target_properties(
  ${PROJECT_NAME}_relocalization
  ${PROJECT_NAME}_generate_tiled_map
  PROPERTIES
    INSTALL_RPATH "$ORIGIN/..")

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
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
  TARGETS ${PROJECT_NAME}_relocalization_lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(
  TARGETS ${PROJECT_NAME}_relocalization
    ${PROJECT_NAME}_generate_tiled_map
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_localization_stability_common
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_stability_common.py)
  set_tests_properties(test_localization_stability_common PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  add_test(NAME test_localization_stability_baseline
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_stability_baseline.py)
  set_tests_properties(test_localization_stability_baseline PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  add_test(NAME test_localization_regression
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_regression.py)
  set_tests_properties(test_localization_regression PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  add_test(NAME test_localization_stability_rss
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_localization_stability_rss.py)
  set_tests_properties(test_localization_stability_rss PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test/integration")
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(test_ground_ring_policy test/test_ground_ring_policy.cpp)
  target_include_directories(test_ground_ring_policy PRIVATE include)
  ament_add_gtest(test_ground_grid_patcher test/test_ground_grid_patcher.cpp)
  target_include_directories(test_ground_grid_patcher PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ground_grid_patcher ${PCL_LIBRARIES})
  ament_add_gtest(test_ground_patch_parameter_surface test/test_ground_patch_parameter_surface.cpp)
  ament_add_gtest(test_auto_relocalization_parameter_surface
    test/test_auto_relocalization_parameter_surface.cpp)
  ament_add_gtest(test_thread_runtime_controller
    test/test_thread_runtime_controller.cpp
    src/runtime/thread_runtime_controller.cpp)
  target_include_directories(test_thread_runtime_controller PRIVATE include)
  target_link_libraries(test_thread_runtime_controller
    Threads::Threads ${GLOG_LIBRARIES})
  ament_add_gtest(test_frame_processing_watchdog
    test/test_frame_processing_watchdog.cpp
    src/runtime/frame_processing_watchdog.cpp)
  target_include_directories(test_frame_processing_watchdog PRIVATE include)
  ament_add_gtest(test_unified_parameter_schema
    test/test_unified_parameter_schema.cpp)
  ament_add_gtest(test_tile_index test/test_tile_index.cpp)
  target_include_directories(test_tile_index PRIVATE include)
  ament_add_gtest(test_tile_cache
    test/test_tile_cache.cpp
    src/localization_map/tile_cache.cpp)
  target_include_directories(test_tile_cache PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_tile_cache ${PCL_LIBRARIES})
  ament_add_gtest(test_global_map_provider
    test/test_global_map_provider.cpp
    src/localization_map/global_map_provider.cpp)
  target_include_directories(test_global_map_provider PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_global_map_provider ${PCL_LIBRARIES})
  ament_add_gtest(test_tiled_map_provider
    test/test_tiled_map_provider.cpp
    src/localization_map/tile_manifest.cpp
    src/localization_map/tile_cache.cpp
    src/localization_map/tiled_map_provider.cpp)
  target_include_directories(test_tiled_map_provider PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_tiled_map_provider
    yaml-cpp ${PCL_LIBRARIES} Threads::Threads)
  ament_add_gtest(test_tile_manifest
    test/test_tile_manifest.cpp
    src/localization_map/tile_manifest.cpp)
  target_include_directories(test_tile_manifest PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_tile_manifest yaml-cpp ${PCL_LIBRARIES})
  ament_add_gtest(test_tile_map_generator
    test/test_tile_map_generator.cpp
    src/localization_map/tile_manifest.cpp
    src/localization_map/tile_map_generator.cpp)
  target_include_directories(test_tile_map_generator PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_tile_map_generator yaml-cpp ${PCL_LIBRARIES})
  ament_add_gtest(test_relocalization_efficiency_surface
    test/test_relocalization_efficiency_surface.cpp)
  ament_add_gtest(test_tiled_map_localization_surface
    test/test_tiled_map_localization_surface.cpp)
  ament_add_gtest(test_bounded_async test/test_bounded_async.cpp)
  target_include_directories(test_bounded_async PRIVATE include)
  ament_add_gtest(test_ivox_map_batch_add
    test/test_ivox_map_batch_add.cpp
    src/ivox_map/ivox_map.cpp
    src/ivox_map/voxel_grid_node.cpp)
  target_include_directories(test_ivox_map_batch_add PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ivox_map_batch_add
    ${PCL_LIBRARIES} ${GLOG_LIBRARIES})
  ament_add_gtest(test_code_hygiene test/test_code_hygiene.cpp)
  ament_add_gtest(test_data_stream_watchdog test/test_data_stream_watchdog.cpp src/localization_health/data_stream_watchdog.cpp)
  target_include_directories(test_data_stream_watchdog PRIVATE include)
  ament_add_gtest(test_imu_data_searcher test/test_imu_data_searcher.cpp)
  target_include_directories(test_imu_data_searcher PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_imu_data_searcher
    ${LIO_SAM_EIGEN_TARGET} ${GLOG_LIBRARIES})
  ament_add_gtest(test_tf_continuity_controller
    test/test_tf_continuity_controller.cpp
    src/localization_health/tf_continuity_controller.cpp)
  target_include_directories(test_tf_continuity_controller PRIVATE
    include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_tf_continuity_controller ${LIO_SAM_EIGEN_TARGET})
  ament_add_gtest(test_recovery_attempt_gate
    test/test_recovery_attempt_gate.cpp
    src/localization_health/recovery_attempt_gate.cpp
    src/localization_health/localization_state_machine.cpp)
  target_include_directories(test_recovery_attempt_gate PRIVATE include)
  ament_add_gtest(test_reinitialization_control
    test/test_reinitialization_control.cpp
    src/localization_health/reinitialization_control.cpp)
  target_include_directories(test_reinitialization_control PRIVATE include)
  ament_add_gtest(test_localization_recovery_coordinator
    test/test_localization_recovery_coordinator.cpp
    src/localization_health/localization_recovery_coordinator.cpp
    src/localization_health/localization_state_machine.cpp
    src/localization_health/recovery_attempt_gate.cpp
    src/localization_health/reinitialization_control.cpp)
  target_include_directories(test_localization_recovery_coordinator PRIVATE include)
  ament_add_gtest(test_localization_state_machine
    test/test_localization_state_machine.cpp
    src/localization_health/localization_state_machine.cpp)
  target_include_directories(test_localization_state_machine PRIVATE include)
  ament_add_gtest(test_lidar_imu_calibration
    test/test_lidar_imu_calibration.cpp)
  target_include_directories(test_lidar_imu_calibration PRIVATE include)
  target_link_libraries(test_lidar_imu_calibration ${LIO_SAM_EIGEN_TARGET})
  ament_add_gtest(test_relocalization_acceptance_policy
    test/test_relocalization_acceptance_policy.cpp)
  target_include_directories(test_relocalization_acceptance_policy PRIVATE include)
  ament_add_gtest(test_ground_map_source test/test_ground_map_source.cpp)
  target_include_directories(test_ground_map_source PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ground_map_source ${PCL_LIBRARIES})
  ament_add_gtest(test_scan_context test/test_scan_context.cpp)
  target_include_directories(test_scan_context PRIVATE include)
  target_link_libraries(test_scan_context ${PROJECT_NAME}_relocalization_lib ${PCL_LIBRARIES})
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
