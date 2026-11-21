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

# Keep normal builds independent of test sources and test dependencies.
option(BUILD_TESTING "Build the testing tree." OFF)

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
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(class_loader REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

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

rosidl_generate_interfaces(${PROJECT_NAME} "msg/CloudInfo.msg" "srv/SaveMapOptions.srv" DEPENDENCIES std_msgs sensor_msgs)

# Pure coarse-voxel support and label policy. No ROS, PCL, or native backend
# dependency, so config parsing and policy tests can link it directly.
add_library(${PROJECT_NAME}_dufomap_temporal_guard STATIC
  src/common/dufomap_temporal_guard.cpp)
target_include_directories(${PROJECT_NAME}_dufomap_temporal_guard PUBLIC include)
set_target_properties(${PROJECT_NAME}_dufomap_temporal_guard PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Shared contract sources: direction-safe transforms, ROS pose/TF conversion,
# immutable configuration, and map artifact layout. Consumers link this instead
# of re-deriving frames, aliases, or artifact paths themselves.
add_library(${PROJECT_NAME}_contracts STATIC
  src/common/dufomap_export.cpp
  src/common/map_artifact_layout.cpp
  src/common/ros_pose_adapter.cpp
  src/common/save_map_request.cpp
  src/common/slam_config.cpp
)
ament_target_dependencies(${PROJECT_NAME}_contracts
  rclcpp geometry_msgs nav_msgs)
target_include_directories(${PROJECT_NAME}_contracts PUBLIC include)
target_link_libraries(${PROJECT_NAME}_contracts ${LIO_SAM_EIGEN_TARGET}
  ${PROJECT_NAME}_dufomap_temporal_guard)
# Consumers include the shared relocalization library, so this archive's
# objects must be position independent.
set_target_properties(${PROJECT_NAME}_contracts PROPERTIES
  POSITION_INDEPENDENT_CODE ON)



# Durable complete-keyframe scan cache used by online DUFOMap export. It only
# depends on the small DufomapPoint value type, so it stays C++17 and can be
# shared by the worker and behavior tests.
add_library(${PROJECT_NAME}_keyframe_scan_cache STATIC
  src/common/keyframe_scan_cache.cpp)
target_include_directories(${PROJECT_NAME}_keyframe_scan_cache PUBLIC include)
set_target_properties(${PROJECT_NAME}_keyframe_scan_cache PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Official DUFOMap/UFOMap core is C++20 and intentionally isolated behind a
# PIMPL so the mapping node and all other production targets stay C++17.
find_package(TBB REQUIRED)
add_library(${PROJECT_NAME}_dufomap_native STATIC
  src/common/dufomap_backend.cpp)
target_include_directories(${PROJECT_NAME}_dufomap_native
  PUBLIC include
  PRIVATE third_party/ufomap/include
          third_party/ufomap/compat)
target_compile_features(${PROJECT_NAME}_dufomap_native PRIVATE cxx_std_20)
target_compile_definitions(${PROJECT_NAME}_dufomap_native PRIVATE UFO_PARALLEL=1)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  target_include_directories(${PROJECT_NAME}_dufomap_native BEFORE PRIVATE
    third_party/ufomap/arm64)
endif()
target_link_libraries(${PROJECT_NAME}_dufomap_native
  PUBLIC TBB::tbb OpenMP::OpenMP_CXX Threads::Threads)
set_target_properties(${PROJECT_NAME}_dufomap_native PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

# Background online state is shared by the mapping node and behavior tests.
add_library(${PROJECT_NAME}_dufomap_online STATIC
  src/common/online_dufomap.cpp)
target_include_directories(${PROJECT_NAME}_dufomap_online PUBLIC
  include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}_dufomap_online
  sensor_msgs pcl_conversions)
target_link_libraries(${PROJECT_NAME}_dufomap_online
  ${PROJECT_NAME}_keyframe_scan_cache ${PROJECT_NAME}_dufomap_native
  ${PROJECT_NAME}_dufomap_temporal_guard ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME}_dufomap_online PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

add_library(${PROJECT_NAME}_ground_algorithms STATIC
  src/ground/current_ground_segmenter.cpp
  src/ground/ground_mesh.cpp
  src/ground/ground_grid_processor.cpp
  src/ground/ground_projection_cache.cpp
  src/ground/online_ground_pipeline.cpp
  src/common/patchworkpp_ground_segmenter.cpp
  third_party/patchworkpp/cpp/common/src/plane_fit.cpp
  third_party/patchworkpp/cpp/patchworkpp/src/patchworkpp.cpp)
target_include_directories(${PROJECT_NAME}_ground_algorithms PUBLIC include ${PCL_INCLUDE_DIRS}
  PRIVATE third_party/patchworkpp/cpp/common/include
          third_party/patchworkpp/cpp/patchworkpp/include)
ament_target_dependencies(${PROJECT_NAME}_ground_algorithms rclcpp nav_msgs geometry_msgs OpenCV)
target_link_libraries(${PROJECT_NAME}_ground_algorithms ${PCL_LIBRARIES} ${LIO_SAM_EIGEN_TARGET})
set_target_properties(${PROJECT_NAME}_ground_algorithms PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
ament_target_dependencies(${PROJECT_NAME}_featureExtraction rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_featureExtraction "${cpp_typesupport_target}") 

add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
ament_target_dependencies(${PROJECT_NAME}_imageProjection rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imageProjection "${cpp_typesupport_target}" ${PROJECT_NAME}_ground_algorithms)

add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
ament_target_dependencies(${PROJECT_NAME}_imuPreintegration rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${GTSAM_LIBRARIES} ${LIO_SAM_EIGEN_TARGET} "${cpp_typesupport_target}")

add_executable(${PROJECT_NAME}_mapOptimization src/mapOptimization.cpp)
ament_target_dependencies(${PROJECT_NAME}_mapOptimization rclcpp rclpy std_msgs sensor_msgs geometry_msgs nav_msgs pcl_conversions pcl_msgs visualization_msgs tf2 tf2_ros tf2_eigen tf2_sensor_msgs tf2_geometry_msgs OpenCV PCL)
target_sources(${PROJECT_NAME}_mapOptimization PRIVATE
  src/ivox_map/ivox_map.cpp
  src/ivox_map/voxel_grid_node.cpp
  src/localization_map/tile_manifest.cpp
  src/localization_map/tile_map_generator.cpp
  src/relocalization/scan_context.cpp
)

if (OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_mapOptimization ${GTSAM_LIBRARIES} "${cpp_typesupport_target}" OpenMP::OpenMP_CXX ${GLOG_LIBRARIES} yaml-cpp)
else()
  target_link_libraries(${PROJECT_NAME}_mapOptimization ${GTSAM_LIBRARIES} "${cpp_typesupport_target}" ${GLOG_LIBRARIES} yaml-cpp)
endif()

target_link_libraries(${PROJECT_NAME}_mapOptimization ${PROJECT_NAME}_ground_algorithms ${PROJECT_NAME}_dufomap_online)

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
  src/localization_health/automatic_recovery_policy.cpp
  src/localization_health/data_stream_watchdog.cpp
  src/localization_health/localization_state_machine.cpp
  src/localization_health/recovery_attempt_gate.cpp
  src/localization_health/localization_recovery_coordinator.cpp
  src/localization_health/reinitialization_control.cpp
  src/localization_health/tf_continuity_controller.cpp
  src/runtime/thread_runtime_controller.cpp
  src/runtime/frame_processing_watchdog.cpp
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

# Every production consumer of the shared contract gets it here rather than
# repeating the dependency at each add_executable/add_library site.
foreach(contract_consumer
    ${PROJECT_NAME}_featureExtraction
    ${PROJECT_NAME}_imageProjection
    ${PROJECT_NAME}_imuPreintegration
    ${PROJECT_NAME}_mapOptimization
    ${PROJECT_NAME}_relocalization_lib)
  target_link_libraries(${contract_consumer} ${PROJECT_NAME}_contracts)
endforeach()

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

# RViz panel for the mapping UI. Kept in a plugin library so the shared
# config/rviz2.rviz can list it in every session; when localization mode does
# not offer /lio_sam/save_map_options the panel reports the service as
# unavailable.
add_library(${PROJECT_NAME}_save_map_panel
  include/lio_sam_hesai/rviz/save_map_dialog.hpp
  include/lio_sam_hesai/rviz/save_map_panel.hpp
  src/rviz/save_map_controller.cpp
  src/rviz/save_map_dialog.cpp
  src/rviz/save_map_panel.cpp
)
target_include_directories(${PROJECT_NAME}_save_map_panel PUBLIC include)
ament_target_dependencies(${PROJECT_NAME}_save_map_panel
  rclcpp rviz_common pluginlib)
target_link_libraries(${PROJECT_NAME}_save_map_panel
  "${cpp_typesupport_target}" Qt5::Widgets)
set_target_properties(${PROJECT_NAME}_save_map_panel PROPERTIES
  AUTOMOC ON
  INSTALL_RPATH "$ORIGIN")
# Registers lio_sam_hesai/SaveMapPanel with rviz_common so the panel can be
# listed in config/rviz2.rviz for both mapping and localization sessions.
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

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

install(FILES third_party/patchworkpp/LICENSE third_party/patchworkpp/SOURCE.md
  DESTINATION share/${PROJECT_NAME}/third_party/patchworkpp)

install(FILES third_party/ufomap/LICENSE third_party/ufomap/LICENSE.liblzf
              third_party/ufomap/SOURCE.md third_party/ufomap/SHA256SUMS
  DESTINATION share/${PROJECT_NAME}/third_party/ufomap)

install(DIRECTORY scripts/dufomap_native
  DESTINATION lib/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)

# /proc/self/exe resolves symlink installs back to the build executable.
# Keep the legacy replay script adjacent in build and install layouts.
configure_file(scripts/dufomap_export_filter.py
  ${CMAKE_CURRENT_BINARY_DIR}/dufomap_export_filter.py COPYONLY)

# Legacy manual replay tools installed next to mapOptimization; production
# online mapping runs the native backend and never launches this script.
install(
  PROGRAMS scripts/dufomap_export_filter.py scripts/install_dufomap_backend.sh
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
  TARGETS ${PROJECT_NAME}_save_map_panel
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_include_directories(include)

# Test sources and their CMake registrations are local to development checkouts.
if(BUILD_TESTING)
  include("${CMAKE_CURRENT_SOURCE_DIR}/test/CMakeLists.txt" OPTIONAL)
endif()

ament_package()
