#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/kdtree/kdtree_flann.h>  // pcl include kdtree_flann throws error if PCL_NO_PRECOMPILE
                                      // is defined before
#define PCL_NO_PRECOMPILE
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "common/lidar_imu_calibration.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>

typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER, LIVOX, LSLIDAR, HESAI};

class ParamServer : public rclcpp::Node
{
public:
    std::string robot_id;

    //Topics
    std::string pointCloudTopic;
    std::string imuTopic;
    std::string odomTopic;
    std::string gpsTopic;

    //Frames
    std::string lidarFrame;
    std::string baselinkFrame;
    std::string odometryFrame;
    std::string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    bool generateScDb;
    std::string savePCDDirectory;
    bool mappingMode;
    float savePCDResolution;
    float saveGroundPCDResolution;
    bool saveGroundPCD;
    std::string groundPCDDirectory;
    std::string groundPCDDirectoryGlobal;
    bool groundFinalUseOdom;
    float groundLeafSize;
    float groundGlobalLeafSize;
    float groundPlaneDistance;
    float groundPlaneMaxAngle;
    int groundPlaneMinPoints;
    bool groundFlattenZ;
    float groundFlattenValue;
    bool alignSavedMapGroundToZero;
    float savedMapGroundZOffset;
    bool groundGridPatchEnable;
    float groundGridPatchResolution;
    float groundGridPatchSearchRadius;
    float groundGridPatchMaxSlopeDeg;
    float groundGridPatchMaxHeightRange;
    float groundGridPatchMaxRoughness;
    float groundGridPatchMaxPredictionMargin;
    float groundGridPatchMinInlierRatio;
    int groundGridPatchMinNeighborCells;
    int groundGridPatchMinSupportQuadrants;

    // Lidar Sensor Configuration
    SensorType sensor = SensorType::OUSTER;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;
    int groundScanStartIndex;
    int groundScanEndIndex;
    float groundAngleThreshold;

    // IMU
    int imuType;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    std::vector<double> extRotV;
    std::vector<double> extRPYV;
    std::vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;
    // Add these angle parameters for Livox
    float ang_bottom = -7.0;     // MID-360's bottom angle
    float ang_res_y = 59.0/16.0; // Vertical resolution

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;
    bool dynamicKeyframeFilterEnable;
    float dynamicSurfKnnMaxDist;
    float dynamicSurfPlaneMaxDist;

    bool globalStaticMapFilterEnable;
    float globalStaticVoxelSize;
    int globalStaticMinKF;
    bool globalStaticFilterCorners;


    float z_tollerance;
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold;
    float surroundingkeyframeAddingAngleThreshold;
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;

    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {
        declare_parameter("sensor_topic.lidar_topic", "points");
        get_parameter("sensor_topic.lidar_topic", pointCloudTopic);
        declare_parameter("sensor_topic.imu_topic", "imu/data");
        get_parameter("sensor_topic.imu_topic", imuTopic);
        declare_parameter("sensor_topic.odom_topic", "lio_sam/odometry/imu");
        get_parameter("sensor_topic.odom_topic", odomTopic);
        declare_parameter("sensor_topic.gps_topic", "lio_sam/odometry/gps");
        get_parameter("sensor_topic.gps_topic", gpsTopic);

        declare_parameter("frames.lidar", "laser_data_frame");
        get_parameter("frames.lidar", lidarFrame);
        declare_parameter("frames.base_link", "base_link");
        get_parameter("frames.base_link", baselinkFrame);
        declare_parameter("frames.odometry", "odom");
        get_parameter("frames.odometry", odometryFrame);
        declare_parameter("frames.map", "map");
        get_parameter("frames.map", mapFrame);

        declare_parameter("gps.use_imu_heading_initialization", false);
        get_parameter("gps.use_imu_heading_initialization", useImuHeadingInitialization);
        declare_parameter("gps.use_elevation", false);
        get_parameter("gps.use_elevation", useGpsElevation);
        declare_parameter("gps.covariance_threshold", 2.0);
        get_parameter("gps.covariance_threshold", gpsCovThreshold);
        declare_parameter("gps.pose_covariance_threshold", 25.0);
        get_parameter("gps.pose_covariance_threshold", poseCovThreshold);

        declare_parameter("map_save.enabled", false);
        get_parameter("map_save.enabled", savePCD);
        declare_parameter("map_save.generate_scan_context_db", true);
        get_parameter("map_save.generate_scan_context_db", generateScDb);
        declare_parameter("map_save.directory", "/Downloads/LOAM/");
        get_parameter("map_save.directory", savePCDDirectory);
        declare_parameter("system.mapping_mode", true);
        get_parameter("system.mapping_mode", mappingMode);
        declare_parameter("map_save.resolution", 0.0);
        get_parameter("map_save.resolution", savePCDResolution);
        declare_parameter("map_save.ground_resolution", 0.0);
        get_parameter("map_save.ground_resolution", saveGroundPCDResolution);
        declare_parameter("map_save.save_ground_map", false);
        get_parameter("map_save.save_ground_map", saveGroundPCD);
        declare_parameter("map_save.ground_directory", "/Downloads/LOAM_ground/");
        get_parameter("map_save.ground_directory", groundPCDDirectory);
        declare_parameter("map_save.ground_global_directory", "/Downloads/LOAM_ground_global/");
        get_parameter("map_save.ground_global_directory", groundPCDDirectoryGlobal);
        declare_parameter("map_save.ground_final_use_odometry", true);
        get_parameter("map_save.ground_final_use_odometry", groundFinalUseOdom);
        declare_parameter("ground_segmentation.leaf_size", 0.1);
        get_parameter("ground_segmentation.leaf_size", groundLeafSize);
        declare_parameter("map_save.ground_global_leaf_size", 0.2);
        get_parameter("map_save.ground_global_leaf_size", groundGlobalLeafSize);
        declare_parameter("ground_segmentation.plane_distance", 0.12);
        get_parameter("ground_segmentation.plane_distance", groundPlaneDistance);
        declare_parameter("ground_segmentation.plane_max_angle_deg", 35.0);
        get_parameter("ground_segmentation.plane_max_angle_deg", groundPlaneMaxAngle);
        declare_parameter("ground_segmentation.plane_min_points", 80);
        get_parameter("ground_segmentation.plane_min_points", groundPlaneMinPoints);
        declare_parameter("ground_segmentation.flatten_z", false);
        get_parameter("ground_segmentation.flatten_z", groundFlattenZ);
        declare_parameter("ground_segmentation.flatten_value", 0.0);
        get_parameter("ground_segmentation.flatten_value", groundFlattenValue);
        declare_parameter("map_save.align_ground_to_zero", false);
        get_parameter("map_save.align_ground_to_zero", alignSavedMapGroundToZero);
        declare_parameter("map_save.ground_z_offset", 0.0);
        get_parameter("map_save.ground_z_offset", savedMapGroundZOffset);
        declare_parameter("ground_segmentation.grid_patch.enable", false);
        get_parameter("ground_segmentation.grid_patch.enable", groundGridPatchEnable);
        declare_parameter("ground_segmentation.grid_patch.resolution", 0.10);
        get_parameter("ground_segmentation.grid_patch.resolution", groundGridPatchResolution);
        declare_parameter("ground_segmentation.grid_patch.search_radius", 0.40);
        get_parameter("ground_segmentation.grid_patch.search_radius", groundGridPatchSearchRadius);
        declare_parameter("ground_segmentation.grid_patch.max_slope_deg", 25.0);
        get_parameter("ground_segmentation.grid_patch.max_slope_deg", groundGridPatchMaxSlopeDeg);
        declare_parameter("ground_segmentation.grid_patch.max_height_range", 0.12);
        get_parameter("ground_segmentation.grid_patch.max_height_range", groundGridPatchMaxHeightRange);
        declare_parameter("ground_segmentation.grid_patch.max_roughness", 0.05);
        get_parameter("ground_segmentation.grid_patch.max_roughness", groundGridPatchMaxRoughness);
        declare_parameter("ground_segmentation.grid_patch.max_prediction_margin", 0.08);
        get_parameter("ground_segmentation.grid_patch.max_prediction_margin", groundGridPatchMaxPredictionMargin);
        declare_parameter("ground_segmentation.grid_patch.min_inlier_ratio", 0.70);
        get_parameter("ground_segmentation.grid_patch.min_inlier_ratio", groundGridPatchMinInlierRatio);
        declare_parameter("ground_segmentation.grid_patch.min_neighbor_cells", 6);
        get_parameter("ground_segmentation.grid_patch.min_neighbor_cells", groundGridPatchMinNeighborCells);
        declare_parameter("ground_segmentation.grid_patch.min_support_quadrants", 3);
        get_parameter("ground_segmentation.grid_patch.min_support_quadrants", groundGridPatchMinSupportQuadrants);

        std::string sensorStr;
        declare_parameter("lidar.sensor_type", "ouster");
        get_parameter("lidar.sensor_type", sensorStr);
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        }
        else if (sensorStr == "HESAI" || sensorStr == "Hesai")
        {
            sensor = SensorType::HESAI;
        }
        else if (sensorStr == "LSLIDAR")
        {
            sensor = SensorType::LSLIDAR;
        }
        else
        {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox'): " << sensorStr);
            rclcpp::shutdown();
        }
        declare_parameter("imu.type", 0);
        get_parameter("imu.type", imuType);
        declare_parameter("lidar.scan_lines", 64);
        get_parameter("lidar.scan_lines", N_SCAN);
        declare_parameter("lidar.horizon_scan", 512);
        get_parameter("lidar.horizon_scan", Horizon_SCAN);
        declare_parameter("lidar.downsample_rate", 1);
        get_parameter("lidar.downsample_rate", downsampleRate);
        declare_parameter("lidar.min_range", 5.5);
        get_parameter("lidar.min_range", lidarMinRange);
        declare_parameter("lidar.max_range", 1000.0);
        get_parameter("lidar.max_range", lidarMaxRange);
        declare_parameter("ground_segmentation.scan_start_index", 0);
        get_parameter("ground_segmentation.scan_start_index", groundScanStartIndex);
        declare_parameter("ground_segmentation.scan_end_index", -1);
        get_parameter("ground_segmentation.scan_end_index", groundScanEndIndex);
        declare_parameter("ground_segmentation.angle_threshold_deg", 10.0);
        get_parameter("ground_segmentation.angle_threshold_deg", groundAngleThreshold);

        declare_parameter("imu.acc_noise_std", 9e-4);
        get_parameter("imu.acc_noise_std", imuAccNoise);
        declare_parameter("imu.gyro_noise_std", 1.6e-4);
        get_parameter("imu.gyro_noise_std", imuGyrNoise);
        declare_parameter("imu.acc_bias_rw_noise_std", 5e-4);
        get_parameter("imu.acc_bias_rw_noise_std", imuAccBiasN);
        declare_parameter("imu.gyro_bias_rw_noise_std", 7e-5);
        get_parameter("imu.gyro_bias_rw_noise_std", imuGyrBiasN);
        declare_parameter("imu.gravity", 9.80511);
        get_parameter("imu.gravity", imuGravity);
        declare_parameter("imu.rpy_weight", 0.01);
        get_parameter("imu.rpy_weight", imuRPYWeight);

        const std::vector<double> identity = {
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0};
        declare_parameter("calibration.rotation", identity);
        get_parameter("calibration.rotation", extRotV);
        declare_parameter("calibration.rpy", identity);
        get_parameter("calibration.rpy", extRPYV);
        const std::vector<double> translation = {0.0, 0.0, 0.10};
        declare_parameter("calibration.translation", translation);
        get_parameter("calibration.translation", extTransV);

        try
        {
            const LidarImuCalibration calibration =
                ParseLidarImuCalibration(extTransV, extRotV, extRPYV);
            extTrans = calibration.mapping_imu_to_lidar_lever_arm;
            extRot = calibration.mapping_imu_to_lidar_rotation;
            extRPY = calibration.mapping_rpy_rotation;
            extQRPY = Eigen::Quaterniond(extRPY);
        }
        catch (const std::exception & error)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Invalid lidar-IMU calibration parameters: %s",
                error.what());
            throw;
        }

        declare_parameter("feature_extraction.edge_threshold", 1.0);
        get_parameter("feature_extraction.edge_threshold", edgeThreshold);
        declare_parameter("feature_extraction.surface_threshold", 0.1);
        get_parameter("feature_extraction.surface_threshold", surfThreshold);
        declare_parameter("feature_extraction.min_edge_features", 10);
        get_parameter("feature_extraction.min_edge_features", edgeFeatureMinValidNum);
        declare_parameter("feature_extraction.min_surface_features", 100);
        get_parameter("feature_extraction.min_surface_features", surfFeatureMinValidNum);

        declare_parameter("voxel_filter.odometry_surface_leaf_size", 0.4);
        get_parameter("voxel_filter.odometry_surface_leaf_size", odometrySurfLeafSize);
        declare_parameter("voxel_filter.mapping_corner_leaf_size", 0.2);
        get_parameter("voxel_filter.mapping_corner_leaf_size", mappingCornerLeafSize);
        declare_parameter("voxel_filter.mapping_surface_leaf_size", 0.4);
        get_parameter("voxel_filter.mapping_surface_leaf_size", mappingSurfLeafSize);
        declare_parameter("dynamic_filter.enable", true);
        get_parameter("dynamic_filter.enable", dynamicKeyframeFilterEnable);
        declare_parameter("dynamic_filter.surface_knn_max_distance", 1.0);
        get_parameter("dynamic_filter.surface_knn_max_distance", dynamicSurfKnnMaxDist);
        declare_parameter("dynamic_filter.surface_plane_max_distance", 0.3);
        get_parameter("dynamic_filter.surface_plane_max_distance", dynamicSurfPlaneMaxDist);

        declare_parameter("static_map_filter.enable", false);
        get_parameter("static_map_filter.enable", globalStaticMapFilterEnable);
        declare_parameter("static_map_filter.voxel_size", 0.4);
        get_parameter("static_map_filter.voxel_size", globalStaticVoxelSize);
        declare_parameter("static_map_filter.min_keyframes", 3);
        get_parameter("static_map_filter.min_keyframes", globalStaticMinKF);
        declare_parameter("static_map_filter.filter_corners", true);
        get_parameter("static_map_filter.filter_corners", globalStaticFilterCorners);

        declare_parameter("motion_constraint.z_tolerance", 1000.0);
        get_parameter("motion_constraint.z_tolerance", z_tollerance);
        declare_parameter("motion_constraint.rotation_tolerance", 1000.0);
        get_parameter("motion_constraint.rotation_tolerance", rotation_tollerance);

        declare_parameter("system.number_of_cores", 4);
        get_parameter("system.number_of_cores", numberOfCores);
        declare_parameter("system.mapping_process_interval", 0.15);
        get_parameter("system.mapping_process_interval", mappingProcessInterval);

        declare_parameter("local_map.keyframe_distance_threshold", 1.0);
        get_parameter("local_map.keyframe_distance_threshold", surroundingkeyframeAddingDistThreshold);
        declare_parameter("local_map.keyframe_angle_threshold", 0.2);
        get_parameter("local_map.keyframe_angle_threshold", surroundingkeyframeAddingAngleThreshold);
        declare_parameter("local_map.keyframe_density", 2.0);
        get_parameter("local_map.keyframe_density", surroundingKeyframeDensity);
        declare_parameter("local_map.search_radius", 50.0);
        get_parameter("local_map.search_radius", surroundingKeyframeSearchRadius);

        declare_parameter("loop_closure.enable", true);
        get_parameter("loop_closure.enable", loopClosureEnableFlag);
        declare_parameter("loop_closure.frequency", 1.0);
        get_parameter("loop_closure.frequency", loopClosureFrequency);
        declare_parameter("loop_closure.surrounding_keyframe_size", 50);
        get_parameter("loop_closure.surrounding_keyframe_size", surroundingKeyframeSize);
        declare_parameter("loop_closure.history_search_radius", 15.0);
        get_parameter("loop_closure.history_search_radius", historyKeyframeSearchRadius);
        declare_parameter("loop_closure.history_search_time_difference", 30.0);
        get_parameter("loop_closure.history_search_time_difference", historyKeyframeSearchTimeDiff);
        declare_parameter("loop_closure.history_search_count", 25);
        get_parameter("loop_closure.history_search_count", historyKeyframeSearchNum);
        declare_parameter("loop_closure.fitness_threshold", 0.3);
        get_parameter("loop_closure.fitness_threshold", historyKeyframeFitnessScore);
        declare_parameter("visualization.global_map_search_radius", 1000.0);
        get_parameter("visualization.global_map_search_radius", globalMapVisualizationSearchRadius);
        declare_parameter("visualization.global_map_pose_density", 10.0);
        get_parameter("visualization.global_map_pose_density", globalMapVisualizationPoseDensity);
        declare_parameter("visualization.global_map_leaf_size", 1.0);
        get_parameter("visualization.global_map_leaf_size", globalMapVisualizationLeafSize);

        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        
        if(imuType==0)
        acc*=imuGravity;
        
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final;
        
        if (imuType == 0)
    	{
        q_final = extQRPY;
    	}
    	else if (imuType == 1)
    	{	
        q_final = q_from * extQRPY;
    	}	
    	else{
        std::cout << "pls set your imu_type, 0 for 6axis and 1 for 9axis" << std::endl;
        }
 	q_final.normalize();
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (std::sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
            rclcpp::shutdown();
        }

        return imu_out;
    }
};


sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double stamp2Sec(const T& stamp)
{
    return rclcpp::Time(stamp).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return std::sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

rmw_qos_profile_t qos_profile{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    1,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile.history,
        qos_profile.depth
    ),
    qos_profile);

rmw_qos_profile_t qos_profile_imu{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    2000,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_imu = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_imu.history,
        qos_profile_imu.depth
    ),
    qos_profile_imu);

rmw_qos_profile_t qos_profile_lidar{
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,//BEST_EFFORT
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
};

auto qos_lidar = rclcpp::QoS(
    rclcpp::QoSInitialization(
        qos_profile_lidar.history,
        qos_profile_lidar.depth
    ),
    qos_profile_lidar);

#endif
