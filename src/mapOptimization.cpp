#include "utility.hpp"
#include "ivox_map/ivox_map.h"
#include "lio_sam/dufomap_backend.h"
#include "lio_sam/online_dufomap.h"
#include "common/dufomap_export.h"
#include "common/keyframe_scan_cache.h"
#include "common/map_artifact_layout.h"
#include "common/map_save_resolution.hpp"
#include "common/save_map_request.h"
#include "common/transformed_keyframe_cache.h"
#include "lio_sam/ground_grid_patcher.h"
#include "lio_sam/ground_grid_processor.h"
#include "lio_sam/ground_mesh.h"
#include "lio_sam/ground_ring_policy.h"
#include "lio_sam/current_ground_segmenter.h"
#include "lio_sam/ground_segmentation_mode.h"
#include "lio_sam/ground_projection_cache.h"
#include "lio_sam/patchworkpp_ground_segmenter.h"
#include "lio_sam_hesai/msg/cloud_info.hpp"
#include "lio_sam_hesai/srv/save_map_options.hpp"
#include "localization_map/tile_map_generator.h"
#include "relocalization/scan_context.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <cmath>
#include <unistd.h>

using namespace gtsam;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;

namespace {

constexpr std::size_t kSurfMapNeighborCount = 5;
constexpr float kSurfMapNeighborMaxDistance = 1.0f;
constexpr float kSurfIVoxResolution = 0.5f;
constexpr std::size_t kCornerMapNeighborCount = 5;
constexpr float kCornerMapNeighborMaxDistance = 1.0f;
constexpr float kCornerIVoxResolution = 0.5f;

// Result of one synchronous map export, reported through both save services.
struct MapExportOutcome
{
    bool success = false;
    std::string message;
};

// HOME is used by the save_map_options destination validation.
std::string HomeDirectory()
{
    const char* home = std::getenv("HOME");
    return home != nullptr ? std::string(home) : std::string();
}

}  // namespace


// ----------------------------------------------------------------------------
// Global-map static voxel filter (anti-ghost):
// Keep voxels that are observed by >= minKF distinct keyframes, output voxel centroids.
// ----------------------------------------------------------------------------
struct VoxelIndex
{
    int ix;
    int iy;
    int iz;

    bool operator==(const VoxelIndex& o) const noexcept
    {
        return ix == o.ix && iy == o.iy && iz == o.iz;
    }
};

struct VoxelIndexHash
{
    size_t operator()(const VoxelIndex& v) const noexcept
    {
        // A simple, reasonably well-mixed hash for 3 ints
        size_t h1 = std::hash<int>{}(v.ix);
        size_t h2 = std::hash<int>{}(v.iy);
        size_t h3 = std::hash<int>{}(v.iz);

        size_t h = h1;
        h ^= h2 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        h ^= h3 + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
        return h;
    }
};

struct VoxelAcc
{
    double sx, sy, sz;
    uint32_t n;
    uint16_t kf_count;
    int last_kf;

    VoxelAcc() : sx(0.0), sy(0.0), sz(0.0), n(0), kf_count(0), last_kf(-1) {}
};

static inline VoxelIndex voxelIndexOf(const PointType& p, float voxelSize)
{
    const double inv = 1.0 / (double)voxelSize;
    VoxelIndex v;
    v.ix = (int)std::floor((double)p.x * inv);
    v.iy = (int)std::floor((double)p.y * inv);
    v.iz = (int)std::floor((double)p.z * inv);
    return v;
}

static void accumulateVoxelMap(std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash>& vox,
                               const pcl::PointCloud<PointType>::Ptr& cloud,
                               int keyframeId,
                               float voxelSize)
{
    for (const auto& pt : cloud->points)
    {
        VoxelIndex idx = voxelIndexOf(pt, voxelSize);
        auto it = vox.find(idx);
        if (it == vox.end())
        {
            VoxelAcc acc;
            acc.last_kf = keyframeId;
            acc.kf_count = 1;
            acc.sx = pt.x; acc.sy = pt.y; acc.sz = pt.z;
            acc.n = 1;
            vox.emplace(idx, acc);
        }
        else
        {
            VoxelAcc& acc = it->second;
            if (acc.last_kf != keyframeId)
            {
                acc.kf_count++;
                acc.last_kf = keyframeId;
            }
            acc.sx += pt.x; acc.sy += pt.y; acc.sz += pt.z;
            acc.n++;
        }
    }
}

static pcl::PointCloud<PointType>::Ptr voxelMapToCloud(const std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash>& vox,
                                                       int minKF)
{
    pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());
    out->reserve(vox.size());

    for (const auto& kv : vox)
    {
        const VoxelAcc& acc = kv.second;
        if ((int)acc.kf_count < minKF || acc.n == 0)
            continue;

        PointType p;
        p.x = (float)(acc.sx / (double)acc.n);
        p.y = (float)(acc.sy / (double)acc.n);
        p.z = (float)(acc.sz / (double)acc.n);
        p.intensity = 1.0f;
        out->push_back(p);
    }

    return out;
}


class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundCloudSurround;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryGlobal;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLaserOdometryIncremental;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubKeyPoses;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubHistoryKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIcpKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrames;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubRecentKeyFrame;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubCloudRegisteredRaw;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubLoopConstraintEdge;

    rclcpp::Service<lio_sam_hesai::srv::SaveMapOptions>::SharedPtr srvSaveMapOptions;
    rclcpp::Subscription<lio_sam_hesai::msg::CloudInfo>::SharedPtr subCloud;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subGPS;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subLoop;

    std::deque<nav_msgs::msg::Odometry> gpsQueue;
    lio_sam_hesai::msg::CloudInfo cloudInfo;

    std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    std::vector<pcl::PointCloud<PointType>::Ptr> groundCloudKeyFrames;

    // Per-session cache of the selected keyframes' full deskewed scans. It is
    // populated for every mapping session, lives outside the map destination,
    // and is retained after the node exits. Ground export inputs use the same
    // session directory when its export algorithm needs them.
    std::filesystem::path dufomapCacheDirectory_;
    std::string dufomapCacheError_;
    // Complete keyframe scans are written after the mapping lock is released.
    // A save waits for these pending writes before it asks the worker to
    // export, so it never observes a half-registered cache.
    std::string completeScanCacheError_;
    std::mutex completeScanCacheMutex_;
    std::condition_variable completeScanCacheCondition_;
    std::size_t pendingCompleteScanWrites_{0};
    // One save at a time owns the frozen snapshot and worker export query.
    std::mutex saveMutex_;
    std::unique_ptr<lio_sam_hesai::OnlineDufomap> online_dufomap_;
    // State of the most recent scan that the mapping thread actually
    // registered. Skipped scans only extrapolate a small IMU-odometry
    // increment relative to this exact keyframe anchor, so a later loop
    // correction moves them together with the anchor.
    int last_processed_anchor_keyframe_{-1};
    lio_sam_hesai::DufomapPose last_processed_anchor_T_scan_{};
    lio_sam_hesai::DufomapPose last_processed_fallback_map_T_scan_{};
    lio_sam_hesai::DufomapPose last_processed_initial_guess_{};
    bool have_last_processed_{false};
    bool have_last_processed_initial_guess_{false};
    std::size_t online_dufomap_skipped_{0};
    std::size_t online_dufomap_rejected_{0};
    std::size_t online_dufomap_enqueue_failures_{0};
    double last_dufomap_health_log_time_{-1.0};
    std::size_t last_dufomap_logged_dropped_{0};
    std::size_t last_dufomap_logged_skipped_{0};
    std::size_t last_dufomap_logged_worker_skipped_{0};
    std::string last_dufomap_logged_error_;
    // Kept when the configured worker cannot start, so a save request fails
    // explicitly instead of silently exporting an unfiltered raw map.
    std::string online_dufomap_startup_error_;
    std::string groundExportCacheError_;
    
    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast; // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast; // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudGroundLast; // ground feature set from imageProjection
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS; // downsampled surf feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    lio_sam_hesai::TransformedKeyframeCache<PointType>
        transformed_keyframe_cache_;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    std::shared_ptr<IVoxMap> ivoxCornerFromMap;
    std::shared_ptr<IVoxMap> ivoxSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    rclcpp::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];

    // Runtime override of the configured loop-closure flag: the node turns it
    // off for itself when it is not running in mapping mode.
    bool loop_closure_enabled_ = false;

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    Eigen::Matrix<float, 6, 6> matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    bool aLoopIsClosed = false;
    std::map<int, int> loopIndexContainer; // from new to old
    std::vector<std::pair<int, int>> loopIndexQueue;
    std::vector<gtsam::Pose3> loopPoseQueue;
    std::vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    std::deque<std_msgs::msg::Float64MultiArray> loopInfoVec;
    std::vector<int> cachedLocalMapKeyIndices;
    bool localMapCacheValid = false;

    nav_msgs::msg::Path globalPath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    mapOptimization(const rclcpp::NodeOptions & options)
        : ParamServer("lio_sam_mapOptimization", options),
          transformed_keyframe_cache_(
              static_cast<std::size_t>(
                  config_.mapping.local_map.transformed_cache_max_mb) *
              1024U * 1024U)
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        // Runtime override rather than configuration: the node turns this off
        // for itself when it is not running in mapping mode, so it lives here
        // instead of in the immutable config_.
        loop_closure_enabled_ = config_.mapping.loop_closure.enable;
        if (!config_.mapping.system.mapping_mode)
        {
            loop_closure_enabled_ = false;
            RCLCPP_WARN(get_logger(), "mappingMode=false: disable loop closure");
        }

        pubKeyPoses = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.trajectory, 1);
        pubLaserCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.global_map, 1);
        pubGroundCloudSurround = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.ground_map, 1);
        pubLaserOdometryGlobal = create_publisher<nav_msgs::msg::Odometry>(config_.pipeline_topics.mapping_odometry, qos);
        pubLaserOdometryIncremental = create_publisher<nav_msgs::msg::Odometry>(
            config_.pipeline_topics.mapping_odometry_incremental, qos);
        pubPath = create_publisher<nav_msgs::msg::Path>(config_.outputs.path, 1);
        br = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        subCloud = create_subscription<lio_sam_hesai::msg::CloudInfo>(
            config_.pipeline_topics.feature_cloud_info, qos,
            std::bind(&mapOptimization::laserCloudInfoHandler, this, std::placeholders::_1));
        subGPS = create_subscription<nav_msgs::msg::Odometry>(
            config_.sensors.gps, 200,
            std::bind(&mapOptimization::gpsHandler, this, std::placeholders::_1));
        subLoop = create_subscription<std_msgs::msg::Float64MultiArray>(
            config_.pipeline_topics.loop_closure_hint, qos,
            std::bind(&mapOptimization::loopInfoHandler, this, std::placeholders::_1));

        if (config_.mapping.system.mapping_mode)
        {
            // Explicit absolute destination and per-save ground/tiled toggles,
            // with a text result for the UI.
            auto optionsSaveMapService = [this](const std::shared_ptr<rmw_request_id_t> request_header, const std::shared_ptr<lio_sam_hesai::srv::SaveMapOptions::Request> req, std::shared_ptr<lio_sam_hesai::srv::SaveMapOptions::Response> res) -> void {
                (void)request_header;
                std::string error;
                const auto options = lio_sam_hesai::MakeSaveMapExportOptions(
                    req->destination,
                    req->resolution,
                    req->ground_resolution,
                    req->save_ground_map,
                    req->save_tiled_map,
                    HomeDirectory(),
                    &error);
                if (!options)
                {
                    RCLCPP_ERROR(get_logger(), "Rejected save_map_options request: %s", error.c_str());
                    res->success = false;
                    res->message = error;
                    return;
                }
                const MapExportOutcome outcome = ExportMap(*options);
                res->success = outcome.success;
                res->message = outcome.message;
            };

            srvSaveMapOptions = create_service<lio_sam_hesai::srv::SaveMapOptions>("lio_sam/save_map_options", optionsSaveMapService);
        }
        else
        {
            RCLCPP_WARN(get_logger(), "mappingMode=false: save_map_options service disabled");
        }
        pubHistoryKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.loop_history, 1);
        pubIcpKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.loop_history, 1);
        pubLoopConstraintEdge = create_publisher<visualization_msgs::msg::MarkerArray>(config_.outputs.loop_constraints, 1);

        pubRecentKeyFrames = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.local_map, 1);
        pubRecentKeyFrame = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.registered_cloud, 1);
        pubCloudRegisteredRaw = create_publisher<sensor_msgs::msg::PointCloud2>(config_.outputs.registered_cloud_raw, 1);

        downSizeFilterCorner.setLeafSize(config_.mapping.voxel_filter.mapping_corner_leaf_size, config_.mapping.voxel_filter.mapping_corner_leaf_size, config_.mapping.voxel_filter.mapping_corner_leaf_size);
        downSizeFilterSurf.setLeafSize(config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size);
        downSizeFilterICP.setLeafSize(config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size);
        downSizeFilterSurroundingKeyPoses.setLeafSize(config_.mapping.local_map.keyframe_density, config_.mapping.local_map.keyframe_density, config_.mapping.local_map.keyframe_density); // for surrounding key poses of scan-to-map optimization

        // Every mapping session keeps its ground-export scan cache outside the
        // map destination and retains it after node exit. Online DUFOMap has
        // its own worker cache below this session directory.
        dufomapCacheDirectory_ = lio_sam_hesai::MakeDufomapSessionDirectory(
            config_.mapping.dufomap.cache_directory,
            static_cast<unsigned long>(::getpid()), &dufomapCacheError_);
        if (dufomapCacheDirectory_.empty())
        {
            RCLCPP_ERROR(get_logger(),
                         "Keyframe scan cache could not be created: %s. "
                         "Online DUFOMap and cached ground exports will fail "
                         "until this is fixed.",
                         dufomapCacheError_.c_str());
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Keyframe scan cache created in %s",
                        dufomapCacheDirectory_.c_str());
        }

        if (config_.mapping.dufomap.enable)
        {
            if (dufomapCacheDirectory_.empty())
            {
                online_dufomap_startup_error_ =
                    "online DUFOMap is enabled but the session cache is "
                    "unavailable: " + dufomapCacheError_;
                RCLCPP_ERROR(get_logger(), "%s",
                             online_dufomap_startup_error_.c_str());
            }
            else
            {
                try
                {
                    online_dufomap_ =
                        std::make_unique<lio_sam_hesai::OnlineDufomap>(
                            config_.mapping.dufomap,
                            (dufomapCacheDirectory_ / "online").string());
                }
                catch (const std::exception& error)
                {
                    online_dufomap_startup_error_ =
                        std::string("online DUFOMap worker failed to start: ") +
                        error.what();
                    RCLCPP_ERROR(get_logger(), "%s",
                                 online_dufomap_startup_error_.c_str());
                }
            }
        }

        allocateMemory();
    }

    ~mapOptimization() override
    {
        // main() joins the loop and visualization threads before the node is
        // destroyed, so no callback can touch the worker after this reset.
        online_dufomap_.reset();
        if (!dufomapCacheDirectory_.empty())
        {
            RCLCPP_INFO(get_logger(), "Retaining keyframe scan cache: %s",
                        dufomapCacheDirectory_.c_str());
        }
    }

    lio_sam_hesai::GroundAlgorithm ExportGroundAlgorithm() const
    {
        return lio_sam_hesai::SelectGroundAlgorithm(
            config_.mapping.ground_segmentation.mode,
            config_.mapping.patchworkpp.mode, lio_sam_hesai::GroundStage::Export);
    }
    std::filesystem::path CachedGroundScanPath(std::size_t index) const
    {
        return dufomapCacheDirectory_ / "ground_scans" / CachedFullScanPath(index).filename();
    }
    std::filesystem::path CachedProjectionPath(std::size_t index) const
    {
        auto name = CachedFullScanPath(index).filename();
        name.replace_extension(".bin");
        return dufomapCacheDirectory_ / "projection" / name;
    }
    bool StoreGroundInputsForExport(std::size_t index, std::string* error)
    {
        const auto algorithm = ExportGroundAlgorithm();
        if (algorithm == lio_sam_hesai::GroundAlgorithm::None)
            return true;
        try
        {
            if (!groundExportCacheError_.empty())
                throw std::runtime_error(groundExportCacheError_);
            if (!dufomapCacheError_.empty())
                throw std::runtime_error(dufomapCacheError_);
            if (dufomapCacheDirectory_.empty())
                throw std::runtime_error("ground export scan cache is unavailable");
            if (algorithm == lio_sam_hesai::GroundAlgorithm::Current)
            {
                if (cloudInfo.cloud_projection_rows !=
                        static_cast<std::uint32_t>(config_.mapping.lidar.scan_lines) ||
                    cloudInfo.cloud_projection_columns !=
                        static_cast<std::uint32_t>(config_.mapping.lidar.horizon_scan))
                    throw std::runtime_error("source ground projection dimensions do not match mapping configuration");
                // Projection indices refer to this exact ordered scan. The
                // online DUFOMap cache stores a different, complete scan.
                auto projected = pcl::PointCloud<PointType>::Ptr(
                    new pcl::PointCloud<PointType>());
                pcl::fromROSMsg(cloudInfo.cloud_deskewed, *projected);
                std::string write_error;
                if (!StoreFullScanForExport(index, projected, &write_error))
                    throw std::runtime_error(write_error);
                lio_sam_hesai::WriteGroundProjectionFile(
                    CachedProjectionPath(index),
                    projected->size(),
                    cloudInfo.cloud_projection_indices,
                    config_.mapping.lidar.scan_lines,
                    config_.mapping.lidar.horizon_scan);
            }
            else
            {
                if (cloudInfo.cloud_export_scan.fields.empty() ||
                    cloudInfo.cloud_export_scan.data.empty())
                    throw std::runtime_error("missing complete pre-projection scan for Patchwork++ export");
                pcl::PointCloud<PointType> complete;
                pcl::fromROSMsg(cloudInfo.cloud_export_scan, complete);
                std::string write_error;
                if (!SaveCloudBinary(CachedGroundScanPath(index), complete, &write_error))
                    throw std::runtime_error(write_error);
            }
        }
        catch (const std::exception& exception)
        {
            groundExportCacheError_ = exception.what();
            if (error) *error = groundExportCacheError_;
            return false;
        }
        if (error) error->clear();
        return true;
    }

    bool RecomputeExportGround(
        const pcl::PointCloud<PointTypePose>::Ptr& poses, std::size_t frame_count,
        const std::string& scan_cache_error, const std::string& ground_cache_error,
        float ground_resolution, pcl::PointCloud<PointType>::Ptr& ground,
        std::string* error)
    {
        try
        {
            if (!scan_cache_error.empty()) throw std::runtime_error(scan_cache_error);
            if (!ground_cache_error.empty()) throw std::runtime_error(ground_cache_error);
            if (frame_count == 0 || poses->size() != frame_count)
                throw std::runtime_error("ground export requires cached keyframes and final poses");
            const auto algorithm = ExportGroundAlgorithm();
            std::unique_ptr<lio_sam_hesai::GroundMeshRasterizer> rasterizer;
            const auto& mesh_config = config_.mapping.map_save.ground_mesh;
            if (mesh_config.enable)
            {
                if (algorithm != lio_sam_hesai::GroundAlgorithm::Current)
                    throw std::runtime_error(
                        "ground_mesh requires Current/RingPlane export mode");
                if (!(ground_resolution > 0.0F))
                    throw std::runtime_error(
                        "ground_mesh requires a positive resolved ground_resolution; got " +
                        std::to_string(ground_resolution));
                lio_sam_hesai::GroundMeshOptions mesh_options;
                mesh_options.resolution = ground_resolution;
                mesh_options.max_edge_length = mesh_config.max_edge_length;
                mesh_options.edge_length_per_meter =
                    mesh_config.edge_length_per_meter;
                mesh_options.max_range_ratio = mesh_config.max_range_ratio;
                mesh_options.max_slope_deg =
                    config_.mapping.ground_segmentation.grid_patch.max_slope_deg;
                mesh_options.max_height_range =
                    config_.mapping.ground_segmentation.grid_patch.max_height_range;
                const auto& static_filter = config_.mapping.static_map_filter;
                if (static_filter.enable && static_filter.voxel_size > 1e-3f &&
                    static_filter.min_keyframes > 1)
                    mesh_options.min_keyframe_support =
                        static_filter.min_keyframes;
                rasterizer =
                    std::make_unique<lio_sam_hesai::GroundMeshRasterizer>(
                        mesh_options);
            }
            std::unique_ptr<lio_sam_hesai::PatchworkppGroundSegmenter> patchwork;
            if (algorithm == lio_sam_hesai::GroundAlgorithm::Patchworkpp)
                patchwork = std::make_unique<lio_sam_hesai::PatchworkppGroundSegmenter>(
                    config_.mapping.patchworkpp);
            const lio_sam_hesai::GroundRingRange output_rings =
                lio_sam_hesai::normalizeGroundScanRange(
                    config_.mapping.ground_segmentation.scan_start_index,
                    config_.mapping.ground_segmentation.scan_end_index,
                    config_.mapping.lidar.scan_lines);
            std::vector<pcl::PointCloud<PointType>::Ptr> frames;
            frames.reserve(frame_count);
            for (std::size_t index = 0; index < frame_count; ++index)
            {
                const auto path = algorithm == lio_sam_hesai::GroundAlgorithm::Current ?
                    CachedFullScanPath(index) : CachedGroundScanPath(index);
                pcl::PointCloud<PointType> scan;
                if (pcl::io::loadPCDFile<PointType>(path.string(), scan) != 0 || scan.empty())
                    throw std::runtime_error("missing/corrupt ground export scan: " + path.string());
                auto frame = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
                if (algorithm == lio_sam_hesai::GroundAlgorithm::Current)
                {
                    auto indices = lio_sam_hesai::ReadGroundProjectionFile(
                        CachedProjectionPath(index), scan.size(),
                        config_.mapping.lidar.scan_lines, config_.mapping.lidar.horizon_scan);
                    auto result = lio_sam_hesai::SegmentProjectedGround(
                        config_.mapping.ground_segmentation, scan, indices,
                        config_.mapping.lidar.scan_lines, config_.mapping.lidar.horizon_scan);
                    if (rasterizer)
                    {
                        const auto& pose = poses->points[index];
                        const Eigen::Matrix4f map_from_scan =
                            pcl::getTransformation(pose.x, pose.y, pose.z,
                                                   pose.roll, pose.pitch,
                                                   pose.yaw)
                                .matrix();
                        rasterizer->AddFrame(
                            scan, indices, result.classification,
                            config_.mapping.lidar.scan_lines,
                            config_.mapping.lidar.horizon_scan, output_rings,
                            map_from_scan);
                    }
                    *frame = std::move(result.ground);
                }
                else
                {
                    for (const auto point : patchwork->Segment(scan))
                    {
                        if (point >= scan.size())
                            throw std::runtime_error("Patchwork++ returned an invalid export index");
                        frame->push_back(scan[point]);
                    }
                }
                frames.push_back(std::move(frame));
            }
            ground = BuildGroundFromSnapshot(frames, poses);
            if (rasterizer)
            {
                const std::size_t appended =
                    rasterizer->AppendMissingSamples(ground.get());
                const auto& statistics = rasterizer->statistics();
                std::cout << "Ground mesh: frames=" << statistics.frames
                          << " acceptedTriangles="
                          << statistics.accepted_triangles
                          << " rasterizedCells=" << statistics.rasterized_cells
                          << " appendedSamples=" << appended << std::endl;
            }
            return true;
        }
        catch (const std::exception& exception)
        {
            if (error) *error = std::string("Ground export failed: ") + exception.what();
            return false;
        }
    }

    // DUFOMap evidence is cached per keyframe from the full deskewed scan, not
    // from the LOAM feature clouds. The file format is ordinary binary PCD so
    // the offline tool can replay the same dataset.
    std::filesystem::path CachedFullScanPath(std::size_t index) const
    {
        std::ostringstream name;
        name << std::setw(6) << std::setfill('0') << index << ".pcd";
        return dufomapCacheDirectory_ / "scans" / name.str();
    }

    bool StoreFullScanForExport(std::size_t index,
                                const pcl::PointCloud<PointType>::Ptr& cloud,
                                std::string* error)
    {
        if (!dufomapCacheError_.empty())
        {
            if (error != nullptr)
                *error = dufomapCacheError_;
            return false;
        }

        std::string reason;
        if (dufomapCacheDirectory_.empty())
        {
            reason = "Export scan cache directory is not available";
        }
        else if (cloud == nullptr || cloud->empty())
        {
            reason = "full deskewed scan for keyframe " + std::to_string(index) +
                     " is empty; map export requires a complete scan";
        }
        if (!reason.empty())
        {
            dufomapCacheError_ = reason;
            if (error != nullptr)
                *error = reason;
            return false;
        }

        const std::filesystem::path path = CachedFullScanPath(index);
        std::error_code filesystem_error;
        std::filesystem::create_directories(path.parent_path(), filesystem_error);
        if (filesystem_error)
        {
            reason = "cannot create map export scan cache " +
                     path.parent_path().string() + ": " +
                     filesystem_error.message();
        }
        else
        {
            try
            {
                if (pcl::io::savePCDFileBinary(path.string(), *cloud) != 0)
                    reason = "cannot write map export scan cache " + path.string();
            }
            catch (const std::exception& exception)
            {
                reason = "cannot write map export scan cache " + path.string() +
                         ": " + exception.what();
            }
        }

        if (!reason.empty())
        {
            dufomapCacheError_ = reason;
            if (error != nullptr)
                *error = reason;
            return false;
        }
        if (error != nullptr)
            error->clear();
        return true;
    }

    std::filesystem::path DufomapCacheDirectory() const
    {
        return dufomapCacheDirectory_;
    }

    // The complete scan cache is intentionally separate from the legacy
    // RingPlane/Patchwork++ scan directories. A missing or corrupt complete
    // scan must fail the export; the worker never falls back to features or
    // to those legacy ground inputs.
    std::filesystem::path CachedCompleteScanPath(std::size_t index) const
    {
        return lio_sam_hesai::KeyframeScanCachePath(
            dufomapCacheDirectory_ / "keyframe_scans", index);
    }

    bool StoreCompleteKeyframeScan(
        std::size_t index,
        const sensor_msgs::msg::PointCloud2& message,
        std::string* error)
    {
        std::string reason;
        if (dufomapCacheDirectory_.empty())
            reason = "complete keyframe scan cache directory is unavailable";
        else if (message.data.empty())
            reason = "complete keyframe scan for keyframe " +
                     std::to_string(index) + " is empty";

        std::vector<lio_sam_hesai::DufomapPoint> points;
        if (reason.empty())
        {
            try
            {
                pcl::PointCloud<PointType> cloud;
                pcl::fromROSMsg(message, cloud);
                points.reserve(cloud.size());
                for (const auto& point : cloud.points)
                    points.push_back(lio_sam_hesai::DufomapPoint{
                        point.x, point.y, point.z, point.intensity});
                if (points.empty())
                    reason = "complete keyframe scan for keyframe " +
                             std::to_string(index) + " produced no points";
            }
            catch (const std::exception& exception)
            {
                reason = "cannot convert complete keyframe scan for keyframe " +
                         std::to_string(index) + ": " + exception.what();
            }
        }

        std::filesystem::path path;
        if (reason.empty())
        {
            path = CachedCompleteScanPath(index);
            std::string write_error;
            if (!lio_sam_hesai::WriteKeyframeScanCacheFile(
                    path, index, points, &write_error))
                reason = write_error;
        }

        if (!reason.empty())
        {
            {
                std::lock_guard<std::mutex> lock(mtx);
                completeScanCacheError_ = reason;
            }
            if (error != nullptr) *error = reason;
            return false;
        }

        // The mapping callback registers the keyframe before this write
        // completes. Attaching the persisted path must not create a new pose
        // generation.
        if (online_dufomap_)
            online_dufomap_->SetKeyframeScanPath(
                static_cast<int>(index), path.string());
        if (error != nullptr) error->clear();
        return true;
    }

    void BeginCompleteScanWrite()
    {
        std::lock_guard<std::mutex> lock(completeScanCacheMutex_);
        ++pendingCompleteScanWrites_;
    }

    void FinishCompleteScanWrite()
    {
        {
            std::lock_guard<std::mutex> lock(completeScanCacheMutex_);
            if (pendingCompleteScanWrites_ > 0U)
                --pendingCompleteScanWrites_;
        }
        completeScanCacheCondition_.notify_all();
    }

    bool WaitForPendingCompleteScanWrites(
        std::chrono::milliseconds timeout, std::string* error)
    {
        std::unique_lock<std::mutex> lock(completeScanCacheMutex_);
        if (!completeScanCacheCondition_.wait_for(
                lock, timeout, [&] { return pendingCompleteScanWrites_ == 0U; }))
        {
            if (error != nullptr)
                *error = "timed out waiting for complete keyframe scan cache writes";
            return false;
        }
        return true;
    }

    // Build the unfiltered ground map from the same keyframe/pose snapshot as
    // the corner/surface export. Ground never enters DUFOMap classification;
    // this only freezes the poses used by the original patch/flatten/save path
    // while the backend runs.
    pcl::PointCloud<PointType>::Ptr BuildGroundFromSnapshot(
        const std::vector<pcl::PointCloud<PointType>::Ptr>& ground_frames,
        const pcl::PointCloud<PointTypePose>::Ptr& poses)
    {
        pcl::PointCloud<PointType>::Ptr ground(new pcl::PointCloud<PointType>());
        const bool filter = config_.mapping.static_map_filter.enable &&
            config_.mapping.static_map_filter.voxel_size > 1e-3f &&
            config_.mapping.static_map_filter.min_keyframes > 1;
        std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash> voxels;
        for (std::size_t index = 0; index < ground_frames.size() && index < poses->size(); ++index)
        {
            if (!ground_frames[index]) continue;
            auto transformed = transformPointCloud(ground_frames[index], &poses->points[index]);
            if (filter)
                accumulateVoxelMap(voxels, transformed, index,
                    config_.mapping.static_map_filter.voxel_size);
            else *ground += *transformed;
        }
        if (filter) ground = voxelMapToCloud(voxels, config_.mapping.static_map_filter.min_keyframes);
        return ground;
    }

    std::array<double, 16> PoseMatrix(const PointTypePose& pose) const
    {
        const Eigen::Affine3f transform = pcl::getTransformation(
            pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw);
        const Eigen::Matrix4f matrix = transform.matrix();
        std::array<double, 16> result{};
        for (int row = 0; row < 4; ++row)
        {
            for (int column = 0; column < 4; ++column)
                result[static_cast<std::size_t>(row * 4 + column)] =
                    static_cast<double>(matrix(row, column));
        }
        return result;
    }

    lio_sam_hesai::DufomapPose PoseMatrixFromValues(
        double x, double y, double z, double roll, double pitch, double yaw) const
    {
        PointTypePose pose;
        pose.x = x;
        pose.y = y;
        pose.z = z;
        pose.roll = roll;
        pose.pitch = pitch;
        pose.yaw = yaw;
        return PoseMatrix(pose);
    }

    lio_sam_hesai::DufomapPose CurrentInitialGuessPose() const
    {
        return PoseMatrixFromValues(
            cloudInfo.initial_guess_x, cloudInfo.initial_guess_y,
            cloudInfo.initial_guess_z, cloudInfo.initial_guess_roll,
            cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
    }

    lio_sam_hesai::DufomapPose CurrentLidarMapPose() const
    {
        return PoseMatrixFromValues(
            transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
    }

    static lio_sam_hesai::DufomapPose IdentityPose()
    {
        return lio_sam_hesai::DufomapPose{
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    }

    static bool PoseToMatrix(const lio_sam_hesai::DufomapPose& pose,
                             Eigen::Matrix4d* matrix)
    {
        if (matrix == nullptr) return false;
        for (int row = 0; row < 4; ++row)
        {
            for (int column = 0; column < 4; ++column)
            {
                const double value =
                    pose[static_cast<std::size_t>(row * 4 + column)];
                if (!std::isfinite(value)) return false;
                (*matrix)(row, column) = value;
            }
        }
        return true;
    }

    static lio_sam_hesai::DufomapPose MatrixToPose(
        const Eigen::Matrix4d& matrix)
    {
        lio_sam_hesai::DufomapPose result{};
        for (int row = 0; row < 4; ++row)
        {
            for (int column = 0; column < 4; ++column)
                result[static_cast<std::size_t>(row * 4 + column)] =
                    matrix(row, column);
        }
        return result;
    }

    // A finite homogeneous transform with an orthonormal right-handed
    // rotation. Every pose used as an anchor or a relative increment passes
    // this check before it is accepted as DUFOMap evidence.
    static bool IsRigidPose(const lio_sam_hesai::DufomapPose& pose)
    {
        Eigen::Matrix4d matrix;
        if (!PoseToMatrix(pose, &matrix)) return false;
        const Eigen::Vector3d bottom = matrix.row(3).head<3>().transpose();
        if (bottom.norm() > 1e-6 ||
            std::abs(matrix(3, 3) - 1.0) > 1e-6)
            return false;
        const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
        if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                    .norm() > 1e-3)
            return false;
        return std::abs(rotation.determinant() - 1.0) <= 1e-3;
    }

    static double TranslationNorm(const lio_sam_hesai::DufomapPose& pose)
    {
        return std::sqrt(pose[3] * pose[3] + pose[7] * pose[7] +
                         pose[11] * pose[11]);
    }

    static double RotationAngleRad(const lio_sam_hesai::DufomapPose& pose)
    {
        Eigen::Matrix4d matrix;
        if (!PoseToMatrix(pose, &matrix)) return std::numeric_limits<double>::infinity();
        const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0);
        const double cosine = std::min(
            1.0, std::max(-1.0, (rotation.trace() - 1.0) * 0.5));
        return std::acos(cosine);
    }

    static lio_sam_hesai::DufomapPose RelativePose(
        const lio_sam_hesai::DufomapPose& from,
        const lio_sam_hesai::DufomapPose& to)
    {
        Eigen::Matrix4d from_matrix;
        Eigen::Matrix4d to_matrix;
        if (!PoseToMatrix(from, &from_matrix) ||
            !PoseToMatrix(to, &to_matrix))
            return lio_sam_hesai::DufomapPose{};
        return MatrixToPose(from_matrix.inverse() * to_matrix);
    }

    static lio_sam_hesai::DufomapPose ComposePose(
        const lio_sam_hesai::DufomapPose& left,
        const lio_sam_hesai::DufomapPose& right)
    {
        Eigen::Matrix4d left_matrix;
        Eigen::Matrix4d right_matrix;
        if (!PoseToMatrix(left, &left_matrix) ||
            !PoseToMatrix(right, &right_matrix))
            return lio_sam_hesai::DufomapPose{};
        return MatrixToPose(left_matrix * right_matrix);
    }

    // A skipped scan may only extrapolate a small, physically plausible
    // increment. A large jump is a new odometry epoch, not usable evidence.
    static bool IsSaneRelativeIncrement(
        const lio_sam_hesai::DufomapPose& relative)
    {
        if (!IsRigidPose(relative)) return false;
        return TranslationNorm(relative) <= 5.0 &&
               RotationAngleRad(relative) <= M_PI / 3.0;
    }

    pcl::PointCloud<PointType>::Ptr FilterKeyframeCloud(
        const pcl::PointCloud<PointType>::Ptr& raw,
        const std::vector<std::uint8_t>* labels) const
    {
        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        if (!raw) return nullptr;
        if (labels == nullptr) return raw;
        if (labels->size() != raw->size())
        {
            RCLCPP_ERROR(get_logger(),
                         "Online DUFOMap label count %zu does not match keyframe cloud %zu",
                         labels->size(), raw->size());
            return nullptr;
        }
        std::vector<lio_sam_hesai::DufomapPoint> points;
        points.reserve(raw->size());
        for (const auto& point : raw->points)
            points.push_back(lio_sam_hesai::DufomapPoint{
                point.x, point.y, point.z, point.intensity});
        const auto kept = lio_sam_hesai::ApplyDufomapLabels(points, labels);
        filtered->reserve(kept.size());
        for (const auto& point : kept)
        {
            PointType output;
            output.x = point.x;
            output.y = point.y;
            output.z = point.z;
            output.intensity = point.intensity;
            filtered->push_back(output);
        }
        return filtered;
    }

    void UpdateOnlineDufomapPoses()
    {
        if (!online_dufomap_) return;
        std::vector<lio_sam_hesai::DufomapPose> poses;
        poses.reserve(cloudKeyPoses6D->size());
        for (const auto& pose : cloudKeyPoses6D->points)
            poses.push_back(PoseMatrix(pose));
        online_dufomap_->UpdateKeyframePoses(poses);
    }

    // Rejects any snapshot that cannot be applied to every frozen keyframe.
    // This runs before a save removes an old artifact or publishes a display
    // cloud, so a mismatch can never look like a successful empty map.
    bool ValidateDufomapLabels(
        const lio_sam_hesai::LabelSnapshot& snapshot,
        std::size_t required_count,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& corners,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& surfs,
        std::string* error) const
    {
        if (snapshot.keyframe_count < required_count ||
            snapshot.labels.size() < required_count)
        {
            if (error != nullptr)
                *error = "online DUFOMap snapshot covers " +
                         std::to_string(snapshot.labels.size()) +
                         " keyframes, but " + std::to_string(required_count) +
                         " were frozen for export";
            return false;
        }
        for (std::size_t index = 0; index < required_count; ++index)
        {
            const std::size_t corner_count =
                index < corners.size() && corners[index] ? corners[index]->size() : 0U;
            const std::size_t surf_count =
                index < surfs.size() && surfs[index] ? surfs[index]->size() : 0U;
            const auto& labels = snapshot.labels[index];
            if (labels.corner.size() != corner_count ||
                labels.surf.size() != surf_count)
            {
                if (error != nullptr)
                {
                    *error = "online DUFOMap label count mismatch at keyframe " +
                             std::to_string(index) + " (corner " +
                             std::to_string(labels.corner.size()) + "/" +
                             std::to_string(corner_count) + ", surf " +
                             std::to_string(labels.surf.size()) + "/" +
                             std::to_string(surf_count) + ")";
                }
                return false;
            }
        }
        return true;
    }

    // Health is logged at most once per few seconds, and only when a counter or
    // error changed, so a persistent worker failure stays observable without
    // flooding the mapping log.
    void MaybeLogOnlineDufomapHealth()
    {
        if (!online_dufomap_) return;
        const double now = timeLaserInfoCur;
        if (last_dufomap_health_log_time_ >= 0.0 &&
            now - last_dufomap_health_log_time_ < 5.0)
            return;
        last_dufomap_health_log_time_ = now;

        const std::size_t dropped = online_dufomap_->dropped_evidence();
        const std::size_t worker_skipped = online_dufomap_->skipped_evidence();
        const std::string error = online_dufomap_->last_error();
        const bool changed =
            dropped != last_dufomap_logged_dropped_ ||
            worker_skipped != last_dufomap_logged_worker_skipped_ ||
            online_dufomap_skipped_ != last_dufomap_logged_skipped_ ||
            error != last_dufomap_logged_error_;
        if (!changed) return;

        RCLCPP_WARN(get_logger(),
                    "Online DUFOMap health: dropped=%zu skipped=%zu "
                    "worker_skipped=%zu rejected=%zu enqueue_failures=%zu "
                    "last_error=%s",
                    dropped, online_dufomap_skipped_, worker_skipped,
                    online_dufomap_rejected_, online_dufomap_enqueue_failures_,
                    error.empty() ? "none" : error.c_str());
        last_dufomap_logged_dropped_ = dropped;
        last_dufomap_logged_skipped_ = online_dufomap_skipped_;
        last_dufomap_logged_worker_skipped_ = worker_skipped;
        last_dufomap_logged_error_ = error;
    }

    bool SaveCloudBinary(const std::filesystem::path& path,
                         const pcl::PointCloud<PointType>& cloud,
                         std::string* error) const
    {
        std::error_code filesystem_error;
        std::filesystem::create_directories(path.parent_path(), filesystem_error);
        if (filesystem_error)
        {
            if (error != nullptr)
                *error = "cannot create " + path.parent_path().string() + ": " +
                         filesystem_error.message();
            return false;
        }

        if (cloud.empty())
        {
            // PCL's binary writer can report an error for a zero-point cloud.
            // An empty feature/ground set is a legitimate export result, so
            // write the canonical empty PCD header explicitly.
            std::ofstream stream(path, std::ios::binary | std::ios::trunc);
            stream << "# .PCD v0.7 - Point Cloud Data file format\n"
                   << "VERSION .7\n"
                   << "FIELDS x y z intensity\n"
                   << "SIZE 4 4 4 4\n"
                   << "TYPE F F F F\n"
                   << "COUNT 1 1 1 1\n"
                   << "WIDTH 0\n"
                   << "HEIGHT 1\n"
                   << "VIEWPOINT 0 0 0 1 0 0 0\n"
                   << "POINTS 0\n"
                   << "DATA binary\n";
            stream.close();
            if (!stream.good())
            {
                if (error != nullptr)
                    *error = "cannot write empty point cloud " + path.string();
                return false;
            }
            return true;
        }

        try
        {
            if (pcl::io::savePCDFileBinary(path.string(), cloud) == 0)
                return true;
            if (error != nullptr)
                *error = "cannot write " + path.string();
        }
        catch (const std::exception& exception)
        {
            if (error != nullptr)
                *error = "cannot write " + path.string() + ": " + exception.what();
        }
        return false;
    }

    bool WriteCloudBinary(const std::filesystem::path& path,
                          const pcl::PointCloud<PointType>& cloud,
                          std::string* error) const
    {
        return SaveCloudBinary(path, cloud, error);
    }

    // Single export body for /lio_sam/save_map_options: the service resolves
    // its request into lio_sam_hesai::SaveMapExportOptions and lands here.
    MapExportOutcome ExportMap(const lio_sam_hesai::SaveMapExportOptions& options)
    {
        // A save freezes one worker generation and owns the only export query.
        std::lock_guard<std::mutex> save_lock(saveMutex_);
        MapExportOutcome outcome;
        // DUFOMap is an online property. A disabled configuration publishes
        // and saves exactly the original keyframe map.
        const bool dufomapEnabled = config_.mapping.dufomap.enable;
        const bool groundGridEnabled =
            config_.mapping.map_save.ground_grid.enable;
        const float groundRes = lio_sam_hesai::ResolveMapSaveResolution(
            options.ground_resolution, config_.mapping.map_save.ground_resolution);
        const bool recomputeGround = options.save_ground_map &&
            ExportGroundAlgorithm() != lio_sam_hesai::GroundAlgorithm::None;
        const bool groundMeshEnabled = options.save_ground_map &&
            config_.mapping.map_save.ground_mesh.enable;
        if (groundMeshEnabled &&
            ExportGroundAlgorithm() != lio_sam_hesai::GroundAlgorithm::Current)
        {
            outcome.message =
                "map_save.ground_mesh.enable requires Current/RingPlane "
                "export mode; got an unsupported ground export mode";
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }
        if (groundMeshEnabled && !(groundRes > 0.0F))
        {
            outcome.message =
                "map_save.ground_mesh.enable requires a positive resolved "
                "ground_resolution; got " +
                std::to_string(groundRes);
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }
        const bool deferFinalWrites = dufomapEnabled || recomputeGround ||
            groundMeshEnabled ||
            (options.save_ground_map && groundGridEnabled);
        if (options.save_ground_map && groundGridEnabled &&
            !(groundRes > 0.0f))
        {
            outcome.message =
                "map_save.ground_grid.enable requires a positive resolved "
                "ground_resolution; got " +
                std::to_string(groundRes);
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }
        const std::string saveMapDirectory = options.directory.string();
        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving map to pcd files ..." << std::endl;
        std::cout << "Save destination: " << saveMapDirectory << std::endl;

        std::error_code directoryError;
        std::filesystem::create_directories(options.directory, directoryError);
        if (directoryError)
        {
            outcome.message = "Unable to create save destination " + saveMapDirectory +
                              ": " + directoryError.message();
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }

        std::unique_ptr<MapArtifactLayout> saveLayout;
        try
        {
            saveLayout = std::make_unique<MapArtifactLayout>(
                options.directory,
                config_.mapping.map_save.tiled_map.output_subdirectory);
        }
        catch (const std::exception& error)
        {
            outcome.message = "Invalid map save layout for " + saveMapDirectory +
                              ": " + error.what();
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }

        // Remove the artifacts a previous save could have written, and nothing
        // else: unrelated files in the destination directory survive.
        std::string artifactError;
        if (!deferFinalWrites &&
            !lio_sam_hesai::RemovePriorMapArtifacts(*saveLayout, &artifactError))
        {
            outcome.message = artifactError;
            RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
            return outcome;
        }
        const MapArtifactLayout& save_layout = *saveLayout;
        std::string failure;
        // Copy the keyframe metadata while mapping/loop closure is locked.
        // The feature-cloud pointers are immutable once keyframes are saved,
        // so the expensive map export can safely run after releasing mtx.
        pcl::PointCloud<PointType>::Ptr keyPoses3DSnapshot(
            new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointTypePose>::Ptr keyPoses6DSnapshot(
            new pcl::PointCloud<PointTypePose>());
        std::vector<pcl::PointCloud<PointType>::Ptr> cornerKeyFramesSnapshot;
        std::vector<pcl::PointCloud<PointType>::Ptr> surfKeyFramesSnapshot;
        std::vector<pcl::PointCloud<PointType>::Ptr> groundKeyFramesSnapshot;
        std::size_t keyFrameCount = 0;
        std::size_t groundFrameCount = 0;
        std::string scanCacheErrorSnapshot;
        std::string groundCacheErrorSnapshot;
        std::string completeCacheErrorSnapshot;
        std::string onlineDufomapErrorSnapshot;
        lio_sam_hesai::DufomapSnapshotRequest dufomapRequest{};
        bool haveDufomapRequest = false;
        {
            std::lock_guard<std::mutex> lock(mtx);
            scanCacheErrorSnapshot = dufomapCacheError_;
            groundCacheErrorSnapshot = groundExportCacheError_;
            completeCacheErrorSnapshot = completeScanCacheError_;
            onlineDufomapErrorSnapshot = online_dufomap_startup_error_;
            *keyPoses3DSnapshot = *cloudKeyPoses3D;
            *keyPoses6DSnapshot = *cloudKeyPoses6D;
            // Keep the historical corner/surface keyframe count. Ground is an
            // independent, optional stream: a session without ground must not
            // truncate the feature map to zero, and a shorter ground history
            // must not hide later corner/surface keyframes.
            keyFrameCount = std::min(
                std::min(keyPoses3DSnapshot->size(), keyPoses6DSnapshot->size()),
                std::min(cornerCloudKeyFrames.size(), surfCloudKeyFrames.size()));
            groundFrameCount =
                std::min(keyFrameCount, groundCloudKeyFrames.size());
            keyPoses3DSnapshot->resize(keyFrameCount);
            keyPoses6DSnapshot->resize(keyFrameCount);
            cornerKeyFramesSnapshot.assign(cornerCloudKeyFrames.begin(),
                                           cornerCloudKeyFrames.begin() + keyFrameCount);
            surfKeyFramesSnapshot.assign(surfCloudKeyFrames.begin(),
                                         surfCloudKeyFrames.begin() + keyFrameCount);
            groundKeyFramesSnapshot.assign(
                groundCloudKeyFrames.begin(),
                groundCloudKeyFrames.begin() + groundFrameCount);
            // Capture the worker request under the same lock as the poses and
            // keyframe clouds. A loop correction that arrives after this point
            // must not hand back labels from a newer generation.
            if (dufomapEnabled && online_dufomap_)
            {
                dufomapRequest =
                    online_dufomap_->CaptureRequest(keyFrameCount);
                haveDufomapRequest = true;
            }
        }

        // Do not hold mtx while waiting for the online worker. The keyframes,
        // poses, and exact snapshot generation are already frozen; any failure
        // here returns before a prior map artifact is touched.
        std::shared_ptr<const lio_sam_hesai::LabelSnapshot> dufomapLabels;
        std::shared_ptr<const lio_sam_hesai::KeyframeScanExport>
            completeScanExport;
        if (dufomapEnabled)
        {
            std::string pendingWriteError;
            if (!WaitForPendingCompleteScanWrites(std::chrono::seconds(30),
                                                  &pendingWriteError))
            {
                outcome.message = pendingWriteError;
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            {
                std::lock_guard<std::mutex> lock(mtx);
                completeCacheErrorSnapshot = completeScanCacheError_;
            }
            if (!completeCacheErrorSnapshot.empty())
            {
                outcome.message = completeCacheErrorSnapshot;
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            if (!online_dufomap_)
            {
                outcome.message = onlineDufomapErrorSnapshot.empty()
                    ? "online DUFOMap is enabled but the worker is unavailable"
                    : onlineDufomapErrorSnapshot;
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            if (!haveDufomapRequest)
            {
                outcome.message =
                    "online DUFOMap snapshot request was not captured with the frozen keyframes";
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            const auto labels = online_dufomap_->Barrier(
                dufomapRequest, std::chrono::seconds(30));
            if (!labels.has_value())
            {
                outcome.message = "online DUFOMap save barrier failed: " +
                                  online_dufomap_->last_error();
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            dufomapLabels = *labels;
            if (dufomapLabels->keyframe_generation !=
                dufomapRequest.keyframe_generation)
            {
                outcome.message =
                    "online DUFOMap snapshot generation advanced during save "
                    "(requested " +
                    std::to_string(dufomapRequest.keyframe_generation) +
                    ", received " +
                    std::to_string(dufomapLabels->keyframe_generation) +
                    "); refusing to mix old map poses with new labels";
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }
            std::string labelError;
            if (!ValidateDufomapLabels(*dufomapLabels, keyFrameCount,
                                       cornerKeyFramesSnapshot,
                                       surfKeyFramesSnapshot, &labelError))
            {
                outcome.message = labelError;
                RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                return outcome;
            }

            // The complete-scan query happens before any prior artifact is
            // removed. A missing/corrupt cache or generation change therefore
            // leaves the previous map untouched.
            std::vector<int> completeScanIndices;
            completeScanIndices.reserve(keyFrameCount);
            for (std::size_t index = 0; index < keyFrameCount; ++index)
                completeScanIndices.push_back(static_cast<int>(index));
            if (!completeScanIndices.empty())
            {
                const auto exported = online_dufomap_->ExportKeyframeScans(
                    dufomapRequest, completeScanIndices, std::chrono::seconds(30));
                if (!exported.has_value())
                {
                    outcome.message =
                        "online DUFOMap complete keyframe scan export failed: " +
                        online_dufomap_->last_error();
                    RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                    return outcome;
                }
                completeScanExport = *exported;
                if (completeScanExport->keyframe_generation !=
                    dufomapRequest.keyframe_generation)
                {
                    outcome.message =
                        "online DUFOMap complete scan export returned a newer "
                        "generation; refusing to mix it with the frozen poses";
                    RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                    return outcome;
                }
                // The backend may have integrated more evidence since the
                // first barrier. Use the feature labels paired with the full
                // scan query so all exported maps use the same evidence.
                dufomapLabels = completeScanExport->feature_labels;
                if (!dufomapLabels ||
                    dufomapLabels->keyframe_generation !=
                        completeScanExport->keyframe_generation ||
                    dufomapLabels->evidence_sequence !=
                        completeScanExport->evidence_sequence ||
                    !ValidateDufomapLabels(*dufomapLabels, keyFrameCount,
                                           cornerKeyFramesSnapshot,
                                           surfKeyFramesSnapshot, &labelError))
                {
                    outcome.message = labelError.empty()
                        ? "online DUFOMap complete scan export has no matching feature labels"
                        : labelError;
                    RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                    return outcome;
                }
            }
            else
            {
                auto emptyExport =
                    std::make_shared<lio_sam_hesai::KeyframeScanExport>();
                emptyExport->keyframe_generation =
                    dufomapRequest.keyframe_generation;
                emptyExport->evidence_sequence =
                    dufomapLabels->evidence_sequence;
                completeScanExport = std::move(emptyExport);
            }
        }

        auto saveTrajectory = [&]() {
            // Save key frame transformations from the same snapshot used below.
            if (pcl::io::savePCDFileBinary(save_layout.Trajectory().string(), *keyPoses3DSnapshot) != 0 &&
                failure.empty())
                failure = "Failed to save " + save_layout.Trajectory().string();
            if (pcl::io::savePCDFileBinary(save_layout.Transformations().string(), *keyPoses6DSnapshot) != 0 &&
                failure.empty())
                failure = "Failed to save " + save_layout.Transformations().string();
        };
        if (!deferFinalWrites)
            saveTrajectory();
        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());
        // When online DUFOMap is enabled, GlobalMap is the complete deskewed
        // keyframe map filtered point-by-point by the worker. Corner/surface
        // features keep their own export path and never leak into this cloud.
        pcl::PointCloud<PointType>::Ptr globalCompleteCloud(
            new pcl::PointCloud<PointType>());
        if (dufomapEnabled && completeScanExport)
        {
            globalCompleteCloud->reserve(completeScanExport->points.size());
            for (const auto& point : completeScanExport->points)
            {
                PointType output;
                output.x = point.x;
                output.y = point.y;
                output.z = point.z;
                output.intensity = point.intensity;
                globalCompleteCloud->push_back(output);
            }
            completeScanExport.reset();
        }
        const bool useStaticFilter = config_.mapping.static_map_filter.enable && config_.mapping.static_map_filter.voxel_size > 1e-3f && config_.mapping.static_map_filter.min_keyframes > 1;
        std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash> voxSurf;
        std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash> voxCorner;

        size_t rawSurfPts = 0;
        size_t rawCornerPts = 0;

        if (useStaticFilter)
        {
            // Reserve a rough amount to reduce rehash; tune if needed.
            voxSurf.reserve((size_t)keyPoses3DSnapshot->size() * 2000);
            if (config_.mapping.static_map_filter.filter_corners)
                voxCorner.reserve((size_t)keyPoses3DSnapshot->size() * 400);
        }

        relocalization::ScanContext scanContextGenerator;
        const bool generateScanContextDb = config_.mapping.map_save.generate_scan_context_db;
        for (int i = 0; i < (int)keyPoses3DSnapshot->size(); i++)
        {
            if (cornerKeyFramesSnapshot[i] == nullptr ||
                surfKeyFramesSnapshot[i] == nullptr)
            {
                RCLCPP_WARN(get_logger(),
                            "Skipping keyframe %d with missing feature cloud", i);
                continue;
            }
            pcl::PointCloud<PointType>::Ptr cornerForMap = cornerKeyFramesSnapshot[i];
            pcl::PointCloud<PointType>::Ptr surfForMap = surfKeyFramesSnapshot[i];
            if (dufomapEnabled)
            {
                const auto& labels = dufomapLabels->labels[i];
                cornerForMap = FilterKeyframeCloud(cornerKeyFramesSnapshot[i], &labels.corner);
                surfForMap = FilterKeyframeCloud(surfKeyFramesSnapshot[i], &labels.surf);
                if (!cornerForMap || !surfForMap)
                {
                    outcome.message =
                        "online DUFOMap label application failed at keyframe " +
                        std::to_string(i) +
                        "; refusing to save an empty or partial map";
                    RCLCPP_ERROR(get_logger(), "%s", outcome.message.c_str());
                    return outcome;
                }
            }
            pcl::PointCloud<PointType>::Ptr cornerT = transformPointCloud(cornerForMap,  &keyPoses6DSnapshot->points[i]);
            pcl::PointCloud<PointType>::Ptr surfT   = transformPointCloud(surfForMap,    &keyPoses6DSnapshot->points[i]);

            if (generateScanContextDb)
            {
                pcl::PointCloud<PointType> localFeatureCloud;
                localFeatureCloud.reserve(cornerKeyFramesSnapshot[i]->size() +
                                          surfKeyFramesSnapshot[i]->size());
                localFeatureCloud += *cornerKeyFramesSnapshot[i];
                localFeatureCloud += *surfKeyFramesSnapshot[i];
                if (!localFeatureCloud.empty())
                {
                    Eigen::Matrix4d keyFramePose =
                        pclPointToAffine3f(keyPoses6DSnapshot->points[i])
                            .cast<double>()
                            .matrix();
                    if (config_.mapping.map_save.align_ground_to_zero)
                        keyFramePose(2, 3) += config_.mapping.map_save.ground_z_offset;
                    scanContextGenerator.AddKeyFrame(
                        scanContextGenerator.MakeDescriptor(localFeatureCloud),
                        keyFramePose);
                }
            }

            rawCornerPts += cornerT->size();
            rawSurfPts   += surfT->size();

            if (useStaticFilter)
            {
                if (config_.mapping.static_map_filter.filter_corners)
                    accumulateVoxelMap(voxCorner, cornerT, i, config_.mapping.static_map_filter.voxel_size);
                else
                    *globalCornerCloud += *cornerT;

                // 对surf做静态一致性筛选
                accumulateVoxelMap(voxSurf, surfT, i, config_.mapping.static_map_filter.voxel_size);
            }
            else
            {
                *globalCornerCloud += *cornerT;
                *globalSurfCloud   += *surfT;
            }

            std::cout << "" << std::flush << "Processing feature cloud " << i << " of " << keyPoses6DSnapshot->size() << " ..." << std::endl;
        }

        if (useStaticFilter)
        {
            // 把大于等于minKF次观测的体素输出成点
            if (config_.mapping.static_map_filter.filter_corners)
                globalCornerCloud = voxelMapToCloud(voxCorner, config_.mapping.static_map_filter.min_keyframes);

            globalSurfCloud = voxelMapToCloud(voxSurf, config_.mapping.static_map_filter.min_keyframes);

            std::cout
                << "Static-map filter: voxel=" << std::fixed << std::setprecision(2)
                << config_.mapping.static_map_filter.voxel_size << " m, minKF=" << config_.mapping.static_map_filter.min_keyframes
                << ", filterCorner=" << (config_.mapping.static_map_filter.filter_corners ? "true" : "false")
                << " | corner " << rawCornerPts << " -> " << globalCornerCloud->size()
                << " | surf "   << rawSurfPts   << " -> " << globalSurfCloud->size()
            << std::endl;
        }

        pcl::PointCloud<PointType>::Ptr preparedGround;
        if (options.save_ground_map)
        {
            if (recomputeGround)
            {
                if (!RecomputeExportGround(keyPoses6DSnapshot, keyFrameCount,
                                           scanCacheErrorSnapshot, groundCacheErrorSnapshot,
                                           groundRes, preparedGround,
                                           &outcome.message))
                    return outcome;
            }
            else
                preparedGround = BuildGroundFromSnapshot(
                    groundKeyFramesSnapshot, keyPoses6DSnapshot);
            try
            {
                if (groundGridEnabled)
                {
                    lio_sam_hesai::GroundGridExportOptions gridOptions;
                    gridOptions.resolution = groundRes;
                    gridOptions.search_radius =
                        config_.mapping.ground_segmentation.grid_patch.search_radius;
                    gridOptions.max_height_range =
                        config_.mapping.ground_segmentation.grid_patch.max_height_range;
                    gridOptions.max_roughness =
                        config_.mapping.ground_segmentation.grid_patch.max_roughness;
                    gridOptions.plane_distance_threshold =
                        config_.mapping.ground_segmentation.plane_distance;
                    gridOptions.max_slope_deg =
                        config_.mapping.ground_segmentation.grid_patch.max_slope_deg;
                    gridOptions.max_prediction_margin =
                        config_.mapping.ground_segmentation.grid_patch.max_prediction_margin;
                    gridOptions.min_plane_inlier_ratio =
                        config_.mapping.ground_segmentation.grid_patch.min_inlier_ratio;
                    gridOptions.min_neighbor_cells =
                        config_.mapping.ground_segmentation.grid_patch.min_neighbor_cells;
                    gridOptions.min_support_quadrants =
                        config_.mapping.ground_segmentation.grid_patch.min_support_quadrants;
                    gridOptions.max_hole_cells =
                        config_.mapping.map_save.ground_grid.max_hole_cells;
                    gridOptions.fill_unobserved_gaps =
                        config_.mapping.ground_segmentation.grid_patch.enable;
                    pcl::PointCloud<lio_sam_hesai::GroundGridPointType> gridded;
                    std::string gridError;
                    if (!lio_sam_hesai::BuildGroundGrid(
                            *preparedGround, gridOptions, &gridded, &gridError))
                    {
                        outcome.message =
                            "Ground grid export failed: " + gridError;
                        return outcome;
                    }
                    preparedGround->swap(gridded);
                }
                else
                {
                    appendGlobalGroundGridPatches(preparedGround);
                    preparedGround = downsampleGroundMapCloud(preparedGround, groundRes);
                }
                shiftSavedMapCloudZ(preparedGround);
                flattenGroundCloudForSave(preparedGround);
            }
            catch (const std::exception& exception)
            {
                outcome.message = std::string("Ground export preprocessing failed: ") + exception.what();
                return outcome;
            }
        }

        if (deferFinalWrites)
        {
            if (!lio_sam_hesai::RemovePriorMapArtifacts(save_layout, &artifactError))
            {
                outcome.message = artifactError;
                return outcome;
            }
            saveTrajectory();
        }

        shiftSavedMapCloudZ(globalCornerCloud);
        shiftSavedMapCloudZ(globalSurfCloud);
        // The complete cloud is already in map coordinates after the worker's
        // classification; only now apply the existing save Z offset.
        shiftSavedMapCloudZ(globalCompleteCloud);
        if (config_.mapping.map_save.align_ground_to_zero)
        {
            std::cout << "Saved map Z offset: " << config_.mapping.map_save.ground_z_offset
                 << " m (align ground to map z=0)" << std::endl;
        }

        const float resolution = lio_sam_hesai::ResolveMapSaveResolution(
            options.resolution, config_.mapping.map_save.resolution);
        if(resolution != 0)
        {
           std::cout << "\n\nSave resolution: " << resolution << std::endl;
           // down-sample and save corner cloud
           downSizeFilterCorner.setInputCloud(globalCornerCloud);
           downSizeFilterCorner.setLeafSize(resolution, resolution, resolution);
           downSizeFilterCorner.filter(*globalCornerCloudDS);
           std::string saveError;
           if (!SaveCloudBinary(save_layout.CornerMap(), *globalCornerCloudDS,
                                &saveError) &&
               failure.empty())
               failure = saveError;
           // down-sample and save surf cloud
           downSizeFilterSurf.setInputCloud(globalSurfCloud);
           downSizeFilterSurf.setLeafSize(resolution, resolution, resolution);
           downSizeFilterSurf.filter(*globalSurfCloudDS);
           if (!SaveCloudBinary(save_layout.SurfaceMap(), *globalSurfCloudDS,
                                &saveError) &&
               failure.empty())
               failure = saveError;
           //pcl::io::savePCDFileBinary(saveMapDirectory + "/localmap.pcd", *globalSurfCloudDS);
           // build and save GlobalMap with same resolution
           if (dufomapEnabled)
           {
               pcl::VoxelGrid<PointType> complete_filter;
               complete_filter.setLeafSize(resolution, resolution, resolution);
               complete_filter.setInputCloud(globalCompleteCloud);
               complete_filter.filter(*globalMapCloud);
               globalCompleteCloud.reset();
           }
           else
           {
               *globalMapCloud += *globalCornerCloudDS;
               *globalMapCloud += *globalSurfCloudDS;
           }
        }
        else
        {
           // save corner cloud
           std::string saveError;
           if (!SaveCloudBinary(save_layout.CornerMap(), *globalCornerCloud,
                                &saveError) &&
               failure.empty())
               failure = saveError;
           // save surf cloud
           if (!SaveCloudBinary(save_layout.SurfaceMap(), *globalSurfCloud,
                                &saveError) &&
               failure.empty())
               failure = saveError;
           // build GlobalMap without downsampling
           if (dufomapEnabled)
               globalMapCloud = std::move(globalCompleteCloud);
           else
           {
               *globalMapCloud += *globalCornerCloud;
               *globalMapCloud += *globalSurfCloud;
           }
        }
        std::string globalSaveError;
        const bool globalMapSaved =
            SaveCloudBinary(save_layout.GlobalMap(), *globalMapCloud,
                            &globalSaveError);
        if (!globalMapSaved && failure.empty())
            failure = globalSaveError;
        bool scanContextDbSaved = !generateScanContextDb;
        if (generateScanContextDb)
        {
            const std::string scanContextPath =
                save_layout.ScanContextDb().string();
            scanContextDbSaved =
                globalMapSaved && scanContextGenerator.Size() > 0 &&
                scanContextGenerator.SaveDB(
                    scanContextPath, MapArtifactLayout::kGlobalMapName);
            if (scanContextDbSaved)
            {
                std::cout << "Scan Context DB (" << scanContextGenerator.Size()
                     << " keyframes) saved: " << scanContextPath << std::endl;
            }
            else
            {
                if (failure.empty())
                    failure = "Failed to save Scan Context DB: " + scanContextPath;
                RCLCPP_ERROR(get_logger(),
                             "Failed to save Scan Context DB: %s",
                             scanContextPath.c_str());
            }
        }
        bool groundSaved = true;
        if (options.save_ground_map && preparedGround && !preparedGround->empty())
        {
            std::string save_error;
            groundSaved = SaveCloudBinary(save_layout.GroundMap(), *preparedGround, &save_error);
            if (!groundSaved && failure.empty()) failure = save_error;
        }

        bool tiledMapSaved = !options.save_tiled_map;
        if (options.save_tiled_map)
        {
            if (globalMapSaved && groundSaved && scanContextDbSaved)
            {
                localization_map::TileMapGeneratorOptions tileOptions;
                tileOptions.map_directory = saveMapDirectory;
                tileOptions.output_subdirectory = config_.mapping.map_save.tiled_map.output_subdirectory;
                tileOptions.tile_size_m = config_.mapping.map_save.tiled_map.tile_size_m;
                tileOptions.overview_resolution_m = config_.mapping.map_save.tiled_map.overview_resolution_m;
                tileOptions.frame_id = config_.frames.map;
                try
                {
                    const auto result = localization_map::GenerateTileMap(tileOptions);
                    tiledMapSaved = true;
                    RCLCPP_INFO(get_logger(), "Saved %zu tiled map cells to %s",
                                result.tile_count,
                                result.output_directory.c_str());
                }
                catch (const std::exception& error)
                {
                    tiledMapSaved = false;
                    if (failure.empty())
                        failure = std::string("Failed to save tiled map: ") + error.what();
                    RCLCPP_ERROR(get_logger(), "Failed to save tiled map: %s", error.what());
                }
            }
            else
            {
                tiledMapSaved = false;
                if (failure.empty())
                    failure = "Skipped tiled map because a required source artifact was not saved";
            }
        }

        downSizeFilterCorner.setLeafSize(config_.mapping.voxel_filter.mapping_corner_leaf_size, config_.mapping.voxel_filter.mapping_corner_leaf_size, config_.mapping.voxel_filter.mapping_corner_leaf_size);
        downSizeFilterSurf.setLeafSize(config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size, config_.mapping.voxel_filter.mapping_surface_leaf_size);
        std::cout << "****************************************************" << std::endl;
        std::cout << "Saving map to pcd files completed\n" << std::endl;

        outcome.success = failure.empty() && groundSaved && scanContextDbSaved && tiledMapSaved;
        if (outcome.success)
        {
            outcome.message = "Saved map to " + saveMapDirectory;
        }
        else if (!failure.empty())
        {
            outcome.message = failure;
        }
        else
        {
            outcome.message = "Map save failed";
        }
        return outcome;
    }

    std::shared_ptr<IVoxMap> CreateSurfIVoxMap() const
    {
        IVoxMap::Options options;
        options.resolution_ = std::max(
            kSurfIVoxResolution,
            static_cast<float>(config_.mapping.voxel_filter.mapping_surface_leaf_size));
        options.nearby_type_ = IVoxMap::NearbyType::NEARBY18;
        options.capacity_ = 1000000;
        options.num_nearest_points_ = kSurfMapNeighborCount;
        return std::make_shared<IVoxMap>(options);
    }

    std::shared_ptr<IVoxMap> CreateCornerIVoxMap() const
    {
        IVoxMap::Options options;
        options.resolution_ = std::max(
            kCornerIVoxResolution,
            static_cast<float>(config_.mapping.voxel_filter.mapping_corner_leaf_size));
        options.nearby_type_ = IVoxMap::NearbyType::NEARBY18;
        options.capacity_ = 1000000;
        options.num_nearest_points_ = kCornerMapNeighborCount;
        return std::make_shared<IVoxMap>(options);
    }

    void BuildIVoxMap(const pcl::PointCloud<PointType>::Ptr &sourceCloud,
                      std::shared_ptr<IVoxMap> *targetMap) const
    {
        if (targetMap == nullptr || !(*targetMap))
            return;

        IVoxMap::PointVector mapPoints;
        mapPoints.reserve(sourceCloud->size());
        for (const auto &point : sourceCloud->points)
            mapPoints.emplace_back(point);

        (*targetMap)->AddPoints(mapPoints);
    }

    void BuildCornerIVoxMap()
    {
        ivoxCornerFromMap = CreateCornerIVoxMap();
        if (laserCloudCornerFromMapDS->empty())
            return;

        BuildIVoxMap(laserCloudCornerFromMapDS, &ivoxCornerFromMap);
    }

    void BuildSurfIVoxMap()
    {
        ivoxSurfFromMap = CreateSurfIVoxMap();
        if (laserCloudSurfFromMapDS->empty())
            return;

        BuildIVoxMap(laserCloudSurfFromMapDS, &ivoxSurfFromMap);
    }

    bool GetCornerNearestNeighborsFromIVox(
        const PointType &queryPoint, IVoxMap::PointVector *nearestPoints) const
    {
        if (nearestPoints == nullptr || !ivoxCornerFromMap)
            return false;

        nearestPoints->clear();
        if (!ivoxCornerFromMap->GetClosestPoint(
                queryPoint, *nearestPoints, kCornerMapNeighborCount,
                kCornerMapNeighborMaxDistance))
        {
            return false;
        }

        return nearestPoints->size() >= kCornerMapNeighborCount;
    }

    bool GetSurfNearestNeighborsFromIVox(
        const PointType &queryPoint, IVoxMap::PointVector *nearestPoints) const
    {
        if (nearestPoints == nullptr || !ivoxSurfFromMap)
            return false;

        nearestPoints->clear();
        if (!ivoxSurfFromMap->GetClosestPoint(
                queryPoint, *nearestPoints, kSurfMapNeighborCount,
                kSurfMapNeighborMaxDistance))
        {
            return false;
        }

        return nearestPoints->size() >= kSurfMapNeighborCount;
    }

    // 使局部子图缓存失效，下一帧重新提取附近关键帧和局部地图。
    void InvalidateLocalMapCache()
    {
        localMapCacheValid = false;
        cachedLocalMapKeyIndices.clear();
    }

    // 收集当前扫描到地图优化所需的关键帧索引集合。
    void CollectLocalMapKeyIndices(std::vector<int> *keyIndices)
    {
        if (keyIndices == nullptr)
            return;

        keyIndices->clear();
        if (cloudKeyPoses3D->empty())
            return;

        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeSurroundingKeyPoses->radiusSearch(
            cloudKeyPoses3D->back(), static_cast<double>(config_.mapping.local_map.search_radius),
            pointSearchInd, pointSearchSqDis);

        surroundingKeyPoses->reserve(pointSearchInd.size());
        for (int keyIndex : pointSearchInd)
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[keyIndex]);

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);

        keyIndices->reserve(
            surroundingKeyPosesDS->size() + std::min<std::size_t>(cloudKeyPoses3D->size(), 64));
        for (auto &point : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(
                point, 1, pointSearchInd, pointSearchSqDis);
            if (!pointSearchInd.empty())
                keyIndices->push_back(pointSearchInd.front());
        }

        const int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                keyIndices->push_back(i);
            else
                break;
        }

        std::sort(keyIndices->begin(), keyIndices->end());
        keyIndices->erase(
            std::unique(keyIndices->begin(), keyIndices->end()),
            keyIndices->end());
    }

    // 判断当前附近关键帧集合是否与缓存一致。
    bool CanReuseLocalMap(const std::vector<int> &keyIndices) const
    {
        return localMapCacheValid && keyIndices == cachedLocalMapKeyIndices;
    }

    // 记录本次构建局部子图时使用的关键帧索引集合。
    void UpdateLocalMapCache(const std::vector<int> &keyIndices)
    {
        cachedLocalMapKeyIndices = keyIndices;
        localMapCacheValid = true;
    }

    // 根据关键帧索引集合拼接局部地图并重建下采样云与 iVox。
    void ExtractCloudByKeyIndices(const std::vector<int> &keyIndices)
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();

        for (int thisKeyInd : keyIndices)
        {
            if (thisKeyInd < 0 || thisKeyInd >= static_cast<int>(cloudKeyPoses3D->size()))
                continue;

            if (pointDistance(cloudKeyPoses3D->points[thisKeyInd], cloudKeyPoses3D->back()) >
                config_.mapping.local_map.search_radius)
            {
                continue;
            }

            const auto *cached = transformed_keyframe_cache_.Find(thisKeyInd);
            if (cached != nullptr)
            {
                *laserCloudCornerFromMap += cached->corner;
                *laserCloudSurfFromMap += cached->surf;
                continue;
            }

            auto corner = transformPointCloud(
                cornerCloudKeyFrames[thisKeyInd],
                &cloudKeyPoses6D->points[thisKeyInd]);
            auto surf = transformPointCloud(
                surfCloudKeyFrames[thisKeyInd],
                &cloudKeyPoses6D->points[thisKeyInd]);
            *laserCloudCornerFromMap += *corner;
            *laserCloudSurfFromMap += *surf;
            transformed_keyframe_cache_.Insert(
                thisKeyInd, std::move(*corner), std::move(*surf));
        }

        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        BuildCornerIVoxMap();

        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
        BuildSurfIVoxMap();
    }

    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>()); // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>()); // surf feature set from odoOptimization
        laserCloudGroundLast.reset(new pcl::PointCloud<PointType>()); // ground feature set from imageProjection
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);
        coeffSelCornerVec.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);
        laserCloudOriCornerFlag.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);
        laserCloudOriSurfVec.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);
        coeffSelSurfVec.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);
        laserCloudOriSurfFlag.resize(config_.mapping.lidar.scan_lines * config_.mapping.lidar.horizon_scan);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        ivoxCornerFromMap = CreateCornerIVoxMap();
        ivoxSurfFromMap = CreateSurfIVoxMap();
        InvalidateLocalMapCache();

        for (int i = 0; i < 6; ++i){
            transformTobeMapped[i] = 0;
        }

        matP.setZero();
    }

    void laserCloudInfoHandler(const lio_sam_hesai::msg::CloudInfo::SharedPtr msgIn)
    {
        // Freeze registration and evidence enqueue together for map saves.
        // The worker queue lock is short and never held during integration.
        std::unique_lock<std::mutex> lock(mtx);
        // extract time stamp
        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = stamp2Sec(msgIn->header.stamp);

        // extract info and feature cloud
        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner,  *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        pcl::fromROSMsg(msgIn->cloud_ground,  *laserCloudGroundLast);

        MaybeLogOnlineDufomapHealth();

        bool processed = false;
        bool keyframe_added = false;
        bool loop_closed = false;
        int anchor_keyframe = -1;
        lio_sam_hesai::DufomapPose anchor_T_scan = IdentityPose();
        lio_sam_hesai::DufomapPose fallback_map_T_scan = IdentityPose();
        lio_sam_hesai::DufomapPose processed_guess{};
        bool processed_guess_valid = false;
        bool cache_complete_scan = false;
        std::size_t complete_scan_index = 0;
        {
            static double timeLastProcessing = -1;
            if (timeLaserInfoCur - timeLastProcessing >= config_.mapping.system.mapping_process_interval)
            {
                timeLastProcessing = timeLaserInfoCur;
                processed = true;

                updateInitialGuess();
                extractSurroundingKeyFrames();
                downsampleCurrentScan();
                scan2MapOptimization();

                const std::size_t keyframe_count_before =
                    cloudKeyPoses3D->size();
                saveKeyFramesAndFactor();
                keyframe_added =
                    cloudKeyPoses3D->size() > keyframe_count_before;
                loop_closed = aLoopIsClosed;

                correctPoses();
                if (loop_closed)
                    UpdateOnlineDufomapPoses();

                publishOdometry();

                if (config_.mapping.system.mapping_mode)
                    publishFrames();

                // Freeze this registered scan's exact pose and anchor state
                // while the mapping data is still under the mutex. Keyframe
                // poses are the post-loop-correction values.
                if (online_dufomap_)
                {
                    const lio_sam_hesai::DufomapPose lidar_map_pose =
                        CurrentLidarMapPose();
                    fallback_map_T_scan = lidar_map_pose;
                    if (keyframe_added && !cloudKeyPoses6D->empty())
                    {
                        // This scan defines the new keyframe, so its local
                        // cloud is exactly at the keyframe pose and must move
                        // with that keyframe after a loop correction.
                        anchor_keyframe = static_cast<int>(
                            cloudKeyPoses6D->size() - 1);
                        anchor_T_scan = IdentityPose();
                        fallback_map_T_scan =
                            PoseMatrix(cloudKeyPoses6D->back());
                    }
                    else if (!cloudKeyPoses6D->empty() &&
                             IsRigidPose(lidar_map_pose))
                    {
                        const lio_sam_hesai::DufomapPose anchor_map_pose =
                            PoseMatrix(cloudKeyPoses6D->back());
                        if (IsRigidPose(anchor_map_pose))
                        {
                            const lio_sam_hesai::DufomapPose relative = RelativePose(
                                anchor_map_pose, lidar_map_pose);
                            if (IsRigidPose(relative))
                            {
                                anchor_keyframe = static_cast<int>(
                                    cloudKeyPoses6D->size() - 1);
                                anchor_T_scan = relative;
                            }
                        }
                    }
                    processed_guess = CurrentInitialGuessPose();
                    processed_guess_valid =
                        cloudInfo.odom_available &&
                        IsRigidPose(processed_guess);
                    have_last_processed_ = true;
                    last_processed_anchor_keyframe_ = anchor_keyframe;
                    last_processed_anchor_T_scan_ = anchor_T_scan;
                    last_processed_fallback_map_T_scan_ = fallback_map_T_scan;
                    last_processed_initial_guess_ = processed_guess;
                    have_last_processed_initial_guess_ = processed_guess_valid;
                }

                if (online_dufomap_ && keyframe_added)
                {
                    // Persist the complete scan independent of ground mode.
                    // The pending counter is raised before the mapping lock is
                    // released so a save can wait for this exact keyframe.
                    cache_complete_scan = true;
                    complete_scan_index = cloudKeyPoses3D->size() - 1;
                    BeginCompleteScanWrite();
                }
            }
        }

        const auto cache_complete = [&]() {
            if (!cache_complete_scan) return;
            // Register the keyframe and enqueue its evidence under mtx before
            // releasing it. Saves then capture both in the same snapshot and
            // wait only for this disk write to finish.
            lock.unlock();
            std::string cache_error;
            try
            {
                StoreCompleteKeyframeScan(
                    complete_scan_index, msgIn->cloud_export_scan, &cache_error);
            }
            catch (const std::exception& exception)
            {
                cache_error = std::string("complete keyframe scan cache failed: ") +
                              exception.what();
                std::lock_guard<std::mutex> cache_lock(mtx);
                completeScanCacheError_ = cache_error;
            }
            FinishCompleteScanWrite();
            if (!cache_error.empty())
                RCLCPP_ERROR(get_logger(),
                             "Failed to cache complete keyframe scan: %s",
                             cache_error.c_str());
        };

        if (!online_dufomap_ || msgIn->cloud_export_scan.data.empty())
        {
            cache_complete();
            return;
        }

        lio_sam_hesai::OnlineDufomapEvidence evidence;
        evidence.cloud = std::make_shared<const sensor_msgs::msg::PointCloud2>(
            msgIn->cloud_export_scan);
        evidence.fallback_map_T_scan = fallback_map_T_scan;

        const auto enqueue = [&](const lio_sam_hesai::OnlineDufomapEvidence& item) {
            if (online_dufomap_->Enqueue(item))
                return true;
            ++online_dufomap_enqueue_failures_;
            if (online_dufomap_enqueue_failures_ % 100U == 1U)
            {
                RCLCPP_WARN(get_logger(),
                            "Online DUFOMap rejected evidence (%zu failures so far)",
                            online_dufomap_enqueue_failures_);
            }
            return false;
        };

        if (processed)
        {
            // A registered scan uses the exact scan-matched pose. Its anchor
            // is always the latest keyframe, never a raw IMU-only guess.
            evidence.anchor_keyframe = anchor_keyframe;
            evidence.anchor_T_scan = anchor_T_scan;
            enqueue(evidence);
            cache_complete();
            return;
        }

        // A skipped scan was not scan-matched. Extrapolate from the last
        // registered scan's keyframe anchor with its saved IMU initial guess:
        //   anchor_T_current = anchor_T_processed *
        //                      inv(processed_guess) * current_guess
        // and accept it only if every matrix and the increment are sane.
        if (!have_last_processed_ ||
            last_processed_anchor_keyframe_ < 0 ||
            !have_last_processed_initial_guess_ ||
            !cloudInfo.odom_available)
        {
            ++online_dufomap_skipped_;
            return;
        }
        const lio_sam_hesai::DufomapPose current_guess =
            CurrentInitialGuessPose();
        if (!IsRigidPose(last_processed_initial_guess_) ||
            !IsRigidPose(current_guess))
        {
            ++online_dufomap_rejected_;
            return;
        }
        const lio_sam_hesai::DufomapPose relative = RelativePose(
            last_processed_initial_guess_, current_guess);
        if (!IsSaneRelativeIncrement(relative))
        {
            ++online_dufomap_rejected_;
            return;
        }
        const lio_sam_hesai::DufomapPose candidate = ComposePose(
            last_processed_anchor_T_scan_, relative);
        if (!IsRigidPose(candidate))
        {
            ++online_dufomap_rejected_;
            return;
        }
        evidence.anchor_keyframe = last_processed_anchor_keyframe_;
        evidence.anchor_T_scan = candidate;
        evidence.fallback_map_T_scan =
            last_processed_fallback_map_T_scan_;
        if (!enqueue(evidence))
            return;
    }

    void gpsHandler(const nav_msgs::msg::Odometry::SharedPtr gpsMsg)
    {
        gpsQueue.push_back(*gpsMsg);
    }

    void pointAssociateToMap(PointType const * const pi, PointType * const po)
    {
        po->x = transPointAssociateToMap(0,0) * pi->x + transPointAssociateToMap(0,1) * pi->y + transPointAssociateToMap(0,2) * pi->z + transPointAssociateToMap(0,3);
        po->y = transPointAssociateToMap(1,0) * pi->x + transPointAssociateToMap(1,1) * pi->y + transPointAssociateToMap(1,2) * pi->z + transPointAssociateToMap(1,3);
        po->z = transPointAssociateToMap(2,0) * pi->x + transPointAssociateToMap(2,1) * pi->y + transPointAssociateToMap(2,2) * pi->z + transPointAssociateToMap(2,3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose* transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
        
        #pragma omp parallel for num_threads(config_.mapping.system.number_of_cores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                                  gtsam::Point3(double(thisPoint.x),    double(thisPoint.y),     double(thisPoint.z)));
    }

    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]), 
                                  gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll  = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw   = transformIn[2];
        return thisPose6D;
    }

    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(1.0);
        while (rclcpp::ok()){
            rate.sleep();
            if (!config_.mapping.system.mapping_mode)
                continue;
            publishGlobalMap();
            publishGlobalGroundMap();
        }
    }

    // 保存地图时可整体平移Z，使导出的PCD地面落到map.z=0。
    void shiftSavedMapCloudZ(const pcl::PointCloud<PointType>::Ptr &cloud) const
    {
        if (!config_.mapping.map_save.align_ground_to_zero || cloud == nullptr)
            return;

        for (auto &point : cloud->points)
            point.z += config_.mapping.map_save.ground_z_offset;
    }

    // 在导出地面地图前按配置压平高度，便于与现有地面导出参数兼容。
    void flattenGroundCloudForSave(const pcl::PointCloud<PointType>::Ptr &cloud) const
    {
        if (!config_.mapping.ground_segmentation.flatten_z || cloud == nullptr)
            return;

        const float target_z = config_.mapping.map_save.align_ground_to_zero ? 0.0f : config_.mapping.ground_segmentation.flatten_value;
        for (auto &point : cloud->points)
            point.z = target_z;
    }

    void appendGlobalGroundGridPatches(pcl::PointCloud<PointType>::Ptr cloud) const
    {
        if (!config_.mapping.ground_segmentation.grid_patch.enable)
            return;

        lio_sam_hesai::GroundGridPatchOptions options;
        options.resolution = config_.mapping.ground_segmentation.grid_patch.resolution;
        options.search_radius = config_.mapping.ground_segmentation.grid_patch.search_radius;
        options.max_slope_deg = config_.mapping.ground_segmentation.grid_patch.max_slope_deg;
        options.plane_distance_threshold = config_.mapping.ground_segmentation.plane_distance;
        options.max_height_range = config_.mapping.ground_segmentation.grid_patch.max_height_range;
        options.max_roughness = config_.mapping.ground_segmentation.grid_patch.max_roughness;
        options.max_prediction_margin = config_.mapping.ground_segmentation.grid_patch.max_prediction_margin;
        options.min_plane_inlier_ratio = config_.mapping.ground_segmentation.grid_patch.min_inlier_ratio;
        options.min_neighbor_cells = config_.mapping.ground_segmentation.grid_patch.min_neighbor_cells;
        options.min_support_quadrants = config_.mapping.ground_segmentation.grid_patch.min_support_quadrants;
        lio_sam_hesai::appendGroundGridPatches(cloud, options);
    }

    pcl::PointCloud<PointType>::Ptr downsampleGroundMapCloud(
        const pcl::PointCloud<PointType>::Ptr& cloud, float leafSize) const
    {
        if (cloud == nullptr || cloud->empty() || leafSize <= 0.0f)
            return cloud;

        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> voxel;
        voxel.setLeafSize(leafSize, leafSize, leafSize);
        voxel.setInputCloud(cloud);
        voxel.filter(*filtered);
        return filtered;
    }

    pcl::PointCloud<PointType>::Ptr makeGroundKeyFrameCloud(
        const pcl::PointCloud<PointType>::Ptr& groundIn)
    {
        pcl::PointCloud<PointType>::Ptr groundLocal(
            new pcl::PointCloud<PointType>());
        if (groundIn == nullptr || groundIn->empty())
            return groundLocal;

        *groundLocal = *groundIn;
        return groundLocal;
    }

    // 根据关键帧索引拼接全局地面点云。关键帧只保存真实量测地面；
    // 高程栅格补点在全局坐标系内只执行一次，避免逐关键帧补点叠加成多层地面。
    pcl::PointCloud<PointType>::Ptr buildGlobalGroundMapFromIndices(
        const std::vector<int> &keyFrameIndices,
        float leafSize)
    {
        pcl::PointCloud<PointType>::Ptr globalGroundKeyFrames(
            new pcl::PointCloud<PointType>());
        if (keyFrameIndices.empty())
            return globalGroundKeyFrames;

        const bool useStaticFilter =
            config_.mapping.static_map_filter.enable && config_.mapping.static_map_filter.voxel_size > 1e-3f &&
            config_.mapping.static_map_filter.min_keyframes > 1;

        if (useStaticFilter)
        {
            std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash> vox;
            vox.reserve(keyFrameIndices.size() * 200);

            for (const int keyIndex : keyFrameIndices)
            {
                if (keyIndex < 0 || keyIndex >= static_cast<int>(groundCloudKeyFrames.size()) ||
                    keyIndex >= static_cast<int>(cloudKeyPoses6D->size()))
                {
                    continue;
                }

                pcl::PointCloud<PointType>::Ptr ground = transformPointCloud(
                    groundCloudKeyFrames[keyIndex], &cloudKeyPoses6D->points[keyIndex]);
                accumulateVoxelMap(vox, ground, keyIndex, config_.mapping.static_map_filter.voxel_size);
            }
            globalGroundKeyFrames = voxelMapToCloud(vox, config_.mapping.static_map_filter.min_keyframes);
        }
        else
        {
            for (const int keyIndex : keyFrameIndices)
            {
                if (keyIndex < 0 || keyIndex >= static_cast<int>(groundCloudKeyFrames.size()) ||
                    keyIndex >= static_cast<int>(cloudKeyPoses6D->size()))
                {
                    continue;
                }

                *globalGroundKeyFrames += *transformPointCloud(
                    groundCloudKeyFrames[keyIndex], &cloudKeyPoses6D->points[keyIndex]);
            }
        }

        appendGlobalGroundGridPatches(globalGroundKeyFrames);
        return downsampleGroundMapCloud(globalGroundKeyFrames, leafSize);
    }

    // 基于所有地面关键帧构建完整全局地面点云，用于最终导出。
    pcl::PointCloud<PointType>::Ptr buildFullGlobalGroundMap(float leafSize)
    {
        std::vector<int> keyFrameIndices;
        keyFrameIndices.reserve(groundCloudKeyFrames.size());
        for (int i = 0; i < static_cast<int>(groundCloudKeyFrames.size()); ++i)
            keyFrameIndices.push_back(i);

        return buildGlobalGroundMapFromIndices(keyFrameIndices, leafSize);
    }

    // 将全局地面点云保存到与全局地图相同的输出目录中。Whether ground is
    // exported at all is decided by the caller's SaveMapExportOptions toggle.
    // `source` is used by the DUFOMap export path: it holds the unfiltered
    // ground rebuilt from the frozen final-pose snapshot, so save-only grid
    // patching, downsampling, shift and flatten are applied here without
    // touching live keyframes.
    bool saveGlobalGroundMap(const std::string &filePath, float leafSize, bool useBinary,
                             const pcl::PointCloud<PointType>::Ptr &source = nullptr)
    {
        (void)useBinary;
        pcl::PointCloud<PointType>::Ptr globalGroundMap;
        if (source != nullptr)
        {
            globalGroundMap.reset(new pcl::PointCloud<PointType>(*source));
            appendGlobalGroundGridPatches(globalGroundMap);
            globalGroundMap = downsampleGroundMapCloud(globalGroundMap, leafSize);
        }
        else
        {
            globalGroundMap = buildFullGlobalGroundMap(leafSize);
        }
        if (globalGroundMap->empty())
        {
            RCLCPP_WARN(get_logger(), "Ground map is empty, skip saving: %s",
                        filePath.c_str());
            return true;
        }

        shiftSavedMapCloudZ(globalGroundMap);
        flattenGroundCloudForSave(globalGroundMap);
        std::string error;
        if (!SaveCloudBinary(filePath, *globalGroundMap, &error))
        {
            RCLCPP_WARN(get_logger(), "Failed to save ground map %s: %s",
                        filePath.c_str(), error.c_str());
            return false;
        }
        return true;
    }

    pcl::PointCloud<PointType>::Ptr buildGlobalGroundMapFromKeyPoses(
        const pcl::PointCloud<PointType>::Ptr &globalMapKeyPosesDS)
    {
        std::vector<int> keyFrameIndices;
        keyFrameIndices.reserve(globalMapKeyPosesDS->size());

        for (const auto &pose : globalMapKeyPosesDS->points)
        {
            if (pointDistance(pose, cloudKeyPoses3D->back()) >
                config_.mapping.visualization.global_map_search_radius)
            {
                continue;
            }

            keyFrameIndices.push_back(static_cast<int>(pose.intensity));
        }

        return buildGlobalGroundMapFromIndices(
            keyFrameIndices, config_.mapping.visualization.global_map_leaf_size);
    }

    void publishGlobalGroundMap()
    {
        if (!config_.mapping.system.mapping_mode)
            return;
        if (pubGroundCloudSurround->get_subscription_count() == 0)
            return;
        if (cloudKeyPoses3D->points.empty() || groundCloudKeyFrames.empty())
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());

        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;

        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(
            cloudKeyPoses3D->back(),
            config_.mapping.visualization.global_map_search_radius,
            pointSearchIndGlobalMap,
            pointSearchSqDisGlobalMap,
            0);
        mtx.unlock();

        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);

        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;
        downSizeFilterGlobalMapKeyPoses.setLeafSize(
            config_.mapping.visualization.global_map_pose_density,
            config_.mapping.visualization.global_map_pose_density,
            config_.mapping.visualization.global_map_pose_density);
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);

        for (auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        pcl::PointCloud<PointType>::Ptr globalGroundKeyFramesDS =
            buildGlobalGroundMapFromKeyPoses(globalMapKeyPosesDS);
        publishCloud(pubGroundCloudSurround, globalGroundKeyFramesDS, timeLaserInfoStamp, config_.frames.odometry);
    }

    void publishGlobalMap()
    {
        if (!config_.mapping.system.mapping_mode)
            return;
        if (pubLaserCloudSurround->get_subscription_count() == 0)
            return;

        // Freeze the exact keyframe clouds and poses this publication uses.
        // The worker label snapshot is checked against the same pose
        // generation below, so a rebuild can never mix old labels with new
        // map poses.
        std::vector<pcl::PointCloud<PointType>::Ptr> corners;
        std::vector<pcl::PointCloud<PointType>::Ptr> surfs;
        pcl::PointCloud<PointTypePose>::Ptr poses(new pcl::PointCloud<PointTypePose>());
        lio_sam_hesai::DufomapSnapshotRequest dufomapRequest{};
        rclcpp::Time mapStamp;
        const bool labelsEnabled = config_.mapping.dufomap.enable;
        {
            std::lock_guard<std::mutex> lock(mtx);
            mapStamp = timeLaserInfoStamp;
            const std::size_t count = std::min(
                std::min(cloudKeyPoses6D->size(), cornerCloudKeyFrames.size()),
                surfCloudKeyFrames.size());
            corners.assign(cornerCloudKeyFrames.begin(), cornerCloudKeyFrames.begin() + count);
            surfs.assign(surfCloudKeyFrames.begin(), surfCloudKeyFrames.begin() + count);
            poses->points.assign(cloudKeyPoses6D->points.begin(),
                                 cloudKeyPoses6D->points.begin() + count);
            if (labelsEnabled)
            {
                if (!online_dufomap_) return;
                // Capture the generation together with the frozen poses. Do
                // not re-read a fresh generation after unlocking.
                dufomapRequest = online_dufomap_->CaptureRequest(count);
            }
        }
        if (poses->points.empty())
            return;

        std::shared_ptr<const lio_sam_hesai::LabelSnapshot> labels;
        std::string labelError;
        if (labelsEnabled)
        {
            labels = online_dufomap_->LatestSnapshot();
            if (!labels ||
                labels->keyframe_generation != dufomapRequest.keyframe_generation)
            {
                // Keep the previous consistent display until the worker has
                // finished rebuilding the generation captured with these poses.
                return;
            }
            // New keyframes can arrive faster than history is classified.
            // Publish the ready prefix instead of starving RViz indefinitely.
            const std::size_t ready_count = std::min(
                poses->size(), std::min(labels->keyframe_count,
                                        labels->labels.size()));
            if (ready_count == 0) return;
            poses->resize(ready_count);
            corners.resize(ready_count);
            surfs.resize(ready_count);
            if (!ValidateDufomapLabels(*labels, poses->size(), corners, surfs,
                                       &labelError))
            {
                // Never publish a partial or raw cloud in place of the last
                // complete cleaned map.
                return;
            }
        }

        pcl::PointCloud<PointType>::Ptr key_positions(new pcl::PointCloud<PointType>());
        key_positions->reserve(poses->size());
        for (std::size_t index = 0; index < poses->points.size(); ++index)
        {
            PointType position;
            position.x = poses->points[index].x;
            position.y = poses->points[index].y;
            position.z = poses->points[index].z;
            position.intensity = static_cast<float>(index);
            key_positions->push_back(position);
        }

        pcl::KdTreeFLANN<PointType> kdtree;
        kdtree.setInputCloud(key_positions);
        std::vector<int> searched_indices;
        std::vector<float> searched_distances;
        kdtree.radiusSearch(key_positions->back(),
                            config_.mapping.visualization.global_map_search_radius,
                            searched_indices, searched_distances, 0);

        pcl::PointCloud<PointType>::Ptr nearby_poses(new pcl::PointCloud<PointType>());
        for (const int index : searched_indices)
            nearby_poses->push_back(key_positions->points[index]);
        pcl::VoxelGrid<PointType> pose_filter;
        pose_filter.setLeafSize(
            config_.mapping.visualization.global_map_pose_density,
            config_.mapping.visualization.global_map_pose_density,
            config_.mapping.visualization.global_map_pose_density);
        pose_filter.setInputCloud(nearby_poses);
        pcl::PointCloud<PointType>::Ptr nearby_poses_ds(new pcl::PointCloud<PointType>());
        pose_filter.filter(*nearby_poses_ds);

        std::vector<int> selected;
        for (auto& point : nearby_poses_ds->points)
        {
            std::vector<int> nearest(1);
            std::vector<float> distance(1);
            kdtree.nearestKSearch(point, 1, nearest, distance);
            point.intensity = static_cast<float>(nearest[0]);
            selected.push_back(nearest[0]);
        }

        pcl::PointCloud<PointType>::Ptr global_frames(new pcl::PointCloud<PointType>());
        const bool static_filter =
            config_.mapping.static_map_filter.enable &&
            config_.mapping.static_map_filter.voxel_size > 1e-3f &&
            config_.mapping.static_map_filter.min_keyframes > 1;
        std::unordered_map<VoxelIndex, VoxelAcc, VoxelIndexHash> voxels;
        if (static_filter)
            voxels.reserve(selected.size() * 200);

        for (const int index : selected)
        {
            if (index < 0 || static_cast<std::size_t>(index) >= poses->size())
                continue;
            pcl::PointCloud<PointType>::Ptr corner = corners[index];
            pcl::PointCloud<PointType>::Ptr surf = surfs[index];
            if (labelsEnabled)
            {
                const auto& keyframe_labels = labels->labels[index];
                corner = FilterKeyframeCloud(corner, &keyframe_labels.corner);
                surf = FilterKeyframeCloud(surf, &keyframe_labels.surf);
                if (!corner || !surf)
                    return;
            }
            pcl::PointCloud<PointType>::Ptr corner_world =
                transformPointCloud(corner, &poses->points[index]);
            pcl::PointCloud<PointType>::Ptr surf_world =
                transformPointCloud(surf, &poses->points[index]);
            if (static_filter)
            {
                if (config_.mapping.static_map_filter.filter_corners)
                    accumulateVoxelMap(voxels, corner_world, index,
                                       config_.mapping.static_map_filter.voxel_size);
                accumulateVoxelMap(voxels, surf_world, index,
                                   config_.mapping.static_map_filter.voxel_size);
            }
            else
            {
                *global_frames += *corner_world;
                *global_frames += *surf_world;
            }
        }
        if (static_filter)
            global_frames = voxelMapToCloud(
                voxels, config_.mapping.static_map_filter.min_keyframes);

        pcl::PointCloud<PointType>::Ptr global_frames_ds(new pcl::PointCloud<PointType>());
        pcl::VoxelGrid<PointType> frame_filter;
        frame_filter.setLeafSize(
            config_.mapping.visualization.global_map_leaf_size,
            config_.mapping.visualization.global_map_leaf_size,
            config_.mapping.visualization.global_map_leaf_size);
        frame_filter.setInputCloud(global_frames);
        frame_filter.filter(*global_frames_ds);

        publishCloud(pubLaserCloudSurround, global_frames_ds, mapStamp,
                     config_.frames.odometry);
    }

    void loopClosureThread()
    {
        if (loop_closure_enabled_ == false)
            return;

        rclcpp::Rate rate(config_.mapping.loop_closure.frequency);
        while (rclcpp::ok())
        {
            rate.sleep();
            performLoopClosure();
            visualizeLoopClosure();
        }
    }

    void loopInfoHandler(const std_msgs::msg::Float64MultiArray::SharedPtr loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;

        loopInfoVec.push_back(*loopMsg);

        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        mtx.lock();
        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
        mtx.unlock();

        // find keys
        int loopKeyCur;
        int loopKeyPre;
        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;

        // extract cloud
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);
            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, config_.mapping.loop_closure.history_search_count);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames->get_subscription_count() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, config_.frames.odometry);
        }

        // ICP Settings
        static pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(config_.mapping.loop_closure.history_search_radius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > config_.mapping.loop_closure.fitness_threshold)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, config_.frames.odometry);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        float noiseScore = icp.getFitnessScore();
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(std::make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }

    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // check loop constraint added before
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), config_.mapping.loop_closure.history_search_radius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > config_.mapping.loop_closure.history_search_time_difference)
            {
                loopKeyPre = id;
                break;
            }
        }

        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        // this function is not used yet, please ignore it
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;

        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();

        if (abs(loopTimeCur - loopTimePre) < config_.mapping.loop_closure.history_search_time_difference)
            return false;

        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        // latest key
        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        if (loopKeyCur == loopKeyPre)
            return false;

        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;
            *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[keyNear]);
        }

        if (nearKeyframes->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;

        visualization_msgs::msg::MarkerArray markerArray;
        // loop nodes
        visualization_msgs::msg::Marker markerNode;
        markerNode.header.frame_id = config_.frames.odometry;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::msg::Marker::ADD;
        markerNode.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3; 
        markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
        markerNode.color.a = 1;
        // loop edges
        visualization_msgs::msg::Marker markerEdge;
        markerEdge.header.frame_id = config_.frames.odometry;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::msg::Marker::ADD;
        markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
        markerEdge.color.a = 1;

        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::msg::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);
        pubLoopConstraintEdge->publish(markerArray);
    }







    



    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imu_roll_init;
            transformTobeMapped[1] = cloudInfo.imu_pitch_init;
            transformTobeMapped[2] = cloudInfo.imu_yaw_init;

            if (!config_.mapping.gps.use_imu_heading_initialization)
                transformTobeMapped[2] = 0;

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odom_available == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(
                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            } else {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;

                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imu_available == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                          transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imu_roll_init, cloudInfo.imu_pitch_init, cloudInfo.imu_yaw_init); // save imu before return;
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses-1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= config_.mapping.loop_closure.surrounding_keyframe_size)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        std::vector<int> localMapKeyIndices;
        CollectLocalMapKeyIndices(&localMapKeyIndices);
        if (CanReuseLocalMap(localMapKeyIndices))
            return;

        ExtractCloudByKeyIndices(localMapKeyIndices);
        UpdateLocalMapCache(localMapKeyIndices);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        // fuse the map
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear(); 
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > config_.mapping.local_map.search_radius)
                continue;

            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            const auto *cached = transformed_keyframe_cache_.Find(thisKeyInd);
            if (cached != nullptr)
            {
                // transformed cloud available
                *laserCloudCornerFromMap += cached->corner;
                *laserCloudSurfFromMap   += cached->surf;
            } else {
                // transformed cloud not available
                auto corner = transformPointCloud(
                    cornerCloudKeyFrames[thisKeyInd],
                    &cloudKeyPoses6D->points[thisKeyInd]);
                auto surf = transformPointCloud(
                    surfCloudKeyFrames[thisKeyInd],
                    &cloudKeyPoses6D->points[thisKeyInd]);
                *laserCloudCornerFromMap += *corner;
                *laserCloudSurfFromMap   += *surf;
                transformed_keyframe_cache_.Insert(
                    thisKeyInd, std::move(*corner), std::move(*surf));
            }
            
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        BuildCornerIVoxMap();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();
        BuildSurfIVoxMap();
    }

    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return; 
        
        // if (config_.mapping.loop_closure.enable == true)
        // {
        //     extractForLoopClosure();    
        // } else {
        //     extractNearby();
        // }

        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(config_.mapping.system.number_of_cores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            IVoxMap::PointVector nearestPoints;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            if (!GetCornerNearestNeighborsFromIVox(pointSel, &nearestPoints))
                continue;

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            float cx = 0.0f;
            float cy = 0.0f;
            float cz = 0.0f;
            for (std::size_t j = 0; j < kCornerMapNeighborCount; ++j) {
                cx += nearestPoints[j].x;
                cy += nearestPoints[j].y;
                cz += nearestPoints[j].z;
            }
            cx /= static_cast<float>(kCornerMapNeighborCount);
            cy /= static_cast<float>(kCornerMapNeighborCount);
            cz /= static_cast<float>(kCornerMapNeighborCount);

            float a11 = 0.0f;
            float a12 = 0.0f;
            float a13 = 0.0f;
            float a22 = 0.0f;
            float a23 = 0.0f;
            float a33 = 0.0f;
            for (std::size_t j = 0; j < kCornerMapNeighborCount; ++j) {
                float ax = nearestPoints[j].x - cx;
                float ay = nearestPoints[j].y - cy;
                float az = nearestPoints[j].z - cz;

                a11 += ax * ax;
                a12 += ax * ay;
                a13 += ax * az;
                a22 += ay * ay;
                a23 += ay * az;
                a33 += az * az;
            }
            a11 /= static_cast<float>(kCornerMapNeighborCount);
            a12 /= static_cast<float>(kCornerMapNeighborCount);
            a13 /= static_cast<float>(kCornerMapNeighborCount);
            a22 /= static_cast<float>(kCornerMapNeighborCount);
            a23 /= static_cast<float>(kCornerMapNeighborCount);
            a33 /= static_cast<float>(kCornerMapNeighborCount);

            matA1.at<float>(0, 0) = a11;
            matA1.at<float>(0, 1) = a12;
            matA1.at<float>(0, 2) = a13;
            matA1.at<float>(1, 0) = a12;
            matA1.at<float>(1, 1) = a22;
            matA1.at<float>(1, 2) = a23;
            matA1.at<float>(2, 0) = a13;
            matA1.at<float>(2, 1) = a23;
            matA1.at<float>(2, 2) = a33;

            cv::eigen(matA1, matD1, matV1);

            if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                float x0 = pointSel.x;
                float y0 = pointSel.y;
                float z0 = pointSel.z;
                float x1 = cx + 0.1f * matV1.at<float>(0, 0);
                float y1 = cy + 0.1f * matV1.at<float>(0, 1);
                float z1 = cz + 0.1f * matV1.at<float>(0, 2);
                float x2 = cx - 0.1f * matV1.at<float>(0, 0);
                float y2 = cy - 0.1f * matV1.at<float>(0, 1);
                float z2 = cz - 0.1f * matV1.at<float>(0, 2);

                float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)) * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

                float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

                float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                          + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

                float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                           + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

                float ld2 = a012 / l12;

                float s = 1 - 0.9f * fabs(ld2);

                coeff.x = s * la;
                coeff.y = s * lb;
                coeff.z = s * lc;
                coeff.intensity = s * ld2;

                if (s > 0.1f) {
                    laserCloudOriCornerVec[i] = pointOri;
                    coeffSelCornerVec[i] = coeff;
                    laserCloudOriCornerFlag[i] = true;
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

        #pragma omp parallel for num_threads(config_.mapping.system.number_of_cores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            IVoxMap::PointVector nearestPoints;

            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            if (!GetSurfNearestNeighborsFromIVox(pointSel, &nearestPoints))
                continue;

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            for (std::size_t j = 0; j < kSurfMapNeighborCount; ++j) {
                matA0(static_cast<int>(j), 0) = nearestPoints[j].x;
                matA0(static_cast<int>(j), 1) = nearestPoints[j].y;
                matA0(static_cast<int>(j), 2) = nearestPoints[j].z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            bool planeValid = true;
            for (std::size_t j = 0; j < kSurfMapNeighborCount; ++j) {
                if (fabs(pa * nearestPoints[j].x +
                         pb * nearestPoints[j].y +
                         pc * nearestPoints[j].z + pd) > 0.2f) {
                    planeValid = false;
                    break;
                }
            }

            if (planeValid) {
                float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointOri.x * pointOri.x
                        + pointOri.y * pointOri.y + pointOri.z * pointOri.z));

                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;

                if (s > 0.1) {
                    laserCloudOriSurfVec[i] = pointOri;
                    coeffSelSurfVec[i] = coeff;
                    laserCloudOriSurfFlag[i] = true;
                }
            }
        }
    }

    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i){
            if (laserCloudOriCornerFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i){
            if (laserCloudOriSurfFlag[i] == true){
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50) {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++) {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0) {

            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--) {
                if (matE.at<float>(0, i) < eignThre[i]) {
                    for (int j = 0; j < 6; j++) {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                } else {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
                            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
                            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
                            pow(matX.at<float>(3, 0) * 100, 2) +
                            pow(matX.at<float>(4, 0) * 100, 2) +
                            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05) {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (laserCloudCornerLastDSNum > config_.mapping.feature_extraction.min_edge_features && laserCloudSurfLastDSNum > config_.mapping.feature_extraction.min_surface_features)
        {
            for (int iterCount = 0; iterCount < 15; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;              
            }

            transformUpdate();
        } else {
            RCLCPP_WARN(get_logger(), "Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imu_available == true)
        {
            if (std::abs(cloudInfo.imu_pitch_init) < 1.4)
            {
                double imuWeight = config_.mapping.imu.rpy_weight;
                tf2::Quaternion imuQuaternion;
                tf2::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }

        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], config_.mapping.motion_constraint.rotation_tolerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], config_.mapping.motion_constraint.rotation_tolerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], config_.mapping.motion_constraint.z_tolerance);

        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;

        if (sensor == SensorType::LIVOX)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->back().time > 1.0)
                return true;
        }

        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5], 
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);

        if (abs(roll)  < config_.mapping.local_map.keyframe_angle_threshold &&
            abs(pitch) < config_.mapping.local_map.keyframe_angle_threshold &&
            abs(yaw)   < config_.mapping.local_map.keyframe_angle_threshold &&
            sqrt(x*x + y*y + z*z) < config_.mapping.local_map.keyframe_distance_threshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }else{
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo   = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size()-1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (gpsQueue.empty())
            return;

        // wait for system initialized and settles down
        if (cloudKeyPoses3D->points.empty())
            return;
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
                return;
        }

        // pose covariance small, no need to correct
        if (poseCovariance(3,3) < config_.mapping.gps.pose_covariance_threshold && poseCovariance(4,4) < config_.mapping.gps.pose_covariance_threshold)
            return;

        // last gps position
        static PointType lastGPSPoint;

        while (!gpsQueue.empty())
        {
            if (stamp2Sec(gpsQueue.front().header.stamp) < timeLaserInfoCur - 0.2)
            {
                // message too old
                gpsQueue.pop_front();
            }
            else if (stamp2Sec(gpsQueue.front().header.stamp) > timeLaserInfoCur + 0.2)
            {
                // message too new
                break;
            }
            else
            {
                nav_msgs::msg::Odometry thisGPS = gpsQueue.front();
                gpsQueue.pop_front();

                // GPS too noisy, skip
                float noise_x = thisGPS.pose.covariance[0];
                float noise_y = thisGPS.pose.covariance[7];
                float noise_z = thisGPS.pose.covariance[14];
                if (noise_x > config_.mapping.gps.covariance_threshold || noise_y > config_.mapping.gps.covariance_threshold)
                    continue;
                float gps_x = thisGPS.pose.pose.position.x;
                float gps_y = thisGPS.pose.pose.position.y;
                float gps_z = thisGPS.pose.pose.position.z;
                if (!config_.mapping.gps.use_elevation)
                {
                    gps_z = transformTobeMapped[5];
                    noise_z = 0.01;
                }

                // GPS not properly initialized (0,0,0)
                if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                    continue;

                // Add GPS every a few meters
                PointType curGPSPoint;
                curGPSPoint.x = gps_x;
                curGPSPoint.y = gps_y;
                curGPSPoint.z = gps_z;
                if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                    continue;
                else
                    lastGPSPoint = curGPSPoint;

                gtsam::Vector Vector3(3);
                Vector3 << std::max(noise_x, 1.0f), std::max(noise_y, 1.0f), std::max(noise_z, 1.0f);
                noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(Vector3);
                gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);
                gtSAMgraph.add(gps_factor);

                aLoopIsClosed = true;
                break;
            }
        }
    }

    void addLoopFactor()
    {
        if (loopIndexQueue.empty())
            return;

        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
        }

        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    pcl::PointCloud<PointType>::Ptr makeSurfKeyFrameCloud()
    {
        pcl::PointCloud<PointType>::Ptr out(new pcl::PointCloud<PointType>());

        const bool canFilter =
            config_.mapping.dynamic_filter.enable &&
            (cloudKeyPoses3D->size() > 3) &&
            (laserCloudSurfFromMapDS->size() > 200);

        if (!canFilter) {
            pcl::copyPointCloud(*laserCloudSurfLastDS, *out);
            return out;
        }

        // 保证坐标变换矩阵是最新的
        updatePointAssociateToMap();

        const float knnMaxSq = config_.mapping.dynamic_filter.surface_knn_max_distance * config_.mapping.dynamic_filter.surface_knn_max_distance;
        IVoxMap::PointVector nearestPoints;

        out->reserve(laserCloudSurfLastDS->size());

        size_t seen_no_match = 0;
        size_t added_no_match = 0;
        size_t kept_match_pass = 0;
        size_t dropped = 0;

        pcl::PointCloud<PointType>::Ptr noMatchCloud(new pcl::PointCloud<PointType>());
        noMatchCloud->reserve(laserCloudSurfLastDS->size());

        // 经验值：至少保留原始 surfDS 的 50%（你这帧总数大约 3000，就先设 1500）
        const size_t minKept = (size_t)std::max(500.0, 0.50 * (double)laserCloudSurfLastDS->size());

        for (int i = 0; i < (int)laserCloudSurfLastDS->size(); ++i) {
            PointType pointOri = laserCloudSurfLastDS->points[i];
            PointType pointSel;
            pointAssociateToMap(&pointOri, &pointSel);

            nearestPoints.clear();
            const bool found = ivoxSurfFromMap &&
                               ivoxSurfFromMap->GetClosestPoint(
                                   pointSel, nearestPoints, kSurfMapNeighborCount,
                                   config_.mapping.dynamic_filter.surface_knn_max_distance);
            float maxNeighborSqDist = 0.0f;
            for (const auto &neighborPoint : nearestPoints) {
                const float dx = pointSel.x - neighborPoint.x;
                const float dy = pointSel.y - neighborPoint.y;
                const float dz = pointSel.z - neighborPoint.z;
                maxNeighborSqDist = std::max(
                    maxNeighborSqDist, dx * dx + dy * dy + dz * dz);
            }

            // 新区域/局部地图稀疏：先放行，避免阻断地图扩展
            if (!found || nearestPoints.size() < kSurfMapNeighborCount ||
                maxNeighborSqDist > knnMaxSq) {
                seen_no_match++;
                noMatchCloud->push_back(pointOri);   // 先缓存
                continue;
            }

            // 5点拟合平面（复用 surfOptimization 思路）
            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;

            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();

            for (int j = 0; j < 5; j++) {
                const auto &pj = nearestPoints[j];
                matA0(j, 0) = pj.x;
                matA0(j, 1) = pj.y;
                matA0(j, 2) = pj.z;
            }

            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1.0f;

            float ps = std::sqrt(pa*pa + pb*pb + pc*pc);
            if (ps < 1e-6f) {
                dropped++;
                continue;
            }
            pa /= ps; pb /= ps; pc /= ps; pd /= ps;

            // 平面有效性检查：沿用 LIO-SAM 常用 0.2m 的一致性门槛
            bool planeValid = true;
            for (int j = 0; j < 5; j++) {
                const auto &pj = nearestPoints[j];
                float dist = std::fabs(pa*pj.x + pb*pj.y + pc*pj.z + pd);
                if (dist > 0.2f) { planeValid = false; break; }
            }
            if (!planeValid) {
                dropped++;
                // 能匹配到附近但几何不一致：更像动态/不稳定，直接不入 keyframe
                continue;
            }

            float pd2 = pa*pointSel.x + pb*pointSel.y + pc*pointSel.z + pd;

            // 最终门控：点到平面距离过大 => 不一致 => 不入 keyframe
            if (std::fabs(pd2) > config_.mapping.dynamic_filter.surface_plane_max_distance) {
                dropped++;
                continue;
            }
            kept_match_pass++;
            out->push_back(pointOri);
        }

        const size_t total = laserCloudSurfLastDS->size();

        // 缺口：还差多少点才能到 minKept
        size_t need = 0;
        if (out->size() < minKept) need = minKept - out->size();

        // 稳定阶段上限（结构丰富场景建议更严格）：最多补 3% 或 150 点
        // 说明：你 canFilter 里已经要求 cloudKeyPoses3D->size()>10，所以这里默认就是“稳定阶段”
        size_t cap = std::min((size_t)(0.03 * (double)total), (size_t)150);

        // 最终预算：只补缺口，且不超过 cap
        size_t budget = std::min(need, cap);

        if (budget > 0 && !noMatchCloud->empty()) {
            pcl::PointCloud<PointType>::Ptr noMatchDS(new pcl::PointCloud<PointType>());
            pcl::VoxelGrid<PointType> vg;

            float leaf = config_.mapping.voxel_filter.mapping_surface_leaf_size * 4.0f;   // 强下采样，降低动态细节残留
            vg.setLeafSize(leaf, leaf, leaf);
            vg.setInputCloud(noMatchCloud); 
            vg.filter(*noMatchDS);

            for (const auto &p : noMatchDS->points) {
                out->push_back(p);
                added_no_match++;
                if (added_no_match >= budget) break;
            }
        }

        //const size_t total = laserCloudSurfLastDS->size();
        const size_t kept  = out->size();
        const double pct_kept = (total ? 100.0 * (double)kept / (double)total : 0.0);
        const double pct_match_pass = (total ? 100.0 * (double)kept_match_pass / (double)total : 0.0);

        RCLCPP_INFO(
            this->get_logger(),
            "Surf KF filter: kept %zu/%zu (%.1f%%) | matchPass=%zu (%.1f%%) | seenNoMatch=%zu (%.1f%%) | addedNoMatch=%zu (%.1f%%) | dropped=%zu",
            kept, total, pct_kept,
            kept_match_pass, pct_match_pass,
            seen_no_match, (total ? 100.0*seen_no_match/total : 0.0),
            added_no_match, (total ? 100.0*added_no_match/total : 0.0),
            dropped
        );

        return out;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
            return;

        // odom factor
        addOdomFactor();

        // gps factor
        addGPSFactor();

        // loop factor
        addLoopFactor();

        // cout << "****************************************************" << endl;
        // gtSAMgraph.print("GTSAM Graph:\n");

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        //save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size()-1);
        // cout << "****************************************************" << endl;
        // isamCurrentEstimate.print("Current estimate: ");

        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity ; // this can be used as index
        thisPose6D.roll  = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw   = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        // cout << "****************************************************" << endl;
        // cout << "Pose covariance:" << endl;
        // cout << isam->marginalCovariance(isamCurrentEstimate.size()-1) << endl << endl;
        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size()-1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        //pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS,  *thisCornerKeyFrame);
        //pcl::copyPointCloud(*laserCloudSurfLastDS,    *thisSurfKeyFrame);
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame = makeSurfKeyFrameCloud();
        pcl::PointCloud<PointType>::Ptr thisGroundKeyFrame =
            makeGroundKeyFrameCloud(laserCloudGroundLast);

        // save key frame cloud
        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        groundCloudKeyFrames.push_back(thisGroundKeyFrame);

        // Register immutable corner/surface targets for the online worker.
        // The keyframe clouds above remain the optimization/output inputs; the
        // worker only receives copies and returns labels.
        if (online_dufomap_)
        {
            std::vector<lio_sam_hesai::DufomapPoint> corner_points;
            std::vector<lio_sam_hesai::DufomapPoint> surf_points;
            corner_points.reserve(thisCornerKeyFrame->size());
            surf_points.reserve(thisSurfKeyFrame->size());
            for (const auto& point : thisCornerKeyFrame->points)
                corner_points.push_back(lio_sam_hesai::DufomapPoint{
                    point.x, point.y, point.z, point.intensity});
            for (const auto& point : thisSurfKeyFrame->points)
                surf_points.push_back(lio_sam_hesai::DufomapPoint{
                    point.x, point.y, point.z, point.intensity});
            online_dufomap_->RegisterKeyframe(
                static_cast<int>(cloudKeyPoses3D->size() - 1),
                std::move(corner_points), std::move(surf_points),
                PoseMatrix(cloudKeyPoses6D->back()));
        }

        if (ExportGroundAlgorithm() != lio_sam_hesai::GroundAlgorithm::None)
        {
            std::string cache_error;
            if (!StoreGroundInputsForExport(cloudKeyPoses3D->size() - 1, &cache_error))
                RCLCPP_ERROR(get_logger(), "Failed to cache ground export input: %s", cache_error.c_str());
        }

        // save path for visualization
        updatePath(thisPose6D);
    }

    void correctPoses()
    {
        if (cloudKeyPoses3D->points.empty())
            return;

        if (aLoopIsClosed == true)
        {
            // clear map cache
            transformed_keyframe_cache_.Clear();
            InvalidateLocalMapCache();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll  = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw   = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();

                updatePath(cloudKeyPoses6D->points[i]);
            }

            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose& pose_in)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = rclcpp::Time(pose_in.time * 1e9);
        pose_stamped.header.frame_id = config_.frames.odometry;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf2::Quaternion q;
        q.setRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        // The optimizer keeps its own float[6] RPY representation; this
        // publication point is the boundary where it becomes a direction-safe
        // odom_T_lidar. The rotation is rebuilt from the same RPY angles in
        // double precision -- matching the previous tf2::setRPY path -- rather
        // than round-tripped through the float Affine3f, whose rounding error
        // (~1e-7) is far above the transform's orthonormality tolerance.
        Eigen::Isometry3d odom_T_lidar_value = Eigen::Isometry3d::Identity();
        odom_T_lidar_value.linear() =
            (Eigen::AngleAxisd(transformTobeMapped[2],
                               Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(transformTobeMapped[1],
                               Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(transformTobeMapped[0],
                               Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        odom_T_lidar_value.translation() =
            Eigen::Vector3d(transformTobeMapped[3], transformTobeMapped[4],
                            transformTobeMapped[5]);
        const OdomTLidar odom_T_lidar(odom_T_lidar_value);
        const nav_msgs::msg::Odometry laserOdometryROS =
            RosPoseAdapter::ToOdometry(odom_T_lidar, config_.frames.odometry,
                                       "odom_mapping", timeLaserInfoStamp);
        pubLaserOdometryGlobal->publish(laserOdometryROS);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::msg::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine; // incremental odometry in affine
        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        } else {
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles (increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imu_available == true)
            {
                if (std::abs(cloudInfo.imu_pitch_init) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf2::Quaternion imuQuaternion;
                    tf2::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imu_roll_init, 0, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imu_pitch_init, 0);
                    tf2::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = config_.frames.odometry;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            tf2::Quaternion quat_tf;
            quat_tf.setRPY(roll, pitch, yaw);
            geometry_msgs::msg::Quaternion quat_msg;
            tf2::convert(quat_tf, quat_msg);
            laserOdomIncremental.pose.pose.orientation = quat_msg;
            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental->publish(laserOdomIncremental);
    }

    void publishFrames()
    {
        if (!config_.mapping.system.mapping_mode)
            return;
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, config_.frames.odometry);
        // Publish surrounding key frames
        publishCloud(pubRecentKeyFrames, laserCloudSurfFromMapDS, timeLaserInfoStamp, config_.frames.odometry);
        // publish registered key frame
        if (pubRecentKeyFrame->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut += *transformPointCloud(laserCloudCornerLastDS,  &thisPose6D);
            *cloudOut += *transformPointCloud(laserCloudSurfLastDS,    &thisPose6D);
            publishCloud(pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, config_.frames.odometry);
        }
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw->get_subscription_count() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut,  &thisPose6D);
            publishCloud(pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, config_.frames.odometry);
        }
        // publish path
        if (pubPath->get_subscription_count() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = config_.frames.odometry;
            pubPath->publish(globalPath);
        }
    }
};


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto MO = std::make_shared<mapOptimization>(options);
    exec.add_node(MO);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Map Optimization Started.\033[0m");

    std::thread loopthread(&mapOptimization::loopClosureThread, MO);
    std::unique_ptr<std::thread> visualizeMapThread;
    if (MO->MappingMode())
        visualizeMapThread = std::make_unique<std::thread>(&mapOptimization::visualizeGlobalMapThread, MO);

    exec.spin();

    rclcpp::shutdown();

    loopthread.join();
    if (visualizeMapThread)
        visualizeMapThread->join();

    return 0;
}
