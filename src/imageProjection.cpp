#include "utility.hpp"
#include "lio_sam_hesai/msg/cloud_info.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cstdlib>
#include <limits>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)

struct LivoxPointXYZIRT {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    //uint8_t tag;
    uint16_t line;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    /*(uint8_t, tag, tag)*/ (uint16_t, line, line) (float, time, time)
)


// Use the Velodyne point format as a common representation
using PointXYZIRT = VelodynePointXYZIRT;

const int queueLength = 2000;

namespace {

constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kMinPlaneNorm = 1e-6f;

// 计算点到平面的无符号距离。
float pointToPlaneDistance(const PointType &point,
                           const pcl::ModelCoefficients &coefficients)
{
    if (coefficients.values.size() < 4)
        return std::numeric_limits<float>::max();

    const float a = coefficients.values[0];
    const float b = coefficients.values[1];
    const float c = coefficients.values[2];
    const float d = coefficients.values[3];
    const float norm = std::sqrt(a * a + b * b + c * c);
    if (norm < kMinPlaneNorm)
        return std::numeric_limits<float>::max();

    return std::fabs(a * point.x + b * point.y + c * point.z + d) / norm;
}

// 判断平面法向量相对竖直方向的倾角是否在允许范围内。
bool isPlaneNormalWithinAngle(const pcl::ModelCoefficients &coefficients,
                              const float max_angle_rad)
{
    if (coefficients.values.size() < 3)
        return false;

    const float a = coefficients.values[0];
    const float b = coefficients.values[1];
    const float c = coefficients.values[2];
    const float norm = std::sqrt(a * a + b * b + c * c);
    if (norm < kMinPlaneNorm)
        return false;

    const float cos_angle = std::fabs(c) / norm;
    const float clamped = std::clamp(cos_angle, 0.0f, 1.0f);
    return std::acos(clamped) <= max_angle_rad;
}

}  // namespace

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLaserCloud;
    rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubExtractedCloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGroundCloudGlobal;
    rclcpp::Publisher<lio_sam_hesai::msg::CloudInfo>::SharedPtr pubLaserCloudInfo;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubNonGroundCloud;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subGlobalMap;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
    std::deque<sensor_msgs::msg::Imu> imuQueue;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom;
    rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
    std::deque<nav_msgs::msg::Odometry> odomQueue;

    std::deque<sensor_msgs::msg::PointCloud2> cloudQueue;
    sensor_msgs::msg::PointCloud2 currentCloudMsg; // 标准点云格式

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn; //VelodynePointXYZIRT点云格式
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;
    pcl::PointCloud<PointType>::Ptr   nonGroundCloud;
    pcl::PointCloud<PointType>::Ptr   groundCloud;
    pcl::PointCloud<PointType>::Ptr   groundCloudDS;
    pcl::PointCloud<PointType>::Ptr   groundCloudScanOnly;
    pcl::PointCloud<PointType>::Ptr   groundCloudScanOnlyDS;
    pcl::PointCloud<PointType>::Ptr   groundCloudGlobalSeed;
    pcl::PointCloud<PointType>::Ptr   globalMapCloud;
    pcl::PointCloud<PointType>::Ptr   patchedGround;
    pcl::PointCloud<PointType>::Ptr   patchedGroundEdge;
    pcl::VoxelGrid<PointType> dsfPatchedGround;
    pcl::VoxelGrid<PointType> groundDownSizeFilter;

    int ringFlag = 0;
    int deskewFlag;
    cv::Mat rangeMat;
    cv::Mat groundMat;
    int firstFrameProcessed;
    std::mutex globalMapMutex;
    bool globalMapReceived;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    bool segmentationOnlyMode_;
    std::string nonGroundCloudTopic_;

    lio_sam_hesai::msg::CloudInfo cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::msg::Header cloudHeader;

    vector<int> columnIdnCountVec;


public:
    // 初始化点云投影节点的订阅、发布和运行时缓存。
    ImageProjection(const rclcpp::NodeOptions & options) :
            ParamServer("lio_sam_imageProjection", options), deskewFlag(0),
            firstFrameProcessed(0), globalMapReceived(false)
    {
        declare_parameter("segmentation_only_mode", false);
        get_parameter("segmentation_only_mode", segmentationOnlyMode_);
        declare_parameter("nonGroundCloudTopic", "lio_sam/deskew/cloud_nonground");
        get_parameter("nonGroundCloudTopic", nonGroundCloudTopic_);

        callbackGroupLidar = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupOdom = create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        auto lidarOpt = rclcpp::SubscriptionOptions();
        lidarOpt.callback_group = callbackGroupLidar;
        auto imuOpt = rclcpp::SubscriptionOptions();
        imuOpt.callback_group = callbackGroupImu;
        auto odomOpt = rclcpp::SubscriptionOptions();
        odomOpt.callback_group = callbackGroupOdom;

        subImu = create_subscription<sensor_msgs::msg::Imu>(
            imuTopic, qos_imu,
            std::bind(&ImageProjection::imuHandler, this, std::placeholders::_1),
            imuOpt);
        subOdom = create_subscription<nav_msgs::msg::Odometry>(
            odomTopic + "_incremental", qos_imu,
            std::bind(&ImageProjection::odometryHandler, this, std::placeholders::_1),
            odomOpt);
        subLaserCloud = create_subscription<sensor_msgs::msg::PointCloud2>(
            pointCloudTopic, qos_lidar,
            std::bind(&ImageProjection::cloudHandler, this, std::placeholders::_1),
            lidarOpt);
        subGlobalMap = create_subscription<sensor_msgs::msg::PointCloud2>(
            globalMapTopic, qos_lidar,
            std::bind(&ImageProjection::globalMapHandler, this, std::placeholders::_1),
            lidarOpt);

        pubExtractedCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/cloud_deskewed", 1);
        pubGroundCloudGlobal = create_publisher<sensor_msgs::msg::PointCloud2>(
            "lio_sam/deskew/ground_cloud_global", 1);
        pubLaserCloudInfo = create_publisher<lio_sam_hesai::msg::CloudInfo>(
            "lio_sam/deskew/cloud_info", qos);
        pubNonGroundCloud = create_publisher<sensor_msgs::msg::PointCloud2>(
            nonGroundCloudTopic_, 1);

        RCLCPP_INFO(
            get_logger(),
            "segmentation_only_mode=%s, nonground topic=%s",
            segmentationOnlyMode_ ? "true" : "false",
            nonGroundCloudTopic_.c_str());

        allocateMemory();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    // 分配点云容器并初始化固定大小的元数据缓冲区。
    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        nonGroundCloud.reset(new pcl::PointCloud<PointType>());
        groundCloud.reset(new pcl::PointCloud<PointType>());
        groundCloudDS.reset(new pcl::PointCloud<PointType>());
        groundCloudScanOnly.reset(new pcl::PointCloud<PointType>());
        groundCloudScanOnlyDS.reset(new pcl::PointCloud<PointType>());
        groundCloudGlobalSeed.reset(new pcl::PointCloud<PointType>());
        globalMapCloud.reset(new pcl::PointCloud<PointType>());
        patchedGround.reset(new pcl::PointCloud<PointType>());
        patchedGroundEdge.reset(new pcl::PointCloud<PointType>());

        dsfPatchedGround.setLeafSize(0.1f, 0.1f, 0.1f);

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.start_ring_index.assign(N_SCAN, 0);
        cloudInfo.end_ring_index.assign(N_SCAN, 0);

        cloudInfo.point_col_ind.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.point_range.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    // 在处理下一帧激光点云前重置当前帧状态。
    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        nonGroundCloud->clear();
        groundCloud->clear();
        groundCloudDS->clear();
        groundCloudScanOnly->clear();
        groundCloudScanOnlyDS->clear();
        groundCloudGlobalSeed->clear();
        patchedGround->clear();
        patchedGroundEdge->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }
        columnIdnCountVec.assign(N_SCAN, 0);
    }

    // 节点退出时不再执行额外的地面点云保存逻辑。
    ~ImageProjection()
    {
    }

    // 缓存经过坐标系转换后的 IMU 数据，供后续去畸变使用。
    void imuHandler(const sensor_msgs::msg::Imu::SharedPtr imuMsg)
    {
        sensor_msgs::msg::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf2::Quaternion orientation;
        // tf2::fromMsg(thisImu.orientation, orientation);
        // tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    // 缓存增量里程计，用于扫描去畸变和全局位姿初值。
    void odometryHandler(const nav_msgs::msg::Odometry::SharedPtr odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    // 缓存最新的全局地图，用于地面提取时的补充或回退。
    void globalMapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr mapMsg)
    {
        pcl::PointCloud<PointType>::Ptr temp(new pcl::PointCloud<PointType>());
        pcl::fromROSMsg(*mapMsg, *temp);
        std::lock_guard<std::mutex> lock(globalMapMutex);
        *globalMapCloud = *temp;
        globalMapReceived = true;
    }

    // 执行单帧点云的完整处理流程，从缓存到发布依次完成。
    void cloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!segmentationOnlyMode_)
        {
            if (!deskewInfo())
                return;
        }
        else
        {
            cloudInfo.imu_available = false;
            cloudInfo.odom_available = false;
            cloudInfo.imu_roll_init = 0.0f;
            cloudInfo.imu_pitch_init = 0.0f;
            cloudInfo.imu_yaw_init = 0.0f;
            cloudInfo.initial_guess_x = 0.0f;
            cloudInfo.initial_guess_y = 0.0f;
            cloudInfo.initial_guess_z = 0.0f;
            cloudInfo.initial_guess_roll = 0.0f;
            cloudInfo.initial_guess_pitch = 0.0f;
            cloudInfo.initial_guess_yaw = 0.0f;
        }

        projectPointCloud();

        groundRemoval();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }
    
    // 为 Livox 点云根据垂直角近似计算 ring 编号。
    int calculateLivoxRing(const PointType& point) {
        // Calculate vertical angle
        float verticalAngle = atan2(point.z, sqrt(point.x * point.x + point.y * point.y)) * 180.0 / M_PI;
        
        // Map vertical angle to ring index
        float minVerticalAngle = -7.0; // degrees
        float maxVerticalAngle = 52.0;  // degrees
        
        // Normalize to [0, N_SCAN-1]
        int ring = (int)((verticalAngle - minVerticalAngle) / (maxVerticalAngle - minVerticalAngle) * (N_SCAN - 1));
        
        // Clamp to valid range
        ring = std::max(0, std::min(ring, N_SCAN - 1));
        
        return ring;
    }

    // 判断输入 ROS 点云是否包含指定字段。
    bool hasCloudField(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & field_name)
    {
        for (const auto & field : cloud.fields)
        {
            if (field.name == field_name)
                return true;
        }
        return false;
    }

    // 将输入 ROS 点云转换为内部格式，并补齐 ring 与时间信息。
    bool cachePointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::VELODYNE)
        {
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);
        }
        else if (sensor == SensorType::HESAI)
        {
            pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

            if (!laserCloudIn->points.empty())
            {
                if (hasCloudField(currentCloudMsg, "timestamp"))
                {
                    sensor_msgs::PointCloud2ConstIterator<double> iter_ts(currentCloudMsg, "timestamp");
                    double t0 = std::numeric_limits<double>::max();
                    std::vector<double> timestamps;
                    timestamps.reserve(laserCloudIn->points.size());

                    for (size_t i = 0; i < laserCloudIn->points.size(); ++i, ++iter_ts)
                    {
                        const double ts = *iter_ts;
                        timestamps.push_back(ts);
                        t0 = std::min(t0, ts);
                    }

                    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
                    {
                        laserCloudIn->points[i].time = static_cast<float>(timestamps[i] - t0);
                    }
                }
                else if (hasCloudField(currentCloudMsg, "time"))
                {
                    sensor_msgs::PointCloud2ConstIterator<float> iter_time(currentCloudMsg, "time");
                    float t0 = std::numeric_limits<float>::max();
                    std::vector<float> rel_times;
                    rel_times.reserve(laserCloudIn->points.size());

                    for (size_t i = 0; i < laserCloudIn->points.size(); ++i, ++iter_time)
                    {
                        const float rel_time = *iter_time;
                        rel_times.push_back(rel_time);
                        t0 = std::min(t0, rel_time);
                    }

                    for (size_t i = 0; i < laserCloudIn->points.size(); ++i)
                    {
                        laserCloudIn->points[i].time = rel_times[i] - t0;
                    }
                }
                else
                {
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 5000,
                        "HESAI point cloud has no timestamp/time field. Deskew will be disabled for this frame.");
                }
            }
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else if (sensor == SensorType::LIVOX)
        {
            pcl::PointCloud<LivoxPointXYZIRT>::Ptr tmpLivoxCloudIn(new pcl::PointCloud<LivoxPointXYZIRT>());
            pcl::moveFromROSMsg(currentCloudMsg, *tmpLivoxCloudIn);
            
            laserCloudIn->points.resize(tmpLivoxCloudIn->size());
            laserCloudIn->is_dense = tmpLivoxCloudIn->is_dense;
            
            for (size_t i = 0; i < tmpLivoxCloudIn->size(); i++)
            {
                auto &src = tmpLivoxCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                
                // For Livox, we'll calculate an artificial ring based on vertical angle
                // since Livox doesn't have traditional rings
                PointType tempPoint;
                tempPoint.x = src.x;
                tempPoint.y = src.y;
                tempPoint.z = src.z;
                dst.ring = calculateLivoxRing(tempPoint);
                
                dst.time = src.time;
            }
        }
        else if (sensor == SensorType::LSLIDAR)
        {
            // LS180S2 适配分支
            // 1. LS180S2 字段定义 (x,y,z,i,ring,time) 与 VelodynePointXYZIRT 兼容，直接转换
            pcl::moveFromROSMsg(currentCloudMsg, *laserCloudIn);

            // 2. 时间归一化处理
            if (!laserCloudIn->points.empty())
            {
                // 获取这一帧第一个点的相对时间
                double start_time_offset = laserCloudIn->points[0].time;

                // 修正 Header 时间戳：将 offset 加到 header 上
                rclcpp::Time current_header_time(currentCloudMsg.header.stamp);
                // 加上 duration (秒)
                currentCloudMsg.header.stamp = (current_header_time + rclcpp::Duration::from_seconds(start_time_offset));

                // 归一化点云时间：所有点减去首点时间，使得首点 time 变为 0
                for (size_t i = 0; i < laserCloudIn->points.size(); i++)
                {
                    PointType tempPoint;
                    tempPoint.x = laserCloudIn->points[i].x;
                    tempPoint.y = laserCloudIn->points[i].y;
                    tempPoint.z = laserCloudIn->points[i].z;
                    laserCloudIn->points[i].ring = calculateLivoxRing(tempPoint);
                    laserCloudIn->points[i].time -= static_cast<float>(start_time_offset);
                }
            }
        }
        else
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Unknown sensor type: " << int(sensor));
            rclcpp::shutdown();
        }

        // remove Nan
        vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn, *laserCloudIn, indices);

        if (laserCloudIn->empty())
        {
            RCLCPP_WARN(get_logger(), "Input point cloud is empty after NaN filtering, skip this frame.");
            return false;
        }

        if (laserCloudIn->is_dense == false)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Input point cloud reports is_dense=false. NaN points have been filtered and processing will continue.");
        }
        laserCloudIn->is_dense = true;

        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = stamp2Sec(cloudHeader.stamp);
        double max_point_time = 0.0;
        for (const auto & point : laserCloudIn->points)
        {
            max_point_time = std::max(max_point_time, static_cast<double>(point.time));
        }
        timeScanEnd = timeScanCur + max_point_time;

        // check ring channel
        // we will skip the ring check in case of velodyne - as we calculate the ring value downstream (line 572)
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)currentCloudMsg.fields.size(); ++i)
            {
                if (currentCloudMsg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                if (sensor == SensorType::VELODYNE || sensor == SensorType::HESAI) {
                    ringFlag = 2;
                } else if (sensor == SensorType::LSLIDAR || sensor == SensorType::LIVOX) {
                    ringFlag = 3; // Use artificial ring calculation for Livox
                }else {
                    RCLCPP_ERROR(get_logger(), "Point cloud ring channel not available, please configure your point cloud data!");
                    rclcpp::shutdown();
                }
            }
        }

        // check point time
        if (deskewFlag == 0)
        {
            deskewFlag = -1;
            for (auto &field : currentCloudMsg.fields)
            {
                if (field.name == "time" || field.name == "t" || field.name == "timestamp")
                {
                    deskewFlag = 1;
                    break;
                }
            }
            if (deskewFlag == -1)
                RCLCPP_WARN(get_logger(), "Point cloud timestamp not available, deskew function disabled, system will drift significantly!");
        }

        return true;
    }

    // 检查 IMU 和里程计是否齐全，并准备去畸变插值所需数据。
    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() ||
            stamp2Sec(imuQueue.front().header.stamp) > timeScanCur ||
            stamp2Sec(imuQueue.back().header.stamp) < timeScanEnd)
        {
            RCLCPP_INFO(get_logger(), "Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    // 在当前扫描时间段内积分 IMU 角速度，用于旋转去畸变。
    void imuDeskewInfo()
    {
        cloudInfo.imu_available = false;

        while (!imuQueue.empty())
        {
            if (stamp2Sec(imuQueue.front().header.stamp) < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::msg::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = stamp2Sec(thisImuMsg.header.stamp);

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imu_roll_init, &cloudInfo.imu_pitch_init, &cloudInfo.imu_yaw_init);
            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imu_available = true;
    }

    // 提取扫描起止时刻的里程计信息，构造位姿初值与运动增量。
    void odomDeskewInfo()
    {
        cloudInfo.odom_available = false;

        while (!odomQueue.empty())
        {
            if (stamp2Sec(odomQueue.front().header.stamp) < timeScanCur - 0.01)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (stamp2Sec(odomQueue.front().header.stamp) > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::msg::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (stamp2Sec(startOdomMsg.header.stamp) < timeScanCur)
                continue;
            else
                break;
        }

        tf2::Quaternion orientation;
        tf2::fromMsg(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initial_guess_x = startOdomMsg.pose.pose.position.x;
        cloudInfo.initial_guess_y = startOdomMsg.pose.pose.position.y;
        cloudInfo.initial_guess_z = startOdomMsg.pose.pose.position.z;
        cloudInfo.initial_guess_roll = roll;
        cloudInfo.initial_guess_pitch = pitch;
        cloudInfo.initial_guess_yaw = yaw;

        cloudInfo.odom_available = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (stamp2Sec(odomQueue.back().header.stamp) < timeScanEnd)
            return;

        nav_msgs::msg::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (stamp2Sec(endOdomMsg.header.stamp) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf2::fromMsg(endOdomMsg.pose.pose.orientation, orientation);
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    // 按点时间戳插值计算该点对应的 IMU 旋转量。
    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    // 在启用位置去畸变时，为点插值计算平移量。
    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    // 利用 IMU 和里程计运动信息将原始点变换到扫描起始时刻坐标系。
    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imu_available == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    // 将输入点云投影到按 ring 和列索引组织的距离图上。
    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        
        if (sensor == SensorType::LIVOX || sensor == SensorType::LSLIDAR) {
            std::fill(columnIdnCountVec.begin(), columnIdnCountVec.end(), 0);
    	}
        // range image projection
        #pragma omp parallel for
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            // if sensor is a velodyne (ringFlag = 2) calculate rowIdn based on number of scans
            if (ringFlag == 2) { 
                float verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                rowIdn = (verticalAngle + (N_SCAN - 1)) / 2.0;
            }
            
            else if (ringFlag == 3) { // Livox - use artificial ring
            	rowIdn = calculateLivoxRing(thisPoint);
            }

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            int columnIdn = -1;
            if (sensor == SensorType::VELODYNE || sensor == SensorType::OUSTER || sensor == SensorType::HESAI)
            {
                float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
                static float ang_res_x = 360.0/float(Horizon_SCAN);
                columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
                if (columnIdn >= Horizon_SCAN)
                    columnIdn -= Horizon_SCAN;
            }
            else if (sensor == SensorType::LSLIDAR || sensor == SensorType::LIVOX)
            {
                float horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI;
            	if (horizonAngle < 0) horizonAngle += 360;
            	columnIdn = columnIdnCountVec[rowIdn];
                columnIdnCountVec[rowIdn] += 1;
                
                // Handle overlapping points in the same cell
                while (columnIdn < Horizon_SCAN && rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)    {
                    columnIdn++;

                }
                if (columnIdn >= Horizon_SCAN) {
                    // Try wrapping around or skip this point
                for (int col = 0; col < Horizon_SCAN; col++) {
                        if (rangeMat.at<float>(rowIdn, col) == FLT_MAX) {
                            columnIdn = col;
                            break;
                        }
                    }
                }
            }


            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }
    }

    // 清空或重建当前帧的地面分类矩阵。
    void resetGroundMatrix()
    {
        if (groundMat.rows != N_SCAN || groundMat.cols != Horizon_SCAN)
        {
            groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
            return;
        }
        groundMat.setTo(cv::Scalar::all(0));
    }

    // 利用低线束相邻点和补点策略构造初始地面种子点云。
    void buildGroundSeedCloud(pcl::PointCloud<PointType>::Ptr seedCloud)
    {
        seedCloud->clear();
        patchedGround->clear();
        patchedGroundEdge->clear();

        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        if (groundScanEnd <= groundScanStart)
            return;

        const float groundAngleThresholdRad = groundAngleThreshold * kDegToRad;
        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            int ringEdge = groundScanStart;
            int closestRingEdge = groundScanEnd;
            bool doPatch = false;
            for (int i = groundScanStart; i < groundScanEnd; ++i)
            {
                if (rangeMat.at<float>(i, j) == FLT_MAX ||
                    rangeMat.at<float>(i + 1, j) == FLT_MAX)
                {
                    groundMat.at<signed char>(i, j) = -1;
                    continue;
                }

                const int lowerInd = j + i * Horizon_SCAN;
                const int upperInd = j + (i + 1) * Horizon_SCAN;
                const PointType &lowerPoint = fullCloud->points[lowerInd];
                const PointType &upperPoint = fullCloud->points[upperInd];

                const float dX = upperPoint.x - lowerPoint.x;
                const float dY = upperPoint.y - lowerPoint.y;
                const float dZ = upperPoint.z - lowerPoint.z;
                const float horizontalDistance = std::sqrt(dX * dX + dY * dY);
                if (horizontalDistance < kMinPlaneNorm)
                    continue;

                const float verticalAngle = std::atan2(dZ, horizontalDistance);
                if (std::fabs(verticalAngle) <= groundAngleThresholdRad)
                {
                    groundMat.at<signed char>(i, j) = 1;
                    groundMat.at<signed char>(i + 1, j) = 1;
                    seedCloud->push_back(lowerPoint);
                    seedCloud->push_back(upperPoint);

                    if (i < closestRingEdge)
                        closestRingEdge = i;

                    const float ds = std::sqrt(dX * dX + dY * dY + dZ * dZ);
                    if (distanceForPatchBetweenRings > 0.0f &&
                        ds < distanceForPatchBetweenRings)
                    {
                        ringEdge = i + 1;
                        const float dt = 1.0f / (ds / 0.1f + 1.0f);
                        for (float t = 0.0f; t <= 1.0f; t += dt)
                        {
                            PointType patchPoint;
                            patchPoint.intensity = 0.0f;
                            patchPoint.x = lowerPoint.x + dX * t;
                            patchPoint.y = lowerPoint.y + dY * t;
                            patchPoint.z = lowerPoint.z + dZ * t;
                            patchedGround->push_back(patchPoint);
                        }
                        PointType patchPoint;
                        patchPoint.intensity = 0.0f;
                        patchPoint.x = upperPoint.x;
                        patchPoint.y = upperPoint.y;
                        patchPoint.z = upperPoint.z;
                        patchedGround->push_back(patchPoint);
                        doPatch = true;
                    }
                }
            }

            const int ringEdgeInd = j + ringEdge * Horizon_SCAN;
            if (rangeMat.at<float>(ringEdge, j) != FLT_MAX)
            {
                PointType edgePoint;
                edgePoint.x = fullCloud->points[ringEdgeInd].x;
                edgePoint.y = fullCloud->points[ringEdgeInd].y;
                edgePoint.z = fullCloud->points[ringEdgeInd].z;
                edgePoint.intensity = 100.0f;
                patchedGroundEdge->push_back(edgePoint);
            }

            (void)closestRingEdge;
            (void)doPatch;
        }

        if (!patchedGround->empty())
        {
            dsfPatchedGround.setInputCloud(patchedGround);
            dsfPatchedGround.filter(*patchedGround);
            *seedCloud += *patchedGround;
        }
        if (!patchedGroundEdge->empty())
        {
            dsfPatchedGround.setInputCloud(patchedGroundEdge);
            dsfPatchedGround.filter(*patchedGroundEdge);
        }
    }

    // 对地面种子点云拟合局部平面，并检查法向方向是否合理。
    bool fitGroundPlane(const pcl::PointCloud<PointType>::Ptr &seedCloud,
                        pcl::ModelCoefficients::Ptr coefficients,
                        pcl::PointIndices::Ptr inliers)
    {
        if (seedCloud->size() < static_cast<size_t>(groundPlaneMinPoints))
            return false;

        pcl::SACSegmentation<PointType> segmentation;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(200);
        segmentation.setDistanceThreshold(groundPlaneDistance);
        segmentation.setInputCloud(seedCloud);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() < static_cast<size_t>(groundPlaneMinPoints))
            return false;

        return isPlaneNormalWithinAngle(*coefficients, groundPlaneMaxAngle * kDegToRad);
    }

    // 判断距离图中的指定单元是否包含有效点。
    bool hasValidGroundCell(int row, int column) const
    {
        return row >= 0 && row < N_SCAN &&
               column >= 0 && column < Horizon_SCAN &&
               rangeMat.at<float>(row, column) != FLT_MAX;
    }

    // 返回用于地面补点和最终地面输出的 ring 行号范围 [start, end]。
    std::pair<int, int> getGroundScanRange() const
    {
        int groundScanStart = groundScanStartIndex;
        int groundScanEnd = groundScanEndIndex;

        if (groundScanEnd < 0)
            groundScanEnd = N_SCAN - 1;

        groundScanStart = std::min(std::max(groundScanStart, 0), N_SCAN - 1);
        groundScanEnd = std::min(std::max(groundScanEnd, 0), N_SCAN - 1);
        if (groundScanStart > groundScanEnd)
            std::swap(groundScanStart, groundScanEnd);

        return {groundScanStart, groundScanEnd};
    }

    // 返回参与地面/非地面判定的 ring 行号范围，默认使用全部线束。
    std::pair<int, int> getGroundClassificationRange() const
    {
        return {0, N_SCAN - 1};
    }

    // 判断两个相邻候选点是否在局部几何上连续，可视为同一片地面。
    bool isGroundConnected(int fromRow, int fromColumn, int toRow, int toColumn) const
    {
        if (!hasValidGroundCell(fromRow, fromColumn) ||
            !hasValidGroundCell(toRow, toColumn))
        {
            return false;
        }

        const PointType &fromPoint =
            fullCloud->points[fromColumn + fromRow * Horizon_SCAN];
        const PointType &toPoint =
            fullCloud->points[toColumn + toRow * Horizon_SCAN];

        const float dX = toPoint.x - fromPoint.x;
        const float dY = toPoint.y - fromPoint.y;
        const float dZ = toPoint.z - fromPoint.z;
        const float horizontalDistance = std::sqrt(dX * dX + dY * dY);
        if (horizontalDistance < kMinPlaneNorm)
            return std::fabs(dZ) <= groundPlaneDistance;

        const float localAngle = std::atan2(std::fabs(dZ), horizontalDistance);
        const float maxLinkAngle = std::max(groundPlaneMaxAngle * kDegToRad,
                                            groundAngleThreshold * 2.0f * kDegToRad);
        return localAngle <= maxLinkAngle;
    }

    // 从 groundMat 中收集真实量测到的地面种子点，避免把人工补点输出到最终地面云。
    void collectMeasuredGroundSeeds(pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        groundOutput->clear();
        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        for (int i = groundScanStart; i <= groundScanEnd; ++i)
        {
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (groundMat.at<signed char>(i, j) != 1 || !hasValidGroundCell(i, j))
                    continue;

                groundOutput->push_back(fullCloud->points[j + i * Horizon_SCAN]);
            }
        }
    }

    // 仅在相邻 ring 都已确认是地面时补充插值点，用于填补扫描线之间的空洞。
    void appendConfirmedGroundPatches(const pcl::ModelCoefficients::Ptr &coefficients,
                                      pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        if (distanceForPatchBetweenRings <= 0.0f)
            return;

        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        const float interpolationSpacing =
            std::max(0.02f, groundLeafSize > 0.0f ? groundLeafSize * 0.5f : 0.05f);

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            for (int i = groundScanStart; i < groundScanEnd; ++i)
            {
                if (groundMat.at<signed char>(i, j) != 1 ||
                    groundMat.at<signed char>(i + 1, j) != 1)
                {
                    continue;
                }

                if (!hasValidGroundCell(i, j) || !hasValidGroundCell(i + 1, j))
                    continue;

                if (!isGroundConnected(i, j, i + 1, j))
                    continue;

                const PointType &lowerPoint =
                    fullCloud->points[j + i * Horizon_SCAN];
                const PointType &upperPoint =
                    fullCloud->points[j + (i + 1) * Horizon_SCAN];

                const float dX = upperPoint.x - lowerPoint.x;
                const float dY = upperPoint.y - lowerPoint.y;
                const float dZ = upperPoint.z - lowerPoint.z;
                const float ds = std::sqrt(dX * dX + dY * dY + dZ * dZ);
                if (ds <= interpolationSpacing ||
                    ds > distanceForPatchBetweenRings)
                {
                    continue;
                }

                const int segmentCount =
                    std::max(2, static_cast<int>(std::ceil(ds / interpolationSpacing)));
                for (int step = 1; step < segmentCount; ++step)
                {
                    const float t = static_cast<float>(step) /
                                    static_cast<float>(segmentCount);

                    PointType patchPoint;
                    patchPoint.x = lowerPoint.x + dX * t;
                    patchPoint.y = lowerPoint.y + dY * t;
                    patchPoint.z = lowerPoint.z + dZ * t;
                    patchPoint.intensity = 0.0f;

                    if (coefficients &&
                        pointToPlaneDistance(patchPoint, *coefficients) >
                            groundPlaneDistance)
                    {
                        continue;
                    }

                    groundOutput->push_back(patchPoint);
                }
            }
        }
    }

    // 计算地面平面在传感器中心正下方的投影点，用于补齐机器人脚底下的地面空洞。
    bool getGroundCenterPointOnPlane(const pcl::ModelCoefficients::Ptr &coefficients,
                                     PointType *centerPoint) const
    {
        if (!coefficients || coefficients->values.size() < 4 || centerPoint == nullptr)
            return false;

        const float a = coefficients->values[0];
        const float b = coefficients->values[1];
        const float c = coefficients->values[2];
        const float d = coefficients->values[3];
        (void)a;
        (void)b;
        if (std::fabs(c) < kMinPlaneNorm)
            return false;

        centerPoint->x = 0.0f;
        centerPoint->y = 0.0f;
        centerPoint->z = -d / c;
        centerPoint->intensity = 0.0f;
        return std::isfinite(centerPoint->z);
    }

    // 将机器人中心到 groundScanStartIndex 对应地面线之间补齐，减小脚底下的近距离空洞。
    void appendGroundPatchToSensorCenter(const pcl::ModelCoefficients::Ptr &coefficients,
                                         pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        PointType centerGroundPoint;
        if (!getGroundCenterPointOnPlane(coefficients, &centerGroundPoint))
            return;

        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        if (groundScanStart > groundScanEnd)
            return;

        const float interpolationSpacing =
            std::max(0.02f, groundLeafSize > 0.0f ? groundLeafSize * 0.5f : 0.05f);
        bool centerPointAdded = false;

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            if (groundMat.at<signed char>(groundScanStart, j) != 1 ||
                !hasValidGroundCell(groundScanStart, j))
            {
                continue;
            }

            const PointType &edgePoint =
                fullCloud->points[j + groundScanStart * Horizon_SCAN];
            const float dX = edgePoint.x - centerGroundPoint.x;
            const float dY = edgePoint.y - centerGroundPoint.y;
            const float dZ = edgePoint.z - centerGroundPoint.z;
            const float ds = std::sqrt(dX * dX + dY * dY + dZ * dZ);
            if (ds <= interpolationSpacing)
                continue;

            if (!centerPointAdded)
            {
                groundOutput->push_back(centerGroundPoint);
                centerPointAdded = true;
            }

            const int segmentCount =
                std::max(2, static_cast<int>(std::ceil(ds / interpolationSpacing)));
            for (int step = 1; step < segmentCount; ++step)
            {
                const float t = static_cast<float>(step) /
                                static_cast<float>(segmentCount);

                PointType patchPoint;
                patchPoint.x = centerGroundPoint.x + dX * t;
                patchPoint.y = centerGroundPoint.y + dY * t;
                patchPoint.z = centerGroundPoint.z + dZ * t;
                patchPoint.intensity = 0.0f;
                groundOutput->push_back(patchPoint);
            }
        }
    }

    // 根据拟合平面按列向上连续提取地面，避免把悬空同平面点误判为地面。
    void extractGroundPointsFromPlane(const pcl::ModelCoefficients::Ptr &coefficients,
                                      pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        groundOutput->clear();
        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        const auto [classificationStart, classificationEnd] =
            getGroundClassificationRange();

        for (int j = 0; j < Horizon_SCAN; ++j)
        {
            int minConfirmedRow = N_SCAN;
            int maxConfirmedRow = -1;
            for (int i = groundScanStart; i <= groundScanEnd; ++i)
            {
                if (!hasValidGroundCell(i, j))
                    continue;

                const int index = j + i * Horizon_SCAN;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= groundPlaneDistance;
                if (groundMat.at<signed char>(i, j) != 1 || !nearPlane)
                    continue;

                groundMat.at<signed char>(i, j) = 1;
                groundOutput->push_back(point);
                minConfirmedRow = std::min(minConfirmedRow, i);
                maxConfirmedRow = std::max(maxConfirmedRow, i);
            }

            if (maxConfirmedRow < 0)
                continue;

            int previousGroundRow = maxConfirmedRow;
            for (int i = maxConfirmedRow + 1; i <= classificationEnd; ++i)
            {
                if (!hasValidGroundCell(i, j))
                    break;

                const int index = j + i * Horizon_SCAN;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= groundPlaneDistance;
                if (!nearPlane || !isGroundConnected(previousGroundRow, j, i, j))
                    break;

                groundMat.at<signed char>(i, j) = 1;
                if (i >= groundScanStart && i <= groundScanEnd)
                    groundOutput->push_back(point);
                previousGroundRow = i;
            }

            previousGroundRow = minConfirmedRow;
            for (int i = minConfirmedRow - 1; i >= classificationStart; --i)
            {
                if (!hasValidGroundCell(i, j))
                    break;

                const int index = j + i * Horizon_SCAN;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= groundPlaneDistance;
                if (!nearPlane || !isGroundConnected(i, j, previousGroundRow, j))
                    break;

                groundMat.at<signed char>(i, j) = 1;
                if (i >= groundScanStart && i <= groundScanEnd)
                    groundOutput->push_back(point);
                previousGroundRow = i;
            }
        }

        appendConfirmedGroundPatches(coefficients, groundOutput);
        appendGroundPatchToSensorCenter(coefficients, groundOutput);
    }

    // 通过种子构造与平面拟合，从当前扫描中提取局部地面。
    bool extractLocalGroundFromScan(pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        pcl::PointCloud<PointType>::Ptr seedCloud(new pcl::PointCloud<PointType>());
        buildGroundSeedCloud(seedCloud);
        if (seedCloud->empty())
            return false;

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        if (!fitGroundPlane(seedCloud, coefficients, inliers))
        {
            collectMeasuredGroundSeeds(groundOutput);
            appendConfirmedGroundPatches(nullptr, groundOutput);
            return !groundOutput->empty();
        }

        extractGroundPointsFromPlane(coefficients, groundOutput);
        if (groundOutput->empty())
        {
            collectMeasuredGroundSeeds(groundOutput);
            appendConfirmedGroundPatches(coefficients, groundOutput);
            appendGroundPatchToSensorCenter(coefficients, groundOutput);
        }
        return !groundOutput->empty();
    }

    // 融合当前扫描地面与可选的全局地图地面补偿结果。
    void groundRemoval()
    {
        groundCloud->clear();
        groundCloudDS->clear();
        groundCloudScanOnly->clear();
        groundCloudScanOnlyDS->clear();
        groundCloudGlobalSeed->clear();

        resetGroundMatrix();

        const bool hasLocalGround = extractLocalGroundFromScan(groundCloudScanOnly);
        const bool needGlobalGroundFallback =
            !hasLocalGround || groundCloudScanOnly->size() <
                                   static_cast<size_t>(groundPlaneMinPoints);
        const bool hasGlobalGround =
            useGlobalMapGround && needGlobalGroundFallback && globalMapReceived &&
            cloudInfo.odom_available &&
            extractGroundFromGlobalMap(groundCloudGlobalSeed);

        if (hasLocalGround)
            *groundCloud = *groundCloudScanOnly;

        if (hasGlobalGround)
        {
            if (groundCloud->empty())
                *groundCloud = *groundCloudGlobalSeed;
        }

        if (!groundCloud->empty())
            downsampleGroundCloud(groundCloud, groundCloudDS);
        if (!groundCloudScanOnly->empty())
            downsampleGroundCloud(groundCloudScanOnly, groundCloudScanOnlyDS);
    }

    // 从全局地图中提取附近地面，作为当前扫描地面的补充或回退方案。
    bool extractGroundFromGlobalMap(pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        groundOutput->clear();

        pcl::PointCloud<PointType>::Ptr mapCopy(new pcl::PointCloud<PointType>());
        {
            std::lock_guard<std::mutex> lock(globalMapMutex);
            if (globalMapCloud->empty())
                return false;
            *mapCopy = *globalMapCloud;
        }

        if (mapCopy->empty())
            return false;

        const float cx = cloudInfo.initial_guess_x;
        const float cy = cloudInfo.initial_guess_y;
        const float cz = cloudInfo.initial_guess_z;

        pcl::CropBox<PointType> crop;
        crop.setInputCloud(mapCopy);
        crop.setMin(Eigen::Vector4f(cx - globalMapGroundRadius,
                                    cy - globalMapGroundRadius,
                                    cz - globalMapGroundZRange, 1.0f));
        crop.setMax(Eigen::Vector4f(cx + globalMapGroundRadius,
                                    cy + globalMapGroundRadius,
                                    cz + globalMapGroundZRange, 1.0f));

        pcl::PointCloud<PointType>::Ptr cropped(new pcl::PointCloud<PointType>());
        crop.filter(*cropped);
        if (cropped->size() < static_cast<size_t>(globalMapGroundMinPoints))
            return false;

        pcl::PointCloud<PointType>::Ptr filtered(new pcl::PointCloud<PointType>());
        if (globalMapGroundVoxelSize > 0.0f)
        {
            pcl::VoxelGrid<PointType> voxelGrid;
            voxelGrid.setLeafSize(globalMapGroundVoxelSize,
                                  globalMapGroundVoxelSize,
                                  globalMapGroundVoxelSize);
            voxelGrid.setInputCloud(cropped);
            voxelGrid.filter(*filtered);
        }
        else
        {
            *filtered = *cropped;
        }

        if (filtered->size() < static_cast<size_t>(globalMapGroundMinPoints))
            return false;

        pcl::SACSegmentation<PointType> segmentation;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(200);
        segmentation.setDistanceThreshold(globalMapGroundDistance);
        segmentation.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
        segmentation.setEpsAngle(globalMapGroundMaxAngle * kDegToRad);
        segmentation.setInputCloud(filtered);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() < static_cast<size_t>(globalMapGroundMinPoints))
            return false;

        pcl::PointCloud<PointType>::Ptr groundOdom(new pcl::PointCloud<PointType>());
        pcl::ExtractIndices<PointType> extract;
        extract.setInputCloud(filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*groundOdom);

        if (groundOdom->empty())
            return false;

        Eigen::Affine3f trans = pcl::getTransformation(
            cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
            cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);

        pcl::transformPointCloud(*groundOdom, *groundOutput, trans.inverse());
        return true;
    }

    // 在需要平面化导出时，将点云高度压到固定的 Z 值。
    void flattenCloudZ(pcl::PointCloud<PointType> &cloud)
    {
        if (!groundFlattenZ)
            return;
        for (auto &pt : cloud.points)
            pt.z = groundFlattenValue;
    }

    // 对单帧地面点云做下采样，便于发布和后续处理。
    void downsampleGroundCloud(const pcl::PointCloud<PointType>::Ptr &source,
                               pcl::PointCloud<PointType>::Ptr target)
    {
        target->clear();
        if (source->empty())
            return;

        if (groundLeafSize <= 0.0f)
        {
            *target = *source;
            flattenCloudZ(*target);
            return;
        }

        groundDownSizeFilter.setLeafSize(groundLeafSize, groundLeafSize, groundLeafSize);
        groundDownSizeFilter.setInputCloud(source);
        groundDownSizeFilter.filter(*target);
        flattenCloudZ(*target);
    }

    // 从距离图中提取有效点，生成有序的输出点云。
    void cloudExtraction()
    {
        int count = 0;
        nonGroundCloud->clear();
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.start_ring_index[i] = count - 1 + 5;
            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.point_col_ind[count] = j;
                    // save range info
                    cloudInfo.point_range[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    if (groundMat.at<signed char>(i, j) != 1)
                        nonGroundCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.end_ring_index[i] = count -1 - 5;
        }
    }

    // 发布去畸变点云、地面结果以及当前帧的 CloudInfo 消息。
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, extractedCloud, cloudHeader.stamp, lidarFrame);
        publishCloud(pubNonGroundCloud, nonGroundCloud, cloudHeader.stamp, lidarFrame);
        pcl::toROSMsg(*groundCloudScanOnlyDS, cloudInfo.cloud_ground);
        cloudInfo.cloud_ground.header.stamp = cloudHeader.stamp;
        cloudInfo.cloud_ground.header.frame_id = lidarFrame;

        pcl::PointCloud<PointType>::Ptr groundMap(new pcl::PointCloud<PointType>());
        if (cloudInfo.odom_available)
        {
            Eigen::Affine3f trans = pcl::getTransformation(
                cloudInfo.initial_guess_x, cloudInfo.initial_guess_y, cloudInfo.initial_guess_z,
                cloudInfo.initial_guess_roll, cloudInfo.initial_guess_pitch, cloudInfo.initial_guess_yaw);
            pcl::transformPointCloud(*groundCloudDS, *groundMap, trans);
        }
        else
        {
            *groundMap = *groundCloudDS;
        }
        publishCloud(pubGroundCloudGlobal, groundMap, cloudHeader.stamp, mapFrame);

        pubLaserCloudInfo->publish(cloudInfo);
        ++firstFrameProcessed;
    }
};

// 启动图像投影节点并持续运行直到系统关闭。
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor exec;

    auto IP = std::make_shared<ImageProjection>(options);
    exec.add_node(IP);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "\033[1;32m----> Image Projection Started.\033[0m");

    exec.spin();

    rclcpp::shutdown();
    return 0;
}
