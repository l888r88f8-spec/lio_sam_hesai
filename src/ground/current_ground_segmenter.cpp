#include "lio_sam/current_ground_segmenter.h"
#include "lio_sam/ground_ring_policy.h"
#include <opencv2/core.hpp>
#include <pcl/common/point_tests.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <algorithm>
#include <cmath>
#include <cfloat>
#include <limits>
#include <stdexcept>

namespace lio_sam_hesai {
namespace {
using PointType = pcl::PointXYZI;
constexpr float kDegToRad = 3.14159265358979323846F / 180.0F;
constexpr float kMinPlaneNorm = 1e-6F;
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


std::size_t CellCount(int rows, int columns) {
  if (rows <= 0 || columns <= 0 ||
      static_cast<std::size_t>(rows) > std::numeric_limits<std::size_t>::max() /
          static_cast<std::size_t>(columns))
    throw std::invalid_argument("invalid ground projection dimensions");
  const auto count=static_cast<std::size_t>(rows)*static_cast<std::size_t>(columns);
  if (count > std::numeric_limits<std::uint32_t>::max())
    throw std::invalid_argument("ground projection exceeds uint32 index range");
  return count;
}
class CurrentGroundEngine {
 public:
  CurrentGroundEngine(const GroundSegmentationConfig& config,
      const pcl::PointCloud<PointType>& organized,
      const std::vector<std::uint8_t>& valid,int rows,int columns)
      : config_(config),rows_(rows),columns_(columns),fullCloud(&organized),
        rangeMat(rows,columns,CV_32F,cv::Scalar(FLT_MAX)) {
    for(std::size_t index=0;index<valid.size();++index)
      if(valid[index]) rangeMat.at<float>(index/columns,index%columns)=0.0F;
    resetGroundMatrix();
  }
  CurrentGroundResult Run() {
    pcl::PointCloud<PointType>::Ptr ground(new pcl::PointCloud<PointType>());
    extractLocalGroundFromScan(ground);
    CurrentGroundResult result;
    result.ground=std::move(*ground);
    result.classification.assign(groundMat.ptr<std::int8_t>(),
        groundMat.ptr<std::int8_t>()+static_cast<std::size_t>(rows_)*columns_);
    return result;
  }
 private:
  const GroundSegmentationConfig& config_;
  int rows_,columns_;
  const pcl::PointCloud<PointType>* fullCloud;
  cv::Mat rangeMat,groundMat;
    // 清空或重建当前帧的地面分类矩阵。
    void resetGroundMatrix()
    {
        if (groundMat.rows != rows_ || groundMat.cols != columns_)
        {
            groundMat = cv::Mat(rows_, columns_, CV_8S, cv::Scalar::all(0));
            return;
        }
        groundMat.setTo(cv::Scalar::all(0));
    }

    // 利用低线束相邻点构造初始地面种子点云。
    void buildGroundSeedCloud(pcl::PointCloud<PointType>::Ptr seedCloud)
    {
        seedCloud->clear();

        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        if (groundScanEnd <= groundScanStart)
            return;

        const float groundAngleThresholdRad = config_.angle_threshold_deg * kDegToRad;
        for (int j = 0; j < columns_; ++j)
        {
            for (int i = groundScanStart; i < groundScanEnd; ++i)
            {
                if (rangeMat.at<float>(i, j) == FLT_MAX ||
                    rangeMat.at<float>(i + 1, j) == FLT_MAX)
                {
                    groundMat.at<signed char>(i, j) = -1;
                    continue;
                }

                const int lowerInd = j + i * columns_;
                const int upperInd = j + (i + 1) * columns_;
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
                }
            }
        }
    }

    // 对地面种子点云拟合局部平面，并检查法向方向是否合理。
    bool fitGroundPlane(const pcl::PointCloud<PointType>::Ptr &seedCloud,
                        pcl::ModelCoefficients::Ptr coefficients,
                        pcl::PointIndices::Ptr inliers)
    {
        if (seedCloud->size() < static_cast<size_t>(config_.plane_min_points))
            return false;

        pcl::SACSegmentation<PointType> segmentation;
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setMaxIterations(200);
        segmentation.setDistanceThreshold(config_.plane_distance);
        segmentation.setInputCloud(seedCloud);
        segmentation.segment(*inliers, *coefficients);

        if (inliers->indices.size() < static_cast<size_t>(config_.plane_min_points))
            return false;

        return isPlaneNormalWithinAngle(*coefficients, config_.plane_max_angle_deg * kDegToRad);
    }

    // 判断距离图中的指定单元是否包含有效点。
    bool hasValidGroundCell(int row, int column) const
    {
        return row >= 0 && row < rows_ &&
               column >= 0 && column < columns_ &&
               rangeMat.at<float>(row, column) != FLT_MAX;
    }

    // 返回用于地面种子的 ring 行号范围 [start, end]。
    std::pair<int, int> getGroundScanRange() const
    {
        const auto range = lio_sam_hesai::normalizeGroundScanRange(
            config_.scan_start_index, config_.scan_end_index, rows_);
        return {range.start, range.end};
    }

    // 返回参与地面/非地面判定的 ring 行号范围。
    // 非地面点云需要从全部 ring 中剔除地面点。
    std::pair<int, int> getGroundClassificationRange() const
    {
        const auto range = lio_sam_hesai::fullGroundClassificationRange(rows_);
        return {range.start, range.end};
    }

    // 地面建图输出仍只使用配置的 ring 范围。
    bool shouldOutputGroundPoint(int row) const
    {
        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        const lio_sam_hesai::GroundRingRange outputRange{
            groundScanStart, groundScanEnd};
        return lio_sam_hesai::isGroundOutputRing(row, outputRange);
    }

    void markGroundCellForNonGroundFiltering(
        int row, int column, const PointType &point,
        pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        groundMat.at<signed char>(row, column) = 1;
        if (shouldOutputGroundPoint(row))
            groundOutput->push_back(point);
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
            fullCloud->points[fromColumn + fromRow * columns_];
        const PointType &toPoint =
            fullCloud->points[toColumn + toRow * columns_];

        const float dX = toPoint.x - fromPoint.x;
        const float dY = toPoint.y - fromPoint.y;
        const float dZ = toPoint.z - fromPoint.z;
        const float horizontalDistance = std::sqrt(dX * dX + dY * dY);
        if (horizontalDistance < kMinPlaneNorm)
            return std::fabs(dZ) <= config_.plane_distance;

        const float localAngle = std::atan2(std::fabs(dZ), horizontalDistance);
        const float maxLinkAngle = std::max(config_.plane_max_angle_deg * kDegToRad,
                                            config_.angle_threshold_deg * 2.0f * kDegToRad);
        return localAngle <= maxLinkAngle;
    }

    // 从 groundMat 中收集真实量测到的地面种子点。
    void collectMeasuredGroundSeeds(pcl::PointCloud<PointType>::Ptr groundOutput)
    {
        groundOutput->clear();
        const auto [groundScanStart, groundScanEnd] = getGroundScanRange();
        for (int i = groundScanStart; i <= groundScanEnd; ++i)
        {
            for (int j = 0; j < columns_; ++j)
            {
                if (groundMat.at<signed char>(i, j) != 1 || !hasValidGroundCell(i, j))
                    continue;

                groundOutput->push_back(fullCloud->points[j + i * columns_]);
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

        for (int j = 0; j < columns_; ++j)
        {
            int minConfirmedRow = rows_;
            int maxConfirmedRow = -1;
            for (int i = groundScanStart; i <= groundScanEnd; ++i)
            {
                if (!hasValidGroundCell(i, j))
                    continue;

                const int index = j + i * columns_;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= config_.plane_distance;
                if (groundMat.at<signed char>(i, j) != 1 || !nearPlane)
                    continue;

                markGroundCellForNonGroundFiltering(i, j, point, groundOutput);
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

                const int index = j + i * columns_;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= config_.plane_distance;
                if (!nearPlane || !isGroundConnected(previousGroundRow, j, i, j))
                    break;

                markGroundCellForNonGroundFiltering(i, j, point, groundOutput);
                previousGroundRow = i;
            }

            previousGroundRow = minConfirmedRow;
            for (int i = minConfirmedRow - 1; i >= classificationStart; --i)
            {
                if (!hasValidGroundCell(i, j))
                    break;

                const int index = j + i * columns_;
                const PointType &point = fullCloud->points[index];
                const bool nearPlane =
                    pointToPlaneDistance(point, *coefficients) <= config_.plane_distance;
                if (!nearPlane || !isGroundConnected(i, j, previousGroundRow, j))
                    break;

                markGroundCellForNonGroundFiltering(i, j, point, groundOutput);
                previousGroundRow = i;
            }
        }

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
            return !groundOutput->empty();
        }

        extractGroundPointsFromPlane(coefficients, groundOutput);
        if (groundOutput->empty())
        {
            collectMeasuredGroundSeeds(groundOutput);
        }
        return !groundOutput->empty();
    }

};
}  // namespace
CurrentGroundResult SegmentCurrentGround(const GroundSegmentationConfig& config,
    const pcl::PointCloud<pcl::PointXYZI>& organized,
    const std::vector<std::uint8_t>& valid,int rows,int columns) {
  const auto count=CellCount(rows,columns);
  if(organized.size()!=count || valid.size()!=count)
    throw std::invalid_argument("ground projection cloud/mask dimensions do not match");
  return CurrentGroundEngine(config,organized,valid,rows,columns).Run();
}
CurrentGroundResult SegmentProjectedGround(const GroundSegmentationConfig& config,
    const pcl::PointCloud<pcl::PointXYZI>& projected,
    const std::vector<std::uint32_t>& indices,int rows,int columns) {
  const auto count=CellCount(rows,columns);
  if(projected.size()!=indices.size())
    throw std::invalid_argument("ground projection index count does not match scan");
  pcl::PointCloud<pcl::PointXYZI> organized;
  organized.resize(count);
  std::vector<std::uint8_t> valid(count,0);
  for(std::size_t index=0;index<indices.size();++index) {
    const auto cell=indices[index];
    if(cell>=count || valid[cell])
      throw std::invalid_argument("ground projection index is duplicate or out of range");
    if(!pcl::isFinite(projected[index]))
      throw std::invalid_argument("ground projection contains a nonfinite point");
    organized[cell]=projected[index];valid[cell]=1;
  }
  return SegmentCurrentGround(config,organized,valid,rows,columns);
}
}  // namespace lio_sam_hesai
