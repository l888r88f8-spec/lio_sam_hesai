#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace lio_sam_hesai {

using GroundGridPointType = pcl::PointXYZI;

struct GroundGridPatchOptions
{
    float resolution = 0.10f;
    float search_radius = 0.40f;
    float max_slope_deg = 25.0f;
    float plane_distance_threshold = 0.12f;
    float max_height_range = 0.12f;
    float max_roughness = 0.05f;
    float max_prediction_margin = 0.08f;
    float min_plane_inlier_ratio = 0.70f;
    int min_neighbor_cells = 6;
    int min_support_quadrants = 3;
    bool require_opposing_axis_support = true;
};

namespace ground_grid_detail {

struct GridKey
{
    int ix = 0;
    int iy = 0;

    bool operator==(const GridKey& other) const noexcept
    {
        return ix == other.ix && iy == other.iy;
    }
};

struct GridKeyHash
{
    std::size_t operator()(const GridKey& key) const noexcept
    {
        const std::size_t h1 = std::hash<int>{}(key.ix);
        const std::size_t h2 = std::hash<int>{}(key.iy);
        return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
    }
};

struct GridCell
{
    std::vector<int> point_indices;
    float median_z = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;
    float roughness = 0.0f;
    bool traversable = false;
};

struct PlaneModel
{
    float a = 0.0f;
    float b = 0.0f;
    float c = 1.0f;
    float d = 0.0f;
};

using GridMap = std::unordered_map<GridKey, GridCell, GridKeyHash>;
using GridKeySet = std::unordered_set<GridKey, GridKeyHash>;

inline float safeResolution(const GroundGridPatchOptions& options)
{
    return std::max(options.resolution, 0.02f);
}

inline GridKey keyForPoint(const GroundGridPointType& point,
                           const GroundGridPatchOptions& options)
{
    const float res = safeResolution(options);
    return {static_cast<int>(std::floor(point.x / res)),
            static_cast<int>(std::floor(point.y / res))};
}

inline GroundGridPointType cellCenter(const GridKey& key,
                                      const GroundGridPatchOptions& options,
                                      float z)
{
    const float res = safeResolution(options);
    GroundGridPointType point;
    point.x = (static_cast<float>(key.ix) + 0.5f) * res;
    point.y = (static_cast<float>(key.iy) + 0.5f) * res;
    point.z = z;
    point.intensity = 0.0f;
    return point;
}

inline float median(std::vector<float> values)
{
    if (values.empty())
        return 0.0f;

    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float result = values[middle];
    if (values.size() % 2 == 0)
    {
        const auto lower = std::max_element(values.begin(), values.begin() + middle);
        result = (*lower + result) * 0.5f;
    }
    return result;
}

inline void updateCellStatistics(const pcl::PointCloud<GroundGridPointType>& cloud,
                                 GridCell* cell,
                                 const GroundGridPatchOptions& options)
{
    if (cell == nullptr || cell->point_indices.empty())
        return;

    std::vector<float> z_values;
    z_values.reserve(cell->point_indices.size());
    for (const int index : cell->point_indices)
    {
        if (index >= 0 && index < static_cast<int>(cloud.size()))
            z_values.push_back(cloud.points[index].z);
    }
    if (z_values.empty())
        return;

    const auto [min_iter, max_iter] =
        std::minmax_element(z_values.begin(), z_values.end());
    cell->min_z = *min_iter;
    cell->max_z = *max_iter;
    cell->median_z = median(z_values);

    float variance = 0.0f;
    for (const float z : z_values)
    {
        const float dz = z - cell->median_z;
        variance += dz * dz;
    }
    cell->roughness = std::sqrt(variance / static_cast<float>(z_values.size()));
    cell->traversable =
        (cell->max_z - cell->min_z) <= options.max_height_range &&
        cell->roughness <= options.max_roughness;
}

inline GridMap buildGrid(const pcl::PointCloud<GroundGridPointType>& cloud,
                         const GroundGridPatchOptions& options)
{
    GridMap grid;
    for (int i = 0; i < static_cast<int>(cloud.size()); ++i)
    {
        const GroundGridPointType& point = cloud.points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z))
        {
            continue;
        }
        grid[keyForPoint(point, options)].point_indices.push_back(i);
    }

    for (auto& entry : grid)
        updateCellStatistics(cloud, &entry.second, options);
    return grid;
}

inline std::vector<GridKey> collectNeighborKeys(const GridKey& key,
                                                const GridMap& grid,
                                                const GroundGridPatchOptions& options)
{
    std::vector<GridKey> neighbors;
    const float res = safeResolution(options);
    const int cell_radius =
        std::max(1, static_cast<int>(std::ceil(options.search_radius / res)));
    const GroundGridPointType center = cellCenter(key, options, 0.0f);
    const float radius_sq = options.search_radius * options.search_radius;

    for (int dx = -cell_radius; dx <= cell_radius; ++dx)
    {
        for (int dy = -cell_radius; dy <= cell_radius; ++dy)
        {
            if (dx == 0 && dy == 0)
                continue;

            GridKey neighbor_key{key.ix + dx, key.iy + dy};
            const auto iter = grid.find(neighbor_key);
            if (iter == grid.end() || !iter->second.traversable)
                continue;

            const GroundGridPointType neighbor_center =
                cellCenter(neighbor_key, options, iter->second.median_z);
            const float diff_x = neighbor_center.x - center.x;
            const float diff_y = neighbor_center.y - center.y;
            if (diff_x * diff_x + diff_y * diff_y <= radius_sq)
                neighbors.push_back(neighbor_key);
        }
    }
    return neighbors;
}

inline bool hasEnoughDirectionalSupport(const GridKey& key,
                                        const std::vector<GridKey>& neighbors,
                                        const GroundGridPatchOptions& options)
{
    if (static_cast<int>(neighbors.size()) < options.min_neighbor_cells)
        return false;

    bool left = false;
    bool right = false;
    bool down = false;
    bool up = false;
    bool quadrants[4] = {false, false, false, false};

    for (const GridKey& neighbor : neighbors)
    {
        const int dx = neighbor.ix - key.ix;
        const int dy = neighbor.iy - key.iy;
        left = left || dx < 0;
        right = right || dx > 0;
        down = down || dy < 0;
        up = up || dy > 0;

        if (dx < 0 && dy < 0)
            quadrants[0] = true;
        else if (dx < 0 && dy > 0)
            quadrants[1] = true;
        else if (dx > 0 && dy < 0)
            quadrants[2] = true;
        else if (dx > 0 && dy > 0)
            quadrants[3] = true;
    }

    const int quadrant_count =
        static_cast<int>(quadrants[0]) + static_cast<int>(quadrants[1]) +
        static_cast<int>(quadrants[2]) + static_cast<int>(quadrants[3]);
    if (quadrant_count < options.min_support_quadrants)
        return false;

    if (!options.require_opposing_axis_support)
        return true;

    return left && right && down && up;
}

inline bool evaluatePlaneZ(const PlaneModel& plane, float x, float y, float* z)
{
    if (z == nullptr || std::fabs(plane.c) < 1e-6f)
        return false;
    *z = -(plane.a * x + plane.b * y + plane.d) / plane.c;
    return std::isfinite(*z);
}

inline float pointToPlaneDistance(const GroundGridPointType& point,
                                  const PlaneModel& plane)
{
    const float norm =
        std::sqrt(plane.a * plane.a + plane.b * plane.b + plane.c * plane.c);
    if (norm < 1e-6f)
        return std::numeric_limits<float>::max();
    return std::fabs(plane.a * point.x + plane.b * point.y +
                     plane.c * point.z + plane.d) /
           norm;
}

inline float planeSlopeDeg(const PlaneModel& plane)
{
    const float norm_xy = std::sqrt(plane.a * plane.a + plane.b * plane.b);
    const float norm_z = std::fabs(plane.c);
    return std::atan2(norm_xy, norm_z) * 180.0f / static_cast<float>(M_PI);
}

inline bool fitPatchPlane(const pcl::PointCloud<GroundGridPointType>::Ptr& points,
                          const GroundGridPatchOptions& options,
                          PlaneModel* plane)
{
    if (plane == nullptr || points == nullptr ||
        static_cast<int>(points->size()) < options.min_neighbor_cells)
    {
        return false;
    }

    pcl::ModelCoefficients coefficients;
    pcl::PointIndices inliers;
    pcl::SACSegmentation<GroundGridPointType> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(80);
    segmentation.setDistanceThreshold(options.plane_distance_threshold);
    segmentation.setInputCloud(points);
    segmentation.segment(inliers, coefficients);

    if (coefficients.values.size() < 4 ||
        static_cast<int>(inliers.indices.size()) < options.min_neighbor_cells)
    {
        return false;
    }

    const float inlier_ratio =
        static_cast<float>(inliers.indices.size()) /
        static_cast<float>(points->size());
    if (inlier_ratio < options.min_plane_inlier_ratio)
        return false;

    plane->a = coefficients.values[0];
    plane->b = coefficients.values[1];
    plane->c = coefficients.values[2];
    plane->d = coefficients.values[3];
    if (planeSlopeDeg(*plane) > options.max_slope_deg)
        return false;

    float residual_sq = 0.0f;
    for (const int index : inliers.indices)
    {
        if (index < 0 || index >= static_cast<int>(points->size()))
            continue;
        const float residual = pointToPlaneDistance(points->points[index], *plane);
        residual_sq += residual * residual;
    }
    const float rmse =
        std::sqrt(residual_sq / static_cast<float>(inliers.indices.size()));
    return rmse <= options.plane_distance_threshold;
}

inline void collectNeighborPoints(const std::vector<GridKey>& neighbor_keys,
                                  const pcl::PointCloud<GroundGridPointType>& cloud,
                                  const GridMap& grid,
                                  pcl::PointCloud<GroundGridPointType>::Ptr points)
{
    points->clear();
    for (const GridKey& key : neighbor_keys)
    {
        const auto iter = grid.find(key);
        if (iter == grid.end())
            continue;
        for (const int index : iter->second.point_indices)
        {
            if (index >= 0 && index < static_cast<int>(cloud.size()))
                points->push_back(cloud.points[index]);
        }
    }
}

inline bool predictedHeightWithinSupport(const GroundGridPointType& patch_point,
                                         const std::vector<GridKey>& neighbor_keys,
                                         const GridMap& grid,
                                         const GroundGridPatchOptions& options)
{
    if (neighbor_keys.empty())
        return false;

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const GridKey& key : neighbor_keys)
    {
        const auto iter = grid.find(key);
        if (iter == grid.end())
            continue;
        min_z = std::min(min_z, iter->second.median_z);
        max_z = std::max(max_z, iter->second.median_z);
    }

    return patch_point.z >= min_z - options.max_prediction_margin &&
           patch_point.z <= max_z + options.max_prediction_margin;
}

inline GridKeySet collectCandidates(const GridMap& grid,
                                    const GroundGridPatchOptions& options)
{
    GridKeySet candidates;
    const float res = safeResolution(options);
    const int cell_radius =
        std::max(1, static_cast<int>(std::ceil(options.search_radius / res)));

    for (const auto& entry : grid)
    {
        if (!entry.second.traversable)
            continue;
        const GridKey& occupied_key = entry.first;
        for (int dx = -cell_radius; dx <= cell_radius; ++dx)
        {
            for (int dy = -cell_radius; dy <= cell_radius; ++dy)
            {
                if (dx == 0 && dy == 0)
                    continue;
                GridKey candidate{occupied_key.ix + dx, occupied_key.iy + dy};
                if (grid.find(candidate) == grid.end())
                    candidates.insert(candidate);
            }
        }
    }
    return candidates;
}

}  // namespace ground_grid_detail

inline std::vector<GroundGridPointType> generateGroundGridPatches(
    const pcl::PointCloud<GroundGridPointType>& cloud,
    const GroundGridPatchOptions& options)
{
    std::vector<GroundGridPointType> patch_points;
    if (cloud.empty() || options.resolution <= 0.0f ||
        options.search_radius <= 0.0f)
    {
        return patch_points;
    }

    const auto grid = ground_grid_detail::buildGrid(cloud, options);
    const auto candidates = ground_grid_detail::collectCandidates(grid, options);
    if (grid.empty() || candidates.empty())
        return patch_points;

    const std::size_t max_patch_points = cloud.size();
    patch_points.reserve(std::min(max_patch_points, candidates.size()));
    pcl::PointCloud<GroundGridPointType>::Ptr neighbors(
        new pcl::PointCloud<GroundGridPointType>());

    for (const auto& candidate : candidates)
    {
        const auto neighbor_keys =
            ground_grid_detail::collectNeighborKeys(candidate, grid, options);
        if (!ground_grid_detail::hasEnoughDirectionalSupport(
                candidate, neighbor_keys, options))
        {
            continue;
        }

        ground_grid_detail::collectNeighborPoints(
            neighbor_keys, cloud, grid, neighbors);
        ground_grid_detail::PlaneModel plane;
        if (!ground_grid_detail::fitPatchPlane(neighbors, options, &plane))
            continue;

        GroundGridPointType center =
            ground_grid_detail::cellCenter(candidate, options, 0.0f);
        float z = 0.0f;
        if (!ground_grid_detail::evaluatePlaneZ(plane, center.x, center.y, &z))
            continue;

        center.z = z;
        if (!ground_grid_detail::predictedHeightWithinSupport(
                center, neighbor_keys, grid, options))
        {
            continue;
        }

        patch_points.push_back(center);
        if (patch_points.size() >= max_patch_points)
            break;
    }

    return patch_points;
}

inline void appendGroundGridPatches(
    pcl::PointCloud<GroundGridPointType>::Ptr cloud,
    const GroundGridPatchOptions& options)
{
    if (cloud == nullptr)
        return;

    const auto patch_points = generateGroundGridPatches(*cloud, options);
    for (const auto& point : patch_points)
        cloud->push_back(point);
}

}  // namespace lio_sam_hesai
