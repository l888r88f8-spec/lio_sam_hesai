#include "lio_sam/ground_grid_processor.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lio_sam_hesai {
namespace {

// Grid indices are kept in a wide integer type so normal metric coordinates do
// not overflow a 32-bit cell index. 2^53 is the largest integer a double can
// represent exactly; coordinates beyond it cannot be mapped reliably.
constexpr std::int64_t kMaxGridIndex = std::int64_t{1} << 53;

struct Key {
  std::int64_t ix = 0;
  std::int64_t iy = 0;

  bool operator==(const Key& other) const noexcept {
    return ix == other.ix && iy == other.iy;
  }
};

struct KeyHash {
  std::size_t operator()(const Key& key) const noexcept {
    const std::size_t h1 = std::hash<std::int64_t>{}(key.ix);
    const std::size_t h2 = std::hash<std::int64_t>{}(key.iy);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

struct KeyLess {
  bool operator()(const Key& lhs, const Key& rhs) const noexcept {
    return lhs.ix != rhs.ix ? lhs.ix < rhs.ix : lhs.iy < rhs.iy;
  }
};

struct BucketKey {
  std::int64_t ix = 0;
  std::int64_t iy = 0;

  bool operator==(const BucketKey& other) const noexcept {
    return ix == other.ix && iy == other.iy;
  }
};

struct BucketKeyHash {
  std::size_t operator()(const BucketKey& key) const noexcept {
    const std::size_t h1 = std::hash<std::int64_t>{}(key.ix);
    const std::size_t h2 = std::hash<std::int64_t>{}(key.iy);
    return h1 ^ (h2 + 0x9e3779b97f4a7c15ULL + (h1 << 6) + (h1 >> 2));
  }
};

struct Layer {
  int count = 0;
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  std::size_t begin = 0;
  std::size_t end = 0;
};

struct Cell {
  // Input indices only, never copied point records. Cleared after selection.
  std::vector<int> point_indices;
  std::vector<Layer> layers;
  bool has_choice = false;
  double representative_x = 0.0;
  double representative_y = 0.0;
  double representative_z = 0.0;
  bool has_local_plane = false;
  double local_plane_z = 0.0;
  bool output = false;
  double output_z = 0.0;
};

using CellMap = std::unordered_map<Key, Cell, KeyHash>;
using BucketMap =
    std::unordered_map<BucketKey, std::vector<Key>, BucketKeyHash>;

struct Sample {
  double dx = 0.0;
  double dy = 0.0;
  double z = 0.0;
};

struct LocalPlane {
  double a = 0.0;
  double b = 0.0;
  double c = 0.0;
};

bool IsFinite(const GroundGridPointType& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

bool KeyForPoint(const GroundGridPointType& point, double resolution,
    Key* key) {
  if (key == nullptr) {
    return false;
  }

  const double grid_x =
      std::floor(static_cast<double>(point.x) / resolution);
  const double grid_y =
      std::floor(static_cast<double>(point.y) / resolution);
  if (!std::isfinite(grid_x) || !std::isfinite(grid_y)) {
    return false;
  }
  if (grid_x < -static_cast<double>(kMaxGridIndex) ||
      grid_x > static_cast<double>(kMaxGridIndex) ||
      grid_y < -static_cast<double>(kMaxGridIndex) ||
      grid_y > static_cast<double>(kMaxGridIndex)) {
    return false;
  }
  key->ix = static_cast<std::int64_t>(grid_x);
  key->iy = static_cast<std::int64_t>(grid_y);
  return true;
}

double CellCenterX(const Key& key, double resolution) {
  return (static_cast<double>(key.ix) + 0.5) * resolution;
}

double CellCenterY(const Key& key, double resolution) {
  return (static_cast<double>(key.iy) + 0.5) * resolution;
}

GroundGridPointType CellCenter(const Key& key, double resolution, double z) {
  GroundGridPointType point;
  point.x = static_cast<float>(CellCenterX(key, resolution));
  point.y = static_cast<float>(CellCenterY(key, resolution));
  point.z = static_cast<float>(z);
  point.intensity = 0.0f;
  return point;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }

  const std::size_t middle = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + middle, values.end());
  double result = values[middle];
  if (values.size() % 2 == 0) {
    const auto lower =
        std::max_element(values.begin(), values.begin() + middle);
    result = (*lower + result) * 0.5;
  }
  return result;
}

void BuildLayers(Cell* cell,
    const pcl::PointCloud<GroundGridPointType>& input,
    const GroundGridExportOptions& options) {
  auto& indices = cell->point_indices;
  std::sort(indices.begin(), indices.end(), [&input](int lhs, int rhs) {
        const float lhs_z = input.points[lhs].z;
        const float rhs_z = input.points[rhs].z;
        return lhs_z != rhs_z ? lhs_z < rhs_z : lhs < rhs;
      });

  const double max_span =
      std::max(0.001, static_cast<double>(options.max_height_range));
  std::size_t start = 0;
  for (std::size_t index = 1; index <= indices.size(); ++index) {
    const bool boundary =
        index == indices.size() ||
        static_cast<double>(input.points[indices[index]].z) -
        static_cast<double>(input.points[indices[start]].z) >
        max_span;
    if (!boundary) {
      continue;
    }

    Layer layer;
    layer.count = static_cast<int>(index - start);
    layer.begin = start;
    layer.end = index;
    for (std::size_t offset = start; offset < index; ++offset) {
      const GroundGridPointType& point = input.points[indices[offset]];
      layer.sum_x += point.x;
      layer.sum_y += point.y;
      layer.sum_z += point.z;
    }
    cell->layers.push_back(layer);
    start = index;
  }
}

bool SolvePlane(const std::vector<Sample>& samples, LocalPlane* plane);
double PlaneResidual(const LocalPlane& plane, const Sample& sample);

void ChooseLayer(Cell* cell, const Key& key,
    const GroundGridExportOptions& options,
    const pcl::PointCloud<GroundGridPointType>& input) {
  if (cell->layers.empty()) {
    return;
  }

  // A real ramp can span several absolute height bands inside one cell.
  // Accept a consistent plane before testing for competing horizontal layers.
  std::vector<Sample> cell_samples;
  const double center_x = CellCenterX(key, options.resolution);
  const double center_y = CellCenterY(key, options.resolution);
  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  cell_samples.reserve(cell->point_indices.size());
  for (int point_index : cell->point_indices) {
    const auto& point = input.points[point_index];
    cell_samples.push_back({point.x - center_x, point.y - center_y, point.z});
    sum_x += point.x;
    sum_y += point.y;
    sum_z += point.z;
  }
  LocalPlane cell_plane;
  if (SolvePlane(cell_samples, &cell_plane)) {
    double max_residual = 0.0;
    for (const auto& sample : cell_samples) {
      max_residual = std::max(max_residual,
              std::fabs(PlaneResidual(cell_plane, sample)));
    }
    const double slope = std::atan(std::hypot(cell_plane.a, cell_plane.b)) *
        180.0 / M_PI;
    if (max_residual <= options.max_roughness &&
        max_residual <= options.plane_distance_threshold &&
        slope <= options.max_slope_deg) {
      const double count = static_cast<double>(cell_samples.size());
      cell->representative_x = sum_x / count;
      cell->representative_y = sum_y / count;
      cell->representative_z = sum_z / count;
      cell->has_choice = true;
      cell->has_local_plane = true;
      cell->local_plane_z = cell_plane.c;
      return;
    }
  }

  int best = 0;
  int second = -1;
  for (int index = 1; index < static_cast<int>(cell->layers.size()); ++index) {
    if (cell->layers[index].count > cell->layers[best].count) {
      second = best;
      best = index;
    } else if (second < 0 ||
        cell->layers[index].count > cell->layers[second].count) {
      second = index;
    }
  }

  const int total = static_cast<int>(cell->point_indices.size());
  const int best_count = cell->layers[best].count;
  if (second >= 0) {
    const int second_count = cell->layers[second].count;
    if (best_count == second_count) {
      return;
    }
    if (second_count >= 2 && best_count * 4LL < total * 3LL) {
      return;
    }
  }

  const Layer& chosen = cell->layers[best];
  if (chosen.count <= 0) {
    return;
  }

  cell->representative_x =
      chosen.sum_x / static_cast<double>(chosen.count);
  cell->representative_y =
      chosen.sum_y / static_cast<double>(chosen.count);
  cell->representative_z =
      chosen.sum_z / static_cast<double>(chosen.count);
  cell->has_choice = std::isfinite(cell->representative_x) &&
      std::isfinite(cell->representative_y) &&
      std::isfinite(cell->representative_z);
  if (!cell->has_choice) {
    return;
  }

  // Keep a local plane fitted only to the selected layer. This is a fallback
  // for cells with too few neighbours and never mixes in another layer.
  if (chosen.end - chosen.begin < 3) {
    return;
  }
  std::vector<Sample> local_samples;
  local_samples.reserve(chosen.end - chosen.begin);
  for (std::size_t offset = chosen.begin; offset < chosen.end; ++offset) {
    const GroundGridPointType& point =
        input.points[cell->point_indices[offset]];
    local_samples.push_back(
      {point.x - center_x, point.y - center_y, point.z});
  }

  LocalPlane local_plane;
  if (!SolvePlane(local_samples, &local_plane)) {
    return;
  }
  double max_residual = 0.0;
  for (const Sample& sample : local_samples) {
    max_residual = std::max(
      max_residual, std::fabs(PlaneResidual(local_plane, sample)));
  }
  const double slope =
      std::atan(std::sqrt(local_plane.a * local_plane.a +
          local_plane.b * local_plane.b)) *
      180.0 / M_PI;
  if (std::isfinite(local_plane.c) &&
      max_residual <= options.plane_distance_threshold &&
      max_residual <= options.max_roughness &&
      std::isfinite(slope) && slope <= options.max_slope_deg) {
    cell->has_local_plane = true;
    cell->local_plane_z = local_plane.c;
  }
}

CellMap BuildCells(const pcl::PointCloud<GroundGridPointType>& input,
    const GroundGridExportOptions& options, bool* had_finite,
    bool* had_unrepresentable) {
  CellMap cells;
  for (int index = 0; index < static_cast<int>(input.size()); ++index) {
    const GroundGridPointType& point = input.points[index];
    if (!IsFinite(point)) {
      continue;
    }
    if (had_finite != nullptr) {
      *had_finite = true;
    }
    Key key;
    if (!KeyForPoint(point, static_cast<double>(options.resolution), &key)) {
      if (had_unrepresentable != nullptr) {
        *had_unrepresentable = true;
      }
      continue;
    }
    cells[key].point_indices.push_back(index);
  }

  for (auto& entry : cells) {
    Cell& cell = entry.second;
    BuildLayers(&cell, input, options);
    ChooseLayer(&cell, entry.first, options, input);
    std::vector<int>().swap(cell.point_indices);
    std::vector<Layer>().swap(cell.layers);
  }
  return cells;
}

class SpatialIndex {
public:
void Build(const CellMap& cells, const GroundGridExportOptions& options) {
  resolution_ = static_cast<double>(options.resolution);
  search_radius_ = static_cast<double>(options.search_radius);
  bucket_size_ = std::max(search_radius_, resolution_);
  for (const auto& entry : cells) {
    if (!entry.second.has_choice) {
      continue;
    }
    BucketKey bucket;
    if (!BucketFor(entry.first, &bucket)) {
      unindexed_.push_back(entry.first);
      continue;
    }
    buckets_[bucket].push_back(entry.first);
  }
}

std::vector<Key> Neighbors(const Key& key, const CellMap& cells,
    bool require_output) const {
  std::vector<Key> keys;
  BucketKey bucket;
  if (BucketFor(key, &bucket)) {
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        const BucketKey neighbor_bucket{bucket.ix + dx,
                                        bucket.iy + dy};
        const auto iter = buckets_.find(neighbor_bucket);
        if (iter == buckets_.end()) {
          continue;
        }
        for (const Key& candidate : iter->second) {
          AddIfNeighbor(key, candidate, cells, require_output,
              &keys);
        }
      }
    }
    for (const Key& candidate : unindexed_) {
      AddIfNeighbor(key, candidate, cells, require_output, &keys);
    }
  } else{
    for (const auto& entry : cells) {
      if (!entry.second.has_choice) {
        continue;
      }
      AddIfNeighbor(key, entry.first, cells, require_output, &keys);
    }
  }

  std::sort(keys.begin(), keys.end(), KeyLess{});
  return keys;
}

private:
bool BucketFor(const Key& key, BucketKey* bucket) const {
  if (bucket == nullptr || !(bucket_size_ > 0.0)) {
    return false;
  }
  const double grid_x = std::floor(CellCenterX(key, resolution_) /
          bucket_size_);
  const double grid_y = std::floor(CellCenterY(key, resolution_) /
          bucket_size_);
  if (!std::isfinite(grid_x) || !std::isfinite(grid_y)) {
    return false;
  }
  if (grid_x < -static_cast<double>(kMaxGridIndex) ||
      grid_x > static_cast<double>(kMaxGridIndex) ||
      grid_y < -static_cast<double>(kMaxGridIndex) ||
      grid_y > static_cast<double>(kMaxGridIndex)) {
    return false;
  }
  bucket->ix = static_cast<std::int64_t>(grid_x);
  bucket->iy = static_cast<std::int64_t>(grid_y);
  return true;
}

void AddIfNeighbor(const Key& key, const Key& candidate,
    const CellMap& cells, bool require_output,
    std::vector<Key>* keys) const {
  if (candidate == key) {
    return;
  }
  const auto iter = cells.find(candidate);
  if (iter == cells.end() || !iter->second.has_choice) {
    return;
  }
  if (require_output && !iter->second.output) {
    return;
  }

  const double diff_x = CellCenterX(candidate, resolution_) -
      CellCenterX(key, resolution_);
  const double diff_y = CellCenterY(candidate, resolution_) -
      CellCenterY(key, resolution_);
  if (diff_x * diff_x + diff_y * diff_y <=
      search_radius_ * search_radius_) {
    keys->push_back(candidate);
  }
}

double resolution_ = 1.0;
double search_radius_ = 0.0;
double bucket_size_ = 1.0;
BucketMap buckets_;
std::vector<Key> unindexed_;
};

std::vector<Sample> CollectSamples(const std::vector<Key>& keys,
    const Key& center, const CellMap& cells,
    double resolution, bool require_output) {
  std::vector<Sample> samples;
  samples.reserve(keys.size());
  const double center_x = CellCenterX(center, resolution);
  const double center_y = CellCenterY(center, resolution);
  for (const Key& key : keys) {
    const auto iter = cells.find(key);
    if (iter == cells.end() || !iter->second.has_choice) {
      continue;
    }
    if (require_output && !iter->second.output) {
      continue;
    }
    const Cell& cell = iter->second;
    samples.push_back({cell.representative_x - center_x,
                       cell.representative_y - center_y,
                       cell.representative_z});
  }
  return samples;
}

bool SolvePlane(const std::vector<Sample>& samples, LocalPlane* plane) {
  if (plane == nullptr || samples.size() < 3) {
    return false;
  }

  Eigen::MatrixXd design(static_cast<Eigen::Index>(samples.size()), 3);
  Eigen::VectorXd rhs(static_cast<Eigen::Index>(samples.size()));
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const Eigen::Index row = static_cast<Eigen::Index>(index);
    design(row, 0) = samples[index].dx;
    design(row, 1) = samples[index].dy;
    design(row, 2) = 1.0;
    rhs(row) = samples[index].z;
  }

  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> decomposition(design);
  if (decomposition.rank() < 3) {
    return false;
  }
  const Eigen::Vector3d solution = decomposition.solve(rhs);
  if (!solution.allFinite()) {
    return false;
  }

  plane->a = solution(0);
  plane->b = solution(1);
  plane->c = solution(2);
  return std::isfinite(plane->a) && std::isfinite(plane->b) &&
         std::isfinite(plane->c);
}

double PlaneResidual(const LocalPlane& plane, const Sample& sample) {
  return plane.a * sample.dx + plane.b * sample.dy + plane.c - sample.z;
}

bool FitPlane(const std::vector<Sample>& samples,
    const GroundGridExportOptions& options, LocalPlane* plane) {
  const int minimum = std::max(3, options.min_neighbor_cells);
  if (static_cast<int>(samples.size()) < minimum) {
    return false;
  }

  std::vector<Sample> inliers = samples;
  LocalPlane fit;
  for (int iteration = 0; iteration < 5; ++iteration) {
    if (!SolvePlane(inliers, &fit)) {
      return false;
    }

    std::vector<Sample> next;
    next.reserve(inliers.size());
    for (const Sample& sample : inliers) {
      if (std::fabs(PlaneResidual(fit, sample)) <=
          options.plane_distance_threshold) {
        next.push_back(sample);
      }
    }
    if (static_cast<int>(next.size()) < minimum) {
      return false;
    }
    const bool converged = next.size() == inliers.size();
    inliers.swap(next);
    if (converged) {
      break;
    }
  }

  if (!SolvePlane(inliers, &fit)) {
    return false;
  }

  const double inlier_ratio =
      static_cast<double>(inliers.size()) /
      static_cast<double>(samples.size());
  if (inlier_ratio < static_cast<double>(options.min_plane_inlier_ratio)) {
    return false;
  }

  double residual_sq = 0.0;
  double max_residual = 0.0;
  for (const Sample& sample : inliers) {
    const double residual = PlaneResidual(fit, sample);
    residual_sq += residual * residual;
    max_residual = std::max(max_residual, std::fabs(residual));
  }
  const double rmse =
      std::sqrt(residual_sq / static_cast<double>(inliers.size()));
  if (!std::isfinite(rmse) || !std::isfinite(max_residual) ||
      max_residual > options.plane_distance_threshold ||
      max_residual > options.max_roughness ||
      rmse > options.max_roughness) {
    return false;
  }

  const double slope =
      std::atan(std::sqrt(fit.a * fit.a + fit.b * fit.b)) * 180.0 /
      M_PI;
  if (!std::isfinite(slope) || slope > options.max_slope_deg) {
    return false;
  }

  *plane = fit;
  return true;
}

bool PredictedWithinSupport(double predicted,
    const std::vector<Sample>& samples,
    const GroundGridExportOptions& options) {
  if (samples.empty()) {
    return false;
  }
  double min_z = std::numeric_limits<double>::max();
  double max_z = std::numeric_limits<double>::lowest();
  for (const Sample& sample : samples) {
    min_z = std::min(min_z, sample.z);
    max_z = std::max(max_z, sample.z);
  }
  return predicted >= min_z - options.max_prediction_margin &&
         predicted <= max_z + options.max_prediction_margin;
}

double SampleMedian(const std::vector<Sample>& samples) {
  std::vector<double> z_values;
  z_values.reserve(samples.size());
  for (const Sample& sample : samples) {
    z_values.push_back(sample.z);
  }
  return Median(std::move(z_values));
}

double OutlierMargin(const GroundGridExportOptions& options) {
  return std::max(3.0 * options.plane_distance_threshold,
             2.0 * options.max_height_range);
}

bool HasEnoughDirectionalSupport(
  const Key& key, const std::vector<Key>& neighbors,
  const GroundGridExportOptions& options) {
  if (static_cast<int>(neighbors.size()) < options.min_neighbor_cells) {
    return false;
  }

  bool left = false;
  bool right = false;
  bool down = false;
  bool up = false;
  bool quadrants[4] = {false, false, false, false};
  for (const Key& neighbor : neighbors) {
    const std::int64_t dx = neighbor.ix - key.ix;
    const std::int64_t dy = neighbor.iy - key.iy;
    left = left || dx < 0;
    right = right || dx > 0;
    down = down || dy < 0;
    up = up || dy > 0;

    if (dx < 0 && dy < 0) {
      quadrants[0] = true;
    } else if (dx < 0 && dy > 0) {
      quadrants[1] = true;
    } else if (dx > 0 && dy < 0) {
      quadrants[2] = true;
    } else if (dx > 0 && dy > 0) {
      quadrants[3] = true;
    }
  }

  const int quadrant_count =
      static_cast<int>(quadrants[0]) + static_cast<int>(quadrants[1]) +
      static_cast<int>(quadrants[2]) + static_cast<int>(quadrants[3]);
  if (quadrant_count < options.min_support_quadrants) {
    return false;
  }

  return left && right && down && up;
}

void SelectInitialOutput(const GroundGridExportOptions& options,
    const SpatialIndex& index, CellMap* cells) {
  for (auto& entry : *cells) {
    Cell& cell = entry.second;
    if (!cell.has_choice) {
      continue;
    }
    cell.output = true;
    cell.output_z = cell.representative_z;
  }

  // Existing measurements need three non-collinear supports; empty cells
  // still require the stricter configured support in FillUnobservedGaps.
  GroundGridExportOptions fit_options = options;
  fit_options.min_neighbor_cells = 3;
  const int minimum = 3;
  const double margin = OutlierMargin(options);
  for (auto& entry : *cells) {
    Cell& cell = entry.second;
    if (!cell.has_choice) {
      continue;
    }
    const auto neighbors = index.Neighbors(entry.first, *cells, false);
    const auto samples =
        CollectSamples(neighbors, entry.first, *cells,
            static_cast<double>(options.resolution), false);
    if (static_cast<int>(samples.size()) < minimum) {
      if (cell.has_local_plane) {
        cell.output_z = cell.local_plane_z;
      }
      continue;
    }

    LocalPlane plane;
    // Existing measurements anchor the plane at their actual XY. Predicting
    // their own cell center may exceed neighbor Z at the edge of a real ramp.
    if (FitPlane(samples, fit_options, &plane)) {
      const Sample measured{
        cell.representative_x - CellCenterX(entry.first, options.resolution),
        cell.representative_y - CellCenterY(entry.first, options.resolution),
        cell.representative_z};
      const double residual = std::fabs(PlaneResidual(plane, measured));
      if (residual > margin) {
        cell.output = false;
        continue;
      }
      if (residual <= options.max_roughness &&
          residual <= options.plane_distance_threshold) {
        cell.output_z = plane.c;
      } else if (cell.has_local_plane) {
        cell.output_z = cell.local_plane_z;
      }
      continue;
    }

    const double median_z = SampleMedian(samples);
    if (std::fabs(cell.representative_z - median_z) > margin) {
      cell.output = false;
      continue;
    }
    // Keep the measured layer: never turn a step into a slope.
    if (cell.has_local_plane) {
      cell.output_z = cell.local_plane_z;
    }
  }
}

void RefineOutputPlanes(const GroundGridExportOptions& options,
    const SpatialIndex& index, CellMap* cells) {
  // Existing measurements need three non-collinear supports; empty cells
  // still require the stricter configured support in FillUnobservedGaps.
  GroundGridExportOptions fit_options = options;
  fit_options.min_neighbor_cells = 3;
  const int minimum = 3;
  for (auto& entry : *cells) {
    Cell& cell = entry.second;
    if (!cell.output) {
      continue;
    }
    const auto neighbors = index.Neighbors(entry.first, *cells, true);
    const auto samples =
        CollectSamples(neighbors, entry.first, *cells,
            static_cast<double>(options.resolution), true);
    if (static_cast<int>(samples.size()) < minimum) {
      continue;
    }

    LocalPlane plane;
    if (!FitPlane(samples, fit_options, &plane)) {
      continue;
    }
    const Sample measured{
      cell.representative_x - CellCenterX(entry.first, options.resolution),
      cell.representative_y - CellCenterY(entry.first, options.resolution),
      cell.representative_z};
    const double residual = std::fabs(PlaneResidual(plane, measured));
    if (residual <= options.max_roughness &&
        residual <= options.plane_distance_threshold) {
      cell.output_z = plane.c;
    }
  }
}

void AddObservedOutput(const CellMap& cells, double resolution,
    std::map<Key, GroundGridPointType, KeyLess>* output) {
  for (const auto& entry : cells) {
    const Cell& cell = entry.second;
    if (!cell.output || !std::isfinite(cell.output_z)) {
      continue;
    }
    const GroundGridPointType point =
        CellCenter(entry.first, resolution, cell.output_z);
    if (IsFinite(point)) {
      (*output)[entry.first] = point;
    }
  }
}

enum class GapState {
  kFilled,
  kRejected,
};

using GapStateMap = std::unordered_map<Key, GapState, KeyHash>;

// Discover one complete four-connected empty component from an observed
// boundary. The search is capped at max_hole_cells + 1 so an unbounded or
// huge empty region is rejected without allocating its bounding box.
bool DiscoverHole(const Key& seed, const CellMap& cells, int max_hole_cells,
    GapStateMap* explored, std::vector<Key>* component,
    std::set<Key, KeyLess>* boundary) {
  const std::array<Key, 4> steps = {
    Key{1, 0}, Key{-1, 0}, Key{0, 1}, Key{0, -1},
  };
  std::vector<Key> queue;
  std::unordered_map<Key, bool, KeyHash> visited;
  queue.push_back(seed);
  visited[seed] = true;

  const auto reject_component = [&]() {
    for (const auto& entry : visited) {
      (*explored)[entry.first] = GapState::kRejected;
    }
    return false;
  };

  std::size_t head = 0;
  while (head < queue.size()) {
    // At most max_hole_cells cells are accepted; one extra queued cell means
    // the component is too large and must stay completely empty.
    if (static_cast<int>(component->size()) >= max_hole_cells) {
      return reject_component();
    }

    const Key key = queue[head++];
    component->push_back(key);
    for (const Key& step : steps) {
      const Key neighbor{key.ix + step.ix, key.iy + step.iy};
      const auto cell = cells.find(neighbor);
      if (cell != cells.end()) {
        if (!cell->second.output) {
          // Touching a rejected observed cell means the enclosure is not
          // trustworthy, even when the empty component itself is small.
          return reject_component();
        }
        boundary->insert(neighbor);
        continue;
      }

      if (explored->find(neighbor) != explored->end()) {
        // A previous over-limit or invalid component owns this empty area.
        // Never fill just the tail that happens to fit inside the limit.
        return reject_component();
      }
      if (visited.find(neighbor) == visited.end()) {
        if (visited.size() >=
            static_cast<std::size_t>(max_hole_cells) + 1U) {
          return reject_component();
        }
        visited[neighbor] = true;
        queue.push_back(neighbor);
      }
    }
  }
  return !component->empty();
}

void FillUnobservedGaps(const GroundGridExportOptions& options,
    const SpatialIndex& index, const CellMap& cells,
    double resolution,
    std::map<Key, GroundGridPointType, KeyLess>* output) {
  if (!options.fill_unobserved_gaps) {
    return;
  }

  const int minimum = std::max(3, options.min_neighbor_cells);
  const std::array<Key, 4> steps = {
    Key{1, 0}, Key{-1, 0}, Key{0, 1}, Key{0, -1},
  };
  GapStateMap explored;
  for (const auto& entry : cells) {
    if (!entry.second.output) {
      continue;
    }
    for (const Key& step : steps) {
      const Key seed{entry.first.ix + step.ix, entry.first.iy + step.iy};
      if (cells.find(seed) != cells.end() ||
          explored.find(seed) != explored.end()) {
        continue;
      }

      std::vector<Key> component;
      std::set<Key, KeyLess> boundary;
      if (!DiscoverHole(seed, cells, options.max_hole_cells, &explored,
              &component, &boundary)) {
        // The discovered cells are cached as rejected so a later boundary
        // seed cannot refill part of the same large/open component.
        continue;
      }

      const auto reject_hole = [&]() {
        for (const Key& key : component) {
          explored[key] = GapState::kRejected;
        }
      };

      Key reference = component.front();
      std::int64_t min_ix = reference.ix;
      std::int64_t max_ix = reference.ix;
      std::int64_t min_iy = reference.iy;
      std::int64_t max_iy = reference.iy;
      for (const Key& key : component) {
        min_ix = std::min(min_ix, key.ix);
        max_ix = std::max(max_ix, key.ix);
        min_iy = std::min(min_iy, key.iy);
        max_iy = std::max(max_iy, key.iy);
      }
      reference = Key{min_ix + (max_ix - min_ix) / 2,
                      min_iy + (max_iy - min_iy) / 2};

      // Merge the original accepted boundary cells with each hole cell's
      // radius support. Filled output points are never fitting evidence and
      // the cell map/index stay immutable.
      std::set<Key, KeyLess> support = boundary;
      for (const Key& key : component) {
        const auto neighbors = index.Neighbors(key, cells, true);
        support.insert(neighbors.begin(), neighbors.end());
      }
      const std::vector<Key> support_keys(support.begin(), support.end());
      if (!HasEnoughDirectionalSupport(reference, support_keys, options)) {
        reject_hole();
        continue;
      }

      const auto samples = CollectSamples(support_keys, reference, cells,
              resolution, true);
      if (static_cast<int>(samples.size()) < minimum) {
        reject_hole();
        continue;
      }

      LocalPlane plane;
      if (!FitPlane(samples, options, &plane) ||
          !PredictedWithinSupport(plane.c, samples, options)) {
        reject_hole();
        continue;
      }

      const double reference_x = CellCenterX(reference, resolution);
      const double reference_y = CellCenterY(reference, resolution);
      // A robust fit may discard a raised boundary as an outlier. Every direct
      // boundary cell must agree with the plane so a step is never bridged.
      bool consistent = true;
      for (const Key& key : boundary) {
        const auto cell = cells.find(key);
        if (cell == cells.end() || !cell->second.output) {
          consistent = false;
          break;
        }
        const Cell& measured_cell = cell->second;
        const Sample measured{
          measured_cell.representative_x - reference_x,
          measured_cell.representative_y - reference_y,
          measured_cell.representative_z};
        const double residual = std::fabs(PlaneResidual(plane, measured));
        if (residual > options.max_roughness ||
            residual > options.plane_distance_threshold) {
          consistent = false;
          break;
        }
      }
      if (!consistent) {
        reject_hole();
        continue;
      }

      const double support_median = SampleMedian(samples);
      if (std::fabs(plane.c - support_median) >
          options.max_prediction_margin + options.max_height_range) {
        reject_hole();
        continue;
      }

      // Stage every prediction before committing anything. If one cell is
      // invalid the whole component stays empty instead of being filled in
      // pieces.
      std::vector<std::pair<Key, GroundGridPointType>> staged;
      staged.reserve(component.size());
      bool valid = true;
      for (const Key& key : component) {
        const double dx = CellCenterX(key, resolution) - reference_x;
        const double dy = CellCenterY(key, resolution) - reference_y;
        const double predicted = plane.a * dx + plane.b * dy + plane.c;
        if (!std::isfinite(predicted) ||
            !PredictedWithinSupport(predicted, samples, options)) {
          valid = false;
          break;
        }
        const GroundGridPointType point =
            CellCenter(key, resolution, predicted);
        if (!IsFinite(point)) {
          valid = false;
          break;
        }
        staged.emplace_back(key, point);
      }
      if (!valid) {
        reject_hole();
        continue;
      }

      for (const auto& item : staged) {
        (*output)[item.first] = item.second;
      }
      for (const Key& key : component) {
        explored[key] = GapState::kFilled;
      }
    }
  }
}

}  // namespace

bool BuildGroundGrid(const pcl::PointCloud<GroundGridPointType>& input,
    const GroundGridExportOptions& options,
    pcl::PointCloud<GroundGridPointType>* output,
    std::string* error) {
  if (output == nullptr) {
    return false;
  }
  output->clear();
  if (error != nullptr) {
    error->clear();
  }

  if (!std::isfinite(options.resolution) || options.resolution <= 0.0f) {
    if (error != nullptr) {
      *error = "ground grid resolution must be positive and finite";
    }
    return false;
  }
  if (!std::isfinite(options.search_radius) || options.search_radius <= 0.0f) {
    if (error != nullptr) {
      *error = "ground grid search radius must be positive and finite";
    }
    return false;
  }
  if (options.max_hole_cells < 1) {
    if (error != nullptr) {
      *error = "ground grid max hole cells must be at least one";
    }
    return false;
  }

  bool had_finite = false;
  bool had_unrepresentable = false;
  CellMap cells = BuildCells(input, options, &had_finite,
          &had_unrepresentable);
  if (had_finite && cells.empty() && had_unrepresentable) {
    if (error != nullptr) {
      *error =
          "ground grid coordinates exceed the supported cell index range";
    }
    return false;
  }

  SpatialIndex index;
  index.Build(cells, options);

  SelectInitialOutput(options, index, &cells);
  RefineOutputPlanes(options, index, &cells);

  std::map<Key, GroundGridPointType, KeyLess> result;
  AddObservedOutput(cells, static_cast<double>(options.resolution), &result);
  FillUnobservedGaps(options, index, cells,
      static_cast<double>(options.resolution), &result);

  output->reserve(result.size());
  for (const auto& entry : result) {
    output->push_back(entry.second);
  }
  return true;
}

}  // namespace lio_sam_hesai
