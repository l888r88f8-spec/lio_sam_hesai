#include "lio_sam/ground_mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lio_sam_hesai {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kGeometryEpsilon = 1.0e-9;
constexpr double kRangeEpsilon = 1.0e-6;
// Scan coordinates are floats; include shared edges despite float rounding.
constexpr double kCellEpsilon = 1.0e-6;
constexpr double kMaxTriangleCells = 1000000.0;
constexpr std::size_t kMaxFrameCells = 2000000;
constexpr std::size_t kMaxWorldCells = 4000000;

bool IsFinite(double value) {
  return std::isfinite(value);
}

void ValidateOptions(const GroundMeshOptions &options) {
  if (!IsFinite(options.resolution) || options.resolution <= 0.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.resolution must be positive and finite");
  }
  if (!IsFinite(options.max_edge_length) || options.max_edge_length <= 0.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.max_edge_length must be positive and finite");
  }
  if (!IsFinite(options.edge_length_per_meter) ||
      options.edge_length_per_meter < 0.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.edge_length_per_meter must be finite and "
            "non-negative");
  }
  if (!IsFinite(options.max_range_ratio) || options.max_range_ratio < 1.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.max_range_ratio must be finite and at least 1");
  }
  if (!IsFinite(options.max_slope_deg) || options.max_slope_deg <= 0.0f ||
      options.max_slope_deg >= 90.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.max_slope_deg must be in (0, 90)");
  }
  if (!IsFinite(options.max_height_range) ||
      options.max_height_range <= 0.0f) {
    throw std::invalid_argument(
            "GroundMeshOptions.max_height_range must be positive and finite");
  }
}

void ValidatePose(const Eigen::Matrix4f &map_from_scan) {
  if (!map_from_scan.allFinite()) {
    throw std::invalid_argument(
        "map_from_scan must contain only finite values");
  }

  const Eigen::Matrix3f rotation = map_from_scan.block<3, 3>(0, 0);
  const Eigen::Matrix3f orthogonality = rotation.transpose() * rotation;
  if (!orthogonality.isApprox(Eigen::Matrix3f::Identity(), 1.0e-3f)) {
    throw std::invalid_argument(
            "map_from_scan rotation must be orthonormal");
  }
  if (std::abs(rotation.determinant() - 1.0f) > 1.0e-3f) {
    throw std::invalid_argument(
            "map_from_scan rotation must have unit determinant");
  }
  if (std::abs(map_from_scan(3, 0)) > 1.0e-6f ||
      std::abs(map_from_scan(3, 1)) > 1.0e-6f ||
      std::abs(map_from_scan(3, 2)) > 1.0e-6f ||
      std::abs(map_from_scan(3, 3) - 1.0f) > 1.0e-6f) {
    throw std::invalid_argument(
            "map_from_scan must be a homogeneous rigid transform");
  }
}

void ValidateFrameDimensions(
  int rows, int columns,
  const std::vector<std::uint32_t> &projection_indices,
  const std::vector<std::int8_t> &classification) {
  if (rows <= 0 || columns <= 0) {
    throw std::invalid_argument("projection rows and columns must be positive");
  }

  const auto row_count = static_cast<std::size_t>(rows);
  const auto column_count = static_cast<std::size_t>(columns);
  if (row_count > std::numeric_limits<std::size_t>::max() / column_count) {
    throw std::invalid_argument("rows * columns overflows size_t");
  }
  const std::size_t cell_count = row_count * column_count;
  if (cell_count > std::numeric_limits<std::uint32_t>::max()) {
    throw std::invalid_argument("rows * columns exceeds uint32 index range");
  }
  if (classification.size() != cell_count) {
    throw std::invalid_argument(
            "classification size must equal rows * columns");
  }
  if (projection_indices.size() > cell_count) {
    throw std::invalid_argument(
            "projection_indices cannot exceed rows * columns");
  }

  std::unordered_set<std::uint32_t> seen;
  seen.reserve(projection_indices.size());
  for (const std::uint32_t index : projection_indices) {
    if (static_cast<std::size_t>(index) >= cell_count) {
      throw std::invalid_argument("projection index out of range");
    }
    if (!seen.insert(index).second) {
      throw std::invalid_argument("duplicate projection index");
    }
  }
}

bool IsFinitePoint(const pcl::PointXYZI &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z);
}

struct CellKey {
  std::int64_t x = 0;
  std::int64_t y = 0;

  bool operator<(const CellKey &other) const {
    return x < other.x || (x == other.x && y < other.y);
  }

  bool operator==(const CellKey &other) const {
    return x == other.x && y == other.y;
  }
};

struct CellKeyHash {
  std::size_t operator()(const CellKey &key) const {
    const std::uint64_t x = static_cast<std::uint64_t>(key.x);
    const std::uint64_t y = static_cast<std::uint64_t>(key.y);
    const std::uint64_t mixed =
        x * 0x9e3779b97f4a7c15ULL ^ (y + 0x9e3779b97f4a7c15ULL +
        (x << 6U) + (x >> 2U));
    return static_cast<std::size_t>(mixed);
  }
};

bool WorldCellIndex(double coordinate, double resolution,
    std::int64_t *index) {
  if (!IsFinite(coordinate) || !IsFinite(resolution) || resolution <= 0.0) {
    return false;
  }
  const double scaled = std::floor(coordinate / resolution);
  if (!IsFinite(scaled) ||
      scaled <= static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      scaled >= static_cast<double>(std::numeric_limits<std::int64_t>::max())) {
    return false;
  }
  *index = static_cast<std::int64_t>(scaled);
  return true;
}

struct HeightLayer {
  double min_z = 0.0;
  double max_z = 0.0;
  double sum_z = 0.0;
  std::size_t count = 0;
};

struct FrameCell {
  double min_z = 0.0;
  double max_z = 0.0;
  double sum_z = 0.0;
  std::size_t count = 0;
};

struct WorldVertex {
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double scan_range = 0.0;
};

WorldVertex MakeWorldVertex(const pcl::PointXYZI &point,
    const Eigen::Matrix4f &map_from_scan) {
  const Eigen::Vector3d scan(static_cast<double>(point.x),
      static_cast<double>(point.y),
      static_cast<double>(point.z));
  const Eigen::Matrix3d rotation =
      map_from_scan.block<3, 3>(0, 0).cast<double>();
  const Eigen::Vector3d translation =
      map_from_scan.block<3, 1>(0, 3).cast<double>();

  WorldVertex vertex;
  vertex.point = rotation * scan + translation;
  vertex.scan_range = scan.norm();
  return vertex;
}

struct Corner {
  bool exists = false;
  bool ground = false;
  bool finite = false;
  WorldVertex vertex;
};

}  // namespace

struct GroundMeshRasterizer::Impl {
  explicit Impl(const GroundMeshOptions &input_options)
    : options(input_options) {
  }

  void AddFrameSample(const CellKey &key, double z,
      std::map<CellKey, FrameCell> *frame_cells) {
    auto inserted = frame_cells->emplace(key, FrameCell{});
    if (inserted.second) {
      if (frame_cells->size() > kMaxFrameCells) {
        throw std::length_error("ground mesh exceeds per-frame cell limit");
      }
      ++stats.rasterized_cells;
    }
    FrameCell &cell = inserted.first->second;
    if (cell.count == 0U) {
      cell.min_z = z;
      cell.max_z = z;
      cell.sum_z = z;
    } else {
      cell.min_z = std::min(cell.min_z, z);
      cell.max_z = std::max(cell.max_z, z);
      cell.sum_z += z;
    }
    ++cell.count;
  }

  void RasterizeTriangle(const std::array<WorldVertex, 3> &vertices,
      std::map<CellKey, FrameCell> *frame_cells) {
    const double resolution = static_cast<double>(options.resolution);
    std::int64_t x_index[3] = {0, 0, 0};
    std::int64_t y_index[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i) {
      if (!WorldCellIndex(vertices[i].point.x(), resolution, &x_index[i]) ||
          !WorldCellIndex(vertices[i].point.y(), resolution, &y_index[i])) {
        return;
      }
    }

    const double x0 = vertices[0].point.x();
    const double y0 = vertices[0].point.y();
    const double z0 = vertices[0].point.z();
    const double x1 = vertices[1].point.x();
    const double y1 = vertices[1].point.y();
    const double z1 = vertices[1].point.z();
    const double x2 = vertices[2].point.x();
    const double y2 = vertices[2].point.y();
    const double z2 = vertices[2].point.z();

    std::int64_t cell_min_x = x_index[0];
    std::int64_t cell_max_x = x_index[0];
    std::int64_t cell_min_y = y_index[0];
    std::int64_t cell_max_y = y_index[0];
    for (int i = 1; i < 3; ++i) {
      cell_min_x = std::min(cell_min_x, x_index[i]);
      cell_max_x = std::max(cell_max_x, x_index[i]);
      cell_min_y = std::min(cell_min_y, y_index[i]);
      cell_max_y = std::max(cell_max_y, y_index[i]);
    }
    const double width = static_cast<double>(cell_max_x) - cell_min_x + 1.0;
    const double height = static_cast<double>(cell_max_y) - cell_min_y + 1.0;
    if (width * height > kMaxTriangleCells) {
      throw std::length_error(
          "ground mesh triangle exceeds cell limit; "
          "increase ground_resolution");
    }

    const double denominator =
        (y1 - y2) * (x0 - x2) + (x2 - x1) * (y0 - y2);
    if (std::abs(denominator) <= kGeometryEpsilon) {
      return;
    }

    for (std::int64_t cell_x = cell_min_x; cell_x <= cell_max_x; ++cell_x) {
      for (std::int64_t cell_y = cell_min_y; cell_y <= cell_max_y; ++cell_y) {
        const double center_x =
            (static_cast<double>(cell_x) + 0.5) * resolution;
        const double center_y =
            (static_cast<double>(cell_y) + 0.5) * resolution;
        const double w0 =
            ((y1 - y2) * (center_x - x2) + (x2 - x1) * (center_y - y2)) /
            denominator;
        const double w1 =
            ((y2 - y0) * (center_x - x2) + (x0 - x2) * (center_y - y2)) /
            denominator;
        const double w2 = 1.0 - w0 - w1;
        if (w0 < -kCellEpsilon || w1 < -kCellEpsilon ||
            w2 < -kCellEpsilon) {
          continue;
        }
        const double z = w0 * z0 + w1 * z1 + w2 * z2;
        if (!IsFinite(z)) {
          continue;
        }
        AddFrameSample(CellKey{cell_x, cell_y}, z, frame_cells);
      }
    }
  }

  bool RejectTriangle(const std::array<WorldVertex, 3> &vertices) {
    const Eigen::Vector3d edge01 = vertices[1].point - vertices[0].point;
    const Eigen::Vector3d edge02 = vertices[2].point - vertices[0].point;
    const Eigen::Vector3d normal = edge01.cross(edge02);
    const double normal_length = normal.norm();
    const double xy_denominator =
        (vertices[1].point.y() - vertices[2].point.y()) *
        (vertices[0].point.x() - vertices[2].point.x()) +
        (vertices[2].point.x() - vertices[1].point.x()) *
        (vertices[0].point.y() - vertices[2].point.y());
    if (!IsFinite(normal_length) || normal_length <= kGeometryEpsilon ||
        !IsFinite(xy_denominator) ||
        std::abs(xy_denominator) <= kGeometryEpsilon) {
      ++stats.rejected_degenerate;
      return true;
    }

    for (int i = 0; i < 3; ++i) {
      const int j = (i + 1) % 3;
      const double edge_length =
          (vertices[i].point - vertices[j].point).norm();
      const double range_min =
          std::min(vertices[i].scan_range, vertices[j].scan_range);
      const double limit =
          static_cast<double>(options.max_edge_length) +
          static_cast<double>(options.edge_length_per_meter) * range_min;
      if (edge_length > limit + kRangeEpsilon) {
        ++stats.rejected_edge;
        return true;
      }
    }

    const double range_min =
        std::min({vertices[0].scan_range, vertices[1].scan_range,
                  vertices[2].scan_range});
    const double range_max =
        std::max({vertices[0].scan_range, vertices[1].scan_range,
                  vertices[2].scan_range});
    if (range_min > kRangeEpsilon &&
        range_max / range_min >
        static_cast<double>(options.max_range_ratio) + kRangeEpsilon) {
      ++stats.rejected_range;
      return true;
    }

    const double cosine =
        std::clamp(std::abs(normal.z()) / normal_length, 0.0, 1.0);
    const double slope_deg = std::acos(cosine) * 180.0 / kPi;
    if (slope_deg >
        static_cast<double>(options.max_slope_deg) + kRangeEpsilon) {
      ++stats.rejected_slope;
      return true;
    }
    return false;
  }

  void ProcessTriangle(const std::array<WorldVertex, 3> &vertices,
      std::map<CellKey, FrameCell> *frame_cells) {
    ++stats.candidate_triangles;
    if (RejectTriangle(vertices)) {
      return;
    }
    ++stats.accepted_triangles;
    RasterizeTriangle(vertices, frame_cells);
  }

  void MergeFrameVotes(const std::map<CellKey, FrameCell> &frame_cells) {
    const double height_limit =
        static_cast<double>(options.max_height_range);
    for (const auto &entry : frame_cells) {
      const FrameCell &frame_cell = entry.second;
      if (frame_cell.count == 0U) {
        continue;
      }
      if (frame_cell.max_z - frame_cell.min_z > height_limit + kRangeEpsilon) {
        ++stats.ambiguous_cells;
        continue;
      }

      const double z = frame_cell.sum_z / static_cast<double>(frame_cell.count);
      std::vector<HeightLayer> &layers = global_cells[entry.first];
      if (global_cells.size() > kMaxWorldCells) {
        throw std::length_error("ground mesh exceeds global cell limit");
      }
      std::size_t best_layer = layers.size();
      double best_span = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < layers.size(); ++i) {
        const double new_min = std::min(layers[i].min_z, z);
        const double new_max = std::max(layers[i].max_z, z);
        const double span = new_max - new_min;
        if (span <= height_limit + kRangeEpsilon && span < best_span) {
          best_layer = i;
          best_span = span;
        }
      }
      if (best_layer == layers.size()) {
        HeightLayer layer;
        layer.min_z = z;
        layer.max_z = z;
        layer.sum_z = z;
        layer.count = 1U;
        layers.push_back(layer);
      } else {
        HeightLayer &layer = layers[best_layer];
        layer.min_z = std::min(layer.min_z, z);
        layer.max_z = std::max(layer.max_z, z);
        layer.sum_z += z;
        ++layer.count;
      }
    }
  }

  GroundMeshOptions options;
  GroundMeshStatistics stats;
  std::map<CellKey, std::vector<HeightLayer> > global_cells;
};

GroundMeshRasterizer::GroundMeshRasterizer(const GroundMeshOptions &options)
  : impl_(new Impl(options)) {
  ValidateOptions(options);
}

GroundMeshRasterizer::~GroundMeshRasterizer() = default;

GroundMeshRasterizer::GroundMeshRasterizer(GroundMeshRasterizer &&) noexcept =
    default;

GroundMeshRasterizer &GroundMeshRasterizer::operator=(
  GroundMeshRasterizer &&) noexcept = default;

void GroundMeshRasterizer::AddFrame(
  const pcl::PointCloud<pcl::PointXYZI> &scan,
  const std::vector<std::uint32_t> &projection_indices,
  const std::vector<std::int8_t> &classification, int rows, int columns,
  GroundRingRange output_rings, const Eigen::Matrix4f &map_from_scan) {
  if (scan.size() != projection_indices.size()) {
    throw std::invalid_argument(
            "scan and projection_indices must have the same size");
  }
  ValidateFrameDimensions(rows, columns, projection_indices, classification);
  ValidatePose(map_from_scan);

  if (output_rings.start < 0 || output_rings.end < output_rings.start ||
      output_rings.end >= rows) {
    throw std::invalid_argument("output_rings must be a valid ring range");
  }

  std::unordered_map<std::uint32_t, std::uint32_t> cell_to_point;
  cell_to_point.reserve(scan.size());
  for (std::size_t i = 0; i < scan.size(); ++i) {
    if (!IsFinitePoint(scan[i])) {
      continue;
    }
    cell_to_point.emplace(projection_indices[i], static_cast<std::uint32_t>(i));
  }

  std::map<CellKey, FrameCell> frame_cells;
  const auto make_corner = [&](std::uint32_t cell_index) {
        Corner corner;
        const auto it = cell_to_point.find(cell_index);
        if (it == cell_to_point.end()) {
          return corner;
        }
        corner.exists = true;
        corner.ground = classification[cell_index] == 1;
        if (!corner.ground) {
          return corner;
        }
        corner.finite = true;
        corner.vertex =
            MakeWorldVertex(scan[static_cast<std::size_t>(it->second)],
                map_from_scan);
        return corner;
      };

  const auto to_world = [](const Corner &corner) {
        return corner.vertex;
      };

  if (columns >= 2) {
    for (int row = output_rings.start; row < output_rings.end; ++row) {
      for (int column = 0; column < columns; ++column) {
        if (columns == 2 && column == 1) {
          continue;
        }
        const int next_column = (column + 1) % columns;
        const auto row_offset =
            static_cast<std::uint32_t>(row) *
            static_cast<std::uint32_t>(columns);
        const auto next_row_offset =
            static_cast<std::uint32_t>(row + 1) *
            static_cast<std::uint32_t>(columns);
        const std::uint32_t column_offset =
            static_cast<std::uint32_t>(column);
        const std::uint32_t next_column_offset =
            static_cast<std::uint32_t>(next_column);

        const Corner corner00 =
            make_corner(row_offset + column_offset);
        const Corner corner01 =
            make_corner(row_offset + next_column_offset);
        const Corner corner10 =
            make_corner(next_row_offset + column_offset);
        const Corner corner11 =
            make_corner(next_row_offset + next_column_offset);

        const int valid_count =
            (corner00.finite ? 1 : 0) + (corner01.finite ? 1 : 0) +
            (corner10.finite ? 1 : 0) + (corner11.finite ? 1 : 0);
        if (valid_count >= 3) {
          for (const Corner *corner :
              {&corner00, &corner01, &corner10, &corner11}) {
            if (corner->exists && !corner->ground) {
              ++impl_->stats.rejected_ground;
            }
          }
        }
        if (valid_count < 3) {
          continue;
        }

        if (valid_count == 3) {
          std::array<WorldVertex, 3> triangle;
          int write_index = 0;
          for (const Corner *corner :
              {&corner00, &corner01, &corner10, &corner11}) {
            if (corner->finite) {
              triangle[write_index++] = to_world(*corner);
            }
          }
          impl_->ProcessTriangle(triangle, &frame_cells);
          continue;
        }

        const double diagonal_00_11 =
            (corner00.vertex.point - corner11.vertex.point).norm();
        const double diagonal_01_10 =
            (corner01.vertex.point - corner10.vertex.point).norm();
        if (diagonal_00_11 <= diagonal_01_10) {
          impl_->ProcessTriangle(
            {to_world(corner00), to_world(corner01), to_world(corner11)},
            &frame_cells);
          impl_->ProcessTriangle(
            {to_world(corner00), to_world(corner11), to_world(corner10)},
            &frame_cells);
        } else {
          impl_->ProcessTriangle(
            {to_world(corner00), to_world(corner01), to_world(corner10)},
            &frame_cells);
          impl_->ProcessTriangle(
            {to_world(corner01), to_world(corner11), to_world(corner10)},
            &frame_cells);
        }
      }
    }
  }

  impl_->MergeFrameVotes(frame_cells);
  ++impl_->stats.frames;
}

std::size_t GroundMeshRasterizer::AppendMissingSamples(
  pcl::PointCloud<pcl::PointXYZI> *global_measured_ground) {
  if (global_measured_ground == nullptr) {
    throw std::invalid_argument("global_measured_ground must not be null");
  }

  const double resolution = static_cast<double>(impl_->options.resolution);
  std::unordered_set<CellKey, CellKeyHash> measured_cells;
  measured_cells.reserve(global_measured_ground->size());
  for (const pcl::PointXYZI &point : *global_measured_ground) {
    CellKey key;
    if (WorldCellIndex(static_cast<double>(point.x), resolution, &key.x) &&
        WorldCellIndex(static_cast<double>(point.y), resolution, &key.y)) {
      measured_cells.insert(key);
    }
  }

  const std::size_t minimum_support =
      std::max<std::size_t>(1U, impl_->options.min_keyframe_support);
  std::size_t appended = 0;
  for (const auto &entry : impl_->global_cells) {
    if (measured_cells.count(entry.first) != 0U) {
      continue;
    }
    const std::vector<HeightLayer> &layers = entry.second;
    if (layers.empty()) {
      continue;
    }

    std::size_t total_votes = 0;
    std::size_t best_layer = 0;
    for (std::size_t i = 0; i < layers.size(); ++i) {
      total_votes += layers[i].count;
      if (layers[i].count > layers[best_layer].count) {
        best_layer = i;
      }
    }

    const HeightLayer &selected = layers[best_layer];
    if (selected.count < minimum_support) {
      continue;
    }
    if (static_cast<double>(selected.count) < 0.75 * static_cast<double>(
          total_votes)) {
      ++impl_->stats.ambiguous_cells;
      continue;
    }

    pcl::PointXYZI point;
    point.x = static_cast<float>((static_cast<double>(entry.first.x) + 0.5) *
        resolution);
    point.y = static_cast<float>((static_cast<double>(entry.first.y) + 0.5) *
        resolution);
    point.z = static_cast<float>(selected.sum_z /
        static_cast<double>(selected.count));
    point.intensity = 1.0f;
    global_measured_ground->push_back(point);
    ++appended;
  }
  return appended;
}

const GroundMeshStatistics &GroundMeshRasterizer::statistics() const {
  return impl_->stats;
}

}  // namespace lio_sam_hesai
