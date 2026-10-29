#include "lio_sam/dufomap_backend.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <ufo/map/integration/integration.hpp>
#include <ufo/map/ufomap.hpp>

#include "common/dufomap_config.h"

namespace lio_sam_hesai {
namespace {

using Map = ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
                     ufo::MapType::LABEL>;
using Clock = std::chrono::steady_clock;

constexpr char kBackendKind[] = "native-cpp-upstream-core";
constexpr char kBackendSource[] =
    "KTH-RPL/dufomap@9e239ddd5995136e14f5212f33382a6ebc59e518";
constexpr char kUfomapRevision[] =
    "fcb20f445d257280c039c07ab756c9d5b37ebd78";
constexpr std::size_t kClassifyBatch = 4096;
// Bounds the later duration_cast well below the steady_clock range so a huge
// but finite timeout cannot overflow the deadline calculation.
constexpr double kMaxTimeoutSeconds = 1.0e9;

struct ValidatedConfig {
  double resolution;
  double d_s;
  int d_p;
  int num_threads;
  double min_range;
  double max_range;
  double timeout_seconds;
};

void Require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::invalid_argument(message);
  }
}

ValidatedConfig ValidateConfig(const DufomapConfig& config) {
  Require(std::isfinite(config.resolution) && config.resolution > 0.0,
          "dufomap_export.resolution must be finite and positive");
  Require(std::isfinite(config.d_s) && config.d_s >= 0.0,
          "dufomap_export.d_s must be finite and non-negative");
  Require(config.d_p >= 0 && config.d_p <= 4,
          "dufomap_export.d_p must be between 0 and 4");
  Require(config.num_threads == 0 || config.num_threads >= 2,
          "dufomap_export.num_threads must be 0 (automatic) or at least 2");
  Require(std::isfinite(config.min_range) && config.min_range >= 0.0,
          "dufomap_export.min_range must be finite and non-negative");
  Require(std::isfinite(config.max_range) && config.max_range >= 0.0 &&
              (config.max_range == 0.0 || config.max_range >= config.min_range),
          "dufomap_export.max_range must be zero or greater than or equal to "
          "min_range");
  Require(std::isfinite(config.timeout_seconds) &&
              config.timeout_seconds >= 0.0 &&
              config.timeout_seconds <= kMaxTimeoutSeconds,
          "dufomap_export.timeout_seconds must be finite, non-negative, and "
          "at most 1e9");
  return {config.resolution, config.d_s, config.d_p, config.num_threads,
          config.min_range, config.max_range, config.timeout_seconds};
}

Clock::time_point Deadline(double seconds) {
  if (seconds <= 0.0) {
    return Clock::time_point::max();
  }
  return Clock::now() +
         std::chrono::duration_cast<Clock::duration>(
             std::chrono::duration<double>(seconds));
}

void ValidatePose(const std::array<double, 16>& pose) {
  for (const double value : pose) {
    Require(std::isfinite(value), "dufomap frame pose must be finite");
  }
  Require(std::abs(pose[12]) <= 1e-8 && std::abs(pose[13]) <= 1e-8 &&
              std::abs(pose[14]) <= 1e-8 && std::abs(pose[15] - 1.0) <= 1e-8,
          "dufomap frame pose must be a homogeneous transform");
}

ufo::Point Point(double x, double y, double z, double limit) {
  Require(std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
              std::abs(x) < limit && std::abs(y) < limit && std::abs(z) < limit,
          "point outside finite DUFOMap coordinate range");
  return ufo::Point(x, y, z);
}

std::string TimeoutMessage(double seconds) {
  return "DUFOMAP native backend timed out after " + std::to_string(seconds) +
         " seconds at a cooperative frame/classification checkpoint; the "
         "in-flight upstream core call cannot be interrupted";
}

}  // namespace

struct DufomapBackend::Impl {
  explicit Impl(const DufomapConfig& config)
      : Impl(ValidateConfig(config)) {
  }

 private:
  explicit Impl(const ValidatedConfig& config)
      : map(config.resolution, 20),
        limit(std::ldexp(config.resolution, 19)),
        min_range(config.min_range),
        max_range(config.max_range),
        timeout_seconds(config.timeout_seconds),
        deadline(Deadline(config.timeout_seconds)) {
    params.inflate_hits_dist = config.d_s;
    params.inflate_unknown = config.d_p;
    params.inflate_unknown_compensation = true;
    params.ray_passthrough_hits = false;
    // The adapter applies its own configurable range gate.
    params.min_range = 0.0F;
    params.num_threads =
        config.num_threads
            ? static_cast<unsigned>(config.num_threads)
            : std::max(2U, std::min(8U, std::thread::hardware_concurrency()));
    // Upstream getMisses has no serial implementation.
    params.parallel = true;
  }

 public:
  void CheckDeadline() const {
    if (Clock::now() > deadline) {
      throw std::runtime_error(TimeoutMessage(timeout_seconds));
    }
  }

  ufo::PointCloud Cloud(const std::vector<DufomapPoint>& local_points,
                        const std::array<double, 16>& pose, bool transform,
                        ufo::Point& origin) const {
    ValidatePose(pose);
    origin = Point(pose[3], pose[7], pose[11], limit);
    ufo::PointCloud cloud;
    cloud.reserve(local_points.size());
    for (const DufomapPoint& point : local_points) {
      double x = point.x;
      double y = point.y;
      double z = point.z;
      if (transform) {
        const double tx = pose[0] * x + pose[1] * y + pose[2] * z + pose[3];
        const double ty = pose[4] * x + pose[5] * y + pose[6] * z + pose[7];
        z = pose[8] * x + pose[9] * y + pose[10] * z + pose[11];
        x = tx;
        y = ty;
      }
      cloud.push_back(Point(x, y, z, limit));
    }
    return cloud;
  }

  bool InRange(const DufomapPoint& point) const {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return false;
    }
    const double range =
        std::sqrt(static_cast<double>(point.x) * point.x +
                  static_cast<double>(point.y) * point.y +
                  static_cast<double>(point.z) * point.z);
    return range >= min_range && (max_range == 0.0 || range <= max_range);
  }

  Map map;
  ufo::IntegrationParams params;
  double limit;
  double min_range;
  double max_range;
  double timeout_seconds;
  Clock::time_point deadline;
  std::size_t integrated_points{0};
};

DufomapBackend::DufomapBackend(const DufomapConfig& config)
    : impl_(std::make_unique<Impl>(config)) {
}

DufomapBackend::~DufomapBackend() = default;

void DufomapBackend::Integrate(
    const std::vector<DufomapPoint>& local_points,
    const std::array<double, 16>& odometry_T_lidar) {
  impl_->CheckDeadline();
  ValidatePose(odometry_T_lidar);
  std::vector<DufomapPoint> selected;
  selected.reserve(local_points.size());
  for (const DufomapPoint& point : local_points) {
    if (impl_->InRange(point)) {
      selected.push_back(point);
    }
  }
  if (selected.empty()) {
    throw std::invalid_argument(
        "DUFOMAP frame has no points after the range gate");
  }
  ufo::Point origin;
  const ufo::PointCloud cloud =
      impl_->Cloud(selected, odometry_T_lidar, true, origin);
  ufo::insertPointCloud(impl_->map, cloud, origin, impl_->params, false);
  impl_->integrated_points += selected.size();
}

void DufomapBackend::Propagate() {
  impl_->CheckDeadline();
  impl_->map.propagateModified();
}

std::vector<std::uint8_t> DufomapBackend::Classify(
    const std::vector<DufomapPoint>& odometry_points) {
  impl_->CheckDeadline();
  std::vector<std::uint8_t> labels;
  labels.reserve(odometry_points.size());
  const auto identity = std::array<double, 16>{1.0, 0.0, 0.0, 0.0,
                                               0.0, 1.0, 0.0, 0.0,
                                               0.0, 0.0, 1.0, 0.0,
                                               0.0, 0.0, 0.0, 1.0};
  for (std::size_t begin = 0; begin < odometry_points.size();
       begin += kClassifyBatch) {
    impl_->CheckDeadline();
    const std::size_t end =
        std::min(odometry_points.size(), begin + kClassifyBatch);
    std::vector<DufomapPoint> batch(odometry_points.begin() + begin,
                                    odometry_points.begin() + end);
    ufo::Point unused_origin;
    const ufo::PointCloud cloud =
        impl_->Cloud(batch, identity, false, unused_origin);
    for (const ufo::Point& point : cloud) {
      labels.push_back(impl_->map.seenFree(point) ? 1U : 0U);
    }
  }
  return labels;
}

std::size_t DufomapBackend::integrated_points() const {
  return impl_->integrated_points;
}

const char* DufomapBackendKind() {
  return kBackendKind;
}

const char* DufomapBackendSource() {
  return kBackendSource;
}

const char* DufomapUfomapRevision() {
  return kUfomapRevision;
}

}  // namespace lio_sam_hesai
