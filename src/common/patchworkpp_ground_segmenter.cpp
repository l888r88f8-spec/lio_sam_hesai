#include "lio_sam/patchworkpp_ground_segmenter.h"

#include "common/slam_config.h"
#include "patchwork/patchworkpp.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace lio_sam_hesai
{
namespace
{
void RequirePositive(double value, const char* name)
{
  if (!std::isfinite(value) || value <= 0.0)
    throw std::invalid_argument(std::string("patchworkpp.") + name +
                                " must be finite and positive");
}

void RequireNonnegative(double value, const char* name)
{
  if (!std::isfinite(value) || value < 0.0)
    throw std::invalid_argument(std::string("patchworkpp.") + name +
                                " must be finite and nonnegative");
}

patchwork::Params Parameters(const PatchworkppConfig& config)
{
  RequirePositive(config.sensor_height, "sensor_height");
  RequireNonnegative(config.min_range, "min_range");
  RequirePositive(config.max_range, "max_range");
  if (config.max_range <= config.min_range)
    throw std::invalid_argument("patchworkpp.max_range must exceed min_range");
  RequirePositive(config.uprightness_thr, "uprightness_thr");
  if (config.uprightness_thr > 1.0)
    throw std::invalid_argument("patchworkpp.uprightness_thr must not exceed 1");
  RequirePositive(config.th_seeds, "th_seeds");
  RequirePositive(config.th_dist, "th_dist");
  RequireNonnegative(config.rnr_intensity_threshold, "rnr_intensity_threshold");

  patchwork::Params params;
  params.sensor_height = config.sensor_height;
  params.min_range = config.min_range;
  params.max_range = config.max_range;
  params.uprightness_thr = config.uprightness_thr;
  params.th_seeds = config.th_seeds;
  params.th_dist = config.th_dist;
  params.enable_RNR = config.enable_rnr;
  params.RNR_intensity_thr = config.rnr_intensity_threshold;
  return params;
}
}  // namespace

struct PatchworkppGroundSegmenter::Impl
{
  explicit Impl(const PatchworkppConfig& config)
      : algorithm(Parameters(config)), enable_rnr(config.enable_rnr)
  {
  }

  patchwork::PatchWorkpp algorithm;
  bool enable_rnr;
};

PatchworkppGroundSegmenter::PatchworkppGroundSegmenter(const PatchworkppConfig& config)
    : impl_(std::make_unique<Impl>(config))
{
}

PatchworkppGroundSegmenter::~PatchworkppGroundSegmenter() = default;

std::vector<std::size_t> PatchworkppGroundSegmenter::Segment(
    const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
  // Upstream PointXYZ::idx and its Eigen index output use signed int.
  if (cloud.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
    throw std::length_error("Patchwork++ cloud exceeds the upstream index range");

  std::vector<std::size_t> input_indices;
  input_indices.reserve(cloud.size());
  for (std::size_t i = 0; i < cloud.size(); ++i)
  {
    const auto& point = cloud.points[i];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
      continue;
    if (impl_->enable_rnr && !std::isfinite(point.intensity))
      throw std::invalid_argument(
          "patchworkpp RNR requires finite intensity for valid XYZ point at input index " +
          std::to_string(i));
    input_indices.push_back(i);
  }
  // Avoid calling the adaptive core for empty input, retaining its scan-stream
  // state without accidentally returning the previous frame's ground indices.
  if (input_indices.empty()) return {};

  Eigen::MatrixXf input(static_cast<Eigen::Index>(input_indices.size()), 4);
  for (std::size_t row = 0; row < input_indices.size(); ++row)
  {
    const auto& point = cloud.points[input_indices[row]];
    input.row(static_cast<Eigen::Index>(row)) << point.x, point.y, point.z, point.intensity;
  }
  impl_->algorithm.estimateGround(std::move(input));
  const auto ground = impl_->algorithm.getGroundIndices();

  std::vector<std::size_t> result;
  result.reserve(static_cast<std::size_t>(ground.size()));
  for (Eigen::Index i = 0; i < ground.size(); ++i)
  {
    if (ground[i] < 0 || static_cast<std::size_t>(ground[i]) >= input_indices.size())
      throw std::runtime_error("Patchwork++ returned an out-of-range input index");
    result.push_back(input_indices[static_cast<std::size_t>(ground[i])]);
  }
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());
  return result;
}
}  // namespace lio_sam_hesai
