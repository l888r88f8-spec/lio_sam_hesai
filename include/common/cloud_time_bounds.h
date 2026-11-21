#ifndef LIO_SAM_HESAI_CLOUD_TIME_BOUNDS_H_
#define LIO_SAM_HESAI_CLOUD_TIME_BOUNDS_H_

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

namespace relocalization {

// Conversion can legitimately remove every point. Validate before any point
// access so an unusable scan is discarded without terminating the worker.
template <typename CloudPtr>
std::optional<std::pair<float, float>> CloudTimeBounds(const CloudPtr& cloud) {
  if (!cloud || cloud->empty()) {
    return std::nullopt;
  }
  float minimum = cloud->front().time;
  float maximum = minimum;
  for (const auto& point : *cloud) {
    if (!std::isfinite(point.time)) {
      return std::nullopt;
    }
    minimum = std::min(minimum, point.time);
    maximum = std::max(maximum, point.time);
  }
  return std::make_pair(minimum, maximum);
}
}  // namespace relocalization

#endif
