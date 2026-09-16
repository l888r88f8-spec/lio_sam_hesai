#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_PREDICTION_ANCHOR_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_PREDICTION_ANCHOR_H

#include "common/data_type.h"

#include <cstdint>

namespace localization_health {

// Identification of the committed localization anchor a TF prediction may use.
// The TF continuity epoch fences predictions across ROS clock jumps: an anchor
// committed before a jump must never be extrapolated with post-jump IMU data.
struct TfPredictionAnchorKey {
  TimeStampUs sensor_timestamp_us{0U};
  std::uint64_t tf_continuity_epoch{0U};
  bool has_anchor{false};
};

// Identification carried by the result whose TF pose is being predicted. It
// must match the committed anchor exactly: a result may only be projected from
// its own measurement, never from a newer frame that raced ahead of it.
struct TfPredictionRequestKey {
  TimeStampUs sensor_timestamp_us{0U};
  std::uint64_t tf_continuity_epoch{0U};
};

enum class TfPredictionAnchorMatch {
  kMatched,
  kNoAnchor,
  kTimestampMismatch,
  kEpochMismatch,
};

[[nodiscard]] constexpr TfPredictionAnchorMatch MatchTfPredictionAnchor(
    const TfPredictionAnchorKey& anchor,
    const TfPredictionRequestKey& request) {
  if (!anchor.has_anchor) {
    return TfPredictionAnchorMatch::kNoAnchor;
  }
  if (anchor.sensor_timestamp_us != request.sensor_timestamp_us) {
    return TfPredictionAnchorMatch::kTimestampMismatch;
  }
  if (anchor.tf_continuity_epoch != request.tf_continuity_epoch) {
    return TfPredictionAnchorMatch::kEpochMismatch;
  }
  return TfPredictionAnchorMatch::kMatched;
}

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_PREDICTION_ANCHOR_H
