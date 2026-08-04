#include "localization_health/tf_continuity_controller.h"

#include <algorithm>
#include <stdexcept>

namespace localization_health {

TfContinuityController::TfContinuityController(Config config) : config_(config) {
  Validate(config_);
}

void TfContinuityController::Validate(const Config& config) {
  if (config.publish_period_us == 0U || config.max_extrapolation_us == 0U) {
    throw std::invalid_argument("Invalid TF continuity controller configuration");
  }
}

void TfContinuityController::Reset() {
  anchor_.reset();
  last_publication_timestamp_us_ = 0U;
  mode_ = TfPublicationMode::kNoAnchor;
  hold_reason_ = TfHoldReason::kNoAnchor;
}

TfPublicationDecision TfContinuityController::RecordMeasured(
    const TfPoseSample& measured, TimeStampUs publication_now_us) {
  anchor_ = measured;
  const TimeStampUs publication_timestamp_us = std::max(
      publication_now_us, last_publication_timestamp_us_);
  last_publication_timestamp_us_ = publication_timestamp_us;
  mode_ = TfPublicationMode::kMeasured;
  hold_reason_ = TfHoldReason::kNone;
  return {mode_, hold_reason_, measured, publication_timestamp_us,
          measured.sensor_timestamp_us, 0U};
}

std::optional<TfPublicationDecision> TfContinuityController::Select(
    TimeStampUs publication_now_us,
    std::optional<TimeStampUs> sensor_now_us,
    const std::optional<TfPoseSample>& extrapolated) {
  if (!anchor_) {
    return std::nullopt;
  }

  if (publication_now_us < last_publication_timestamp_us_ ||
      publication_now_us - last_publication_timestamp_us_ <
          config_.publish_period_us) {
    return std::nullopt;
  }

  const TfPoseSample& anchor = *anchor_;
  if (extrapolated && sensor_now_us.has_value() &&
      sensor_now_us.value() >= anchor.sensor_timestamp_us &&
      sensor_now_us.value() - anchor.sensor_timestamp_us <=
          config_.max_extrapolation_us &&
      extrapolated->sensor_timestamp_us >= anchor.sensor_timestamp_us &&
      extrapolated->sensor_timestamp_us <= sensor_now_us.value()) {
    const TimeStampUs span_us =
        extrapolated->sensor_timestamp_us - anchor.sensor_timestamp_us;
    mode_ = TfPublicationMode::kExtrapolated;
    hold_reason_ = TfHoldReason::kNone;
    last_publication_timestamp_us_ = publication_now_us;
    return TfPublicationDecision{mode_, hold_reason_, *extrapolated,
                                 publication_now_us,
                                 anchor.sensor_timestamp_us, span_us};
  }

  mode_ = TfPublicationMode::kHeld;
  hold_reason_ = extrapolated ? TfHoldReason::kExtrapolationLimit
                              : TfHoldReason::kImuUnavailable;
  last_publication_timestamp_us_ = publication_now_us;
  return TfPublicationDecision{mode_, hold_reason_, anchor,
                               publication_now_us,
                               anchor.sensor_timestamp_us, 0U};
}

TfPublicationMode TfContinuityController::mode() const { return mode_; }

TfHoldReason TfContinuityController::hold_reason() const {
  return hold_reason_;
}

std::optional<TimeStampUs>
TfContinuityController::anchor_sensor_timestamp_us() const {
  if (!anchor_) {
    return std::nullopt;
  }
  return anchor_->sensor_timestamp_us;
}

}  // namespace localization_health
