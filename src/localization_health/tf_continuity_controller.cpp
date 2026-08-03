#include "localization_health/tf_continuity_controller.h"

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

TfPublicationDecision TfContinuityController::RecordMeasured(
    const TfPoseSample& measured) {
  anchor_ = measured;
  last_publish_us_ = measured.timestamp_us;
  mode_ = TfPublicationMode::kMeasured;
  hold_reason_ = TfHoldReason::kNone;
  return {mode_, hold_reason_, measured, measured.timestamp_us, 0U};
}

std::optional<TfPublicationDecision> TfContinuityController::Select(
    TimeStampUs now_us, const std::optional<TfPoseSample>& extrapolated) {
  if (!anchor_) {
    return std::nullopt;
  }

  if (now_us < last_publish_us_ ||
      now_us - last_publish_us_ < config_.publish_period_us) {
    return std::nullopt;
  }

  const TfPoseSample& anchor = *anchor_;
  if (extrapolated && extrapolated->timestamp_us >= anchor.timestamp_us) {
    const TimeStampUs span_us =
        extrapolated->timestamp_us - anchor.timestamp_us;
    if (span_us <= config_.max_extrapolation_us) {
      mode_ = TfPublicationMode::kExtrapolated;
      hold_reason_ = TfHoldReason::kNone;
      last_publish_us_ = now_us;
      return TfPublicationDecision{mode_, hold_reason_, *extrapolated,
                                   anchor.timestamp_us, span_us};
    }
  }

  mode_ = TfPublicationMode::kHeld;
  hold_reason_ = extrapolated ? TfHoldReason::kExtrapolationLimit
                              : TfHoldReason::kImuUnavailable;
  last_publish_us_ = now_us;
  TfPoseSample held = anchor;
  held.timestamp_us = now_us;
  return TfPublicationDecision{mode_, hold_reason_, held, anchor.timestamp_us,
                               0U};
}

TfPublicationMode TfContinuityController::mode() const { return mode_; }

TfHoldReason TfContinuityController::hold_reason() const {
  return hold_reason_;
}

std::optional<TimeStampUs> TfContinuityController::anchor_timestamp_us() const {
  if (!anchor_) {
    return std::nullopt;
  }
  return anchor_->timestamp_us;
}

}  // namespace localization_health
