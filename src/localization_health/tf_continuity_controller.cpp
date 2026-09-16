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
  last_emitted_pose_.reset();
  last_publication_timestamp_us_ = 0U;
  mode_ = TfPublicationMode::kNoAnchor;
  hold_reason_ = TfHoldReason::kNoAnchor;
}

bool TfContinuityController::IsCurrentHorizonSample(
    const TfPoseSample& sample, const TfPoseSample& anchor) const {
  if (sample.sensor_timestamp_us < anchor.sensor_timestamp_us) {
    return false;
  }
  if (sample.sensor_timestamp_us - anchor.sensor_timestamp_us >
      config_.max_extrapolation_us) {
    return false;
  }
  if (last_emitted_pose_.has_value() &&
      sample.sensor_timestamp_us < last_emitted_pose_->sensor_timestamp_us) {
    return false;
  }
  return true;
}

std::optional<TfPublicationDecision> TfContinuityController::RecordMeasured(
    const TfPoseSample& measured, TimeStampUs publication_now_us,
    const std::optional<TfPoseSample>& projected) {
  anchor_ = measured;
  const TimeStampUs publication_timestamp_us = std::max(
      publication_now_us, last_publication_timestamp_us_);

  if (projected.has_value() &&
      IsCurrentHorizonSample(projected.value(), measured)) {
    // The measurement itself is already projected onto the current sensor
    // horizon, so it is the newest pose this process can publish.
    last_emitted_pose_ = projected;
    last_publication_timestamp_us_ = publication_timestamp_us;
    mode_ = TfPublicationMode::kMeasured;
    hold_reason_ = TfHoldReason::kNone;
    return TfPublicationDecision{mode_, hold_reason_, projected.value(),
                                 publication_timestamp_us,
                                 measured.sensor_timestamp_us,
                                 projected->sensor_timestamp_us -
                                     measured.sensor_timestamp_us};
  }

  if (!last_emitted_pose_.has_value()) {
    // No pose has ever been emitted: never relabel a historical measurement
    // as the current TF pose to fill the gap.
    return std::nullopt;
  }

  mode_ = TfPublicationMode::kHeld;
  hold_reason_ = projected.has_value() ? TfHoldReason::kExtrapolationLimit
                                       : TfHoldReason::kImuUnavailable;
  last_publication_timestamp_us_ = publication_timestamp_us;
  return TfPublicationDecision{mode_, hold_reason_, last_emitted_pose_.value(),
                               publication_timestamp_us,
                               anchor_.value().sensor_timestamp_us, 0U};
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

  const TfPoseSample& anchor = anchor_.value();
  if (extrapolated && sensor_now_us.has_value() &&
      sensor_now_us.value() >= anchor.sensor_timestamp_us &&
      sensor_now_us.value() - anchor.sensor_timestamp_us <=
          config_.max_extrapolation_us &&
      extrapolated->sensor_timestamp_us <= sensor_now_us.value() &&
      IsCurrentHorizonSample(extrapolated.value(), anchor)) {
    const TimeStampUs span_us =
        extrapolated->sensor_timestamp_us - anchor.sensor_timestamp_us;
    mode_ = TfPublicationMode::kExtrapolated;
    hold_reason_ = TfHoldReason::kNone;
    last_emitted_pose_ = extrapolated;
    last_publication_timestamp_us_ = publication_now_us;
    return TfPublicationDecision{mode_, hold_reason_, extrapolated.value(),
                                 publication_now_us,
                                 anchor.sensor_timestamp_us, span_us};
  }

  if (!last_emitted_pose_.has_value()) {
    // Nothing has reached TF yet, so there is no pose to hold.
    return std::nullopt;
  }

  mode_ = TfPublicationMode::kHeld;
  hold_reason_ = extrapolated ? TfHoldReason::kExtrapolationLimit
                              : TfHoldReason::kImuUnavailable;
  last_publication_timestamp_us_ = publication_now_us;
  return TfPublicationDecision{mode_, hold_reason_, last_emitted_pose_.value(),
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

std::optional<TimeStampUs>
TfContinuityController::last_emitted_sensor_timestamp_us() const {
  if (!last_emitted_pose_) {
    return std::nullopt;
  }
  return last_emitted_pose_->sensor_timestamp_us;
}

bool TfContinuityController::IsPoseStale(
    const std::optional<TimeStampUs>& latest_imu_sensor_us,
    bool is_tracking) const {
  if (!last_emitted_pose_) {
    // Startup without a TF pose is not staleness, but a tracking vehicle with
    // no emitted pose has no TF continuity at all.
    return is_tracking;
  }
  if (!latest_imu_sensor_us.has_value()) {
    return false;
  }
  if (latest_imu_sensor_us.value() <=
      last_emitted_pose_->sensor_timestamp_us) {
    // A sensor clock that moved backwards is a stream fault, not a held pose.
    return false;
  }
  return latest_imu_sensor_us.value() -
             last_emitted_pose_->sensor_timestamp_us >
         config_.max_extrapolation_us;
}

}  // namespace localization_health
