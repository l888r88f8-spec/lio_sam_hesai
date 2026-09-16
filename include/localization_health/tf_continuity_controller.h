#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H

#include "common/data_type.h"
#include "common/rigid_transform.h"

#include <optional>

namespace localization_health {

enum class TfPublicationMode { kMeasured, kExtrapolated, kHeld, kNoAnchor };

enum class TfHoldReason {
  kNone,
  kImuUnavailable,
  kExtrapolationLimit,
  kNoAnchor
};

struct TfPoseSample {
  MapTBase map_T_base;
  TimeStampUs sensor_timestamp_us;
};

struct TfPublicationDecision {
  TfPublicationMode mode;
  TfHoldReason hold_reason;
  TfPoseSample sample;
  TimeStampUs publication_timestamp_us;
  TimeStampUs anchor_sensor_timestamp_us;
  TimeStampUs extrapolation_duration_us;
};

class TfContinuityController {
 public:
  struct Config {
    TimeStampUs publish_period_us;
    TimeStampUs max_extrapolation_us;
  };

  explicit TfContinuityController(Config config);

  static void Validate(const Config& config);
  void Reset();

  // Records a new measurement as the prediction anchor. `projected` is that
  // same measurement already brought to the newest valid IMU sensor time;
  // without it no historical measurement may be relabelled as the current TF
  // pose, so the decision is absent until a current-horizon pose exists.
  std::optional<TfPublicationDecision> RecordMeasured(
      const TfPoseSample& measured, TimeStampUs publication_now_us,
      const std::optional<TfPoseSample>& projected = std::nullopt);
  std::optional<TfPublicationDecision> Select(
      TimeStampUs publication_now_us,
      std::optional<TimeStampUs> sensor_now_us,
      const std::optional<TfPoseSample>& extrapolated);

  [[nodiscard]] TfPublicationMode mode() const;
  [[nodiscard]] TfHoldReason hold_reason() const;
  [[nodiscard]] std::optional<TimeStampUs>
  anchor_sensor_timestamp_us() const;
  [[nodiscard]] std::optional<TimeStampUs>
  last_emitted_sensor_timestamp_us() const;

  // True once the newest valid IMU sensor time runs further than the
  // configured extrapolation bound beyond the pose actually emitted: the held
  // TF pose no longer describes the vehicle. Before any emission only an
  // already-tracking vehicle is stale (it has no TF continuity at all); a
  // sensor clock that moved backwards is a stream fault, not staleness.
  [[nodiscard]] bool IsPoseStale(
      const std::optional<TimeStampUs>& latest_imu_sensor_us,
      bool is_tracking) const;

 private:
  // A publishable sample must be on the newest known sensor horizon: it may
  // not precede its own measurement anchor, may not exceed the configured
  // extrapolation bound, and may not resolve behind the pose already emitted.
  [[nodiscard]] bool IsCurrentHorizonSample(
      const TfPoseSample& sample, const TfPoseSample& anchor) const;

  Config config_;
  std::optional<TfPoseSample> anchor_;
  // Last pose that actually reached TF. Held publications reuse this pose;
  // before the first emission there is nothing to hold, so no TF is faked.
  std::optional<TfPoseSample> last_emitted_pose_;
  TimeStampUs last_publication_timestamp_us_{0U};
  TfPublicationMode mode_{TfPublicationMode::kNoAnchor};
  TfHoldReason hold_reason_{TfHoldReason::kNoAnchor};
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H
