#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H

#include "common/data_type.h"

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
  Mat4d map_base_pose;
  TimeStampUs timestamp_us;
};

struct TfPublicationDecision {
  TfPublicationMode mode;
  TfHoldReason hold_reason;
  TfPoseSample sample;
  TimeStampUs anchor_timestamp_us;
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

  TfPublicationDecision RecordMeasured(const TfPoseSample& measured);
  std::optional<TfPublicationDecision> Select(
      TimeStampUs now_us, const std::optional<TfPoseSample>& extrapolated);

  [[nodiscard]] TfPublicationMode mode() const;
  [[nodiscard]] TfHoldReason hold_reason() const;
  [[nodiscard]] std::optional<TimeStampUs> anchor_timestamp_us() const;

 private:
  Config config_;
  std::optional<TfPoseSample> anchor_;
  TimeStampUs last_publish_us_{0U};
  TimeStampUs last_emitted_timestamp_us_{0U};
  TfPublicationMode mode_{TfPublicationMode::kNoAnchor};
  TfHoldReason hold_reason_{TfHoldReason::kNoAnchor};
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_TF_CONTINUITY_CONTROLLER_H
