#include "localization_health/automatic_recovery_policy.h"

#include <algorithm>
#include <stdexcept>

namespace localization_health {
namespace {

bool IsPositiveFinite(int value) { return value > 0; }

std::uint64_t MsToUs(int ms) {
  return static_cast<std::uint64_t>(ms) * 1000U;
}

}  // namespace

const char* ToString(AutoRecoveryDecision decision) {
  switch (decision) {
    case AutoRecoveryDecision::kStartRound:
      return "START_ROUND";
    case AutoRecoveryDecision::kCoolingDown:
      return "COOLDOWN";
    case AutoRecoveryDecision::kAttemptExpired:
      return "ATTEMPT_EXPIRED";
  }
  return "UNKNOWN";
}

const char* ToString(RoundTimeoutReason reason) {
  switch (reason) {
    case RoundTimeoutReason::kNone:
      return "none";
    case RoundTimeoutReason::kRoundDeadline:
      return "round_deadline";
    case RoundTimeoutReason::kAttemptDeadline:
      return "attempt_deadline";
  }
  return "unknown";
}

const char* ToString(SeedDecision decision) {
  switch (decision) {
    case SeedDecision::kUseSeed:
      return "use_seed";
    case SeedDecision::kNoSeed:
      return "no_seed";
    case SeedDecision::kReplayEpochChanged:
      return "replay_epoch_changed";
    case SeedDecision::kNoNewerFrame:
      return "no_newer_frame";
    case SeedDecision::kTimestampRollback:
      return "timestamp_rollback";
    case SeedDecision::kSourceInvalid:
      return "source_invalid";
  }
  return "unknown";
}

const char* ToString(SeedQualification qualification) {
  switch (qualification) {
    case SeedQualification::kNone:
      return "unqualified";
    case SeedQualification::kEarlyAccepted:
      return "early_accepted";
    case SeedQualification::kConsistencyPassed:
      return "consistency_passed";
  }
  return "unknown";
}

void AutomaticSeedLifecycle::Reset() { hit_count_ = 0; }

SeedQualification AutomaticSeedLifecycle::ObserveFreshCandidate(
    bool early_accepted, bool has_reference, double translation_delta_m,
    double yaw_delta_rad, int required_frames,
    double translation_threshold_m, double yaw_threshold_rad) {
  if (early_accepted) {
    // Early acceptance already satisfies the independent-frame requirement;
    // keep no stale consistency count for a later reuse.
    hit_count_ = 0;
    return SeedQualification::kEarlyAccepted;
  }
  if (required_frames <= 0) {
    // An invalid 0/0 requirement must never qualify a candidate.
    hit_count_ = 0;
    return SeedQualification::kNone;
  }
  if (has_reference) {
    const bool consistent =
        translation_delta_m <= translation_threshold_m &&
        yaw_delta_rad <= yaw_threshold_rad;
    hit_count_ = consistent ? hit_count_ + 1 : 1;
  } else {
    hit_count_ = 1;
  }
  return hit_count_ >= required_frames
             ? SeedQualification::kConsistencyPassed
             : SeedQualification::kNone;
}


const char* ToString(SourceFreshness freshness) {
  switch (freshness) {
    case SourceFreshness::kFresh:
      return "fresh";
    case SourceFreshness::kStaleAge:
      return "stale_age";
    case SourceFreshness::kTimestampRollback:
      return "timestamp_rollback";
    case SourceFreshness::kNoLatestInput:
      return "no_latest_input";
    case SourceFreshness::kInvalidTimestamp:
      return "invalid_timestamp";
  }
  return "unknown";
}

AutomaticRecoveryPolicy::AutomaticRecoveryPolicy(
    const AutomaticRecoveryConfig& config)
    : config_(config),
      round_budget_us_(MsToUs(config.round_timeout_ms)),
      attempt_budget_us_(MsToUs(config.attempt_timeout_ms)),
      cooldown_us_(MsToUs(config.cooldown_ms)),
      max_source_age_us_(MsToUs(config.max_source_age_ms)) {
  if (!IsPositiveFinite(config_.round_timeout_ms) ||
      !IsPositiveFinite(config_.attempt_timeout_ms) ||
      !IsPositiveFinite(config_.cooldown_ms) ||
      !IsPositiveFinite(config_.max_source_age_ms)) {
    throw std::invalid_argument(
        "automatic recovery budgets must be positive");
  }
  if (config_.attempt_timeout_ms < config_.round_timeout_ms) {
    throw std::invalid_argument(
        "automatic recovery attempt budget must be >= round budget");
  }
}

void AutomaticRecoveryPolicy::BeginAttempt(std::uint64_t now_us) {
  // Coalesce repeated external requests: an active attempt keeps its original
  // budget, and cooldown is never bypassed by another edge.
  if (attempt_active_ || InCooldown(now_us)) {
    return;
  }
  attempt_active_ = true;
  round_active_ = false;
  cooldown_armed_ = false;
  attempt_start_us_ = now_us;
  attempt_deadline_us_ = now_us + attempt_budget_us_;
  round_start_us_ = now_us;
  round_deadline_us_ = now_us;
  cooldown_until_us_ = 0U;
}

void AutomaticRecoveryPolicy::CancelForManual(std::uint64_t /*now_us*/) {
  attempt_active_ = false;
  round_active_ = false;
  cooldown_armed_ = false;
  cooldown_until_us_ = 0U;
}

bool AutomaticRecoveryPolicy::EnsureAttempt(std::uint64_t now_us,
                                            bool input_healthy) {
  if (attempt_active_) {
    return true;
  }
  if (InCooldown(now_us)) {
    return false;
  }
  if (!input_healthy) {
    return false;
  }
  // Cooldown elapsed (or there never was one) and the stream is healthy:
  // restart without waiting for an external recovery edge.
  BeginAttempt(now_us);
  return true;
}

bool AutomaticRecoveryPolicy::InCooldown(std::uint64_t now_us) const {
  return cooldown_armed_ && now_us < cooldown_until_us_;
}

void AutomaticRecoveryPolicy::EnterCooldown(std::uint64_t now_us) {
  attempt_active_ = false;
  round_active_ = false;
  cooldown_armed_ = true;
  cooldown_until_us_ = now_us + cooldown_us_;
}

AutoRecoveryDecision AutomaticRecoveryPolicy::BeginRound(std::uint64_t now_us) {
  if (!attempt_active_) {
    return InCooldown(now_us) ? AutoRecoveryDecision::kCoolingDown
                              : AutoRecoveryDecision::kAttemptExpired;
  }
  if (now_us >= attempt_deadline_us_) {
    EnterCooldown(now_us);
    return AutoRecoveryDecision::kAttemptExpired;
  }
  if (!round_active_) {
    round_active_ = true;
    round_start_us_ = now_us;
    round_deadline_us_ = now_us + round_budget_us_;
  }
  return AutoRecoveryDecision::kStartRound;
}

bool AutomaticRecoveryPolicy::RoundDeadlineExceeded(
    std::uint64_t now_us) const {
  return RoundTimeout(now_us) != RoundTimeoutReason::kNone;
}

RoundTimeoutReason AutomaticRecoveryPolicy::RoundTimeout(
    std::uint64_t now_us) const {
  if (!round_active_) {
    return RoundTimeoutReason::kNone;
  }
  // A round that began just before the overall attempt deadline must not
  // commit past it: the effective deadline is the earlier of the two.
  if (now_us >= attempt_deadline_us_) {
    return RoundTimeoutReason::kAttemptDeadline;
  }
  if (now_us > round_deadline_us_) {
    return RoundTimeoutReason::kRoundDeadline;
  }
  return RoundTimeoutReason::kNone;
}

bool AutomaticRecoveryPolicy::AttemptDeadlineExceeded(
    std::uint64_t now_us) const {
  return attempt_active_ && now_us >= attempt_deadline_us_;
}

bool AutomaticRecoveryPolicy::ExpireAttemptIfDue(std::uint64_t now_us) {
  if (!AttemptDeadlineExceeded(now_us)) {
    return false;
  }
  EnterCooldown(now_us);
  return true;
}

std::uint64_t AutomaticRecoveryPolicy::RoundElapsedMs(
    std::uint64_t now_us) const {
  if (!round_active_ || now_us < round_start_us_) {
    return 0U;
  }
  return (now_us - round_start_us_) / 1000U;
}

std::uint64_t AutomaticRecoveryPolicy::AttemptElapsedMs(
    std::uint64_t now_us) const {
  if (now_us < attempt_start_us_) {
    return 0U;
  }
  return (now_us - attempt_start_us_) / 1000U;
}

bool AutomaticRecoveryPolicy::RecordRoundFailure(std::uint64_t now_us,
                                                bool* attempt_expired) {
  round_active_ = false;
  if (attempt_expired != nullptr) {
    *attempt_expired = false;
  }
  if (!attempt_active_) {
    return false;
  }
  if (now_us >= attempt_deadline_us_) {
    EnterCooldown(now_us);
    if (attempt_expired != nullptr) {
      *attempt_expired = true;
    }
    return false;
  }
  // Attempt budget remains: the next suitable frame retries immediately.
  return true;
}

void AutomaticRecoveryPolicy::RecordRoundPending(std::uint64_t /*now_us*/) {
  // A consistency-pending round is a normal end of round, not a failure. The
  // attempt stays active so the next frame can add a consistency observation.
  round_active_ = false;
}

void AutomaticRecoveryPolicy::RecordRoundSuccess(std::uint64_t /*now_us*/) {
  attempt_active_ = false;
  round_active_ = false;
  cooldown_armed_ = false;
  cooldown_until_us_ = 0U;
}

SourceFreshness AutomaticRecoveryPolicy::EvaluateSourceFreshness(
    std::uint64_t source_timestamp_us,
    std::optional<std::uint64_t> latest_lidar_timestamp_us,
    std::optional<std::uint64_t> latest_imu_timestamp_us,
    std::uint64_t max_source_age_us) {
  if (source_timestamp_us == 0U) {
    return SourceFreshness::kInvalidTimestamp;
  }

  // The newest raw lidar sample is authoritative. Checking it separately from
  // the lidar/IMU maximum keeps a rolled-back raw lidar stream visible even
  // when the IMU timestamp is still ahead of the candidate frame.
  if (latest_lidar_timestamp_us.has_value() &&
      latest_lidar_timestamp_us.value() != 0U &&
      latest_lidar_timestamp_us.value() < source_timestamp_us) {
    return SourceFreshness::kTimestampRollback;
  }

  std::uint64_t latest_timestamp_us = 0U;
  bool has_latest = false;
  const auto consider = [&](const std::optional<std::uint64_t>& stamp) {
    if (stamp.has_value() && stamp.value() != 0U) {
      latest_timestamp_us = has_latest
                                ? std::max(latest_timestamp_us, stamp.value())
                                : stamp.value();
      has_latest = true;
    }
  };
  consider(latest_lidar_timestamp_us);
  consider(latest_imu_timestamp_us);
  if (!has_latest) {
    return SourceFreshness::kNoLatestInput;
  }

  if (latest_timestamp_us < source_timestamp_us) {
    return SourceFreshness::kTimestampRollback;
  }
  const std::uint64_t age_us = latest_timestamp_us - source_timestamp_us;
  if (age_us > max_source_age_us) {
    return SourceFreshness::kStaleAge;
  }
  return SourceFreshness::kFresh;
}

SeedDecision AutomaticRecoveryPolicy::EvaluateSeedUsability(
    std::uint64_t seed_source_timestamp_us, std::uint64_t seed_epoch,
    std::uint64_t current_frame_timestamp_us, std::uint64_t current_epoch,
    std::optional<std::uint64_t> latest_lidar_timestamp_us) {
  if (seed_source_timestamp_us == 0U ||
      current_frame_timestamp_us == 0U) {
    return SeedDecision::kSourceInvalid;
  }
  // Replay/rollback resets the input epoch. A seed from the previous playback
  // must never be revalidated against the new sensor stream.
  if (seed_epoch != current_epoch) {
    return SeedDecision::kReplayEpochChanged;
  }
  if (!latest_lidar_timestamp_us.has_value() ||
      latest_lidar_timestamp_us.value() == 0U) {
    return SeedDecision::kNoNewerFrame;
  }
  if (current_frame_timestamp_us < seed_source_timestamp_us) {
    return SeedDecision::kTimestampRollback;
  }
  if (latest_lidar_timestamp_us.value() <= seed_source_timestamp_us) {
    return SeedDecision::kNoNewerFrame;
  }
  // A queued frame cannot be newer than the newest raw lidar sample. If it is,
  // the raw stream rolled back (or was reset) after this frame was queued.
  if (current_frame_timestamp_us > latest_lidar_timestamp_us.value()) {
    return SeedDecision::kTimestampRollback;
  }
  if (current_frame_timestamp_us <= seed_source_timestamp_us) {
    return SeedDecision::kNoNewerFrame;
  }
  return SeedDecision::kUseSeed;
}

}  // namespace localization_health
