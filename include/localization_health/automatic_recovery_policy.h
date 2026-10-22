#ifndef LIO_SAM_HESAI_LOCALIZATION_HEALTH_AUTOMATIC_RECOVERY_POLICY_H
#define LIO_SAM_HESAI_LOCALIZATION_HEALTH_AUTOMATIC_RECOVERY_POLICY_H

#include <cstdint>
#include <optional>

namespace localization_health {

// Budgets for one automatic relocalization round (Scan Context candidates +
// final registration) and for the whole automatic recovery attempt (several
// consistency frames and retries). These are deliberately separate from the
// 500 ms tracking frame budget: an automatic attempt is not tracking and must
// not be dropped by the tracking watchdog.
struct AutomaticRecoveryConfig {
  int round_timeout_ms{1000};
  int attempt_timeout_ms{5000};
  int cooldown_ms{2000};
  // Maximum sensor-time age of the candidate source frame relative to the
  // newest available lidar/IMU input. Never compared against wall clock.
  int max_source_age_ms{500};
};

enum class AutoRecoveryDecision {
  kStartRound,
  kCoolingDown,
  kAttemptExpired,
};

enum class SourceFreshness {
  kFresh,
  kStaleAge,
  kTimestampRollback,
  kNoLatestInput,
  kInvalidTimestamp,
};

// Which budget is exhausted when an automatic round must stop. Keeping the
// two reasons separate makes it possible to log round vs attempt timeouts.
enum class RoundTimeoutReason {
  kNone,
  kRoundDeadline,
  kAttemptDeadline,
};

// Whether a previously selected Scan Context seed may be revalidated on the
// current lidar frame instead of running Scan Context again.
enum class SeedDecision {
  kUseSeed,
  kNoSeed,
  kReplayEpochChanged,
  kNoNewerFrame,
  kTimestampRollback,
  kSourceInvalid,
};

// A Scan Context candidate becomes a reusable seed only after early acceptance
// or the full independent consistency count. A pending candidate (for example
// 1/3) is never retained as a seed.
enum class SeedQualification {
  kNone,
  kEarlyAccepted,
  kConsistencyPassed,
};

const char* ToString(AutoRecoveryDecision decision);
const char* ToString(SourceFreshness freshness);
const char* ToString(RoundTimeoutReason reason);
const char* ToString(SeedDecision decision);
const char* ToString(SeedQualification qualification);

// Production-shared consistency accounting and seed qualification. Localization
// stores the reference pose (needed for registration) and passes the measured
// delta for each fresh Scan Context candidate. Once a seed is qualified, the
// next frame calls EvaluateSeedUsability and does not call ObserveFreshCandidate
// again, so a 3/3 seed cannot accumulate a fourth observation.
class AutomaticSeedLifecycle {
 public:
  void Reset();

  [[nodiscard]] SeedQualification ObserveFreshCandidate(
      bool early_accepted, bool has_reference, double translation_delta_m,
      double yaw_delta_rad, int required_frames,
      double translation_threshold_m, double yaw_threshold_rad);

  [[nodiscard]] int hit_count() const { return hit_count_; }

  [[nodiscard]] static bool CanReuse(SeedQualification qualification) {
    return qualification != SeedQualification::kNone;
  }

 private:
  int hit_count_{0};
};

// Cooperative, steady-clock deadline policy for automatic relocalization.
// A deadline is only enforced where the worker checks it; PCL iterations are
// not preemptible, so this is not a hard real-time guarantee.
class AutomaticRecoveryPolicy {
 public:
  explicit AutomaticRecoveryPolicy(const AutomaticRecoveryConfig& config);

  // Starts a fresh attempt budget only when no attempt is active and no
  // cooldown is running. Repeated external LOST_ENTRY/STREAM_RECOVERED edges
  // therefore coalesce instead of extending an active attempt or bypassing
  // cooldown.
  void BeginAttempt(std::uint64_t now_us);
  // Manual initialization has priority and cancels automatic recovery.
  void CancelForManual(std::uint64_t now_us);

  // Starts an attempt when none is active and the request is healthy. After an
  // expired attempt this restarts automatically once the cooldown elapsed, so
  // no new LOST_ENTRY/STREAM_RECOVERED edge is required.
  [[nodiscard]] bool EnsureAttempt(std::uint64_t now_us, bool input_healthy);

  [[nodiscard]] bool AttemptActive() const { return attempt_active_; }
  [[nodiscard]] bool InCooldown(std::uint64_t now_us) const;
  [[nodiscard]] bool RoundActive() const { return round_active_; }

  // Applies the overall attempt budget and the round budget. On an expired
  // attempt it enters cooldown and returns kAttemptExpired.
  [[nodiscard]] AutoRecoveryDecision BeginRound(std::uint64_t now_us);
  // True when the active round passed either its own budget or the overall
  // attempt deadline (whichever is earlier).
  [[nodiscard]] bool RoundDeadlineExceeded(std::uint64_t now_us) const;
  [[nodiscard]] RoundTimeoutReason RoundTimeout(std::uint64_t now_us) const;
  // Expires a due attempt even when no healthy frame is available to call
  // BeginRound. This keeps a bad stream from postponing the overall deadline.
  bool ExpireAttemptIfDue(std::uint64_t now_us);
  [[nodiscard]] bool AttemptDeadlineExceeded(std::uint64_t now_us) const;
  [[nodiscard]] std::uint64_t RoundDeadlineUs() const {
    return round_deadline_us_;
  }
  [[nodiscard]] std::uint64_t AttemptDeadlineUs() const {
    return attempt_deadline_us_;
  }
  [[nodiscard]] std::uint64_t RoundElapsedMs(std::uint64_t now_us) const;
  [[nodiscard]] std::uint64_t AttemptElapsedMs(std::uint64_t now_us) const;

  // Ends the current round. Returns true while the attempt remains active and
  // the next suitable frame may start another round. Sets attempt_expired when
  // the whole attempt budget was consumed and cooldown was entered.
  bool RecordRoundFailure(std::uint64_t now_us, bool* attempt_expired);
  // Consistency is still pending: end the round, keep the attempt alive.
  void RecordRoundPending(std::uint64_t now_us);
  void RecordRoundSuccess(std::uint64_t now_us);

  [[nodiscard]] const AutomaticRecoveryConfig& config() const {
    return config_;
  }

  // Same-sensor-time-domain check. latest_lidar/latest_imu are the newest
  // available input timestamps, not wall clock. Missing or rolled-back inputs
  // are rejected rather than treated as fresh.
  static SourceFreshness EvaluateSourceFreshness(
      std::uint64_t source_timestamp_us,
      std::optional<std::uint64_t> latest_lidar_timestamp_us,
      std::optional<std::uint64_t> latest_imu_timestamp_us,
      std::uint64_t max_source_age_us);

  // A cached candidate seed may be revalidated only on a strictly newer raw
  // lidar frame in the same (non-replay) epoch. The final commit still runs
  // the normal freshness gate against the newest raw inputs.
  static SeedDecision EvaluateSeedUsability(
      std::uint64_t seed_source_timestamp_us,
      std::uint64_t seed_epoch,
      std::uint64_t current_frame_timestamp_us,
      std::uint64_t current_epoch,
      std::optional<std::uint64_t> latest_lidar_timestamp_us);

 private:
  void EnterCooldown(std::uint64_t now_us);

  AutomaticRecoveryConfig config_;
  std::uint64_t round_budget_us_{0U};
  std::uint64_t attempt_budget_us_{0U};
  std::uint64_t cooldown_us_{0U};
  std::uint64_t max_source_age_us_{0U};

  bool attempt_active_{false};
  bool round_active_{false};
  bool cooldown_armed_{false};
  std::uint64_t attempt_start_us_{0U};
  std::uint64_t attempt_deadline_us_{0U};
  std::uint64_t round_start_us_{0U};
  std::uint64_t round_deadline_us_{0U};
  std::uint64_t cooldown_until_us_{0U};
};

}  // namespace localization_health

#endif  // LIO_SAM_HESAI_LOCALIZATION_HEALTH_AUTOMATIC_RECOVERY_POLICY_H
