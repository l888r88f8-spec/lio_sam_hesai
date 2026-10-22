#include "localization_health/automatic_recovery_policy.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

namespace {

using localization_health::AutomaticRecoveryConfig;
using localization_health::AutomaticRecoveryPolicy;
using localization_health::AutomaticSeedLifecycle;
using localization_health::AutoRecoveryDecision;
using localization_health::SeedDecision;
using localization_health::SeedQualification;
using localization_health::SourceFreshness;

constexpr std::uint64_t kMs = 1000U;

AutomaticRecoveryConfig DefaultConfig() {
  AutomaticRecoveryConfig config;
  config.round_timeout_ms = 1000;
  config.attempt_timeout_ms = 5000;
  config.cooldown_ms = 2000;
  config.max_source_age_ms = 500;
  return config;
}

TEST(AutomaticRecoveryPolicy, RoundFailureRetriesOnNextFrameWithinAttempt) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_TRUE(policy.AttemptActive());
  ASSERT_EQ(policy.BeginRound(10U), AutoRecoveryDecision::kStartRound);

  bool attempt_expired = true;
  EXPECT_TRUE(policy.RecordRoundFailure(374U * kMs, &attempt_expired));
  EXPECT_FALSE(attempt_expired);
  EXPECT_TRUE(policy.AttemptActive());
  EXPECT_FALSE(policy.RoundDeadlineExceeded(400U * kMs));

  // The very next suitable frame starts another round; no new LOST_ENTRY or
  // STREAM_RECOVERED edge is required.
  EXPECT_EQ(policy.BeginRound(500U * kMs), AutoRecoveryDecision::kStartRound);
  EXPECT_TRUE(policy.RoundDeadlineExceeded(1501U * kMs));
}

TEST(AutomaticRecoveryPolicy, AttemptDeadlineEntersCooldownThenHealthyFrameRetries) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(100U * kMs);
  ASSERT_EQ(policy.BeginRound(100U * kMs), AutoRecoveryDecision::kStartRound);

  bool attempt_expired = false;
  EXPECT_FALSE(policy.RecordRoundFailure(5101U * kMs, &attempt_expired));
  EXPECT_TRUE(attempt_expired);
  EXPECT_FALSE(policy.AttemptActive());
  EXPECT_TRUE(policy.InCooldown(6000U * kMs));

  // Still cooling down: no round, even on a healthy frame.
  EXPECT_FALSE(policy.EnsureAttempt(6000U * kMs, true));
  // Cooldown elapsed but stream unhealthy: still no busy retry.
  EXPECT_FALSE(policy.EnsureAttempt(7101U * kMs, false));
  EXPECT_FALSE(policy.AttemptActive());
  // Cooldown elapsed and stream healthy: restart without an external event.
  EXPECT_TRUE(policy.EnsureAttempt(7102U * kMs, true));
  EXPECT_TRUE(policy.AttemptActive());
  EXPECT_FALSE(policy.InCooldown(7102U * kMs));
  EXPECT_EQ(policy.BeginRound(7102U * kMs), AutoRecoveryDecision::kStartRound);
}

TEST(AutomaticRecoveryPolicy, AttemptDeadlineIsCheckedAtRoundStart) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  // A long stall without a completed round must not extend the attempt.
  EXPECT_EQ(policy.BeginRound(5001U * kMs),
            AutoRecoveryDecision::kAttemptExpired);
  EXPECT_FALSE(policy.AttemptActive());
  EXPECT_TRUE(policy.InCooldown(5001U * kMs));
}

TEST(AutomaticRecoveryPolicy, ManualInitializationCancelsActiveAutomaticRound) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(0U), AutoRecoveryDecision::kStartRound);
  ASSERT_TRUE(policy.AttemptActive());

  policy.CancelForManual(100U * kMs);
  EXPECT_FALSE(policy.AttemptActive());
  EXPECT_FALSE(policy.RoundDeadlineExceeded(200U * kMs));
  EXPECT_FALSE(policy.InCooldown(100U * kMs));
  EXPECT_EQ(policy.BeginRound(200U * kMs),
            AutoRecoveryDecision::kAttemptExpired);
}

TEST(AutomaticRecoveryPolicy, AutomaticRoundBudgetIsIndependentOfTrackingBudget) {
  AutomaticRecoveryConfig config = DefaultConfig();
  config.round_timeout_ms = 1000;
  AutomaticRecoveryPolicy policy(config);
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(0U), AutoRecoveryDecision::kStartRound);

  // The tracking watchdog would have dropped this frame after 500 ms; the
  // automatic round is still valid and the attempt is still open.
  EXPECT_FALSE(policy.RoundDeadlineExceeded(600U * kMs));
  EXPECT_FALSE(policy.RoundDeadlineExceeded(1000U * kMs));
  EXPECT_TRUE(policy.RoundDeadlineExceeded(1001U * kMs));
  EXPECT_TRUE(policy.AttemptActive());

  bool attempt_expired = true;
  EXPECT_TRUE(policy.RecordRoundFailure(1001U * kMs, &attempt_expired));
  EXPECT_FALSE(attempt_expired);
  EXPECT_TRUE(policy.AttemptActive());
}

TEST(AutomaticRecoveryPolicy, ConsistencyPendingEndsRoundButKeepsAttempt) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(0U), AutoRecoveryDecision::kStartRound);
  policy.RecordRoundPending(200U * kMs);
  EXPECT_FALSE(policy.RoundDeadlineExceeded(500U * kMs));
  EXPECT_TRUE(policy.AttemptActive());
  EXPECT_EQ(policy.BeginRound(3000U * kMs), AutoRecoveryDecision::kStartRound);
}

TEST(AutomaticRecoveryPolicy, RejectsInvalidBudgets) {
  AutomaticRecoveryConfig config = DefaultConfig();
  config.round_timeout_ms = 0;
  EXPECT_THROW(AutomaticRecoveryPolicy{config}, std::invalid_argument);

  config = DefaultConfig();
  config.attempt_timeout_ms = 999;
  EXPECT_THROW(AutomaticRecoveryPolicy{config}, std::invalid_argument);

  config = DefaultConfig();
  config.max_source_age_ms = 0;
  EXPECT_THROW(AutomaticRecoveryPolicy{config}, std::invalid_argument);
}

TEST(AutomaticRecoveryPolicy, SourceFreshnessUsesSensorTimeDomainOnly) {
  // 300 ms in the sensor time domain is fresh even if wall clock advanced a lot.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::optional<std::uint64_t>{10'300'000U},
                std::optional<std::uint64_t>{10'150'000U}, 500U * kMs),
            SourceFreshness::kFresh);
  // Multi-second old source frame (544 ms behind the newest sensor sample).
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::optional<std::uint64_t>{10'544'000U},
                std::optional<std::uint64_t>{10'500'000U}, 500U * kMs),
            SourceFreshness::kStaleAge);
  // Timestamp rollback: the "latest" sample is older than the source frame.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::optional<std::uint64_t>{9'900'000U},
                std::optional<std::uint64_t>{9'950'000U}, 500U * kMs),
            SourceFreshness::kTimestampRollback);
  // No newest input at all must not be reported as fresh.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::nullopt, std::nullopt, 500U * kMs),
            SourceFreshness::kNoLatestInput);
  // Invalid source timestamp must not be reported as fresh.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                0U, std::optional<std::uint64_t>{10'000'000U},
                std::nullopt, 500U * kMs),
            SourceFreshness::kInvalidTimestamp);
  // IMU alone can prove freshness; it is the same sensor time domain.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::nullopt,
                std::optional<std::uint64_t>{10'100'000U}, 500U * kMs),
            SourceFreshness::kFresh);
}

TEST(AutomaticRecoveryPolicy, SuccessClearsAttemptAndCooldown) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(0U), AutoRecoveryDecision::kStartRound);
  policy.RecordRoundSuccess(800U * kMs);
  EXPECT_FALSE(policy.AttemptActive());
  EXPECT_FALSE(policy.InCooldown(800U * kMs));
  EXPECT_FALSE(policy.RoundDeadlineExceeded(900U * kMs));
}


TEST(AutomaticRecoveryPolicy,
     RoundCannotCommitPastOverallAttemptDeadlineRegression) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(4900U * kMs),
            AutoRecoveryDecision::kStartRound);
  EXPECT_TRUE(policy.RoundDeadlineExceeded(5001U * kMs));
  EXPECT_EQ(policy.RoundTimeout(5001U * kMs),
            localization_health::RoundTimeoutReason::kAttemptDeadline);
}

TEST(AutomaticRecoveryPolicy,
     RepeatedExternalRequestsDoNotExtendActiveAttemptRegression) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  policy.BeginAttempt(4000U * kMs);
  EXPECT_EQ(policy.BeginRound(5001U * kMs),
            AutoRecoveryDecision::kAttemptExpired);
}

TEST(AutomaticRecoveryPolicy,
     ExternalRequestCannotBypassCooldownRegression) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(0U);
  ASSERT_EQ(policy.BeginRound(5001U * kMs),
            AutoRecoveryDecision::kAttemptExpired);
  policy.BeginAttempt(6000U * kMs);
  EXPECT_TRUE(policy.InCooldown(6000U * kMs));
  EXPECT_FALSE(policy.AttemptActive());
}

TEST(AutomaticRecoveryPolicy,
     ExpireAttemptIfDueWorksWithoutHealthyBeginRound) {
  AutomaticRecoveryPolicy policy(DefaultConfig());
  policy.BeginAttempt(100U * kMs);
  EXPECT_FALSE(policy.ExpireAttemptIfDue(5099U * kMs));
  EXPECT_TRUE(policy.ExpireAttemptIfDue(5100U * kMs));
  EXPECT_FALSE(policy.AttemptActive());
  EXPECT_TRUE(policy.InCooldown(5100U * kMs));
}

TEST(AutomaticRecoveryPolicy,
     RawLidarRollbackCannotBeHiddenByAdvancedImu) {
  // source=10.0 s, raw lidar=9.9 s, IMU=10.1 s. Taking max(lidar, imu) would
  // report fresh; the raw lidar rollback must win.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                10'000'000U, std::optional<std::uint64_t>{9'900'000U},
                std::optional<std::uint64_t>{10'100'000U}, 500U * kMs),
            SourceFreshness::kTimestampRollback);
}

TEST(AutomaticSeedLifecycle, OneOfThreeAndZeroOfZeroAreNotReusable) {
  AutomaticSeedLifecycle lifecycle;
  const auto one_of_three = lifecycle.ObserveFreshCandidate(
      false, false, 0.0, 0.0, 3, 0.5, 0.1);
  EXPECT_EQ(one_of_three, SeedQualification::kNone);
  EXPECT_EQ(lifecycle.hit_count(), 1);
  EXPECT_FALSE(AutomaticSeedLifecycle::CanReuse(one_of_three));

  lifecycle.Reset();
  const auto zero_of_zero = lifecycle.ObserveFreshCandidate(
      false, false, 0.0, 0.0, 0, 0.5, 0.1);
  EXPECT_EQ(zero_of_zero, SeedQualification::kNone);
  EXPECT_EQ(lifecycle.hit_count(), 0);
  EXPECT_FALSE(AutomaticSeedLifecycle::CanReuse(zero_of_zero));
}

TEST(AutomaticSeedLifecycle,
     ThreeIndependentFramesQualifyAndReuseAddsNoObservation) {
  AutomaticSeedLifecycle lifecycle;
  EXPECT_EQ(lifecycle.ObserveFreshCandidate(false, false, 0.0, 0.0, 3, 0.5,
                                            0.1),
            SeedQualification::kNone);
  EXPECT_EQ(lifecycle.hit_count(), 1);
  EXPECT_EQ(lifecycle.ObserveFreshCandidate(false, true, 0.0, 0.0, 3, 0.5,
                                            0.1),
            SeedQualification::kNone);
  EXPECT_EQ(lifecycle.hit_count(), 2);
  const auto qualified = lifecycle.ObserveFreshCandidate(
      false, true, 0.0, 0.0, 3, 0.5, 0.1);
  ASSERT_EQ(qualified, SeedQualification::kConsistencyPassed);
  EXPECT_EQ(lifecycle.hit_count(), 3);
  EXPECT_TRUE(AutomaticSeedLifecycle::CanReuse(qualified));

  // Production seed-reuse path: EvaluateSeedUsability + final freshness, and
  // deliberately no ObserveFreshCandidate call.
  constexpr std::uint64_t seed_ts = 10'000'000U;
  constexpr std::uint64_t epoch = 3U;
  const std::uint64_t newer_frame_ts = seed_ts + 700U * kMs;
  const std::uint64_t latest_raw_ts = seed_ts + 720U * kMs;
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                seed_ts, epoch, newer_frame_ts, epoch,
                std::optional<std::uint64_t>{latest_raw_ts}),
            SeedDecision::kUseSeed);
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                newer_frame_ts, std::optional<std::uint64_t>{latest_raw_ts},
                std::nullopt, 500U * kMs),
            SourceFreshness::kFresh);
  // A qualified 3/3 seed must not become 4/4 just because it is reused.
  EXPECT_EQ(lifecycle.hit_count(), 3);
}

TEST(AutomaticSeedLifecycle,
     EarlyAcceptedSeedReusesWithoutConsistencyCount) {
  AutomaticSeedLifecycle lifecycle;
  const auto early = lifecycle.ObserveFreshCandidate(true, false, 0.0, 0.0, 3,
                                                     0.5, 0.1);
  EXPECT_EQ(early, SeedQualification::kEarlyAccepted);
  EXPECT_EQ(lifecycle.hit_count(), 0);
  EXPECT_TRUE(AutomaticSeedLifecycle::CanReuse(early));

  constexpr std::uint64_t seed_ts = 20'000'000U;
  constexpr std::uint64_t epoch = 5U;
  const std::uint64_t newer_frame_ts = seed_ts + 100U * kMs;
  const std::uint64_t latest_raw_ts = seed_ts + 120U * kMs;
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                seed_ts, epoch, newer_frame_ts, epoch,
                std::optional<std::uint64_t>{latest_raw_ts}),
            SeedDecision::kUseSeed);
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSourceFreshness(
                newer_frame_ts, std::optional<std::uint64_t>{latest_raw_ts},
                std::nullopt, 500U * kMs),
            SourceFreshness::kFresh);
  EXPECT_EQ(lifecycle.hit_count(), 0);
}

TEST(AutomaticSeedLifecycle,
     QueuedFrameAheadOfLatestRawInputIsTimestampRollback) {
  // Queued frame 10.3 s, seed 10.0 s, newest raw lidar 10.2 s: the queued frame
  // cannot have come from the current raw stream.
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                10'000'000U, 3U, 10'300'000U, 3U,
                std::optional<std::uint64_t>{10'200'000U}),
            SeedDecision::kTimestampRollback);
}

TEST(AutomaticRecoveryPolicy, SeedRejectedAcrossReplayEpochAndNoNewerFrame) {
  constexpr std::uint64_t seed_ts = 10'000'000U;
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                seed_ts, 3U, seed_ts + 100'000U, 4U,
                std::optional<std::uint64_t>{seed_ts + 120'000U}),
            localization_health::SeedDecision::kReplayEpochChanged);
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                seed_ts, 3U, seed_ts, 3U,
                std::optional<std::uint64_t>{seed_ts}),
            localization_health::SeedDecision::kNoNewerFrame);
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                seed_ts, 3U, seed_ts - 1U, 3U,
                std::optional<std::uint64_t>{seed_ts + 100'000U}),
            localization_health::SeedDecision::kTimestampRollback);
  EXPECT_EQ(AutomaticRecoveryPolicy::EvaluateSeedUsability(
                0U, 3U, seed_ts, 3U,
                std::optional<std::uint64_t>{seed_ts + 100'000U}),
            localization_health::SeedDecision::kSourceInvalid);
}

}  // namespace
