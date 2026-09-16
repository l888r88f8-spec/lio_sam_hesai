#include "localization_health/tf_continuity_controller.h"

#include <gtest/gtest.h>

#include <optional>

namespace {

using localization_health::TfContinuityController;
using localization_health::TfHoldReason;
using localization_health::TfPoseSample;
using localization_health::TfPublicationMode;

TfPoseSample Sample(TimeStampUs sensor_timestamp_us, double x) {
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation().x() = x;

  TfPoseSample sample;
  sample.map_T_base = MapTBase(value);
  sample.sensor_timestamp_us = sensor_timestamp_us;
  return sample;
}

TfContinuityController MakeController() {
  return TfContinuityController({100000U, 500000U});
}

// Establishes one emitted current-horizon pose: the measurement at 1.0s is
// projected to the 1.2s IMU horizon before it may reach TF.
void EstablishEmittedPose(TfContinuityController& controller) {
  const auto decision = controller.RecordMeasured(
      Sample(1000000U, 1.0), 1200000U, Sample(1200000U, 1.2));
  ASSERT_TRUE(decision.has_value());
  ASSERT_EQ(decision->mode, TfPublicationMode::kMeasured);
  ASSERT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  ASSERT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
}

TEST(TfContinuityController, DoesNotPublishBeforeItHasAnAnchor) {
  auto controller = MakeController();

  EXPECT_FALSE(controller.Select(100000U, std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);
  EXPECT_FALSE(controller.anchor_sensor_timestamp_us().has_value());
}

TEST(TfContinuityController, PublishesProjectedMeasurementAtCurrentHorizon) {
  auto controller = MakeController();

  const auto decision = controller.RecordMeasured(
      Sample(1000000U, 1.0), 1200000U, Sample(1200000U, 1.2));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1200000U);
  EXPECT_EQ(decision->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 200000U);

  EXPECT_EQ(controller.mode(), TfPublicationMode::kMeasured);
  ASSERT_TRUE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(*controller.anchor_sensor_timestamp_us(), 1000000U);
}

TEST(TfContinuityController, DoesNotPublishDelayedMeasurementAsCurrentPose) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  // A result measured at 1.05s is delivered at 1.3s without a projection to
  // the current horizon. It must not be relabelled as the current TF pose.
  const auto decision =
      controller.RecordMeasured(Sample(1050000U, 0.5), 1300000U);

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1300000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 0U);

  // The raw measurement is still retained as the anchor for later frames.
  ASSERT_TRUE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(*controller.anchor_sensor_timestamp_us(), 1050000U);
}

TEST(TfContinuityController, DoesNotSnapBackFromExtrapolatedPoseToDelayedMeasurement) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);
  // The fallback extrapolated the anchor to the current horizon: X=2.0 at 1.5s.
  const auto extrapolated = controller.Select(
      1500000U, 1500000U, Sample(1500000U, 2.0));
  ASSERT_TRUE(extrapolated.has_value());
  ASSERT_EQ(extrapolated->mode, TfPublicationMode::kExtrapolated);
  ASSERT_DOUBLE_EQ(extrapolated->sample.map_T_base.Translation().x(), 2.0);

  // A delayed measurement at X=1.0, measured at 1.05s, is delivered at 1.6s
  // without a projection. Publishing it would be the recorded sawtooth.
  const auto decision =
      controller.RecordMeasured(Sample(1050000U, 1.0), 1600000U);

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1500000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 2.0);
  EXPECT_EQ(decision->publication_timestamp_us, 1600000U);

  // A projection of that same measurement is allowed to move TF again.
  const auto projected = controller.RecordMeasured(
      Sample(1250000U, 1.0), 1700000U, Sample(1700000U, 2.3));
  ASSERT_TRUE(projected.has_value());
  EXPECT_EQ(projected->mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(projected->sample.sensor_timestamp_us, 1700000U);
  EXPECT_DOUBLE_EQ(projected->sample.map_T_base.Translation().x(), 2.3);
  EXPECT_EQ(projected->extrapolation_duration_us, 450000U);
}

TEST(TfContinuityController, RejectsProjectionBehindTheLastEmittedHorizon) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  // A projection that resolves behind the already emitted horizon is a stale
  // frame: hold the last published pose instead of moving TF backwards in time.
  const auto decision = controller.RecordMeasured(
      Sample(1100000U, 1.1), 1300000U, Sample(1150000U, 1.15));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1300000U);
}

TEST(TfContinuityController, RejectsProjectionBeyondTheExtrapolationLimit) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  // The anchor at 1.05s projected to 1.65s spans 0.6s, beyond the 0.5s bound.
  const auto decision = controller.RecordMeasured(
      Sample(1050000U, 1.05), 1700000U, Sample(1650000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1700000U);
}

TEST(TfContinuityController, DoesNotFabricateTfBeforeTheFirstProjection) {
  auto controller = MakeController();

  const auto decision = controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  EXPECT_FALSE(decision.has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);

  // The anchor exists, but Select must not publish it as the current pose.
  EXPECT_FALSE(controller.Select(1100000U, std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
}

TEST(TfContinuityController, PublishesAValidPredictionAsExtrapolated) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  const auto decision = controller.Select(
      1200000U, 1200000U, Sample(1200000U, 3.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_EQ(decision->publication_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 3.0);
  EXPECT_EQ(decision->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 200000U);
}

TEST(TfContinuityController, HoldsLastEmittedPoseForMissingOrOverLimitPrediction) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  const auto missing = controller.Select(1300000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(missing.has_value());
  EXPECT_EQ(missing->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(missing->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(missing->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(missing->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(missing->publication_timestamp_us, 1300000U);
  EXPECT_EQ(missing->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(missing->extrapolation_duration_us, 0U);

  const auto over_limit = controller.Select(
      1900000U, 1900000U, Sample(1900000U, 3.0));
  ASSERT_TRUE(over_limit.has_value());
  EXPECT_EQ(over_limit->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(over_limit->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(over_limit->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(over_limit->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(over_limit->publication_timestamp_us, 1900000U);
  EXPECT_EQ(over_limit->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(over_limit->extrapolation_duration_us, 0U);
}

TEST(TfContinuityController, RateLimitsFallbackAndReplacesAnchorWithProjection) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  EXPECT_FALSE(controller.Select(1250000U, std::nullopt, std::nullopt).has_value());

  const auto measured = controller.RecordMeasured(
      Sample(1100000U, 2.0), 1300000U, Sample(1300000U, 2.3));
  ASSERT_TRUE(measured.has_value());
  EXPECT_EQ(measured->mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(measured->sample.sensor_timestamp_us, 1300000U);
  EXPECT_DOUBLE_EQ(measured->sample.map_T_base.Translation().x(), 2.3);
  EXPECT_EQ(measured->publication_timestamp_us, 1300000U);
  EXPECT_EQ(measured->anchor_sensor_timestamp_us, 1100000U);
  EXPECT_EQ(measured->extrapolation_duration_us, 200000U);

  const auto held = controller.Select(1400000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(held->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(held->sample.sensor_timestamp_us, 1300000U);
  EXPECT_DOUBLE_EQ(held->sample.map_T_base.Translation().x(), 2.3);
  EXPECT_EQ(held->publication_timestamp_us, 1400000U);
  EXPECT_EQ(held->anchor_sensor_timestamp_us, 1100000U);
}

TEST(TfContinuityController, KeepsPublicationTimestampMonotonic) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  // The projection is newer, but the ROS clock advanced less than the previous
  // publication already did: the emitted stamp must not move backwards.
  const auto late = controller.RecordMeasured(
      Sample(1100000U, 2.0), 1100000U, Sample(1250000U, 2.25));

  ASSERT_TRUE(late.has_value());
  EXPECT_EQ(late->mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(late->sample.sensor_timestamp_us, 1250000U);
  EXPECT_DOUBLE_EQ(late->sample.map_T_base.Translation().x(), 2.25);
  EXPECT_EQ(late->publication_timestamp_us, 1200000U);
  EXPECT_EQ(late->anchor_sensor_timestamp_us, 1100000U);
}

TEST(TfContinuityController, HoldsWhenAnchorAgeExceedsExtrapolationLimit) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  const auto decision = controller.Select(
      10000000U, 10000000U, Sample(1400000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 10000000U);
}

TEST(TfContinuityController, HoldsWhenPredictionTimestampIsInTheFuture) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  const auto decision = controller.Select(
      1300000U, 1300000U, Sample(1400000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1300000U);
}

TEST(TfContinuityController, HoldsWhenPredictionHorizonIsBehindTheLastEmittedPose) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  const auto decision = controller.Select(
      1300000U, 1300000U, Sample(1150000U, 5.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_T_base.Translation().x(), 1.2);
  EXPECT_EQ(decision->publication_timestamp_us, 1300000U);
}

TEST(TfContinuityController, ResetForgetsEmittedPoseAndAcceptsLowerSensorTimeline) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);
  ASSERT_TRUE(controller.Select(1300000U, std::nullopt, std::nullopt).has_value());

  controller.Reset();

  EXPECT_FALSE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);

  // After a clock jump a lower sensor timeline is accepted, but the historical
  // measurement is still not published as a current pose.
  const auto measured = controller.RecordMeasured(Sample(100000U, 7.0), 100000U);
  EXPECT_FALSE(measured.has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  ASSERT_TRUE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(*controller.anchor_sensor_timestamp_us(), 100000U);

  const auto projected = controller.RecordMeasured(
      Sample(150000U, 7.0), 200000U, Sample(200000U, 7.5));
  ASSERT_TRUE(projected.has_value());
  EXPECT_EQ(projected->mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(projected->sample.sensor_timestamp_us, 200000U);
  EXPECT_DOUBLE_EQ(projected->sample.map_T_base.Translation().x(), 7.5);
  EXPECT_EQ(projected->publication_timestamp_us, 200000U);

  const auto fallback = controller.Select(
      300000U, 300000U, Sample(300000U, 8.0));
  ASSERT_TRUE(fallback.has_value());
  EXPECT_EQ(fallback->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(fallback->sample.sensor_timestamp_us, 300000U);
  EXPECT_EQ(fallback->publication_timestamp_us, 300000U);
  EXPECT_DOUBLE_EQ(fallback->sample.map_T_base.Translation().x(), 8.0);
}

TEST(TfContinuityController, ExtrapolatesSensorTimelineAcrossRosEpoch) {
  auto controller = MakeController();
  constexpr TimeStampUs kRosUnixTimestampUs = 1781000000000000U;
  controller.RecordMeasured(Sample(1000000U, 1.0),
                            kRosUnixTimestampUs + 200000U,
                            Sample(1200000U, 1.2));

  const auto decision = controller.Select(
      kRosUnixTimestampUs + 300000U, 1400000U, Sample(1400000U, 1.4));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1400000U);
  EXPECT_EQ(decision->publication_timestamp_us,
            kRosUnixTimestampUs + 300000U);
  EXPECT_EQ(decision->publication_timestamp_us - kRosUnixTimestampUs,
            300000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 400000U);
}

}  // namespace

namespace {

// A TF pose may only be held for the configured extrapolation bound. Beyond
// that the pose no longer describes the vehicle and navigation must know.
TEST(TfContinuityController, DoesNotReportStaleBeforeAnyEmission) {
  auto controller = MakeController();

  EXPECT_FALSE(controller.IsPoseStale(9000000U, false));
  EXPECT_FALSE(controller.IsPoseStale(std::nullopt, false));
  EXPECT_FALSE(controller.last_emitted_sensor_timestamp_us().has_value());
}

TEST(TfContinuityController, TrackingWithoutAnyEmissionIsStale) {
  auto controller = MakeController();

  // Startup/relocalization without a TF pose is not staleness, but a tracking
  // vehicle with no emitted pose has no TF continuity at all.
  EXPECT_FALSE(controller.IsPoseStale(1000000U, false));
  EXPECT_TRUE(controller.IsPoseStale(1000000U, true));
}

TEST(TfContinuityController, ReportsStaleOnlyBeyondTheExtrapolationBound) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  ASSERT_TRUE(controller.last_emitted_sensor_timestamp_us().has_value());
  EXPECT_EQ(*controller.last_emitted_sensor_timestamp_us(), 1200000U);

  // A short hold is transient and stays healthy.
  EXPECT_FALSE(controller.IsPoseStale(1500000U, true));
  // Exactly at the bound is still allowed; one microsecond beyond is stale.
  EXPECT_FALSE(controller.IsPoseStale(1700000U, true));
  EXPECT_TRUE(controller.IsPoseStale(1700001U, true));
}

TEST(TfContinuityController, ClockReversalIsNotStale) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);

  // A sensor clock that jumped backwards is a stream fault, not a held pose.
  EXPECT_FALSE(controller.IsPoseStale(100000U, true));
  EXPECT_FALSE(controller.IsPoseStale(std::nullopt, true));
}

TEST(TfContinuityController, HeldEmissionAdvancesStalenessReference) {
  auto controller = MakeController();
  EstablishEmittedPose(controller);
  ASSERT_TRUE(controller.Select(1900000U, std::nullopt, std::nullopt).has_value());

  // The held publication did not move the sensor horizon, so staleness is
  // still measured from the last emitted pose at 1.2s, not from 1.9s.
  EXPECT_TRUE(controller.IsPoseStale(1900000U, true));
}

}  // namespace
