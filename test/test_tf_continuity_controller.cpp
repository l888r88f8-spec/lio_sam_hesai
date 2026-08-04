#include "localization_health/tf_continuity_controller.h"

#include <gtest/gtest.h>

namespace {

using localization_health::TfContinuityController;
using localization_health::TfHoldReason;
using localization_health::TfPoseSample;
using localization_health::TfPublicationMode;

TfPoseSample Sample(TimeStampUs sensor_timestamp_us, double x) {
  TfPoseSample sample;
  sample.map_base_pose = Mat4d::Identity();
  sample.map_base_pose(0, 3) = x;
  sample.sensor_timestamp_us = sensor_timestamp_us;
  return sample;
}

TfContinuityController MakeController() {
  return TfContinuityController({100000U, 500000U});
}

TEST(TfContinuityController, DoesNotPublishBeforeItHasAnAnchor) {
  auto controller = MakeController();

  EXPECT_FALSE(controller.Select(100000U, std::nullopt, std::nullopt).has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);
  EXPECT_FALSE(controller.anchor_sensor_timestamp_us().has_value());
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
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 3.0);
  EXPECT_EQ(decision->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 200000U);
}

TEST(TfContinuityController, HoldsAnchorForMissingOrOverLimitPrediction) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  const auto missing = controller.Select(1100000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(missing.has_value());
  EXPECT_EQ(missing->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(missing->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(missing->sample.sensor_timestamp_us, 1000000U);
  EXPECT_EQ(missing->publication_timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(missing->sample.map_base_pose(0, 3), 1.0);
  EXPECT_EQ(missing->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(missing->extrapolation_duration_us, 0U);

  const auto over_limit = controller.Select(
      1600000U, 1600000U, Sample(1600000U, 2.0));
  ASSERT_TRUE(over_limit.has_value());
  EXPECT_EQ(over_limit->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(over_limit->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(over_limit->sample.sensor_timestamp_us, 1000000U);
  EXPECT_EQ(over_limit->publication_timestamp_us, 1600000U);
  EXPECT_DOUBLE_EQ(over_limit->sample.map_base_pose(0, 3), 1.0);
  EXPECT_EQ(over_limit->anchor_sensor_timestamp_us, 1000000U);
  EXPECT_EQ(over_limit->extrapolation_duration_us, 0U);
}

TEST(TfContinuityController, RateLimitsFallbackAndReplacesAnchorWithMeasurement) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  EXPECT_FALSE(controller.Select(1050000U, std::nullopt, std::nullopt).has_value());

  const auto measured = controller.RecordMeasured(
      Sample(1100000U, 2.0), 1100000U);
  EXPECT_EQ(measured.mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured.hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(measured.sample.sensor_timestamp_us, 1100000U);
  EXPECT_EQ(measured.publication_timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(measured.sample.map_base_pose(0, 3), 2.0);

  const auto held = controller.Select(1200000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(held->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(held->sample.sensor_timestamp_us, 1100000U);
  EXPECT_EQ(held->publication_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(held->sample.map_base_pose(0, 3), 2.0);
  EXPECT_EQ(held->anchor_sensor_timestamp_us, 1100000U);
}

TEST(TfContinuityController, HoldsWhenAnchorAgeExceedsExtrapolationLimit) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  const auto decision = controller.Select(
      10000000U, 10000000U, Sample(1400000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->publication_timestamp_us, 10000000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 1.0);
}

TEST(TfContinuityController, HoldsWhenPredictionTimestampIsInTheFuture) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);

  const auto decision = controller.Select(
      1200000U, 1200000U, Sample(1300000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->publication_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 1.0);
}

TEST(TfContinuityController, ClampsLateMeasurementToLastEmittedTimestamp) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);
  const auto held = controller.Select(1100000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(held.has_value());
  ASSERT_EQ(held->publication_timestamp_us, 1100000U);

  const auto measured = controller.RecordMeasured(
      Sample(1050000U, 2.0), 1050000U);

  EXPECT_EQ(measured.mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured.sample.sensor_timestamp_us, 1050000U);
  EXPECT_EQ(measured.publication_timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(measured.sample.map_base_pose(0, 3), 2.0);
  ASSERT_TRUE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(*controller.anchor_sensor_timestamp_us(), 1050000U);
}

TEST(TfContinuityController, ExtrapolatesSensorPredictionRegardlessOfRosStamp) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);
  const auto held = controller.Select(1100000U, std::nullopt, std::nullopt);
  ASSERT_TRUE(held.has_value());
  ASSERT_EQ(held->publication_timestamp_us, 1100000U);

  const auto decision = controller.Select(
      1200000U, 1200000U, Sample(1050000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1050000U);
  EXPECT_EQ(decision->publication_timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 2.0);
}

TEST(TfContinuityController, ResetAcceptsLowerEpochMeasurementAndFallback) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0), 1000000U);
  ASSERT_TRUE(controller.Select(1100000U, std::nullopt, std::nullopt).has_value());

  controller.Reset();

  EXPECT_FALSE(controller.anchor_sensor_timestamp_us().has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);

  const auto measured = controller.RecordMeasured(
      Sample(100000U, 2.0), 100000U);
  EXPECT_EQ(measured.mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured.sample.sensor_timestamp_us, 100000U);
  EXPECT_EQ(measured.publication_timestamp_us, 100000U);

  const auto fallback = controller.Select(
      200000U, 200000U, Sample(200000U, 3.0));
  ASSERT_TRUE(fallback.has_value());
  EXPECT_EQ(fallback->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(fallback->sample.sensor_timestamp_us, 200000U);
  EXPECT_EQ(fallback->publication_timestamp_us, 200000U);
  EXPECT_DOUBLE_EQ(fallback->sample.map_base_pose(0, 3), 3.0);
}

TEST(TfContinuityController, ExtrapolatesSensorTimelineAcrossRosEpoch) {
  auto controller = MakeController();
  constexpr TimeStampUs kRosUnixTimestampUs = 1781000000000000U;
  controller.RecordMeasured(Sample(1000000U, 1.0), kRosUnixTimestampUs);

  const auto decision = controller.Select(
      kRosUnixTimestampUs + 100000U, 1200000U, Sample(1200000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.sensor_timestamp_us, 1200000U);
  EXPECT_EQ(decision->publication_timestamp_us,
            kRosUnixTimestampUs + 100000U);
  EXPECT_EQ(decision->publication_timestamp_us - kRosUnixTimestampUs,
            100000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 200000U);
}

}  // namespace
