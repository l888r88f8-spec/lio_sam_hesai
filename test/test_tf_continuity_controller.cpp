#include "localization_health/tf_continuity_controller.h"

#include <gtest/gtest.h>

namespace {

using localization_health::TfContinuityController;
using localization_health::TfHoldReason;
using localization_health::TfPoseSample;
using localization_health::TfPublicationMode;

TfPoseSample Sample(TimeStampUs timestamp_us, double x) {
  TfPoseSample sample;
  sample.map_base_pose = Mat4d::Identity();
  sample.map_base_pose(0, 3) = x;
  sample.timestamp_us = timestamp_us;
  return sample;
}

TfContinuityController MakeController() {
  return TfContinuityController({100000U, 500000U});
}

TEST(TfContinuityController, DoesNotPublishBeforeItHasAnAnchor) {
  auto controller = MakeController();

  EXPECT_FALSE(controller.Select(100000U, std::nullopt).has_value());
  EXPECT_EQ(controller.mode(), TfPublicationMode::kNoAnchor);
  EXPECT_EQ(controller.hold_reason(), TfHoldReason::kNoAnchor);
  EXPECT_FALSE(controller.anchor_timestamp_us().has_value());
}

TEST(TfContinuityController, PublishesAValidPredictionAsExtrapolated) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));

  const auto decision = controller.Select(1200000U, Sample(1200000U, 3.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kExtrapolated);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(decision->sample.timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 3.0);
  EXPECT_EQ(decision->anchor_timestamp_us, 1000000U);
  EXPECT_EQ(decision->extrapolation_duration_us, 200000U);
}

TEST(TfContinuityController, HoldsAnchorForMissingOrOverLimitPrediction) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));

  const auto missing = controller.Select(1100000U, std::nullopt);
  ASSERT_TRUE(missing.has_value());
  EXPECT_EQ(missing->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(missing->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(missing->sample.timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(missing->sample.map_base_pose(0, 3), 1.0);
  EXPECT_EQ(missing->anchor_timestamp_us, 1000000U);
  EXPECT_EQ(missing->extrapolation_duration_us, 0U);

  const auto over_limit = controller.Select(1600000U, Sample(1600000U, 2.0));
  ASSERT_TRUE(over_limit.has_value());
  EXPECT_EQ(over_limit->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(over_limit->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(over_limit->sample.timestamp_us, 1600000U);
  EXPECT_DOUBLE_EQ(over_limit->sample.map_base_pose(0, 3), 1.0);
  EXPECT_EQ(over_limit->anchor_timestamp_us, 1000000U);
  EXPECT_EQ(over_limit->extrapolation_duration_us, 0U);
}

TEST(TfContinuityController, RateLimitsFallbackAndReplacesAnchorWithMeasurement) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));

  EXPECT_FALSE(controller.Select(1050000U, std::nullopt).has_value());

  const auto measured = controller.RecordMeasured(Sample(1100000U, 2.0));
  EXPECT_EQ(measured.mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured.hold_reason, TfHoldReason::kNone);
  EXPECT_EQ(measured.sample.timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(measured.sample.map_base_pose(0, 3), 2.0);

  const auto held = controller.Select(1200000U, std::nullopt);
  ASSERT_TRUE(held.has_value());
  EXPECT_EQ(held->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(held->hold_reason, TfHoldReason::kImuUnavailable);
  EXPECT_EQ(held->sample.timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(held->sample.map_base_pose(0, 3), 2.0);
  EXPECT_EQ(held->anchor_timestamp_us, 1100000U);
}

TEST(TfContinuityController, HoldsWhenAnchorAgeExceedsExtrapolationLimit) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));

  const auto decision = controller.Select(10000000U, Sample(1400000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.timestamp_us, 10000000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 1.0);
}

TEST(TfContinuityController, HoldsWhenPredictionTimestampIsInTheFuture) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));

  const auto decision = controller.Select(1200000U, Sample(1300000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 1.0);
}

TEST(TfContinuityController, ClampsLateMeasurementToLastEmittedTimestamp) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));
  const auto held = controller.Select(1100000U, std::nullopt);
  ASSERT_TRUE(held.has_value());
  ASSERT_EQ(held->sample.timestamp_us, 1100000U);

  const auto measured = controller.RecordMeasured(Sample(1050000U, 2.0));

  EXPECT_EQ(measured.mode, TfPublicationMode::kMeasured);
  EXPECT_EQ(measured.sample.timestamp_us, 1100000U);
  EXPECT_DOUBLE_EQ(measured.sample.map_base_pose(0, 3), 2.0);
  ASSERT_TRUE(controller.anchor_timestamp_us().has_value());
  EXPECT_EQ(*controller.anchor_timestamp_us(), 1050000U);
}

TEST(TfContinuityController, HoldsWhenPredictionPredatesLastEmittedTimestamp) {
  auto controller = MakeController();
  controller.RecordMeasured(Sample(1000000U, 1.0));
  const auto held = controller.Select(1100000U, std::nullopt);
  ASSERT_TRUE(held.has_value());
  ASSERT_EQ(held->sample.timestamp_us, 1100000U);

  const auto decision = controller.Select(1200000U, Sample(1050000U, 2.0));

  ASSERT_TRUE(decision.has_value());
  EXPECT_EQ(decision->mode, TfPublicationMode::kHeld);
  EXPECT_EQ(decision->hold_reason, TfHoldReason::kExtrapolationLimit);
  EXPECT_EQ(decision->sample.timestamp_us, 1200000U);
  EXPECT_DOUBLE_EQ(decision->sample.map_base_pose(0, 3), 1.0);
}

}  // namespace
