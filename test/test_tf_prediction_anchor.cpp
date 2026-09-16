#include "localization_health/tf_prediction_anchor.h"

#include <gtest/gtest.h>

namespace {

using localization_health::MatchTfPredictionAnchor;
using localization_health::TfPredictionAnchorKey;
using localization_health::TfPredictionAnchorMatch;
using localization_health::TfPredictionRequestKey;

TEST(TfPredictionAnchorMatch, RejectsPredictionWithoutACommittedAnchor) {
  const TfPredictionAnchorKey no_anchor{0U, 0U, false};
  const TfPredictionRequestKey request{1000000U, 3U};

  EXPECT_EQ(MatchTfPredictionAnchor(no_anchor, request),
            TfPredictionAnchorMatch::kNoAnchor);
}

TEST(TfPredictionAnchorMatch, RejectsRequestFromAnOlderResultThanTheAnchor) {
  // The localization thread already committed anchor B while System still
  // processes the queued result A: projecting A from B would publish B's frame
  // as if it were A's measurement.
  const TfPredictionAnchorKey committed_anchor{2000000U, 3U, true};
  const TfPredictionRequestKey older_result{1000000U, 3U};

  EXPECT_EQ(MatchTfPredictionAnchor(committed_anchor, older_result),
            TfPredictionAnchorMatch::kTimestampMismatch);
}

TEST(TfPredictionAnchorMatch, RejectsRequestNewerThanTheCommittedAnchor) {
  const TfPredictionAnchorKey committed_anchor{1000000U, 3U, true};
  const TfPredictionRequestKey newer_result{2000000U, 3U};

  EXPECT_EQ(MatchTfPredictionAnchor(committed_anchor, newer_result),
            TfPredictionAnchorMatch::kTimestampMismatch);
}

TEST(TfPredictionAnchorMatch, RejectsAnchorFromBeforeAClockJumpEpoch) {
  // Same sensor timestamp, but the epoch was advanced by a ROS clock jump:
  // post-jump IMU data must not extrapolate a pre-jump anchor.
  const TfPredictionAnchorKey pre_jump_anchor{1000000U, 3U, true};
  const TfPredictionRequestKey post_jump_result{1000000U, 4U};

  EXPECT_EQ(MatchTfPredictionAnchor(pre_jump_anchor, post_jump_result),
            TfPredictionAnchorMatch::kEpochMismatch);
}

TEST(TfPredictionAnchorMatch, AcceptsTheExactCommittedResultPair) {
  const TfPredictionAnchorKey committed_anchor{1000000U, 3U, true};
  const TfPredictionRequestKey same_result{1000000U, 3U};

  EXPECT_EQ(MatchTfPredictionAnchor(committed_anchor, same_result),
            TfPredictionAnchorMatch::kMatched);
}

}  // namespace
