#include "imu/imu_data_searcher.h"

#include <gtest/gtest.h>

#include <cstdint>

namespace {

IMUData Sample(std::uint64_t timestamp_us) {
  IMUData data;
  data.timestamp_ = timestamp_us;
  return data;
}

TEST(IMUDataSearcher, ClearsCacheBeforeAcceptingRewoundTimestamp) {
  IMUDataSearcher searcher(8U);
  ASSERT_TRUE(searcher.CacheData(Sample(1000000U)));
  EXPECT_FALSE(searcher.CacheData(Sample(100000U)));

  searcher.Clear();
  EXPECT_FALSE(searcher.LatestData().has_value());
  ASSERT_TRUE(searcher.CacheData(Sample(100000U)));

  const auto latest = searcher.LatestData();
  ASSERT_TRUE(latest.has_value());
  EXPECT_EQ(latest->timestamp_, 100000U);
}

}  // namespace
