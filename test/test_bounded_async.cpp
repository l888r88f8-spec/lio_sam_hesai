#include "common/bounded_async.h"

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <future>
#include <mutex>
#include <vector>

TEST(BoundedAsync, UsesReportedHardwareConcurrencyAsAnUpperBound) {
  EXPECT_EQ(AutomaticAsyncWorkerCount(0, 8), 0U);
  EXPECT_EQ(AutomaticAsyncWorkerCount(8, 0), 1U);
  EXPECT_EQ(AutomaticAsyncWorkerCount(8, 4), 4U);
  EXPECT_EQ(AutomaticAsyncWorkerCount(3, 8), 3U);
}

TEST(BoundedAsync, RunsEveryTaskOnceWithinTheWorkerLimit) {
  constexpr std::size_t kTaskCount = 7;
  constexpr std::size_t kWorkerCount = 3;

  std::mutex mutex;
  std::condition_variable condition;
  std::size_t active = 0;
  std::size_t max_active = 0;
  std::size_t started = 0;
  bool release = false;
  std::vector<int> execution_counts(kTaskCount, 0);

  auto run = std::async(std::launch::async, [&]() {
    return RunBoundedAsync(kTaskCount, kWorkerCount, [&](std::size_t index) {
      std::unique_lock<std::mutex> lock(mutex);
      ++active;
      ++started;
      ++execution_counts[index];
      max_active = std::max(max_active, active);
      condition.notify_all();
      condition.wait(lock, [&]() { return release; });
      --active;
      return index * 10U;
    });
  });

  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(condition.wait_for(lock, std::chrono::seconds(2), [&]() {
      return started >= kWorkerCount;
    }));
    EXPECT_EQ(max_active, kWorkerCount);
    release = true;
  }
  condition.notify_all();

  const auto results = run.get();
  ASSERT_EQ(results.size(), kTaskCount);
  for (std::size_t index = 0; index < kTaskCount; ++index) {
    EXPECT_EQ(execution_counts[index], 1);
    EXPECT_EQ(results[index], index * 10U);
  }
  EXPECT_LE(max_active, kWorkerCount);
}
