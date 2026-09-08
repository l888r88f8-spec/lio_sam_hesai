#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include "localization_map/tile_cache.h"

namespace {

using localization_map::TileCache;
using localization_map::TileCacheConfig;
using localization_map::TileIndex;
using localization_map::TilePayload;

TilePayload MakePayload(std::size_t bytes, std::size_t global_points = 1,
                        std::size_t ground_points = 0,
                        float first_intensity = 0.0F) {
  PCLPointCloudXYZI::Ptr global(new PCLPointCloudXYZI);
  global->resize(global_points);
  if (!global->empty()) {
    global->front().intensity = first_intensity;
  }

  PCLPointCloudXYZI::Ptr ground(new PCLPointCloudXYZI);
  ground->resize(ground_points);
  return {global, ground, bytes};
}

TEST(TileCache, TracksHitsAndMissesAndReturnsInsertedPayload) {
  TileCache cache(TileCacheConfig{2, 100});
  const TileIndex index{3, -2};
  TilePayload payload = MakePayload(24, 2, 1, 7.0F);
  const auto expected_global = payload.global;
  const auto expected_ground = payload.ground;

  EXPECT_EQ(cache.Find(index), nullptr);
  ASSERT_TRUE(cache.Insert(index, std::move(payload)));
  const std::shared_ptr<const TilePayload> found = cache.Find(index);
  const std::shared_ptr<const TilePayload> found_again = cache.Find(index);

  ASSERT_NE(found, nullptr);
  EXPECT_EQ(found.get(), found_again.get());
  EXPECT_EQ(found->global.get(), expected_global.get());
  EXPECT_EQ(found->ground.get(), expected_ground.get());
  EXPECT_EQ(found->global->size(), 2U);
  EXPECT_EQ(found->ground->size(), 1U);
  EXPECT_FLOAT_EQ(found->global->front().intensity, 7.0F);
  const auto snapshot = cache.Snapshot();
  EXPECT_EQ(snapshot.cached_tiles, 1U);
  EXPECT_EQ(snapshot.bytes, 24U);
  EXPECT_EQ(snapshot.hits, 2U);
  EXPECT_EQ(snapshot.misses, 1U);
}

TEST(TileCache, ReturnedPayloadSurvivesEviction) {
  TileCache cache(TileCacheConfig{1, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10, 1, 0, 6.0F)));
  const auto retained = cache.Find(TileIndex{0, 0});
  ASSERT_NE(retained, nullptr);

  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));

  EXPECT_EQ(cache.Find(TileIndex{0, 0}), nullptr);
  EXPECT_EQ(retained->bytes, 10U);
  EXPECT_FLOAT_EQ(retained->global->front().intensity, 6.0F);
}

TEST(TileCache, ReleasesEvictedAndReplacedPayloadsOutsideCacheLock) {
  const pid_t child = fork();
  ASSERT_NE(child, -1);
  if (child == 0) {
    alarm(2);
    TileCache cache(TileCacheConfig{1, 100});
    std::size_t release_count = 0;
    const auto make_reentrant_payload = [&cache, &release_count] {
      PCLPointCloudXYZI::ConstPtr cloud(
          new PCLPointCloudXYZI,
          [&cache, &release_count](const PCLPointCloudXYZI* released) {
            cache.Snapshot();
            ++release_count;
            delete released;
          });
      return TilePayload{std::move(cloud), PCLPointCloudXYZI::ConstPtr{}, 10};
    };

    if (!cache.Insert(TileIndex{0, 0}, make_reentrant_payload()) ||
        !cache.Insert(TileIndex{1, 0}, MakePayload(10)) ||
        release_count != 1 ||
        !cache.Insert(TileIndex{1, 0}, make_reentrant_payload()) ||
        !cache.Insert(TileIndex{1, 0}, MakePayload(10)) ||
        release_count != 2) {
      _exit(1);
    }
    alarm(0);
    _exit(0);
  }

  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status))
      << "child terminated by signal " << WTERMSIG(status);
  EXPECT_EQ(WEXITSTATUS(status), 0);
}

TEST(TileCache, CountsGlobalAndGroundCloudsAsOneLogicalTile) {
  TileCache cache(TileCacheConfig{1, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(60, 4, 6)));

  const auto payload = cache.Find(TileIndex{0, 0});
  ASSERT_NE(payload, nullptr);
  EXPECT_EQ(payload->global->size(), 4U);
  EXPECT_EQ(payload->ground->size(), 6U);
  EXPECT_EQ(cache.Snapshot().cached_tiles, 1U);
}

TEST(TileCache, EvictsLeastRecentlyUsedAndFindMakesEntryMostRecent) {
  TileCache cache(TileCacheConfig{2, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  ASSERT_NE(cache.Find(TileIndex{0, 0}), nullptr);

  ASSERT_TRUE(cache.Insert(TileIndex{2, 0}, MakePayload(10)));

  EXPECT_EQ(cache.Find(TileIndex{1, 0}), nullptr);
  EXPECT_NE(cache.Find(TileIndex{0, 0}), nullptr);
  EXPECT_NE(cache.Find(TileIndex{2, 0}), nullptr);
  EXPECT_EQ(cache.Snapshot().evictions, 1U);
}

TEST(TileCache, MultiplePinsRequireMatchingUnpinsBeforeEviction) {
  TileCache cache(TileCacheConfig{1, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  cache.Pin(TileIndex{0, 0});
  cache.Pin(TileIndex{0, 0});

  EXPECT_FALSE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  cache.Unpin(TileIndex{0, 0});
  EXPECT_FALSE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  cache.Unpin(TileIndex{0, 0});
  EXPECT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  EXPECT_EQ(cache.Find(TileIndex{0, 0}), nullptr);
}

TEST(TileCache, RejectsWhenAllPossibleVictimsArePinned) {
  TileCache cache(TileCacheConfig{2, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(40)));
  cache.Pin(TileIndex{0, 0});
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(40)));
  cache.Pin(TileIndex{1, 0});

  EXPECT_FALSE(cache.Insert(TileIndex{2, 0}, MakePayload(10)));

  const auto snapshot = cache.Snapshot();
  EXPECT_EQ(snapshot.cached_tiles, 2U);
  EXPECT_EQ(snapshot.bytes, 80U);
  EXPECT_EQ(snapshot.hits, 0U);
  EXPECT_EQ(snapshot.misses, 0U);
  EXPECT_EQ(snapshot.evictions, 0U);
  EXPECT_EQ(snapshot.budget_rejections, 1U);
}

TEST(TileCache, EnforcesTileCountBudgetIndependently) {
  TileCache cache(TileCacheConfig{2, 1000});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  ASSERT_TRUE(cache.Insert(TileIndex{2, 0}, MakePayload(10)));

  const auto snapshot = cache.Snapshot();
  EXPECT_EQ(snapshot.cached_tiles, 2U);
  EXPECT_EQ(snapshot.bytes, 20U);
  EXPECT_EQ(snapshot.evictions, 1U);
}

TEST(TileCache, EnforcesByteBudgetIndependently) {
  TileCache cache(TileCacheConfig{10, 50});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(30)));
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(30)));

  const auto snapshot = cache.Snapshot();
  EXPECT_EQ(snapshot.cached_tiles, 1U);
  EXPECT_EQ(snapshot.bytes, 30U);
  EXPECT_EQ(snapshot.evictions, 1U);
}

TEST(TileCache, RejectsSingleOverBudgetPayloadWithoutMutatingCache) {
  TileCache cache(TileCacheConfig{2, 50});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(20, 1, 0, 1.0F)));

  EXPECT_FALSE(cache.Insert(TileIndex{1, 0}, MakePayload(51)));

  const auto snapshot = cache.Snapshot();
  EXPECT_EQ(snapshot.cached_tiles, 1U);
  EXPECT_EQ(snapshot.bytes, 20U);
  EXPECT_EQ(snapshot.evictions, 0U);
  EXPECT_EQ(snapshot.budget_rejections, 1U);
  const auto original = cache.Find(TileIndex{0, 0});
  ASSERT_NE(original, nullptr);
  EXPECT_FLOAT_EQ(original->global->front().intensity, 1.0F);
}

TEST(TileCache, ReplacementUpdatesPayloadAndBytesAndBecomesMostRecent) {
  TileCache cache(TileCacheConfig{2, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10, 1, 0, 1.0F)));
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(20)));

  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(30, 2, 0, 9.0F)));
  ASSERT_TRUE(cache.Insert(TileIndex{2, 0}, MakePayload(40)));

  EXPECT_EQ(cache.Find(TileIndex{1, 0}), nullptr);
  const auto replacement = cache.Find(TileIndex{0, 0});
  ASSERT_NE(replacement, nullptr);
  EXPECT_EQ(replacement->bytes, 30U);
  EXPECT_EQ(replacement->global->size(), 2U);
  EXPECT_FLOAT_EQ(replacement->global->front().intensity, 9.0F);
  EXPECT_EQ(cache.Snapshot().bytes, 70U);
}

TEST(TileCache, ReplacementRetainsPinCount) {
  TileCache cache(TileCacheConfig{1, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  cache.Pin(TileIndex{0, 0});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(20)));

  EXPECT_FALSE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  cache.Unpin(TileIndex{0, 0});
  EXPECT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
}

TEST(TileCache, FailedReplacementPreservesPayloadBytesOrderPinsAndCounters) {
  TileCache cache(TileCacheConfig{2, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(40)));
  cache.Pin(TileIndex{1, 0});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(40, 1, 0, 3.0F)));

  EXPECT_FALSE(
      cache.Insert(TileIndex{0, 0}, MakePayload(90, 1, 0, 8.0F)));

  const auto failed = cache.Snapshot();
  EXPECT_EQ(failed.cached_tiles, 2U);
  EXPECT_EQ(failed.bytes, 80U);
  EXPECT_EQ(failed.hits, 0U);
  EXPECT_EQ(failed.misses, 0U);
  EXPECT_EQ(failed.evictions, 0U);
  EXPECT_EQ(failed.budget_rejections, 1U);

  cache.Unpin(TileIndex{1, 0});
  ASSERT_TRUE(cache.Insert(TileIndex{2, 0}, MakePayload(30)));
  EXPECT_EQ(cache.Find(TileIndex{1, 0}), nullptr);
  const auto original = cache.Find(TileIndex{0, 0});
  ASSERT_NE(original, nullptr);
  EXPECT_EQ(original->bytes, 40U);
  EXPECT_FLOAT_EQ(original->global->front().intensity, 3.0F);
}

TEST(TileCache, FailedInsertionPreservesLruOrderAndPins) {
  TileCache cache(TileCacheConfig{2, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(40)));
  cache.Pin(TileIndex{0, 0});
  ASSERT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(40)));
  cache.Pin(TileIndex{1, 0});
  ASSERT_FALSE(cache.Insert(TileIndex{2, 0}, MakePayload(10)));

  cache.ClearPins();
  ASSERT_TRUE(cache.Insert(TileIndex{2, 0}, MakePayload(10)));

  EXPECT_EQ(cache.Find(TileIndex{0, 0}), nullptr);
  EXPECT_NE(cache.Find(TileIndex{1, 0}), nullptr);
}

TEST(TileCache, ClearPinsPermitsLaterEviction) {
  TileCache cache(TileCacheConfig{1, 100});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  cache.Pin(TileIndex{0, 0});
  ASSERT_FALSE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));

  cache.ClearPins();
  EXPECT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
  EXPECT_EQ(cache.Find(TileIndex{0, 0}), nullptr);
}

TEST(TileCache, MissingPinsAndExtraUnpinsAreNoOps) {
  TileCache cache(TileCacheConfig{1, 100});
  cache.Pin(TileIndex{9, 9});
  cache.Unpin(TileIndex{9, 9});
  ASSERT_TRUE(cache.Insert(TileIndex{0, 0}, MakePayload(10)));
  cache.Unpin(TileIndex{0, 0});
  cache.Unpin(TileIndex{0, 0});

  EXPECT_TRUE(cache.Insert(TileIndex{1, 0}, MakePayload(10)));
}

TEST(TileCache, ZeroInEitherBudgetMakesCacheAlwaysRejecting) {
  TileCache no_tiles(TileCacheConfig{0, 100});
  TileCache no_bytes(TileCacheConfig{1, 0});

  EXPECT_FALSE(no_tiles.Insert(TileIndex{0, 0}, MakePayload(0)));
  EXPECT_FALSE(no_bytes.Insert(TileIndex{0, 0}, MakePayload(0)));
  EXPECT_EQ(no_tiles.Snapshot().budget_rejections, 1U);
  EXPECT_EQ(no_bytes.Snapshot().budget_rejections, 1U);
}

TEST(TileCache, ConcurrentOperationsKeepSnapshotsWithinBudgets) {
  constexpr std::size_t kMaxTiles = 8;
  constexpr std::size_t kMaxBytes = 800;
  TileCache cache(TileCacheConfig{kMaxTiles, kMaxBytes});
  for (int key = 0; key < static_cast<int>(kMaxTiles); ++key) {
    ASSERT_TRUE(cache.Insert(TileIndex{key, 0}, MakePayload(50)));
  }

  std::atomic<bool> start{false};
  std::atomic<bool> consistent{true};
  const auto await_start = [&start] {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  };
  const auto check_snapshot = [&] {
    const auto snapshot = cache.Snapshot();
    if (snapshot.cached_tiles > kMaxTiles || snapshot.bytes > kMaxBytes) {
      consistent.store(false, std::memory_order_relaxed);
    }
  };

  std::thread inserter([&] {
    await_start();
    for (int iteration = 0; iteration < 1000; ++iteration) {
      cache.Insert(TileIndex{iteration % 12, 0},
                   MakePayload(25U + static_cast<std::size_t>(iteration % 4)));
      check_snapshot();
    }
  });
  std::thread finder([&] {
    await_start();
    for (int iteration = 0; iteration < 2000; ++iteration) {
      cache.Find(TileIndex{iteration % 12, 0});
      check_snapshot();
    }
  });
  std::thread observer([&] {
    await_start();
    for (int iteration = 0; iteration < 2000; ++iteration) {
      check_snapshot();
    }
  });

  start.store(true, std::memory_order_release);
  inserter.join();
  finder.join();
  observer.join();

  EXPECT_TRUE(consistent.load(std::memory_order_relaxed));
  const auto final_snapshot = cache.Snapshot();
  EXPECT_LE(final_snapshot.cached_tiles, kMaxTiles);
  EXPECT_LE(final_snapshot.bytes, kMaxBytes);
}

}  // namespace
