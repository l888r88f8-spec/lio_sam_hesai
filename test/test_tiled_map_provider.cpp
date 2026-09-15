#include <gtest/gtest.h>

#include <pcl/io/pcd_io.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "localization_map/tile_manifest.h"
#include "localization_map/tiled_map_provider.h"

namespace {

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using localization_map::MapMode;
using localization_map::MapRequestResult;
using localization_map::MapRequestState;
using localization_map::TileIndex;
using localization_map::TileLoadResult;
using localization_map::TileManifest;
using localization_map::TiledMapProvider;
using localization_map::TiledMapProviderConfig;

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    static std::atomic<std::uint64_t> next_id{0};
    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    root_ = fs::temp_directory_path() /
            ("tiled_map_provider_" + std::to_string(stamp) + "_" +
             std::to_string(next_id.fetch_add(1)));
    fs::create_directories(root_);
  }

  ~TemporaryDirectory() {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path& root() const { return root_; }

 private:
  fs::path root_;
};

PCLPointXYZI Point(float x, float y, float z, float intensity = 1.0F) {
  PCLPointXYZI point;
  point.x = x;
  point.y = y;
  point.z = z;
  point.intensity = intensity;
  return point;
}

PCLPointCloudXYZI::ConstPtr Cloud(float x, float y, float z,
                                  float intensity = 1.0F) {
  PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
  cloud->push_back(Point(x, y, z, intensity));
  return cloud;
}

void SaveCloud(const fs::path& path,
               const std::vector<PCLPointXYZI>& points) {
  PCLPointCloudXYZI cloud;
  cloud.points.assign(points.begin(), points.end());
  cloud.width = static_cast<std::uint32_t>(points.size());
  cloud.height = 1U;
  cloud.is_dense = true;
  ASSERT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
}

TileManifest EmptyManifest() {
  TileManifest manifest;
  manifest.tile_size_m = 100.0;
  manifest.origin = {0.0, 0.0};
  manifest.source.global_map_name = "GlobalMap.pcd";
  manifest.source.ground_map_name = "GroundMap.pcd";
  return manifest;
}

void SaveEmptyManifest(const fs::path& root,
                       bool with_overview = false) {
  TileManifest manifest = EmptyManifest();
  if (with_overview) {
    SaveCloud(root / "overview.pcd", {Point(1, 2, 3, 4)});
    manifest.overview =
        localization_map::OverviewMetadata{"overview.pcd", 5.0};
  }
  manifest.Save(root);
}

TiledMapProviderConfig Config(const fs::path& root, int active_radius = 0,
                              int prefetch_radius = 0) {
  TiledMapProviderConfig config;
  config.root = root;
  config.active_radius = active_radius;
  config.prefetch_radius = prefetch_radius;
  config.max_cached_tiles = 64U;
  config.max_cache_bytes = 64U * 1024U * 1024U;
  config.registration_voxel_size = 0.1F;
  return config;
}

Mat4d Pose(double x, double y = 0.0, double z = 0.0) {
  Mat4d pose = Mat4d::Identity();
  pose.block<3, 1>(0, 3) = Vec3d{x, y, z};
  return pose;
}

TileLoadResult SuccessfulTile(TileIndex index) {
  TileLoadResult result;
  result.index = index;
  result.global_present = true;
  result.global_ok = true;
  result.ground_present = true;
  result.ground_ok = true;
  result.payload.global =
      Cloud(static_cast<float>(index.x * 100),
            static_cast<float>(index.y * 100), 5.0F);
  result.payload.ground =
      Cloud(static_cast<float>(index.x * 100),
            static_cast<float>(index.y * 100), 1.0F);
  result.payload.bytes = 2U * sizeof(PCLPointXYZI);
  return result;
}

MapRequestResult WaitForTracking(TiledMapProvider& provider,
                                 const Mat4d& pose,
                                 std::optional<Vec2d> velocity = std::nullopt,
                                 bool force = false) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    auto result = provider.RequestTracking(pose, velocity, force);
    if (result.state != MapRequestState::kPending) {
      return result;
    }
    std::this_thread::sleep_for(1ms);
  }
  return {};
}

void WaitForCacheMisses(TiledMapProvider& provider, std::uint64_t count) {
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline &&
         provider.Diagnostics().cache_misses < count) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_GE(provider.Diagnostics().cache_misses, count);
}

class BlockingLoader {
 public:
  TileLoadResult operator()(TileIndex index) {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(index);
    threads_.push_back(std::this_thread::get_id());
    condition_.notify_all();
    condition_.wait(lock, [&] { return released_; });
    return SuccessfulTile(index);
  }

  void WaitForCalls(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(condition_.wait_for(lock, 2s,
                                    [&] { return calls_.size() >= count; }));
  }

  void Release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

  std::size_t CallCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_.size();
  }

  std::thread::id FirstThread() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return threads_.front();
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool released_{false};
  std::vector<TileIndex> calls_;
  std::vector<std::thread::id> threads_;
};

class PriorityLoader {
 public:
  TileLoadResult operator()(TileIndex index) {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(index);
    condition_.notify_all();
    if (calls_.size() == 2U) {
      condition_.wait(lock, [&] { return release_second_; });
    }
    return SuccessfulTile(index);
  }

  void WaitForCalls(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(condition_.wait_for(lock, 2s,
                                    [&] { return calls_.size() >= count; }));
  }

  void ReleaseSecond() {
    std::lock_guard<std::mutex> lock(mutex_);
    release_second_ = true;
    condition_.notify_all();
  }

  std::vector<TileIndex> Calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  bool release_second_{false};
  std::vector<TileIndex> calls_;
};

class RecordingLoader {
 public:
  TileLoadResult operator()(TileIndex index) {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.push_back(index);
    condition_.notify_all();
    return SuccessfulTile(index);
  }

  void WaitForCalls(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(condition_.wait_for(lock, 2s,
                                    [&] { return calls_.size() >= count; }));
  }

  std::vector<TileIndex> Calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::vector<TileIndex> calls_;
};

class StepLoader {
 public:
  TileLoadResult operator()(TileIndex index) {
    std::unique_lock<std::mutex> lock(mutex_);
    calls_.push_back(index);
    const std::size_t ordinal = calls_.size();
    condition_.notify_all();
    condition_.wait(lock, [&] { return released_ >= ordinal; });
    completed_ = ordinal;
    condition_.notify_all();
    return SuccessfulTile(index);
  }

  void WaitForCalls(std::size_t count) {
    std::unique_lock<std::mutex> lock(mutex_);
    ASSERT_TRUE(condition_.wait_for(lock, 2s,
                                    [&] { return calls_.size() >= count; }));
  }

  void ReleaseNextAndWait() {
    std::unique_lock<std::mutex> lock(mutex_);
    ++released_;
    condition_.notify_all();
    ASSERT_TRUE(condition_.wait_for(lock, 2s,
                                    [&] { return completed_ >= released_; }));
  }

  void ReleaseAll() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = std::numeric_limits<std::size_t>::max();
    condition_.notify_all();
  }

  std::vector<TileIndex> Calls() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t released_{0U};
  std::size_t completed_{0U};
  std::vector<TileIndex> calls_;
};

TEST(TiledMapProvider, PrepareLoadsOnlyManifestAndOptionalOverview) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root(), true);
  std::atomic<int> calls{0};
  auto loader = [&](TileIndex index) {
    ++calls;
    return SuccessfulTile(index);
  };

  auto config = Config(directory.root());
  config.load_overview = false;
  TiledMapProvider without_overview(config, loader);
  ASSERT_TRUE(without_overview.Prepare(nullptr));
  EXPECT_EQ(calls.load(), 0);
  EXPECT_EQ(without_overview.Overview(), nullptr);

  config.load_overview = true;
  TiledMapProvider with_overview(config, loader);
  ASSERT_TRUE(with_overview.Prepare(nullptr));
  EXPECT_EQ(calls.load(), 0);
  ASSERT_NE(with_overview.Overview(), nullptr);
  EXPECT_EQ(with_overview.Overview()->size(), 1U);
  const auto diagnostics = with_overview.Diagnostics();
  EXPECT_EQ(diagnostics.mode, MapMode::kTiled);
  EXPECT_EQ(diagnostics.manifest_version, TileManifest::kSchemaVersion);
  EXPECT_EQ(diagnostics.cached_tiles, 0U);
  EXPECT_EQ(diagnostics.load_state, "ready");
}

TEST(TiledMapProvider, MissingOrCorruptOptionalOverviewDoesNotFailPrepare) {
  TemporaryDirectory missing_directory;
  SaveEmptyManifest(missing_directory.root());
  auto missing_config = Config(missing_directory.root());
  missing_config.load_overview = true;
  TiledMapProvider missing(missing_config, SuccessfulTile);
  ASSERT_TRUE(missing.Prepare(nullptr));
  EXPECT_EQ(missing.Overview(), nullptr);
  EXPECT_FALSE(missing.Diagnostics().last_error.empty());

  TemporaryDirectory corrupt_directory;
  {
    std::ofstream output(corrupt_directory.root() / "overview.pcd");
    output << "not a point cloud\n";
  }
  TileManifest manifest = EmptyManifest();
  manifest.overview =
      localization_map::OverviewMetadata{"overview.pcd", 5.0};
  manifest.Save(corrupt_directory.root());
  auto corrupt_config = Config(corrupt_directory.root());
  corrupt_config.load_overview = true;
  TiledMapProvider corrupt(corrupt_config, SuccessfulTile);
  ASSERT_TRUE(corrupt.Prepare(nullptr));
  EXPECT_EQ(corrupt.Overview(), nullptr);
  EXPECT_FALSE(corrupt.Diagnostics().last_error.empty());
}

TEST(TiledMapProvider, TrackingIsNonblockingAndLoadsOnWorkerThread) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(Config(directory.root()),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  const std::thread::id requester = std::this_thread::get_id();

  const auto pending =
      provider.RequestTracking(Pose(0), std::nullopt, false);
  EXPECT_EQ(pending.state, MapRequestState::kPending);
  loader.WaitForCalls(1U);
  EXPECT_NE(loader.FirstThread(), requester);
  loader.Release();
  const auto ready = WaitForTracking(provider, Pose(0));
  ASSERT_EQ(ready.state, MapRequestState::kReady);
  ASSERT_NE(ready.snapshot, nullptr);
  EXPECT_TRUE(ready.previous_snapshot_covers_center);
  EXPECT_EQ(ready.snapshot->indices, (std::vector<TileIndex>{{0, 0}}));

  const auto unchanged =
      provider.RequestTracking(Pose(1), std::nullopt, false);
  EXPECT_EQ(unchanged.state, MapRequestState::kUnchanged);
  EXPECT_TRUE(unchanged.previous_snapshot_covers_center);
  EXPECT_EQ(unchanged.snapshot.get(), ready.snapshot.get());
}

TEST(TiledMapProvider, CandidateAndTrackingCoalesceTheSameInflightTile) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(Config(directory.root()),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.WaitForCalls(1U);

  auto candidate = std::async(std::launch::async, [&] {
    return provider.LoadCandidate(Pose(0), 1s);
  });
  EXPECT_EQ(candidate.wait_for(20ms), std::future_status::timeout);
  loader.Release();
  EXPECT_EQ(candidate.get().state, MapRequestState::kReady);
  EXPECT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
  EXPECT_EQ(loader.CallCount(), 1U);
}

TEST(TiledMapProvider, CandidateReturnsAtItsTimeoutAndShutdownJoins) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(Config(directory.root()),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto start = std::chrono::steady_clock::now();
  const auto result = provider.LoadCandidate(Pose(0), 20ms);
  const auto elapsed = std::chrono::steady_clock::now() - start;
  EXPECT_EQ(result.state, MapRequestState::kUnavailable);
  EXPECT_NE(result.error.find("timed out"), std::string::npos);
  EXPECT_GE(elapsed, 15ms);
  EXPECT_LT(elapsed, 500ms);

  auto shutdown = std::async(std::launch::async, [&] { provider.Shutdown(); });
  EXPECT_EQ(shutdown.wait_for(20ms), std::future_status::timeout);
  loader.Release();
  EXPECT_EQ(shutdown.wait_for(2s), std::future_status::ready);
  provider.Shutdown();
}

TEST(TiledMapProvider, CandidateTimeoutCancelsItsQueuedTileLoads) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(
      Config(directory.root(), 1, 1),
      [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));

  EXPECT_EQ(provider.LoadCandidate(Pose(0), 20ms).state,
            MapRequestState::kUnavailable);
  loader.WaitForCalls(1U);
  loader.Release();
  provider.Shutdown();
  EXPECT_EQ(loader.CallCount(), 1U);
}

TEST(TiledMapProvider, CenterFailureIsUnavailableAndNeighborFailureDegrades) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto center_failure = [](TileIndex index) {
    auto result = SuccessfulTile(index);
    if (index == TileIndex{0, 0}) {
      result.global_ok = false;
      result.payload = {};
      result.error = "center failed";
    }
    return result;
  };
  TiledMapProvider failed_center(Config(directory.root(), 1, 1),
                                 center_failure);
  ASSERT_TRUE(failed_center.Prepare(nullptr));
  const auto unavailable = failed_center.LoadCandidate(Pose(0), 1s);
  EXPECT_EQ(unavailable.state, MapRequestState::kUnavailable);
  EXPECT_FALSE(unavailable.error.empty());

  auto neighbor_failure = [](TileIndex index) {
    auto result = SuccessfulTile(index);
    if (index == TileIndex{1, 0}) {
      result.global_ok = false;
      result.payload = {};
      result.error = "neighbor failed";
    }
    return result;
  };
  TiledMapProvider degraded(Config(directory.root(), 1, 1),
                            neighbor_failure);
  ASSERT_TRUE(degraded.Prepare(nullptr));
  const auto ready = degraded.LoadCandidate(Pose(0), 1s);
  ASSERT_EQ(ready.state, MapRequestState::kReady);
  ASSERT_NE(ready.snapshot, nullptr);
  EXPECT_TRUE(ready.snapshot->degraded);
  EXPECT_EQ(ready.snapshot->missing_indices,
            (std::vector<TileIndex>{{1, 0}}));
  EXPECT_EQ(ready.snapshot->indices.size(), 8U);
}

TEST(TiledMapProvider, GroundFailureFallsBackToMatchingGlobalTile) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto loader = [](TileIndex index) {
    auto result = SuccessfulTile(index);
    result.ground_ok = false;
    result.payload.ground.reset();
    result.payload.bytes = sizeof(PCLPointXYZI);
    result.error = "ground failed";
    return result;
  };
  TiledMapProvider provider(Config(directory.root()), loader);
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto result = provider.LoadCandidate(Pose(0), 1s);
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_TRUE(result.snapshot->degraded);
  ASSERT_EQ(result.snapshot->registration_cloud->size(), 1U);
  ASSERT_EQ(result.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(result.snapshot->ground_cloud->front().z, 5.0F);

  const auto cached = provider.LoadCandidate(Pose(0), 1s);
  ASSERT_EQ(cached.state, MapRequestState::kReady);
  EXPECT_TRUE(cached.snapshot->degraded);
  ASSERT_EQ(cached.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(cached.snapshot->ground_cloud->front().z, 5.0F);
}

TEST(TiledMapProvider, RunsCandidatesFifoBeforeLatestActive) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  PriorityLoader loader;
  TiledMapProvider provider(Config(directory.root(), 0, 1),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));

  ASSERT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
  loader.WaitForCalls(2U);
  auto first_candidate = std::async(std::launch::async, [&] {
    return provider.LoadCandidate(Pose(200), 1s);
  });
  WaitForCacheMisses(provider, 10U);
  auto second_candidate = std::async(std::launch::async, [&] {
    return provider.LoadCandidate(Pose(300), 1s);
  });
  WaitForCacheMisses(provider, 11U);
  EXPECT_EQ(provider.RequestTracking(Pose(100), std::nullopt, false).state,
            MapRequestState::kPending);
  EXPECT_EQ(provider.RequestTracking(Pose(400), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.ReleaseSecond();
  loader.WaitForCalls(5U);
  EXPECT_EQ(first_candidate.get().state, MapRequestState::kReady);
  EXPECT_EQ(second_candidate.get().state, MapRequestState::kReady);

  const auto calls = loader.Calls();
  ASSERT_EQ(calls.size(), 5U);
  EXPECT_EQ(calls[0], (TileIndex{0, 0}));
  EXPECT_EQ(calls[1], (TileIndex{-1, -1}));
  EXPECT_EQ(calls[2], (TileIndex{2, 0}));
  EXPECT_EQ(calls[3], (TileIndex{3, 0}));
  EXPECT_EQ(calls[4], (TileIndex{4, 0}));
}

TEST(TiledMapProvider, PromotionLeavesDiagnosticsReadyAfterValidWorkCompletes) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(
      Config(directory.root(), 1, 1),
      [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.WaitForCalls(1U);

  auto candidate = std::async(std::launch::async, [&] {
    return provider.LoadCandidate(Pose(0), 1s);
  });
  WaitForCacheMisses(provider, 18U);
  loader.Release();
  ASSERT_EQ(candidate.get().state, MapRequestState::kReady);
  ASSERT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.Diagnostics().load_state, "ready");
}

TEST(TiledMapProvider, CandidateLoadRunsBeforeReadyActiveAssembly) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  StepLoader loader;
  TiledMapProvider provider(Config(directory.root()),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  ASSERT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.WaitForCalls(1U);
  auto candidate = std::async(std::launch::async, [&] {
    return provider.LoadCandidate(Pose(100), 1s);
  });
  WaitForCacheMisses(provider, 2U);

  loader.ReleaseNextAndWait();
  loader.WaitForCalls(2U);
  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.ReleaseNextAndWait();
  EXPECT_EQ(candidate.get().state, MapRequestState::kReady);
  EXPECT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
}

TEST(TiledMapProvider, PrefetchSortsForwardThenUsesTileIndexForTies) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  RecordingLoader loader;
  TiledMapProvider provider(Config(directory.root(), 0, 1),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  ASSERT_EQ(WaitForTracking(provider, Pose(0), Vec2d{1.0, 0.0}).state,
            MapRequestState::kReady);
  loader.WaitForCalls(9U);

  const auto calls = loader.Calls();
  ASSERT_EQ(calls.size(), 9U);
  EXPECT_EQ(calls[0], (TileIndex{0, 0}));
  EXPECT_EQ(calls[1], (TileIndex{1, -1}));
  EXPECT_EQ(calls[2], (TileIndex{1, 0}));
  EXPECT_EQ(calls[3], (TileIndex{1, 1}));
  EXPECT_EQ(calls[4], (TileIndex{0, -1}));
  EXPECT_EQ(calls[5], (TileIndex{0, 1}));
  EXPECT_EQ(calls[6], (TileIndex{-1, -1}));
  EXPECT_EQ(calls[7], (TileIndex{-1, 0}));
  EXPECT_EQ(calls[8], (TileIndex{-1, 1}));
}

TEST(TiledMapProvider, NewActiveRequestCancelsQueuedOldPrefetch) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  StepLoader loader;
  TiledMapProvider provider(Config(directory.root(), 0, 1),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));

  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.WaitForCalls(1U);
  loader.ReleaseNextAndWait();
  ASSERT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
  loader.WaitForCalls(2U);

  EXPECT_EQ(provider.RequestTracking(Pose(100), std::nullopt, false).state,
            MapRequestState::kPending);
  EXPECT_EQ(provider.RequestTracking(Pose(200), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.ReleaseNextAndWait();
  loader.WaitForCalls(3U);
  EXPECT_EQ(loader.Calls()[2], (TileIndex{2, 0}));
  loader.ReleaseNextAndWait();
  std::this_thread::sleep_for(20ms);
  const std::size_t calls_after_latest_active = loader.Calls().size();
  loader.ReleaseAll();
  provider.Shutdown();
  EXPECT_EQ(calls_after_latest_active, 3U);
}

TEST(TiledMapProvider, BudgetRejectionMakesCandidateUnavailable) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto config = Config(directory.root());
  config.max_cache_bytes = sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, SuccessfulTile);
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto result = provider.LoadCandidate(Pose(0), 1s);
  EXPECT_EQ(result.state, MapRequestState::kUnavailable);
  EXPECT_NE(result.error.find("budget"), std::string::npos);
}

TEST(TiledMapProvider, ExtremeFinitePoseReturnsUnavailableWithoutThrowing) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  TiledMapProvider provider(Config(directory.root(), 1, 1), SuccessfulTile);
  ASSERT_TRUE(provider.Prepare(nullptr));
  Mat4d pose = Pose(static_cast<double>(std::numeric_limits<int>::max()) *
                    100.0);
  ASSERT_TRUE(pose.allFinite());
  EXPECT_NO_THROW({
    EXPECT_EQ(provider.LoadCandidate(pose, 1ms).state,
              MapRequestState::kUnavailable);
  });
  EXPECT_NO_THROW({
    EXPECT_EQ(provider.RequestTracking(pose, std::nullopt, false).state,
              MapRequestState::kUnavailable);
  });
}

TEST(TiledMapProvider, CachePinsCurrentSnapshotAndStaysWithinBudgets) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  std::mutex mutex;
  std::map<TileIndex, int> calls;
  auto loader = [&](TileIndex index) {
    std::lock_guard<std::mutex> lock(mutex);
    ++calls[index];
    return SuccessfulTile(index);
  };
  auto config = Config(directory.root());
  config.max_cached_tiles = 2U;
  config.max_cache_bytes = 4U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, loader);
  ASSERT_TRUE(provider.Prepare(nullptr));

  EXPECT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.LoadCandidate(Pose(100), 1s).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.LoadCandidate(Pose(200), 1s).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.LoadCandidate(Pose(0), 1s).state,
            MapRequestState::kReady);
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(calls[(TileIndex{0, 0})], 1);
    EXPECT_EQ(calls[(TileIndex{1, 0})], 1);
    EXPECT_EQ(calls[(TileIndex{2, 0})], 1);
  }
  auto diagnostics = provider.Diagnostics();
  EXPECT_LE(diagnostics.cached_tiles, 2U);
  EXPECT_LE(diagnostics.cache_mib,
            static_cast<double>(config.max_cache_bytes) / (1024.0 * 1024.0));

  EXPECT_EQ(WaitForTracking(provider, Pose(200)).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.LoadCandidate(Pose(300), 1s).state,
            MapRequestState::kReady);
  EXPECT_EQ(provider.LoadCandidate(Pose(200), 1s).state,
            MapRequestState::kReady);
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(calls[(TileIndex{2, 0})], 1);
  }
}

TEST(TiledMapProvider, MinimumCapacityCacheCanMoveToNewActiveCenter) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto config = Config(directory.root());
  config.max_cached_tiles = 1U;
  config.max_cache_bytes = 2U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, SuccessfulTile);
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto first = WaitForTracking(provider, Pose(0));
  ASSERT_EQ(first.state, MapRequestState::kReady);

  const auto second = WaitForTracking(provider, Pose(100));
  ASSERT_EQ(second.state, MapRequestState::kReady);
  EXPECT_NE(second.snapshot.get(), first.snapshot.get());
  EXPECT_EQ(second.snapshot->indices, (std::vector<TileIndex>{{1, 0}}));
  EXPECT_EQ(first.snapshot->indices, (std::vector<TileIndex>{{0, 0}}));
  EXPECT_EQ(provider.Diagnostics().cached_tiles, 1U);
}

TEST(TiledMapProvider, FailedActiveReplacementRepinsPreviousSnapshot) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  std::mutex mutex;
  std::map<TileIndex, int> calls;
  auto loader = [&](TileIndex index) {
    std::lock_guard<std::mutex> lock(mutex);
    ++calls[index];
    auto result = SuccessfulTile(index);
    if (index == TileIndex{1, 0}) {
      result.global_ok = false;
      result.payload = {};
    }
    return result;
  };
  auto config = Config(directory.root());
  config.max_cached_tiles = 1U;
  config.max_cache_bytes = 2U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, loader);
  ASSERT_TRUE(provider.Prepare(nullptr));
  ASSERT_EQ(WaitForTracking(provider, Pose(0)).state,
            MapRequestState::kReady);

  EXPECT_EQ(WaitForTracking(provider, Pose(100)).state,
            MapRequestState::kUnavailable);
  EXPECT_EQ(provider.LoadCandidate(Pose(200), 1s).state,
            MapRequestState::kUnavailable);
  EXPECT_EQ(provider.LoadCandidate(Pose(0), 1s).state,
            MapRequestState::kReady);
  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(calls[(TileIndex{0, 0})], 1);
}

TEST(TiledMapProvider, OversizedActiveNeighborhoodFailsAndRetainsPreviousMap) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  std::mutex mutex;
  std::map<TileIndex, int> calls;
  auto loader = [&](TileIndex index) {
    std::lock_guard<std::mutex> lock(mutex);
    ++calls[index];
    auto result = SuccessfulTile(index);
    if (index.x == 2) {
      result.payload.bytes = 8U * sizeof(PCLPointXYZI);
    }
    return result;
  };
  auto config = Config(directory.root(), 1, 1);
  config.max_cached_tiles = 9U;
  config.max_cache_bytes = 18U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, loader);
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto previous = WaitForTracking(provider, Pose(0));
  ASSERT_EQ(previous.state, MapRequestState::kReady);

  const auto oversized = WaitForTracking(provider, Pose(100));
  EXPECT_EQ(oversized.state, MapRequestState::kUnavailable);
  EXPECT_NE(oversized.error.find("budget"), std::string::npos);
  const auto unchanged = provider.RequestTracking(Pose(0), std::nullopt, false);
  ASSERT_EQ(unchanged.state, MapRequestState::kUnchanged);
  EXPECT_EQ(unchanged.snapshot.get(), previous.snapshot.get());
  EXPECT_EQ(provider.LoadCandidate(Pose(0), 1s).state,
            MapRequestState::kReady);
  std::lock_guard<std::mutex> lock(mutex);
  for (const TileIndex& index :
       localization_map::SquareNeighborhood(TileIndex{0, 0}, 1)) {
    EXPECT_EQ(calls[index], 1);
  }
}

TEST(TiledMapProvider, LateDisjointBudgetFailureRestoresPreviousTiles) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  std::mutex mutex;
  std::map<TileIndex, int> calls;
  auto loader = [&](TileIndex index) {
    std::lock_guard<std::mutex> lock(mutex);
    ++calls[index];
    auto result = SuccessfulTile(index);
    if (index == TileIndex{11, 1}) {
      result.payload.bytes = 8U * sizeof(PCLPointXYZI);
    }
    return result;
  };
  auto config = Config(directory.root(), 1, 1);
  config.max_cached_tiles = 9U;
  config.max_cache_bytes = 18U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, loader);
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto previous = WaitForTracking(provider, Pose(0));
  ASSERT_EQ(previous.state, MapRequestState::kReady);

  const auto failed = WaitForTracking(provider, Pose(1000));
  EXPECT_EQ(failed.state, MapRequestState::kUnavailable);
  EXPECT_NE(failed.error.find("budget"), std::string::npos);
  const auto unchanged = provider.RequestTracking(Pose(0), std::nullopt, false);
  ASSERT_EQ(unchanged.state, MapRequestState::kUnchanged);
  EXPECT_EQ(unchanged.snapshot.get(), previous.snapshot.get());
  EXPECT_EQ(provider.LoadCandidate(Pose(0), 1s).state,
            MapRequestState::kReady);
  std::lock_guard<std::mutex> lock(mutex);
  for (const TileIndex& index :
       localization_map::SquareNeighborhood(TileIndex{0, 0}, 1)) {
    EXPECT_EQ(calls[index], 1);
  }
}

TEST(TiledMapProvider, TrackingPollDoesNotAssembleLargeSnapshot) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  PCLPointCloudXYZI::Ptr large_cloud(new PCLPointCloudXYZI);
  large_cloud->resize(200000U);
  large_cloud->width = static_cast<std::uint32_t>(large_cloud->size());
  large_cloud->height = 1U;
  std::atomic<int> calls{0};
  auto loader = [&](TileIndex index) {
    ++calls;
    TileLoadResult result;
    result.index = index;
    result.global_present = true;
    result.global_ok = true;
    result.ground_present = true;
    result.ground_ok = true;
    result.payload.global = large_cloud;
    result.payload.ground = large_cloud;
    result.payload.bytes =
        2U * large_cloud->size() * sizeof(PCLPointXYZI);
    return result;
  };
  auto config = Config(directory.root(), 1, 1);
  config.max_cached_tiles = 9U;
  config.max_cache_bytes = 256U * 1024U * 1024U;
  TiledMapProvider provider(config, loader);
  ASSERT_TRUE(provider.Prepare(nullptr));
  ASSERT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (calls.load() < 9 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  ASSERT_EQ(calls.load(), 9);
  std::this_thread::sleep_for(5ms);

  const auto started = std::chrono::steady_clock::now();
  const auto poll = provider.RequestTracking(Pose(0), std::nullopt, false);
  const auto elapsed = std::chrono::steady_clock::now() - started;
  EXPECT_LT(elapsed, 20ms);
  EXPECT_TRUE(poll.state == MapRequestState::kPending ||
              poll.state == MapRequestState::kReady);
  if (poll.state == MapRequestState::kPending) {
    EXPECT_EQ(WaitForTracking(provider, Pose(0)).state,
              MapRequestState::kReady);
  }
}

TEST(TiledMapProvider, RequestRetainsTerminalPayloadsAcrossCacheEviction) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto config = Config(directory.root(), 1, 1);
  config.max_cached_tiles = 9U;
  config.max_cache_bytes = 2U * sizeof(PCLPointXYZI);
  TiledMapProvider provider(config, SuccessfulTile);
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto result = provider.LoadCandidate(Pose(0), 1s);
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_EQ(result.snapshot->indices.size(), 9U);
  EXPECT_EQ(result.snapshot->registration_cloud->size(), 9U);
  EXPECT_EQ(provider.Diagnostics().cached_tiles, 1U);
}

TEST(TiledMapProvider, SupersededTrackingRequestIsNotPublishedLater) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  BlockingLoader loader;
  TiledMapProvider provider(Config(directory.root()),
                            [&](TileIndex index) { return loader(index); });
  ASSERT_TRUE(provider.Prepare(nullptr));
  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.WaitForCalls(1U);
  EXPECT_EQ(provider.RequestTracking(Pose(100), std::nullopt, false).state,
            MapRequestState::kPending);
  loader.Release();
  ASSERT_EQ(WaitForTracking(provider, Pose(100)).state,
            MapRequestState::kReady);

  EXPECT_EQ(provider.RequestTracking(Pose(0), std::nullopt, false).state,
            MapRequestState::kPending);
}

TEST(TiledMapProvider, ProductionLoaderValidatesCountsAndLoadsClouds) {
  TemporaryDirectory directory;
  fs::create_directories(directory.root() / "global");
  fs::create_directories(directory.root() / "ground");
  const auto global_path = directory.root() / "global/0_0.pcd";
  const auto ground_path = directory.root() / "ground/0_0.pcd";
  SaveCloud(global_path, {Point(1, 2, 3, 4), Point(10, 20, 30, 40)});
  SaveCloud(ground_path, {Point(1, 2, -1, 5)});
  TileManifest manifest = EmptyManifest();
  manifest.global_tiles[{0, 0}] =
      {"global/0_0.pcd", 2U, fs::file_size(global_path)};
  manifest.ground_tiles[{0, 0}] =
      {"ground/0_0.pcd", 1U, fs::file_size(ground_path)};
  manifest.Save(directory.root());

  TiledMapProvider provider(Config(directory.root(), 1, 1));
  ASSERT_TRUE(provider.Prepare(nullptr));
  const auto loaded = provider.LoadCandidate(Pose(0), 1s);
  ASSERT_EQ(loaded.state, MapRequestState::kReady);
  EXPECT_EQ(loaded.snapshot->registration_cloud->size(), 2U);
  EXPECT_EQ(loaded.snapshot->indices.size(), 1U);
  EXPECT_FALSE(loaded.snapshot->degraded);
  ASSERT_EQ(loaded.snapshot->ground_cloud->size(), 1U);
  EXPECT_FLOAT_EQ(loaded.snapshot->ground_cloud->front().z, -1.0F);

  provider.Shutdown();
  manifest.global_tiles[{0, 0}].point_count = 3U;
  manifest.Save(directory.root());
  TiledMapProvider bad_count(Config(directory.root()));
  ASSERT_TRUE(bad_count.Prepare(nullptr));
  EXPECT_EQ(bad_count.LoadCandidate(Pose(0), 1s).state,
            MapRequestState::kUnavailable);
}

TEST(TiledMapProvider, ProductionLoaderRejectsTileReplacedBySymlink) {
  TemporaryDirectory directory;
  fs::create_directories(directory.root() / "global");
  const auto global_path = directory.root() / "global/0_0.pcd";
  const auto replacement_path = directory.root() / "replacement.pcd";
  SaveCloud(global_path, {Point(1, 2, 3, 4)});
  TileManifest manifest = EmptyManifest();
  manifest.global_tiles[{0, 0}] =
      {"global/0_0.pcd", 1U, fs::file_size(global_path)};
  manifest.Save(directory.root());

  TiledMapProvider provider(Config(directory.root()));
  ASSERT_TRUE(provider.Prepare(nullptr));
  fs::rename(global_path, replacement_path);
  fs::create_symlink("../replacement.pcd", global_path);

  const auto result = provider.LoadCandidate(Pose(0), 1s);
  EXPECT_EQ(result.state, MapRequestState::kUnavailable);
  EXPECT_EQ(result.error, "center global tile [0,0] is unavailable");
  const auto diagnostics = provider.Diagnostics();
  EXPECT_NE(diagnostics.last_error.find("[0,0]"), std::string::npos);
  EXPECT_NE(diagnostics.last_error.find("global/0_0.pcd"), std::string::npos);
}

TEST(TiledMapProvider, InvalidConfigOrManifestFailsWithStableDiagnostics) {
  TemporaryDirectory directory;
  SaveEmptyManifest(directory.root());
  auto config = Config(directory.root(), 1, 0);
  TiledMapProvider invalid(config, SuccessfulTile);
  std::string error;
  EXPECT_FALSE(invalid.Prepare(&error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(invalid.Diagnostics().load_state, "error");
  EXPECT_EQ(invalid.Diagnostics().last_error, error);
  EXPECT_EQ(invalid.LoadCandidate(Pose(0), 1ms).state,
            MapRequestState::kUnavailable);

  TemporaryDirectory missing;
  TiledMapProvider missing_manifest(Config(missing.root()), SuccessfulTile);
  std::string missing_error;
  EXPECT_FALSE(missing_manifest.Prepare(&missing_error));
  EXPECT_FALSE(missing_error.empty());
  EXPECT_EQ(missing_manifest.Diagnostics().last_error, missing_error);
}

}  // namespace

// --- Real on-disk manifest contract -----------------------------------------
//
// The tests below use a real manifest and real PCD files read by the default
// loader. The injected-loader tests above stay as they are: they cover worker
// scheduling, not what "declared" means on disk.

namespace {

std::uintmax_t SaveTileFile(const fs::path& path) {
  PCLPointCloudXYZI cloud;
  cloud.push_back(Point(1.0F, 2.0F, 3.0F));
  cloud.width = 1U;
  cloud.height = 1U;
  cloud.is_dense = true;
  EXPECT_EQ(pcl::io::savePCDFileBinary(path.string(), cloud), 0);
  std::error_code error;
  return fs::file_size(path, error);
}

// Declares exactly `declared` global tiles and writes a real PCD file for each,
// recording the byte size the manifest validates against.
void SaveRealManifest(const fs::path& root,
                      const std::vector<TileIndex>& declared) {
  TileManifest manifest = EmptyManifest();
  fs::create_directories(root / "global");
  for (const TileIndex& index : declared) {
    const fs::path relative =
        fs::path("global") /
        (std::to_string(index.x) + "_" + std::to_string(index.y) + ".pcd");
    const std::uintmax_t bytes = SaveTileFile(root / relative);
    manifest.global_tiles.emplace(
        index, localization_map::TileFileMetadata{relative, 1U, bytes});
  }
  manifest.Save(root);
  ASSERT_TRUE(fs::is_regular_file(root / "manifest.yaml"));
}

fs::path TileFile(const fs::path& root, TileIndex index) {
  return root / "global" /
         (std::to_string(index.x) + "_" + std::to_string(index.y) + ".pcd");
}

}  // namespace

TEST(TiledMapProvider, UndeclaredHoleInsideBoundsIsNotRequestedOrDegraded) {
  // (0,0) and (1,1) are declared; (1,0) and (0,1) sit inside the same
  // bounding rectangle but are not declared. They must be skipped silently.
  TemporaryDirectory directory;
  SaveRealManifest(directory.root(), {TileIndex{0, 0}, TileIndex{1, 1}});

  TiledMapProvider provider(Config(directory.root(), 1, 1));
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto result = WaitForTracking(provider, Pose(0.0));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_FALSE(result.snapshot->degraded);
  EXPECT_TRUE(result.snapshot->missing_indices.empty());
  EXPECT_EQ(result.snapshot->indices.size(), 2U);
  EXPECT_EQ(provider.Diagnostics().load_failures, 0U);
}

TEST(TiledMapProvider, UndeclaredCenterIsUnavailable) {
  TemporaryDirectory directory;
  SaveRealManifest(directory.root(), {TileIndex{1, 1}});

  TiledMapProvider provider(Config(directory.root(), 1, 1));
  ASSERT_TRUE(provider.Prepare(nullptr));

  const auto result = provider.LoadCandidate(Pose(0.0), 1s);
  EXPECT_EQ(result.state, MapRequestState::kUnavailable);
  EXPECT_NE(result.error.find("[0,0]"), std::string::npos) << result.error;
}

TEST(TiledMapProvider, DeclaredCenterLostAtRuntimeFailsTheRequestByName) {
  TemporaryDirectory directory;
  SaveRealManifest(directory.root(), {TileIndex{0, 0}, TileIndex{1, 1}});

  TiledMapProvider provider(Config(directory.root(), 1, 1));
  ASSERT_TRUE(provider.Prepare(nullptr));

  // The manifest still declares (0,0); only its payload disappears.
  fs::remove(TileFile(directory.root(), TileIndex{0, 0}));

  const auto result = provider.LoadCandidate(Pose(0.0), 1s);
  EXPECT_EQ(result.state, MapRequestState::kUnavailable);
  EXPECT_NE(result.error.find("[0,0]"), std::string::npos) << result.error;
  EXPECT_GT(provider.Diagnostics().load_failures, 0U);
}

TEST(TiledMapProvider, DeclaredNeighborLostAtRuntimeDegradesAndNamesIt) {
  TemporaryDirectory directory;
  SaveRealManifest(directory.root(), {TileIndex{0, 0}, TileIndex{1, 1}});

  TiledMapProvider provider(Config(directory.root(), 1, 1));
  ASSERT_TRUE(provider.Prepare(nullptr));

  fs::remove(TileFile(directory.root(), TileIndex{1, 1}));

  const auto result = WaitForTracking(provider, Pose(0.0));
  ASSERT_EQ(result.state, MapRequestState::kReady);
  ASSERT_NE(result.snapshot, nullptr);
  EXPECT_TRUE(result.snapshot->degraded);
  EXPECT_EQ(result.snapshot->missing_indices,
            (std::vector<TileIndex>{TileIndex{1, 1}}));
  EXPECT_EQ(result.snapshot->indices,
            (std::vector<TileIndex>{TileIndex{0, 0}}));
  EXPECT_GT(provider.Diagnostics().load_failures, 0U);
  EXPECT_NE(provider.Diagnostics().last_error.find("[1,1]"),
            std::string::npos) << provider.Diagnostics().last_error;
}
