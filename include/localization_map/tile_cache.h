#ifndef LOCALIZATION_MAP__TILE_CACHE_H_
#define LOCALIZATION_MAP__TILE_CACHE_H_

#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "common/data_type.h"
#include "localization_map/tile_index.h"

namespace localization_map {

struct TilePayload {
  // Insert trusts bytes and retains const cloud handles without deep-copying.
  // Before insertion, callers must relinquish or freeze every mutable alias to
  // these clouds and must not mutate them afterward.
  PCLPointCloudXYZI::ConstPtr global;
  PCLPointCloudXYZI::ConstPtr ground;
  std::size_t bytes{0};
};

struct TileCacheConfig {
  // A zero value for either limit disables storage, so every insert rejects.
  std::size_t max_tiles{25};
  std::size_t max_bytes{1024U * 1024U * 1024U};
};

struct TileCacheSnapshot {
  std::size_t cached_tiles{0};
  std::size_t bytes{0};
  std::uint64_t hits{0};
  std::uint64_t misses{0};
  std::uint64_t evictions{0};
  std::uint64_t budget_rejections{0};
};

class TileCache {
 public:
  explicit TileCache(TileCacheConfig config);

  std::shared_ptr<const TilePayload> Find(const TileIndex& index);
  bool Insert(const TileIndex& index, TilePayload payload);

  // Pin and Unpin are reference-counted for existing entries; missing entries
  // are no-ops, and callers must match every effective Pin with an Unpin. A
  // single active-set owner must serialize insert/pin/commit transitions; this
  // is not a general-purpose per-client lease API.
  void Pin(const TileIndex& index);
  void Unpin(const TileIndex& index);

  // Invalidates every outstanding pin count. The caller must hold exclusive,
  // quiescent ownership of the cache's pin lifecycle while calling this.
  void ClearPins();
  TileCacheSnapshot Snapshot() const;

 private:
  struct Entry {
    std::shared_ptr<const TilePayload> payload;
    std::list<TileIndex>::iterator lru_position;
    std::size_t pin_count{0};
  };

  const TileCacheConfig config_;
  mutable std::mutex mutex_;
  std::unordered_map<TileIndex, Entry, TileIndexHash> entries_;
  std::list<TileIndex> lru_;
  std::size_t bytes_{0};
  std::uint64_t hits_{0};
  std::uint64_t misses_{0};
  std::uint64_t evictions_{0};
  std::uint64_t budget_rejections_{0};
};

}  // namespace localization_map

#endif  // LOCALIZATION_MAP__TILE_CACHE_H_
