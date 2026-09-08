#include "localization_map/tile_cache.h"

#include <limits>
#include <utility>
#include <vector>

namespace localization_map {

TileCache::TileCache(TileCacheConfig config) : config_(config) {}

std::shared_ptr<const TilePayload> TileCache::Find(const TileIndex& index) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(index);
  if (entry == entries_.end()) {
    ++misses_;
    return nullptr;
  }

  ++hits_;
  lru_.splice(lru_.begin(), lru_, entry->second.lru_position);
  return entry->second.payload;
}

bool TileCache::Insert(const TileIndex& index, TilePayload payload) {
  std::unique_lock<std::mutex> lock(mutex_);

  if (config_.max_tiles == 0 || config_.max_bytes == 0 ||
      payload.bytes > config_.max_bytes) {
    ++budget_rejections_;
    return false;
  }

  const auto existing = entries_.find(index);
  const bool replacing = existing != entries_.end();
  const std::size_t replaced_bytes =
      replacing ? existing->second.payload->bytes : 0U;

  std::size_t projected_count = entries_.size();
  if (!replacing) {
    if (projected_count == std::numeric_limits<std::size_t>::max()) {
      ++budget_rejections_;
      return false;
    }
    ++projected_count;
  }

  std::size_t retained_bytes = bytes_ - replaced_bytes;
  const std::size_t maximum_retained_bytes =
      config_.max_bytes - payload.bytes;
  std::vector<TileIndex> victims;
  // Limits are intentionally small, so an O(n) LRU scan avoids extra state.
  for (auto lru_position = lru_.rbegin();
       lru_position != lru_.rend() &&
       (projected_count > config_.max_tiles ||
        retained_bytes > maximum_retained_bytes);
       ++lru_position) {
    if (*lru_position == index) {
      continue;
    }

    const auto candidate = entries_.find(*lru_position);
    if (candidate == entries_.end() || candidate->second.pin_count > 0) {
      continue;
    }

    victims.push_back(*lru_position);
    --projected_count;
    retained_bytes -= candidate->second.payload->bytes;
  }

  if (projected_count > config_.max_tiles ||
      retained_bytes > maximum_retained_bytes) {
    ++budget_rejections_;
    return false;
  }

  std::shared_ptr<const TilePayload> stored_payload;
  std::vector<std::shared_ptr<const TilePayload>> outgoing_payloads;
  try {
    stored_payload = std::make_shared<const TilePayload>(std::move(payload));
    outgoing_payloads.reserve(victims.size() + (replacing ? 1U : 0U));
  } catch (...) {
    lock.unlock();
    throw;
  }

  if (!replacing) {
    bool lru_inserted = false;
    try {
      lru_.push_front(index);
      lru_inserted = true;
      entries_.emplace(index, Entry{stored_payload, lru_.begin(), 0});
    } catch (...) {
      if (lru_inserted) {
        lru_.pop_front();
      }
      lock.unlock();
      throw;
    }
  }

  for (const TileIndex& victim : victims) {
    const auto victim_entry = entries_.find(victim);
    bytes_ -= victim_entry->second.payload->bytes;
    outgoing_payloads.push_back(std::move(victim_entry->second.payload));
    lru_.erase(victim_entry->second.lru_position);
    entries_.erase(victim_entry);
    ++evictions_;
  }

  if (replacing) {
    bytes_ -= replaced_bytes;
    outgoing_payloads.push_back(std::move(existing->second.payload));
    existing->second.payload = stored_payload;
    lru_.splice(lru_.begin(), lru_, existing->second.lru_position);
  }
  bytes_ += stored_payload->bytes;
  lock.unlock();
  outgoing_payloads.clear();
  return true;
}

void TileCache::Pin(const TileIndex& index) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(index);
  if (entry != entries_.end() &&
      entry->second.pin_count != std::numeric_limits<std::size_t>::max()) {
    ++entry->second.pin_count;
  }
}

void TileCache::Unpin(const TileIndex& index) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto entry = entries_.find(index);
  if (entry != entries_.end() && entry->second.pin_count > 0) {
    --entry->second.pin_count;
  }
}

void TileCache::ClearPins() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto& entry : entries_) {
    entry.second.pin_count = 0;
  }
}

TileCacheSnapshot TileCache::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return {entries_.size(), bytes_, hits_, misses_, evictions_,
          budget_rejections_};
}

}  // namespace localization_map
