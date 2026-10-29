#ifndef LIO_SAM_HESAI_COMMON_TRANSFORMED_KEYFRAME_CACHE_H
#define LIO_SAM_HESAI_COMMON_TRANSFORMED_KEYFRAME_CACHE_H

#include <cstddef>
#include <iterator>
#include <list>
#include <unordered_map>
#include <utility>

#include <pcl/point_cloud.h>

namespace lio_sam_hesai {

// Bounded LRU cache for transformed corner/surface keyframe point buffers.
// The budget limits retained point-buffer bytes only; it is neither a
// whole-process RSS limit nor a cap on temporary transform peaks.
template <typename PointT>
class TransformedKeyframeCache {
 public:
  using PointCloud = pcl::PointCloud<PointT>;

  struct Entry {
    PointCloud corner;
    PointCloud surf;
  };

  explicit TransformedKeyframeCache(std::size_t max_bytes)
      : max_bytes_(max_bytes) {}

  TransformedKeyframeCache(const TransformedKeyframeCache&) = delete;
  TransformedKeyframeCache& operator=(const TransformedKeyframeCache&) = delete;

  // Returns null on a miss. A hit refreshes the entry's LRU position.
  const Entry* Find(int key) {
    const auto found = index_.find(key);
    if (found == index_.end()) {
      return nullptr;
    }
    entries_.splice(entries_.begin(), entries_, found->second);
    return &entries_.front().entry;
  }

  // Retains moved-in clouds when they fit. A zero budget or an entry larger
  // than the whole budget bypasses retention without evicting other entries.
  bool Insert(int key, PointCloud corner, PointCloud surf) {
    Erase(key);
    const std::size_t entry_bytes = BufferBytes(corner) + BufferBytes(surf);
    if (max_bytes_ == 0 || entry_bytes > max_bytes_) {
      return false;
    }
    while (!entries_.empty() && bytes_ + entry_bytes > max_bytes_) {
      EvictLeastRecentlyUsed();
    }
    Entry entry;
    entry.corner = std::move(corner);
    entry.surf = std::move(surf);
    entries_.push_front(Node{key, std::move(entry)});
    index_[key] = entries_.begin();
    bytes_ += entry_bytes;
    return true;
  }

  void Clear() {
    entries_.clear();
    index_.clear();
    bytes_ = 0;
  }

  std::size_t bytes() const { return bytes_; }

  std::size_t size() const { return entries_.size(); }

  std::size_t max_bytes() const { return max_bytes_; }

  // Retained buffer bytes use vector capacity, not size: capacity still owns
  // memory after a cloud shrinks.
  static std::size_t BufferBytes(const PointCloud& cloud) {
    return cloud.points.capacity() * sizeof(PointT);
  }

 private:
  struct Node {
    int key;
    Entry entry;
  };

  void Erase(int key) {
    const auto found = index_.find(key);
    if (found == index_.end()) {
      return;
    }
    bytes_ -= BufferBytes(found->second->entry.corner) +
              BufferBytes(found->second->entry.surf);
    entries_.erase(found->second);
    index_.erase(found);
  }

  void EvictLeastRecentlyUsed() {
    auto oldest = std::prev(entries_.end());
    bytes_ -= BufferBytes(oldest->entry.corner) +
              BufferBytes(oldest->entry.surf);
    index_.erase(oldest->key);
    entries_.erase(oldest);
  }

  std::size_t max_bytes_;
  std::size_t bytes_{0};
  // Front is most recently used; list iterators stay valid across splices.
  std::list<Node> entries_;
  std::unordered_map<int, typename std::list<Node>::iterator> index_;
};

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_TRANSFORMED_KEYFRAME_CACHE_H
