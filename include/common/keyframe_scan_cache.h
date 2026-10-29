#ifndef LIO_SAM_HESAI_COMMON_KEYFRAME_SCAN_CACHE_H
#define LIO_SAM_HESAI_COMMON_KEYFRAME_SCAN_CACHE_H

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

#include "lio_sam/dufomap_backend.h"

namespace lio_sam_hesai {

// One persisted complete deskewed keyframe scan. The file is deliberately
// separate from the legacy RingPlane/Patchwork++ export caches: those may hold
// projected or algorithm-specific scans and must never stand in for a complete
// keyframe scan.
std::filesystem::path KeyframeScanCachePath(
    const std::filesystem::path& directory, std::size_t index);

// Writes magic, version, keyframe index, point count, and the exact point data.
// The write is atomic (temporary file then rename), so an interrupted write
// cannot leave a file that looks valid.
bool WriteKeyframeScanCacheFile(const std::filesystem::path& path,
                                std::size_t index,
                                const std::vector<DufomapPoint>& points,
                                std::string* error);

// Reads and fully validates one cache file. Rejects a missing, truncated,
// renamed (index mismatch), empty, or non-finite scan with an explicit error.
bool ReadKeyframeScanCacheFile(const std::filesystem::path& path,
                               std::size_t expected_index,
                               std::vector<DufomapPoint>* points,
                               std::string* error);

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_COMMON_KEYFRAME_SCAN_CACHE_H
