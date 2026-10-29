#include "common/keyframe_scan_cache.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace lio_sam_hesai {
namespace {

constexpr char kCacheMagic[8] = {'L', 'S', 'K', 'F', 'U', 'L', 'L', '1'};
constexpr std::uint32_t kCacheVersion = 1;
constexpr std::uint64_t kMaxPoints = 100000000U;
constexpr char kCacheName[] = "complete keyframe scan cache";

std::uint64_t HeaderSize() {
  return sizeof(kCacheMagic) + sizeof(std::uint32_t) * 2U +
         sizeof(std::uint64_t) * 2U;
}

void SetError(std::string* error, const std::string& message) {
  if (error != nullptr) *error = message;
}

std::string PathText(const std::filesystem::path& path) {
  return path.string();
}

bool Finite(const DufomapPoint& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && std::isfinite(point.intensity);
}

}  // namespace

std::filesystem::path KeyframeScanCachePath(
    const std::filesystem::path& directory, std::size_t index) {
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << index << ".bin";
  return directory / name.str();
}

bool WriteKeyframeScanCacheFile(const std::filesystem::path& path,
                                std::size_t index,
                                const std::vector<DufomapPoint>& points,
                                std::string* error) {
  if (points.empty() || points.size() > kMaxPoints) {
    SetError(error, std::string("invalid point count for ") + kCacheName + " " +
                        PathText(path));
    return false;
  }
  for (const auto& point : points) {
    if (!Finite(point)) {
      SetError(error, "non-finite point in " + std::string(kCacheName) + " " +
                          PathText(path));
      return false;
    }
  }
  std::error_code filesystem_error;
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path(), filesystem_error);
  }
  if (filesystem_error) {
    SetError(error, "cannot create " + PathText(path.parent_path()) + ": " +
                        filesystem_error.message());
    return false;
  }

  const std::filesystem::path temporary = path.string() + ".tmp";
  std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);
  const std::uint32_t version = kCacheVersion;
  const std::uint32_t reserved = 0;
  const std::uint64_t point_count = points.size();
  const std::uint64_t keyframe_index = index;
  stream.write(kCacheMagic, sizeof(kCacheMagic));
  stream.write(reinterpret_cast<const char*>(&version), sizeof(version));
  stream.write(reinterpret_cast<const char*>(&reserved), sizeof(reserved));
  stream.write(reinterpret_cast<const char*>(&keyframe_index),
               sizeof(keyframe_index));
  stream.write(reinterpret_cast<const char*>(&point_count),
               sizeof(point_count));
  stream.write(reinterpret_cast<const char*>(points.data()),
               static_cast<std::streamsize>(points.size() *
                                            sizeof(DufomapPoint)));
  stream.close();
  if (!stream.good()) {
    std::filesystem::remove(temporary, filesystem_error);
    SetError(error, "cannot write " + std::string(kCacheName) + " " +
                        PathText(path));
    return false;
  }

  std::filesystem::rename(temporary, path, filesystem_error);
  if (filesystem_error) {
    const std::string reason = filesystem_error.message();
    std::error_code cleanup_error;
    std::filesystem::remove(temporary, cleanup_error);
    SetError(error, "cannot finalize " + std::string(kCacheName) + " " +
                        PathText(path) + ": " + reason);
    return false;
  }
  if (error != nullptr) error->clear();
  return true;
}

bool ReadKeyframeScanCacheFile(const std::filesystem::path& path,
                               std::size_t expected_index,
                               std::vector<DufomapPoint>* points,
                               std::string* error) {
  if (points == nullptr) {
    SetError(error, std::string(kCacheName) + " output is null");
    return false;
  }
  std::error_code type_error;
  if (!std::filesystem::is_regular_file(path, type_error)) {
    SetError(error, std::string(kCacheName) + " is not a regular file: " +
                        PathText(path));
    return false;
  }
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    SetError(error, "missing " + std::string(kCacheName) + " " +
                        PathText(path));
    return false;
  }

  char magic[sizeof(kCacheMagic)]{};
  std::uint32_t version = 0;
  std::uint32_t reserved = 0;
  std::uint64_t keyframe_index = 0;
  std::uint64_t point_count = 0;
  stream.read(magic, sizeof(magic));
  stream.read(reinterpret_cast<char*>(&version), sizeof(version));
  stream.read(reinterpret_cast<char*>(&reserved), sizeof(reserved));
  stream.read(reinterpret_cast<char*>(&keyframe_index),
              sizeof(keyframe_index));
  stream.read(reinterpret_cast<char*>(&point_count), sizeof(point_count));
  if (!stream.good() ||
      std::memcmp(magic, kCacheMagic, sizeof(magic)) != 0 ||
      version != kCacheVersion || point_count == 0 ||
      point_count > kMaxPoints) {
    SetError(error, "corrupt or unsupported " + std::string(kCacheName) + " " +
                        PathText(path));
    return false;
  }
  if (keyframe_index != expected_index) {
    SetError(error, "keyframe index mismatch in " + std::string(kCacheName) +
                        " " + PathText(path) + " (expected " +
                        std::to_string(expected_index) + ", found " +
                        std::to_string(keyframe_index) + ")");
    return false;
  }

  std::error_code filesystem_error;
  const std::uintmax_t file_size = std::filesystem::file_size(path,
                                                              filesystem_error);
  const std::uint64_t expected_size =
      HeaderSize() + point_count * sizeof(DufomapPoint);
  if (filesystem_error || file_size != expected_size) {
    SetError(error, "truncated " + std::string(kCacheName) + " " +
                        PathText(path));
    return false;
  }

  points->resize(static_cast<std::size_t>(point_count));
  stream.read(reinterpret_cast<char*>(points->data()),
              static_cast<std::streamsize>(points->size() *
                                           sizeof(DufomapPoint)));
  if (!stream.good() || stream.peek() != std::char_traits<char>::eof()) {
    SetError(error, "truncated " + std::string(kCacheName) + " " +
                        PathText(path));
    return false;
  }
  for (const auto& point : *points) {
    if (!Finite(point)) {
      SetError(error, "non-finite point in " + std::string(kCacheName) + " " +
                          PathText(path));
      return false;
    }
  }
  if (error != nullptr) error->clear();
  return true;
}

}  // namespace lio_sam_hesai
