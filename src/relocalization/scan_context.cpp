#include "relocalization/scan_context.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <new>
#include <utility>
#include <vector>

#include <unistd.h>

namespace relocalization {
namespace {

constexpr std::array<char, 8> kDbMagic = {'S', 'C', 'D', 'B', '0', '0', '0', '2'};
constexpr std::uint32_t kDbVersion = 2U;
constexpr std::uint32_t kEndianMarker = 0x01020304U;
constexpr std::uint32_t kMaxTagLength = 4096U;
constexpr std::uint64_t kMaxEntryCount = 500000U;
constexpr std::uint64_t kMaxDbPayloadBytes = 512U * 1024U * 1024U;
constexpr int kMaxDescriptorDimension = 2048;
constexpr double kTwoPi = 6.28318530717958647692;

template <typename T>
bool WriteValue(std::ostream& stream, const T& value) {
  stream.write(reinterpret_cast<const char*>(&value), sizeof(T));
  return stream.good();
}

template <typename T>
bool ReadValue(std::istream& stream, T* value) {
  stream.read(reinterpret_cast<char*>(value), sizeof(T));
  return stream.good();
}

bool CheckedMultiply(std::uint64_t lhs, std::uint64_t rhs,
                     std::uint64_t* result) {
  if (lhs != 0U && rhs > std::numeric_limits<std::uint64_t>::max() / lhs) {
    return false;
  }
  *result = lhs * rhs;
  return true;
}

bool CalculatePayloadBytes(const ScanContextConfig& config,
                           std::uint64_t count,
                           std::uint64_t* payload_bytes) {
  if (count > kMaxEntryCount) {
    return false;
  }

  std::uint64_t descriptor_values = 0U;
  std::uint64_t descriptor_bytes = 0U;
  if (!CheckedMultiply(static_cast<std::uint64_t>(config.num_ring),
                       static_cast<std::uint64_t>(config.num_sector),
                       &descriptor_values) ||
      !CheckedMultiply(descriptor_values, sizeof(float), &descriptor_bytes) ||
      descriptor_bytes > std::numeric_limits<std::uint64_t>::max() -
                             16U * sizeof(double)) {
    return false;
  }

  return CheckedMultiply(count, descriptor_bytes + 16U * sizeof(double),
                         payload_bytes) &&
         *payload_bytes <= kMaxDbPayloadBytes;
}

bool CreateTemporaryPath(const std::string& path, std::string* temporary_path) {
  if (path.empty() || temporary_path == nullptr) {
    return false;
  }

  std::string template_path = path + ".tmp.XXXXXX";
  std::vector<char> buffer(template_path.begin(), template_path.end());
  buffer.push_back('\0');
  const int descriptor = mkstemp(buffer.data());
  if (descriptor < 0) {
    return false;
  }
  if (close(descriptor) != 0) {
    std::remove(buffer.data());
    return false;
  }

  *temporary_path = buffer.data();
  return true;
}

bool IsFiniteConfig(const ScanContextConfig& config) {
  if (config.num_ring <= 0 || config.num_ring > kMaxDescriptorDimension ||
      config.num_sector <= 0 || config.num_sector > kMaxDescriptorDimension ||
      config.knn_candidates <= 0 || config.min_valid_points < 0) {
    return false;
  }
  const double ring_size = config.max_radius / config.num_ring;
  return std::isfinite(config.max_radius) &&
         std::isfinite(config.min_radius) &&
         std::isfinite(config.height_offset) &&
         std::isfinite(config.ringkey_max_dist) &&
         config.max_radius > config.min_radius && config.min_radius >= 0.0 &&
         config.ringkey_max_dist >= 0.0 && std::isfinite(ring_size) &&
         ring_size >= std::numeric_limits<double>::min();
}

bool IsValidDescriptor(const ScanContextDescriptor& descriptor,
                       const ScanContextConfig& config) {
  return descriptor.rows() == config.num_ring &&
         descriptor.cols() == config.num_sector && descriptor.allFinite() &&
         (descriptor.array() >= 0.0F).all();
}

bool IsPlausiblePose(const Eigen::Matrix4d& pose) {
  if (!pose.allFinite()) {
    return false;
  }

  const Eigen::Vector4d expected_last_row(0.0, 0.0, 0.0, 1.0);
  if ((pose.row(3).transpose() - expected_last_row).norm() > 1.0e-6) {
    return false;
  }

  const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
  return std::fabs(rotation.determinant() - 1.0) < 1.0e-3 &&
         (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() <
             1.0e-3;
}

int CountPopulatedBins(const ScanContextDescriptor& descriptor) {
  return static_cast<int>((descriptor.array() > 0.0F).count());
}

bool ReadConfig(std::istream& stream, ScanContextConfig* config) {
  std::int32_t num_ring = 0;
  std::int32_t num_sector = 0;
  std::int32_t knn_candidates = 0;
  std::int32_t min_valid_points = 0;
  if (!ReadValue(stream, &num_ring) || !ReadValue(stream, &num_sector) ||
      !ReadValue(stream, &config->max_radius) ||
      !ReadValue(stream, &config->min_radius) ||
      !ReadValue(stream, &config->height_offset) ||
      !ReadValue(stream, &knn_candidates) ||
      !ReadValue(stream, &config->ringkey_max_dist) ||
      !ReadValue(stream, &min_valid_points)) {
    return false;
  }
  config->num_ring = num_ring;
  config->num_sector = num_sector;
  config->knn_candidates = knn_candidates;
  config->min_valid_points = min_valid_points;
  return IsFiniteConfig(*config);
}

bool WriteConfig(std::ostream& stream, const ScanContextConfig& config) {
  const std::int32_t num_ring = config.num_ring;
  const std::int32_t num_sector = config.num_sector;
  const std::int32_t knn_candidates = config.knn_candidates;
  const std::int32_t min_valid_points = config.min_valid_points;
  return WriteValue(stream, num_ring) && WriteValue(stream, num_sector) &&
         WriteValue(stream, config.max_radius) &&
         WriteValue(stream, config.min_radius) &&
         WriteValue(stream, config.height_offset) &&
         WriteValue(stream, knn_candidates) &&
         WriteValue(stream, config.ringkey_max_dist) &&
         WriteValue(stream, min_valid_points);
}

}  // namespace

ScanContextDescriptor ScanContext::MakeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>& cloud) const {
  if (!IsFiniteConfig(config_)) {
    return {};
  }

  ScanContextDescriptor descriptor = ScanContextDescriptor::Zero(
      config_.num_ring, config_.num_sector);
  const double ring_size = config_.max_radius / config_.num_ring;
  const double sector_size = kTwoPi / config_.num_sector;
  if (!std::isfinite(ring_size) ||
      ring_size < std::numeric_limits<double>::min()) {
    return {};
  }

  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      continue;
    }

    const double range = std::hypot(static_cast<double>(point.x),
                                    static_cast<double>(point.y));
    if (range < config_.min_radius || range >= config_.max_radius) {
      continue;
    }

    double azimuth = std::atan2(static_cast<double>(point.y),
                                static_cast<double>(point.x));
    if (azimuth < 0.0) {
      azimuth += kTwoPi;
    }

    const int ring = std::min(
        static_cast<int>(range / ring_size), config_.num_ring - 1);
    const int sector = std::min(
        static_cast<int>(azimuth / sector_size), config_.num_sector - 1);
    const double shifted_height =
        std::max(0.0, static_cast<double>(point.z) + config_.height_offset);
    if (!std::isfinite(shifted_height)) {
      continue;
    }
    const float height = static_cast<float>(std::min(
        shifted_height, static_cast<double>(std::numeric_limits<float>::max())));
    descriptor(ring, sector) = std::max(descriptor(ring, sector), height);
  }

  return descriptor;
}

ScanContextRingKey ScanContext::MakeRingKey(
    const ScanContextDescriptor& descriptor) {
  ScanContextRingKey ring_key(descriptor.rows());
  for (int row = 0; row < descriptor.rows(); ++row) {
    ring_key(row) = descriptor.cols() == 0 ? 0.0F : descriptor.row(row).mean();
  }
  return ring_key;
}

void ScanContext::AddKeyFrame(const ScanContextDescriptor& descriptor,
                              const Eigen::Matrix4d& pose) {
  if (!IsFiniteConfig(config_) || !IsValidDescriptor(descriptor, config_) ||
      !IsPlausiblePose(pose)) {
    return;
  }

  descriptors_.push_back(descriptor);
  poses_.push_back(pose);
  ring_keys_.push_back(MakeRingKey(descriptor));
}

std::pair<float, int> ScanContext::DistanceBetween(
    const ScanContextDescriptor& query,
    const ScanContextDescriptor& database) {
  if (query.rows() != database.rows() || query.cols() != database.cols() ||
      query.rows() <= 0 || query.cols() <= 0 || !query.allFinite() ||
      !database.allFinite()) {
    return {std::numeric_limits<float>::infinity(), 0};
  }

  const int num_sector = query.cols();
  float best_distance = std::numeric_limits<float>::infinity();
  int best_shift = 0;

  for (int shift = 0; shift < num_sector; ++shift) {
    float total_distance = 0.0F;
    int valid_columns = 0;
    for (int column = 0; column < num_sector; ++column) {
      const auto query_column = query.col(column);
      const auto database_column =
          database.col((column + shift) % num_sector);
      const float query_norm = query_column.norm();
      const float database_norm = database_column.norm();
      if (query_norm < 1.0e-6F || database_norm < 1.0e-6F) {
        continue;
      }

      const float cosine = std::clamp(
          query_column.dot(database_column) / (query_norm * database_norm),
          -1.0F, 1.0F);
      total_distance += 1.0F - cosine;
      ++valid_columns;
    }

    if (valid_columns == 0) {
      continue;
    }
    const float distance = total_distance / valid_columns;
    if (distance < best_distance) {
      best_distance = distance;
      best_shift = shift;
    }
  }

  return {best_distance, best_shift};
}

bool ScanContext::SaveDB(const std::string& path,
                         const std::string& map_tag) const {
  if (!IsFiniteConfig(config_) || map_tag.size() > kMaxTagLength ||
      descriptors_.size() != poses_.size() ||
      descriptors_.size() != ring_keys_.size()) {
    return false;
  }
  for (std::size_t index = 0; index < descriptors_.size(); ++index) {
    if (!IsValidDescriptor(descriptors_[index], config_) ||
        !IsPlausiblePose(poses_[index])) {
      return false;
    }
  }

  const std::uint32_t tag_length =
      static_cast<std::uint32_t>(map_tag.size());
  const std::uint64_t count =
      static_cast<std::uint64_t>(descriptors_.size());
  std::uint64_t payload_bytes = 0U;
  if (!CalculatePayloadBytes(config_, count, &payload_bytes)) {
    return false;
  }

  std::string temporary_path;
  if (!CreateTemporaryPath(path, &temporary_path)) {
    return false;
  }
  std::ofstream stream(temporary_path,
                       std::ios::binary | std::ios::trunc | std::ios::out);
  if (!stream.is_open()) {
    std::remove(temporary_path.c_str());
    return false;
  }

  stream.write(kDbMagic.data(), static_cast<std::streamsize>(kDbMagic.size()));
  const bool header_ok = stream.good() && WriteValue(stream, kDbVersion) &&
                         WriteValue(stream, kEndianMarker) &&
                         WriteConfig(stream, config_) &&
                         WriteValue(stream, tag_length) &&
                         WriteValue(stream, count);
  if (!header_ok) {
    stream.close();
    std::remove(temporary_path.c_str());
    return false;
  }

  stream.write(map_tag.data(), static_cast<std::streamsize>(map_tag.size()));
  for (std::size_t index = 0; stream.good() && index < descriptors_.size();
       ++index) {
    const auto& descriptor = descriptors_[index];
    for (int row = 0; stream.good() && row < descriptor.rows(); ++row) {
      for (int column = 0; column < descriptor.cols(); ++column) {
        if (!WriteValue(stream, descriptor(row, column))) {
          break;
        }
      }
    }
    for (int row = 0; stream.good() && row < 4; ++row) {
      for (int column = 0; column < 4; ++column) {
        if (!WriteValue(stream, poses_[index](row, column))) {
          break;
        }
      }
    }
  }

  stream.close();
  if (!stream.good()) {
    std::remove(temporary_path.c_str());
    return false;
  }
  if (std::rename(temporary_path.c_str(), path.c_str()) != 0) {
    std::remove(temporary_path.c_str());
    return false;
  }
  return true;
}

bool ScanContext::LoadDB(const std::string& path, std::string* map_tag_out) {
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream.is_open()) {
    return false;
  }
  const std::streamoff file_size = stream.tellg();
  if (file_size < 0) {
    return false;
  }
  stream.seekg(0, std::ios::beg);

  std::array<char, kDbMagic.size()> magic{};
  std::uint32_t version = 0U;
  std::uint32_t endian_marker = 0U;
  ScanContextConfig loaded_config;
  std::uint32_t tag_length = 0U;
  std::uint64_t count = 0U;
  stream.read(magic.data(), static_cast<std::streamsize>(magic.size()));
  if (!stream.good() || std::memcmp(magic.data(), kDbMagic.data(), magic.size()) !=
                            0 ||
      !ReadValue(stream, &version) || !ReadValue(stream, &endian_marker) ||
      version != kDbVersion || endian_marker != kEndianMarker ||
      !ReadConfig(stream, &loaded_config) || !ReadValue(stream, &tag_length) ||
      !ReadValue(stream, &count) || tag_length > kMaxTagLength ||
      count > kMaxEntryCount) {
    return false;
  }

  std::uint64_t all_entry_bytes = 0U;
  if (!CalculatePayloadBytes(loaded_config, count, &all_entry_bytes)) {
    return false;
  }

  const std::streamoff header_end = stream.tellg();
  if (header_end < 0 || header_end > file_size) {
    return false;
  }
  const std::uint64_t remaining =
      static_cast<std::uint64_t>(file_size - header_end);
  if (remaining < tag_length ||
      remaining - tag_length != all_entry_bytes) {
    return false;
  }

  std::string loaded_tag(tag_length, '\0');
  if (tag_length > 0U) {
    stream.read(loaded_tag.data(), static_cast<std::streamsize>(tag_length));
    if (!stream.good()) {
      return false;
    }
  }

  std::vector<ScanContextDescriptor,
              Eigen::aligned_allocator<ScanContextDescriptor>> descriptors;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
  std::vector<ScanContextRingKey,
              Eigen::aligned_allocator<ScanContextRingKey>> ring_keys;
  try {
    descriptors.reserve(static_cast<std::size_t>(count));
    poses.reserve(static_cast<std::size_t>(count));
    ring_keys.reserve(static_cast<std::size_t>(count));

    for (std::uint64_t index = 0U; index < count; ++index) {
      ScanContextDescriptor descriptor(loaded_config.num_ring,
                                       loaded_config.num_sector);
      for (int row = 0; row < descriptor.rows(); ++row) {
        for (int column = 0; column < descriptor.cols(); ++column) {
          if (!ReadValue(stream, &descriptor(row, column))) {
            return false;
          }
        }
      }

      Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
      for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
          if (!ReadValue(stream, &pose(row, column))) {
            return false;
          }
        }
      }
      if (!IsValidDescriptor(descriptor, loaded_config) ||
          !IsPlausiblePose(pose)) {
        return false;
      }
      descriptors.push_back(std::move(descriptor));
      poses.push_back(pose);
      ring_keys.push_back(MakeRingKey(descriptors.back()));
    }
  } catch (const std::bad_alloc&) {
    return false;
  }

  config_ = loaded_config;
  descriptors_.swap(descriptors);
  poses_.swap(poses);
  ring_keys_.swap(ring_keys);
  if (map_tag_out != nullptr) {
    *map_tag_out = std::move(loaded_tag);
  }
  return true;
}

std::vector<ScanContextCandidate> ScanContext::Query(
    const ScanContextDescriptor& query, int top_n) const {
  std::vector<ScanContextCandidate> result;
  if (top_n <= 0 || Empty() || !IsFiniteConfig(config_) ||
      !IsValidDescriptor(query, config_) ||
      CountPopulatedBins(query) < config_.min_valid_points) {
    return result;
  }

  const ScanContextRingKey query_key = MakeRingKey(query);
  std::vector<std::pair<float, std::size_t>> ring_distances;
  ring_distances.reserve(ring_keys_.size());
  for (std::size_t index = 0; index < ring_keys_.size(); ++index) {
    if (ring_keys_[index].size() != query_key.size()) {
      continue;
    }
    ring_distances.emplace_back((ring_keys_[index] - query_key).squaredNorm(),
                                index);
  }
  if (ring_distances.empty()) {
    return result;
  }

  const std::size_t candidate_count = std::min<std::size_t>(
      ring_distances.size(), static_cast<std::size_t>(std::max(
                                 config_.knn_candidates, top_n)));
  const auto ring_less = [](const auto& lhs, const auto& rhs) {
    return lhs.first == rhs.first ? lhs.second < rhs.second : lhs.first < rhs.first;
  };
  std::partial_sort(ring_distances.begin(),
                    ring_distances.begin() + candidate_count,
                    ring_distances.end(), ring_less);

  std::vector<std::pair<ScanContextCandidate, std::size_t>> scored;
  scored.reserve(candidate_count);
  const double sector_angle = kTwoPi / config_.num_sector;
  const float max_ring_distance =
      static_cast<float>(config_.ringkey_max_dist * config_.ringkey_max_dist);
  for (std::size_t candidate = 0; candidate < candidate_count; ++candidate) {
    if (ring_distances[candidate].first > max_ring_distance) {
      continue;
    }
    const std::size_t index = ring_distances[candidate].second;
    const auto [distance, shift] = DistanceBetween(query, descriptors_[index]);
    if (!std::isfinite(distance)) {
      continue;
    }
    ScanContextCandidate scored_candidate;
    scored_candidate.pose = poses_[index];
    scored_candidate.yaw_shift = shift * sector_angle;
    scored_candidate.score = distance;
    scored.emplace_back(scored_candidate, index);
  }

  std::sort(scored.begin(), scored.end(), [](const auto& lhs, const auto& rhs) {
    if (lhs.first.score == rhs.first.score) {
      return lhs.second < rhs.second;
    }
    return lhs.first.score < rhs.first.score;
  });
  const std::size_t result_count = std::min<std::size_t>(
      scored.size(), static_cast<std::size_t>(top_n));
  result.reserve(result_count);
  for (std::size_t index = 0; index < result_count; ++index) {
    result.push_back(scored[index].first);
  }
  return result;
}

}  // namespace relocalization
