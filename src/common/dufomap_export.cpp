#include "common/dufomap_export.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace lio_sam_hesai {
namespace {

bool Finite(double value) {
  return std::isfinite(value);
}

std::string EscapeJson(const std::string& value) {
  std::ostringstream out;
  for (const char character : value) {
    switch (character) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20U) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(character))
              << std::dec;
        } else {
          out << character;
        }
        break;
    }
  }
  return out.str();
}

void WritePose(std::ostringstream& out, const std::array<double, 16>& pose) {
  out << '[';
  for (std::size_t index = 0; index < pose.size(); ++index) {
    if (index != 0U) {
      out << ',';
    }
    out << pose[index];
  }
  out << ']';
}

}  // namespace

bool ValidateDufomapExportConfig(const DufomapExportConfig& config,
                                 std::string* error) {
  std::string reason;
  if (!Finite(config.resolution) || config.resolution <= 0.0) {
    reason = "dufomap_export.resolution must be finite and greater than zero";
  } else if (!Finite(config.d_s) || config.d_s < 0.0) {
    reason = "dufomap_export.d_s must be finite and non-negative";
  } else if (config.d_p < 0 || config.d_p > 4) {
    reason = "dufomap_export.d_p must be between 0 and 4";
  } else if (config.num_threads < 0 || config.num_threads == 1) {
    reason = "dufomap_export.num_threads must be 0 (automatic) or at least 2";
  } else if (!Finite(config.min_range) || config.min_range < 0.0) {
    reason = "dufomap_export.min_range must be finite and non-negative";
  } else if (!Finite(config.max_range) || config.max_range < 0.0 ||
           (config.max_range > 0.0 && config.max_range < config.min_range)) {
    reason = "dufomap_export.max_range must be zero or greater than or equal "
             "to min_range";
  } else if (!Finite(config.timeout_seconds) ||
           config.timeout_seconds < 0.0 ||
           config.timeout_seconds > 1.0e9) {
    reason = "dufomap_export.timeout_seconds must be finite, non-negative, "
             "and at most 1e9";
  }

  if (error != nullptr) {
    *error = reason;
  }
  return reason.empty();
}

bool ValidateDufomapFilterCompatibility(
    bool use_dufomap, const StaticMapFilterConfig& static_filter,
    std::string* error) {
  const bool legacy_active = static_filter.enable &&
                             static_filter.voxel_size > 1e-3 &&
                             static_filter.min_keyframes > 1;
  if (use_dufomap && legacy_active) {
    if (error != nullptr) {
      *error =
          "use_dufomap=true and static_map_filter.enable are mutually "
          "exclusive: the legacy voxel filter merges points into centroids "
          "before export, which destroys the original point coordinates that "
          "DUFOMap must preserve. Disable static_map_filter.enable or save "
          "with use_dufomap=false.";
    }
    return false;
  }
  if (error != nullptr) {
    error->clear();
  }
  return true;
}

std::string SerializeDufomapManifest(const DufomapManifest& manifest) {
  const DufomapExportConfig& params = manifest.parameters;
  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\n";
  out << "  \"format\": \"lio_sam_dufomap_export\",\n";
  out << "  \"version\": 1,\n";
  out << "  \"backend\": \"" << EscapeJson(manifest.backend) << "\",\n";
  out << "  \"backend_source\": \"" << EscapeJson(manifest.backend_source)
      << "\",\n";
  out << "  \"ufomap_revision\": \"" << EscapeJson(manifest.ufomap_revision)
      << "\",\n";
  out << "  \"parameters\": {\n";
  out << "    \"resolution\": " << params.resolution << ",\n";
  out << "    \"d_s\": " << params.d_s << ",\n";
  out << "    \"d_p\": " << params.d_p << ",\n";
  out << "    \"num_threads\": " << params.num_threads << ",\n";
  out << "    \"min_range\": " << params.min_range << ",\n";
  out << "    \"max_range\": " << params.max_range << ",\n";
  out << "    \"if_propagate\": true,\n";
  out << "    \"if_cluster\": false\n";
  out << "  },\n";
  out << "  \"frames\": [\n";
  for (std::size_t index = 0; index < manifest.frames.size(); ++index) {
    const DufomapFrameRecord& frame = manifest.frames[index];
    out << "    {\"index\": " << frame.index << ", \"scan\": \""
        << EscapeJson(frame.scan) << "\", \"pose\": ";
    WritePose(out, frame.pose);
    out << '}';
    if (index + 1U != manifest.frames.size()) {
      out << ',';
    }
    out << '\n';
  }
  out << "  ],\n";
  out << "  \"targets\": [\n";
  for (std::size_t index = 0; index < manifest.targets.size(); ++index) {
    const DufomapTargetRecord& target = manifest.targets[index];
    out << "    {\"name\": \"" << EscapeJson(target.name)
        << "\", \"input\": \"" << EscapeJson(target.input)
        << "\", \"output\": \"" << EscapeJson(target.output) << "\"}";
    if (index + 1U != manifest.targets.size()) {
      out << ',';
    }
    out << '\n';
  }
  out << "  ],\n";
  out << "  \"summary\": \"" << EscapeJson(manifest.summary) << "\"\n";
  out << "}\n";
  return out.str();
}

std::filesystem::path MakeDufomapSessionDirectory(
    const std::string& configured_base, std::uint64_t process_id,
    std::string* error) {
  std::error_code filesystem_error;
  std::filesystem::path base = configured_base.empty()
                                   ? std::filesystem::temp_directory_path(
                                         filesystem_error)
                                   : std::filesystem::path(configured_base);
  if (filesystem_error || base.empty()) {
    if (error != nullptr) {
      *error = "Unable to resolve the DUFOMap cache base directory: " +
               filesystem_error.message();
    }
    return {};
  }

  std::filesystem::create_directories(base, filesystem_error);
  if (filesystem_error) {
    if (error != nullptr) {
      *error = "Unable to create the DUFOMap cache base directory " +
               base.string() + ": " + filesystem_error.message();
    }
    return {};
  }

  const auto stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
  const std::filesystem::path session =
      base / ("session_" + std::to_string(process_id) + "_" +
              std::to_string(stamp));
  std::filesystem::create_directories(session / "scans", filesystem_error);
  if (filesystem_error) {
    std::error_code cleanup_error;
    std::filesystem::remove_all(session, cleanup_error);
    if (error != nullptr) {
      *error = "Unable to create the DUFOMap session cache " +
               session.string() + ": " + filesystem_error.message();
    }
    return {};
  }
  if (error != nullptr) {
    error->clear();
  }
  return session;
}

}  // namespace lio_sam_hesai
