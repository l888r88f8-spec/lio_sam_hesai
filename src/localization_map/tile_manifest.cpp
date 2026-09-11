#include "common/map_artifact_layout.h"
#include "localization_map/tile_manifest.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

namespace localization_map {
namespace {

namespace fs = std::filesystem;

[[noreturn]] void Fail(const std::string& message);

class UniqueFileDescriptor {
 public:
  explicit UniqueFileDescriptor(int descriptor) : descriptor_(descriptor) {}
  ~UniqueFileDescriptor() {
    if (descriptor_ >= 0) {
      ::close(descriptor_);
    }
  }

  UniqueFileDescriptor(const UniqueFileDescriptor&) = delete;
  UniqueFileDescriptor& operator=(const UniqueFileDescriptor&) = delete;

  int get() const { return descriptor_; }

  void CloseChecked(const char* label) {
    const int descriptor = descriptor_;
    descriptor_ = -1;
    if (::close(descriptor) != 0) {
      Fail(std::string("unable to close ") + label + ": " + std::strerror(errno));
    }
  }

 private:
  int descriptor_{-1};
};

[[noreturn]] void Fail(const std::string& message) {
  throw std::runtime_error(message);
}

YAML::Node Required(const YAML::Node& parent, const char* key,
                    const char* context) {
  const YAML::Node value = parent[key];
  if (!value || value.IsNull()) {
    Fail(std::string("manifest is missing ") + context + "." + key);
  }
  return value;
}

std::string ScalarString(const YAML::Node& value, const char* name) {
  if (!value.IsScalar()) {
    Fail(std::string(name) + " must be a scalar");
  }
  try {
    return value.as<std::string>();
  } catch (const YAML::Exception&) {
    Fail(std::string("invalid ") + name);
  }
}

std::string NonemptyString(const YAML::Node& value, const char* name) {
  const std::string result = ScalarString(value, name);
  if (result.empty()) {
    Fail(std::string(name) + " must be nonempty");
  }
  return result;
}

template <typename Integer>
Integer ParseInteger(const YAML::Node& value, const char* name) {
  const std::string text = ScalarString(value, name);
  Integer result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    Fail(std::string("invalid ") + name);
  }
  return result;
}

std::uintmax_t NonnegativeBytes(const YAML::Node& value, const char* name) {
  const std::string text = ScalarString(value, name);
  if (text.empty() || text.front() == '-') {
    Fail(std::string("invalid ") + name);
  }
  std::uintmax_t result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    Fail(std::string("invalid ") + name);
  }
  return result;
}

double FiniteDouble(const YAML::Node& value, const char* name, bool positive) {
  if (!value.IsScalar()) {
    Fail(std::string(name) + " must be a scalar");
  }
  try {
    const double result = value.as<double>();
    if (!std::isfinite(result) || (positive && result <= 0.0)) {
      Fail(std::string(name) + (positive ? " must be finite and positive"
                                         : " must be finite"));
    }
    return result;
  } catch (const YAML::Exception&) {
    Fail(std::string("invalid ") + name);
  }
}

TileIndex ParseTileIndex(const YAML::Node& value, const char* kind) {
  if (!value.IsSequence() || value.size() != 2U) {
    Fail(std::string(kind) + " tile index must contain exactly two integers");
  }
  return {ParseInteger<int>(value[0], "tile index"),
          ParseInteger<int>(value[1], "tile index")};
}

fs::path CanonicalRoot(const fs::path& root) {
  std::error_code error;
  const fs::path canonical = fs::weakly_canonical(root, error);
  if (error) {
    Fail("unable to canonicalize tiled map root: " + error.message());
  }
  return canonical;
}

int OpenRootDirectory(const fs::path& root) {
  const fs::path canonical_root = CanonicalRoot(root);
  const int descriptor = ::open(canonical_root.c_str(),
                                O_RDONLY | O_CLOEXEC | O_DIRECTORY | O_NOFOLLOW);
  if (descriptor < 0) {
    Fail("unable to open tiled map root: " + std::string(std::strerror(errno)));
  }
  return descriptor;
}

std::string ReadManifestFile(const fs::path& root) {
  UniqueFileDescriptor root_descriptor(OpenRootDirectory(root));
  const int manifest_fd =
      ::openat(root_descriptor.get(), MapArtifactLayout::kManifestName,
               O_RDONLY | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK);
  if (manifest_fd < 0) {
    if (errno == ELOOP) {
      Fail("manifest.yaml must be a regular file and not a symlink");
    }
    Fail("unable to open manifest.yaml: " + std::string(std::strerror(errno)));
  }
  UniqueFileDescriptor manifest_descriptor(manifest_fd);

  struct stat status {};
  if (::fstat(manifest_descriptor.get(), &status) != 0) {
    Fail("unable to inspect manifest.yaml: " + std::string(std::strerror(errno)));
  }
  if (!S_ISREG(status.st_mode)) {
    Fail("manifest.yaml must be a regular file and not a symlink");
  }

  std::string contents;
  std::array<char, 8192> buffer{};
  while (true) {
    const ssize_t count = ::read(manifest_descriptor.get(), buffer.data(), buffer.size());
    if (count > 0) {
      contents.append(buffer.data(), static_cast<std::size_t>(count));
      continue;
    }
    if (count == 0) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    Fail("unable to read manifest.yaml: " + std::string(std::strerror(errno)));
  }
  manifest_descriptor.CloseChecked("manifest.yaml");
  return contents;
}

fs::path ResolveManifestPath(const fs::path& root, const fs::path& relative) {
  if (relative.empty()) {
    Fail("manifest path must be nonempty");
  }
  if (relative.is_absolute()) {
    Fail("manifest path must be relative");
  }

  const fs::path canonical_root = CanonicalRoot(root);
  std::error_code error;
  const fs::path canonical_candidate = fs::weakly_canonical(root / relative, error);
  if (error) {
    Fail("unable to canonicalize manifest path: " + error.message());
  }
  const auto mismatch = std::mismatch(
      canonical_root.begin(), canonical_root.end(), canonical_candidate.begin(),
      canonical_candidate.end());
  if (mismatch.first != canonical_root.end()) {
    Fail("manifest path escapes tiled map root");
  }
  return canonical_candidate;
}

struct ResolvedFile {
  fs::path path;
  const char* label;
  std::optional<std::uintmax_t> bytes;
};

void CheckExists(const ResolvedFile& file) {
  std::error_code error;
  const bool exists = fs::exists(file.path, error);
  if (error) {
    Fail(std::string("unable to inspect ") + file.label + ": " + error.message());
  }
  if (!exists) {
    Fail(std::string(file.label) + " is missing");
  }
}

void CheckRegularFile(const ResolvedFile& file) {
  std::error_code error;
  const bool regular = fs::is_regular_file(file.path, error);
  if (error) {
    Fail(std::string("unable to inspect ") + file.label + ": " + error.message());
  }
  if (!regular) {
    Fail(std::string(file.label) + " is not regular");
  }
}

void CheckByteSize(const ResolvedFile& file) {
  if (!file.bytes.has_value()) {
    return;
  }
  std::error_code error;
  const std::uintmax_t actual = fs::file_size(file.path, error);
  if (error) {
    Fail(std::string("unable to inspect ") + file.label + ": " + error.message());
  }
  if (actual != *file.bytes) {
    Fail("tile byte size mismatch");
  }
}

void ValidateScalars(const TileManifest& manifest) {
  if (manifest.schema_version != TileManifest::kSchemaVersion) {
    Fail("unsupported schema_version");
  }
  if (manifest.frame_id.empty()) {
    Fail("frame_id must be nonempty");
  }
  if (manifest.point_type != "XYZI") {
    Fail("point_type must be XYZI");
  }
  if (!std::isfinite(manifest.tile_size_m) || manifest.tile_size_m <= 0.0) {
    Fail("tile_size_m must be finite and positive");
  }
  if (!std::isfinite(manifest.origin[0]) || !std::isfinite(manifest.origin[1])) {
    Fail("origin_xy values must be finite");
  }
  if (manifest.source.global_map_name.empty() || manifest.source.ground_map_name.empty()) {
    Fail("source map names must be nonempty");
  }
  if (manifest.overview.has_value() &&
      (!std::isfinite(manifest.overview->resolution_m) ||
       manifest.overview->resolution_m <= 0.0)) {
    Fail("overview resolution_m must be finite and positive");
  }
}

void ValidateFiles(const fs::path& root, const TileManifest& manifest) {
  // These checks assume the atomically published map root remains immutable.
  // Future on-demand tile openers must repeat safe open-time validation.
  std::vector<ResolvedFile> files;
  files.reserve(manifest.global_tiles.size() + manifest.ground_tiles.size() +
                (manifest.overview.has_value() ? 1U : 0U));
  for (const auto& entry : manifest.global_tiles) {
    files.push_back({ResolveManifestPath(root, entry.second.path), "tile file",
                     entry.second.bytes});
  }
  for (const auto& entry : manifest.ground_tiles) {
    files.push_back({ResolveManifestPath(root, entry.second.path), "tile file",
                     entry.second.bytes});
  }
  if (manifest.overview.has_value()) {
    files.push_back({ResolveManifestPath(root, manifest.overview->path), "overview file",
                     std::nullopt});
  }
  for (const ResolvedFile& file : files) {
    CheckExists(file);
  }
  for (const ResolvedFile& file : files) {
    CheckRegularFile(file);
  }
  for (const ResolvedFile& file : files) {
    CheckByteSize(file);
  }
}

void WriteAll(int descriptor, const std::string& contents) {
  const char* data = contents.data();
  std::size_t remaining = contents.size();
  while (remaining > 0U) {
    const ssize_t written = ::write(descriptor, data, remaining);
    if (written < 0 && errno == EINTR) {
      continue;
    }
    if (written <= 0) {
      if (written == 0) {
        errno = EIO;
      }
      Fail(std::string("unable to write manifest.yaml: ") + std::strerror(errno));
    }
    data += written;
    remaining -= static_cast<std::size_t>(written);
  }
}

void WriteManifestFile(const fs::path& root, const std::string& contents) {
  UniqueFileDescriptor root_descriptor(OpenRootDirectory(root));
  static std::atomic<std::uint64_t> next_temporary_id{0};

  std::string temporary_name;
  int temporary_fd = -1;
  for (int attempt = 0; attempt < 128; ++attempt) {
    temporary_name = std::string(".") + MapArtifactLayout::kManifestName + ".tmp." +
                     std::to_string(::getpid()) + "." +
                     std::to_string(next_temporary_id.fetch_add(1));
    temporary_fd =
        ::openat(root_descriptor.get(), temporary_name.c_str(),
                 O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW, 0644);
    if (temporary_fd >= 0) {
      break;
    }
    if (errno != EEXIST) {
      Fail(std::string("unable to create manifest temporary file: ") +
           std::strerror(errno));
    }
  }
  if (temporary_fd < 0) {
    Fail("unable to create unique manifest temporary file");
  }

  UniqueFileDescriptor temporary_descriptor(temporary_fd);
  bool renamed = false;
  try {
    WriteAll(temporary_descriptor.get(), contents);
    if (::fsync(temporary_descriptor.get()) != 0) {
      Fail(std::string("unable to sync manifest temporary file: ") + std::strerror(errno));
    }
    temporary_descriptor.CloseChecked("manifest temporary file");

    if (::renameat(root_descriptor.get(), temporary_name.c_str(), root_descriptor.get(),
                   MapArtifactLayout::kManifestName) != 0) {
      Fail("unable to replace manifest.yaml: " + std::string(std::strerror(errno)));
    }
    renamed = true;
    if (::fsync(root_descriptor.get()) != 0) {
      Fail("unable to sync tiled map root: " + std::string(std::strerror(errno)));
    }
  } catch (...) {
    if (!renamed) {
      ::unlinkat(root_descriptor.get(), temporary_name.c_str(), 0);
    }
    throw;
  }
}

std::map<TileIndex, TileFileMetadata> ParseTiles(const YAML::Node& tiles,
                                                 const char* kind) {
  if (!tiles.IsSequence()) {
    Fail(std::string(kind) + " tiles must be a sequence");
  }
  std::map<TileIndex, TileFileMetadata> result;
  for (const YAML::Node& tile : tiles) {
    if (!tile.IsMap()) {
      Fail(std::string(kind) + " tile must be a map");
    }
    const TileIndex index = ParseTileIndex(Required(tile, "index", "tile"), kind);
    TileFileMetadata metadata;
    metadata.path = ScalarString(Required(tile, "path", "tile"), "tile path");
    metadata.point_count = ParseInteger<std::uint64_t>(
        Required(tile, "point_count", "tile"), "point_count");
    metadata.bytes = NonnegativeBytes(Required(tile, "bytes", "tile"), "tile bytes");
    if (!result.emplace(index, std::move(metadata)).second) {
      Fail(std::string("duplicate ") + kind + " tile index");
    }
  }
  return result;
}

TileManifest LoadUnchecked(const fs::path& root) {
  const YAML::Node document = YAML::Load(ReadManifestFile(root));
  if (!document.IsMap()) {
    Fail("manifest root must be a map");
  }

  TileManifest manifest;
  manifest.schema_version = ParseInteger<int>(Required(document, "schema_version", "manifest"),
                                              "schema_version");
  if (manifest.schema_version != TileManifest::kSchemaVersion) {
    Fail("unsupported schema_version");
  }
  manifest.frame_id = NonemptyString(Required(document, "frame_id", "manifest"), "frame_id");
  manifest.point_type = NonemptyString(Required(document, "point_type", "manifest"), "point_type");
  manifest.tile_size_m = FiniteDouble(Required(document, "tile_size_m", "manifest"),
                                      "tile_size_m", true);

  const YAML::Node origin = Required(document, "origin_xy", "manifest");
  if (!origin.IsSequence() || origin.size() != 2U) {
    Fail("origin_xy must contain exactly two values");
  }
  manifest.origin = {FiniteDouble(origin[0], "origin_xy", false),
                     FiniteDouble(origin[1], "origin_xy", false)};

  const YAML::Node source = Required(document, "source", "manifest");
  if (!source.IsMap()) {
    Fail("source must be a map");
  }
  manifest.source.global_map_name = NonemptyString(
      Required(source, "global_map_name", "source"), "global_map_name");
  manifest.source.global_map_bytes = NonnegativeBytes(
      Required(source, "global_map_bytes", "source"), "global_map_bytes");
  manifest.source.ground_map_name = NonemptyString(
      Required(source, "ground_map_name", "source"), "ground_map_name");
  manifest.source.ground_map_bytes = NonnegativeBytes(
      Required(source, "ground_map_bytes", "source"), "ground_map_bytes");

  const YAML::Node overview = document["overview"];
  if (overview && !overview.IsNull()) {
    if (!overview.IsMap()) {
      Fail("overview must be a map");
    }
    OverviewMetadata metadata;
    metadata.path = ScalarString(Required(overview, "path", "overview"), "overview path");
    metadata.resolution_m = FiniteDouble(
        Required(overview, "resolution_m", "overview"), "overview resolution_m", true);
    manifest.overview = std::move(metadata);
  }

  ValidateScalars(manifest);

  const YAML::Node tiles = Required(document, "tiles", "manifest");
  if (!tiles.IsMap()) {
    Fail("tiles must be a map");
  }
  manifest.global_tiles = ParseTiles(Required(tiles, "global", "tiles"), "global");
  manifest.ground_tiles = ParseTiles(Required(tiles, "ground", "tiles"), "ground");

  ValidateFiles(root, manifest);
  return manifest;
}

void EmitTiles(YAML::Emitter& output, const std::map<TileIndex, TileFileMetadata>& tiles) {
  output << YAML::BeginSeq;
  for (const auto& entry : tiles) {
    output << YAML::BeginMap;
    output << YAML::Key << "index" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << entry.first.x << entry.first.y << YAML::EndSeq;
    output << YAML::Key << "path" << YAML::Value << entry.second.path.generic_string();
    output << YAML::Key << "point_count" << YAML::Value << entry.second.point_count;
    output << YAML::Key << "bytes" << YAML::Value << entry.second.bytes;
    output << YAML::EndMap;
  }
  output << YAML::EndSeq;
}

}  // namespace

TileManifest TileManifest::Load(const fs::path& root) {
  try {
    return LoadUnchecked(root);
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("invalid manifest YAML: ") + error.what());
  } catch (const fs::filesystem_error& error) {
    throw std::runtime_error(std::string("manifest filesystem error: ") + error.what());
  } catch (const std::runtime_error&) {
    throw;
  }
}

void TileManifest::Save(const fs::path& root) const {
  try {
    ValidateScalars(*this);
    ValidateFiles(root, *this);

    YAML::Emitter output;
    output << YAML::BeginMap;
    output << YAML::Key << "schema_version" << YAML::Value << schema_version;
    output << YAML::Key << "frame_id" << YAML::Value << frame_id;
    output << YAML::Key << "point_type" << YAML::Value << point_type;
    output << YAML::Key << "tile_size_m" << YAML::Value << tile_size_m;
    output << YAML::Key << "origin_xy" << YAML::Value << YAML::Flow << YAML::BeginSeq
           << origin[0] << origin[1] << YAML::EndSeq;
    output << YAML::Key << "source" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "global_map_name" << YAML::Value << source.global_map_name;
    output << YAML::Key << "global_map_bytes" << YAML::Value << source.global_map_bytes;
    output << YAML::Key << "ground_map_name" << YAML::Value << source.ground_map_name;
    output << YAML::Key << "ground_map_bytes" << YAML::Value << source.ground_map_bytes;
    output << YAML::EndMap;
    output << YAML::Key << "tiles" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "global" << YAML::Value;
    EmitTiles(output, global_tiles);
    output << YAML::Key << "ground" << YAML::Value;
    EmitTiles(output, ground_tiles);
    output << YAML::EndMap;
    if (overview.has_value()) {
      output << YAML::Key << "overview" << YAML::Value << YAML::BeginMap;
      output << YAML::Key << "path" << YAML::Value << overview->path.generic_string();
      output << YAML::Key << "resolution_m" << YAML::Value << overview->resolution_m;
      output << YAML::EndMap;
    }
    output << YAML::EndMap;
    if (!output.good()) {
      Fail("failed to emit manifest YAML");
    }
    WriteManifestFile(root, std::string(output.c_str()) + '\n');
  } catch (const YAML::Exception& error) {
    throw std::runtime_error(std::string("invalid manifest YAML: ") + error.what());
  } catch (const fs::filesystem_error& error) {
    throw std::runtime_error(std::string("manifest filesystem error: ") + error.what());
  } catch (const std::runtime_error&) {
    throw;
  }
}

}  // namespace localization_map
