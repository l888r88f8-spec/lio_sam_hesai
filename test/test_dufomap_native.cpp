#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "common/dufomap_config.h"
#include "lio_sam/dufomap_backend.h"

namespace {

using lio_sam_hesai::DufomapBackend;
using DufomapExportConfig = ::DufomapExportConfig;
using lio_sam_hesai::DufomapPoint;

std::array<double, 16> Identity() {
  return {1.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 1.0};
}

std::array<double, 16> Translation(double x, double y, double z) {
  auto pose = Identity();
  pose[3] = x;
  pose[7] = y;
  pose[11] = z;
  return pose;
}

DufomapExportConfig DefaultConfig() {
  DufomapExportConfig config;
  config.resolution = 0.1;
  config.d_s = 0.2;
  config.d_p = 2;
  config.min_range = 0.1;
  config.max_range = 20.0;
  config.num_threads = 2;
  return config;
}

std::vector<DufomapPoint> Wall(double local_x, int bound) {
  std::vector<DufomapPoint> points;
  for (int y = -bound; y <= bound; ++y) {
    for (int z = -bound; z <= bound; ++z) {
      points.push_back(DufomapPoint{
          static_cast<float>(local_x), 0.1F * static_cast<float>(y),
          0.1F * static_cast<float>(z), 10.0F});
    }
  }
  return points;
}

TEST(DufomapBackend, ReportsPinnedOfficialProvenance) {
  EXPECT_STREQ(lio_sam_hesai::DufomapBackendKind(),
               "native-cpp-upstream-core");
  EXPECT_STREQ(lio_sam_hesai::DufomapBackendSource(),
               "KTH-RPL/dufomap@9e239ddd5995136e14f5212f33382a6ebc59e518");
  EXPECT_STREQ(lio_sam_hesai::DufomapUfomapRevision(),
               "fcb20f445d257280c039c07ab756c9d5b37ebd78");
}

TEST(DufomapBackend, ClassifiesRealStaticMovingAndUnknownPoints) {
  DufomapBackend backend(DefaultConfig());
  const std::vector<DufomapPoint> moving{
      {3.2F, 0.2F, 0.2F, 20.0F}};
  auto first = Wall(6.0, 10);
  first.insert(first.end(), moving.begin(), moving.end());
  backend.Integrate(first, Identity());
  backend.Integrate(Wall(5.0, 10), Translation(1.0, 0.0, 0.0));
  backend.Propagate();

  const std::vector<DufomapPoint> targets{
      {6.0F, 0.2F, 0.2F, 1.0F},   // static wall
      {3.2F, 0.2F, 0.2F, 2.0F},   // seen through from the second pose
      {8.2F, 0.2F, 0.2F, 3.0F}};  // never observed, stays unknown
  const std::vector<std::uint8_t> labels = backend.Classify(targets);
  ASSERT_EQ(labels.size(), targets.size());
  EXPECT_EQ(labels[0], 0U);
  EXPECT_EQ(labels[1], 1U);
  EXPECT_EQ(labels[2], 0U);
}

TEST(DufomapBackend, CountsOnlyPointsInsideTheRangeGate) {
  DufomapExportConfig config = DefaultConfig();
  config.min_range = 2.0;
  config.max_range = 7.0;
  DufomapBackend backend(config);
  backend.Integrate({{1.0F, 0.0F, 0.0F, 1.0F},
                     {3.0F, 0.0F, 0.0F, 1.0F},
                     {8.0F, 0.0F, 0.0F, 1.0F}},
                    Identity());
  EXPECT_EQ(backend.integrated_points(), 1U);
}

TEST(DufomapBackend, RejectsUnsupportedAndInvalidParameters) {
  using Mutation = std::function<void(DufomapExportConfig&)>;
  for (const auto& mutate : std::vector<Mutation>{
           [](DufomapExportConfig& c) { c.resolution = 0.0; },
           [](DufomapExportConfig& c) { c.d_s = -1.0; },
           [](DufomapExportConfig& c) { c.d_p = 5; },
           [](DufomapExportConfig& c) { c.num_threads = 1; },
           [](DufomapExportConfig& c) { c.min_range = -1.0; },
           [](DufomapExportConfig& c) { c.max_range = 0.05; },
           [](DufomapExportConfig& c) { c.timeout_seconds = -1.0; },
           [](DufomapExportConfig& c) {
             c.timeout_seconds = std::numeric_limits<double>::quiet_NaN();
           },
           [](DufomapExportConfig& c) {
             c.timeout_seconds = std::numeric_limits<double>::infinity();
           },
           [](DufomapExportConfig& c) {
             c.timeout_seconds = std::numeric_limits<double>::max();
           }}) {
    DufomapExportConfig config = DefaultConfig();
    mutate(config);
    EXPECT_THROW(DufomapBackend backend(config), std::invalid_argument);
  }
}

TEST(DufomapBackend, RejectsInvalidPoseAndTargetCoordinates) {
  DufomapBackend backend(DefaultConfig());
  auto pose = Identity();
  pose[15] = 2.0;
  EXPECT_THROW(backend.Integrate(Wall(6.0, 1), pose), std::invalid_argument);

  const std::vector<DufomapPoint> targets{
      {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F, 0.0F}};
  EXPECT_THROW(backend.Classify(targets), std::invalid_argument);
}

TEST(DufomapBackend, TimeoutIsCheckedBetweenCooperativeSteps) {
  DufomapExportConfig config = DefaultConfig();
  config.timeout_seconds = 1e-9;
  DufomapBackend backend(config);
  try {
    backend.Integrate(Wall(6.0, 1), Identity());
    FAIL() << "expected the cooperative deadline to expire";
  } catch (const std::exception& error) {
    const std::string message = error.what();
    EXPECT_NE(message.find("timed out"), std::string::npos) << message;
    EXPECT_NE(message.find("cooperative"), std::string::npos) << message;
  }
}

}  // namespace
