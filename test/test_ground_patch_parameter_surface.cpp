#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string readProjectFile(const std::string& relative_path)
{
    std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

void expectAbsentFromParameterSurface(const std::string& token)
{
    const std::array<std::string, 3> files = {
        "include/lio_sam/utility.hpp",
        "config/mapping.yaml",
        "config/relocalization.yaml",
    };

    for (const auto& file : files)
    {
        const std::string text = readProjectFile(file);
        EXPECT_EQ(text.find(token), std::string::npos)
            << token << " is still present in " << file;
    }
}

void expectPresentInParameterSurface(const std::string& token)
{
    const std::array<std::string, 2> files = {
        "include/lio_sam/utility.hpp",
        "config/mapping.yaml",
    };

    for (const auto& file : files)
    {
        const std::string text = readProjectFile(file);
        EXPECT_NE(text.find(token), std::string::npos)
            << token << " is missing from " << file;
    }
}

}  // namespace

TEST(GroundPatchParameterSurface, RemovesDeprecatedPatchParameters)
{
    const std::array<std::string, 12> deprecated_tokens = {
        "globalMapTopic",
        "distanceForPatchBetweenRings",
        "groundPatchHorizontalEnable",
        "groundPatchHorizontalMaxDistance",
        "groundPatchToSensorCenterEnable",
        "useGlobalMapGround",
        "globalMapGroundRadius",
        "globalMapGroundZRange",
        "globalMapGroundVoxelSize",
        "globalMapGroundDistance",
        "globalMapGroundMaxAngle",
        "globalMapGroundMinPoints",
    };

    for (const auto& token : deprecated_tokens)
        expectAbsentFromParameterSurface(token);
}

TEST(GroundPatchParameterSurface, KeepsGroundGridPatchParameters)
{
    const std::array<std::string, 10> active_tokens = {
        "groundGridPatchEnable",
        "groundGridPatchResolution",
        "groundGridPatchSearchRadius",
        "groundGridPatchMaxSlopeDeg",
        "groundGridPatchMaxHeightRange",
        "groundGridPatchMaxRoughness",
        "groundGridPatchMaxPredictionMargin",
        "groundGridPatchMinInlierRatio",
        "groundGridPatchMinNeighborCells",
        "groundGridPatchMinSupportQuadrants",
    };

    for (const auto& token : active_tokens)
        expectPresentInParameterSurface(token);
}
