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
    const std::string text = readProjectFile("include/lio_sam/utility.hpp");
    EXPECT_NE(text.find("\"" + token + "\""), std::string::npos)
        << token << " is missing from include/lio_sam/utility.hpp";
}

std::size_t findYamlBlockEnd(const std::string& text,
                             std::size_t block_start,
                             std::size_t indentation)
{
    std::size_t line_start = text.find('\n', block_start);
    while (line_start != std::string::npos && line_start + 1 < text.size())
    {
        ++line_start;
        const std::size_t line_end = text.find('\n', line_start);
        const std::size_t first = text.find_first_not_of(' ', line_start);
        if (first != std::string::npos &&
            (line_end == std::string::npos || first < line_end) &&
            text[first] != '#' && first - line_start <= indentation)
        {
            return line_start;
        }
        line_start = line_end;
    }
    return text.size();
}

void expectPresentInMappingYaml(const std::string& leaf)
{
    const std::string text = readProjectFile("config/mapping.yaml");
    const std::size_t ground = text.find("\n    ground_segmentation:");
    ASSERT_NE(ground, std::string::npos)
        << "ground_segmentation is missing from config/mapping.yaml";
    const std::size_t ground_end = findYamlBlockEnd(text, ground, 4);
    const std::size_t grid_patch = text.find("\n      grid_patch:", ground);
    ASSERT_NE(grid_patch, std::string::npos)
        << "ground_segmentation.grid_patch is missing from config/mapping.yaml";
    ASSERT_LT(grid_patch, ground_end)
        << "grid_patch is not nested under ground_segmentation";
    const std::size_t grid_patch_end = findYamlBlockEnd(text, grid_patch, 6);
    const std::size_t yaml_leaf =
        text.find("\n        " + leaf + ":", grid_patch);
    EXPECT_TRUE(yaml_leaf != std::string::npos &&
                yaml_leaf < grid_patch_end)
        << "ground_segmentation.grid_patch." << leaf
        << " is missing from config/mapping.yaml";
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
    const std::array<std::string, 10> canonical_names = {
        "ground_segmentation.grid_patch.enable",
        "ground_segmentation.grid_patch.resolution",
        "ground_segmentation.grid_patch.search_radius",
        "ground_segmentation.grid_patch.max_slope_deg",
        "ground_segmentation.grid_patch.max_height_range",
        "ground_segmentation.grid_patch.max_roughness",
        "ground_segmentation.grid_patch.max_prediction_margin",
        "ground_segmentation.grid_patch.min_inlier_ratio",
        "ground_segmentation.grid_patch.min_neighbor_cells",
        "ground_segmentation.grid_patch.min_support_quadrants",
    };
    const std::array<std::string, 10> yaml_leaves = {
        "enable",
        "resolution",
        "search_radius",
        "max_slope_deg",
        "max_height_range",
        "max_roughness",
        "max_prediction_margin",
        "min_inlier_ratio",
        "min_neighbor_cells",
        "min_support_quadrants",
    };

    for (const auto& name : canonical_names)
        expectPresentInParameterSurface(name);

    for (const auto& leaf : yaml_leaves)
        expectPresentInMappingYaml(leaf);
}
