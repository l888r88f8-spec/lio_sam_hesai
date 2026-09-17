#include "common/lidar_imu_calibration.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <fstream>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{

struct StaticTransform
{
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
};

std::string ReadProjectFile(const std::string & relative_path)
{
  std::ifstream file(std::string(PROJECT_SOURCE_DIR) + "/" + relative_path);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open " + relative_path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

StaticTransform ReadStaticTransform(
  const std::string & launch_text,
  const std::string & child_frame)
{
  const std::regex arguments_pattern(
    R"regex(arguments\s*=\s*\[([\s\S]*?)\]\s*,)regex");
  const std::regex quoted_value_pattern(R"regex("([^"]*)")regex");

  for (std::sregex_iterator block(
      launch_text.begin(), launch_text.end(), arguments_pattern);
    block != std::sregex_iterator(); ++block)
  {
    const std::string arguments = (*block)[1].str();
    std::vector<std::string> values;
    for (std::sregex_iterator value(
        arguments.begin(), arguments.end(), quoted_value_pattern);
      value != std::sregex_iterator(); ++value)
    {
      values.push_back((*value)[1].str());
    }
    if (values.size() != 8U ||
      values[6] != "base_link" || values[7] != child_frame)
    {
      continue;
    }

    const double yaw = std::stod(values[3]);
    const double pitch = std::stod(values[4]);
    const double roll = std::stod(values[5]);
    return {
      Eigen::Vector3d(
        std::stod(values[0]), std::stod(values[1]), std::stod(values[2])),
      (
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix()
    };
  }

  throw std::runtime_error(
          "Missing base_link -> " + child_frame + " static transform");
}

Eigen::Vector3d ReadCalibrationTranslation(const std::string & yaml_text)
{
  const std::size_t calibration = yaml_text.find("\n    calibration:");
  if (calibration == std::string::npos) {
    throw std::runtime_error("Missing calibration YAML group");
  }
  const std::size_t translation =
    yaml_text.find("\n      translation:", calibration);
  const std::size_t open = yaml_text.find('[', translation);
  const std::size_t close = yaml_text.find(']', open);
  if (translation == std::string::npos ||
    open == std::string::npos || close == std::string::npos)
  {
    throw std::runtime_error("Missing calibration.translation YAML value");
  }

  std::string values = yaml_text.substr(open + 1, close - open - 1);
  std::replace(values.begin(), values.end(), ',', ' ');
  std::istringstream stream(values);
  Eigen::Vector3d result;
  if (!(stream >> result.x() >> result.y() >> result.z())) {
    throw std::runtime_error("Invalid calibration.translation YAML value");
  }
  double extra = 0.0;
  if (stream >> extra) {
    throw std::runtime_error("calibration.translation must have three values");
  }
  return result;
}

std::string ReadPublishStaticTfDefault(const std::string & launch_text)
{
  const std::regex default_pattern(
    R"regex(publish_static_tf_declare\s*=\s*DeclareLaunchArgument\s*\()regex"
    R"regex(\s*"publish_static_tf"\s*,\s*default_value\s*=\s*"([^"]+)")regex");
  std::smatch match;
  if (!std::regex_search(launch_text, match, default_pattern)) {
    throw std::runtime_error("Missing publish_static_tf launch default");
  }
  return match[1].str();
}

}  // namespace

TEST(LidarImuCalibration, PreservesCanonicalLidarToImuDirection)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> rotation = {
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0};
  const std::vector<double> rpy = {
      1.0, 0.0,  0.0,
      0.0, 0.0, -1.0,
      0.0, 1.0,  0.0};

  Eigen::Matrix3d expected_rotation;
  expected_rotation << 0.0, -1.0, 0.0,
                       1.0,  0.0, 0.0,
                       0.0,  0.0, 1.0;
  Eigen::Matrix3d expected_rpy;
  expected_rpy << 1.0, 0.0,  0.0,
                  0.0, 0.0, -1.0,
                  0.0, 1.0,  0.0;

  const auto calibration =
      ParseLidarImuCalibration(translation, rotation, rpy);

  EXPECT_TRUE(calibration.imu_T_lidar.Rotation().isApprox(expected_rotation));
  EXPECT_TRUE(calibration.imu_T_lidar.Translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.10)));
  EXPECT_TRUE(calibration.mapping_imu_to_lidar_rotation.isApprox(
      expected_rotation.transpose()));
  EXPECT_TRUE(calibration.mapping_rpy_rotation.isApprox(expected_rpy));
  EXPECT_TRUE(calibration.imu_T_lidar.Matrix().row(3).isApprox(
      Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)));
  // The inverse is derived rather than configured independently.
  EXPECT_TRUE((calibration.imu_T_lidar * calibration.lidar_T_imu)
                  .Matrix().isApprox(Eigen::Matrix4d::Identity()));
}

TEST(LidarImuCalibration, DerivesMappingLeverArmsInLidarAxes)
{
  const std::vector<double> translation = {1.0, 2.0, 3.0};
  const std::vector<double> rotation = {
      0.0, -1.0, 0.0,
      1.0,  0.0, 0.0,
      0.0,  0.0, 1.0};
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

  const auto calibration =
      ParseLidarImuCalibration(translation, rotation, identity);

  EXPECT_TRUE(calibration.mapping_imu_to_lidar_lever_arm.isApprox(
      Eigen::Vector3d(2.0, -1.0, 3.0)));
}

TEST(LidarImuCalibration, MappingPreintegrationUsesDerivedLeverArm)
{
  const std::string loader =
      ReadProjectFile("src/common/slam_config.cpp");
  const std::string preintegration =
      ReadProjectFile("src/imuPreintegration.cpp");

  EXPECT_NE(loader.find(
      "config.calibration.mapping_imu_to_lidar_lever_arm ="),
      std::string::npos);
  const std::size_t imu2_lidar = preintegration.find(
      "gtsam::Pose3 imu2Lidar = gtsam::Pose3(");
  const std::size_t lidar2_imu = preintegration.find(
      "gtsam::Pose3 lidar2Imu = imu2Lidar.inverse();");
  ASSERT_NE(imu2_lidar, std::string::npos);
  ASSERT_NE(lidar2_imu, std::string::npos);
  ASSERT_LT(imu2_lidar, lidar2_imu);
  const std::string imu2_lidar_definition = preintegration.substr(
      imu2_lidar, lidar2_imu - imu2_lidar);

  EXPECT_NE(imu2_lidar_definition.find("gtsam::Rot3(1, 0, 0, 0)"),
      std::string::npos);
  EXPECT_NE(imu2_lidar_definition.find(
      "config_.calibration.mapping_imu_to_lidar_lever_arm.x()"),
      std::string::npos);
  EXPECT_EQ(imu2_lidar_definition.find(
      "gtsam::Point3(-config_.calibration.mapping_imu_to_lidar_lever_arm"),
      std::string::npos);

  // The canonical transform is stored un-inverted by the shared loader; the
  // inverse is derived from it rather than configured separately.
  EXPECT_NE(loader.find("ParseLidarImuCalibration("), std::string::npos);
  EXPECT_NE(loader.find(
      "config.calibration.imu_T_lidar = parsed.imu_T_lidar;"),
      std::string::npos);
  EXPECT_NE(loader.find(
      "config.calibration.lidar_T_imu = parsed.lidar_T_imu;"),
      std::string::npos);
  EXPECT_EQ(loader.find(
      "config.calibration.imu_T_lidar = parsed.imu_T_lidar.Inverse();"),
      std::string::npos);
}

TEST(LidarImuCalibration, PlanDocumentsMappingLeverArmDirection)
{
  const std::string plan = ReadProjectFile(
      "docs/superpowers/plans/2026-07-24-unified-ros-parameters.md");

  EXPECT_NE(plan.find("mapping_imu_to_lidar_lever_arm"), std::string::npos);
  EXPECT_NE(plan.find("R^{lidar}_{imu} * t^{imu}_{lidar}"),
      std::string::npos);
}

TEST(LidarImuCalibration, LaunchStaticTransformsMatchCanonicalCalibration)
{
  const Eigen::Vector3d mapping_translation = ReadCalibrationTranslation(
    ReadProjectFile("config/mapping.yaml"));
  const Eigen::Vector3d relocalization_translation = ReadCalibrationTranslation(
    ReadProjectFile("config/relocalization.yaml"));
  ASSERT_TRUE(mapping_translation.isApprox(relocalization_translation));

  const std::array<std::pair<std::string, std::string>, 2> launch_files = {{
    {"launch/localization.launch.py", "true"},
    {"launch/localization_with_nonground.launch.py", "true"},
  }};
  for (const auto & launch_file : launch_files) {
    const std::string launch_text = ReadProjectFile(launch_file.first);
    const StaticTransform base_to_lidar =
      ReadStaticTransform(launch_text, "hesai_lidar");
    const StaticTransform base_to_imu =
      ReadStaticTransform(launch_text, "imu_link");

    Eigen::Isometry3d base_lidar = Eigen::Isometry3d::Identity();
    base_lidar.linear() = base_to_lidar.rotation;
    base_lidar.translation() = base_to_lidar.translation;
    Eigen::Isometry3d base_imu = Eigen::Isometry3d::Identity();
    base_imu.linear() = base_to_imu.rotation;
    base_imu.translation() = base_to_imu.translation;
    const Eigen::Isometry3d imu_lidar = base_imu.inverse() * base_lidar;

    EXPECT_TRUE(
      base_lidar.translation().isApprox(
        Eigen::Vector3d(0.02, 0.0, 0.32))) << launch_file.first;
    EXPECT_TRUE(imu_lidar.translation().isApprox(mapping_translation))
      << launch_file.first;
    EXPECT_TRUE(imu_lidar.linear().isApprox(Eigen::Matrix3d::Identity()))
      << launch_file.first;
    EXPECT_EQ(
      ReadPublishStaticTfDefault(launch_text), launch_file.second)
      << launch_file.first;
  }
}

TEST(LidarImuCalibration, RejectsInvalidTranslationLength)
{
  const std::vector<double> short_translation(2, 0.0);
  const std::vector<double> long_translation(4, 0.0);
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

  EXPECT_THROW(
      ParseLidarImuCalibration(short_translation, identity, identity),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(long_translation, identity, identity),
      std::invalid_argument);
}

TEST(LidarImuCalibration, RejectsInvalidRotationLength)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> short_rotation(8, 0.0);
  const std::vector<double> long_rotation(10, 0.0);
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};

  EXPECT_THROW(
      ParseLidarImuCalibration(translation, short_rotation, identity),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(translation, long_rotation, identity),
      std::invalid_argument);
}

TEST(LidarImuCalibration, RejectsInvalidRpyLength)
{
  const std::vector<double> translation = {0.0, 0.0, 0.10};
  const std::vector<double> identity = {
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0};
  const std::vector<double> short_rpy(8, 0.0);
  const std::vector<double> long_rpy(10, 0.0);

  EXPECT_THROW(
      ParseLidarImuCalibration(translation, identity, short_rpy),
      std::invalid_argument);
  EXPECT_THROW(
      ParseLidarImuCalibration(translation, identity, long_rpy),
      std::invalid_argument);
}
