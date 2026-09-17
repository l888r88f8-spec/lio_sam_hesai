#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <QApplication>
#include <QByteArray>
#include <QtGlobal>
#include <class_loader/class_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rviz_common/panel.hpp>

namespace
{

constexpr char kPanelLibrary[] = SAVE_MAP_PANEL_LIBRARY;
constexpr char kPluginXml[] = SAVE_MAP_PANEL_PLUGIN_XML;
// The registered C++ type; plugin_description.xml maps it to the RViz plugin
// name lio_sam_hesai/SaveMapPanel.
constexpr char kPanelClass[] = "lio_sam_hesai::rviz::SaveMapPanel";
constexpr char kPluginName[] = "lio_sam_hesai/SaveMapPanel";
// RViz looks panels up with pluginlib using the package that exports the
// rviz_common::Panel base class. The same lookup is reproduced here so a wrong
// base class package (for example the ROS 1 "rviz" package) fails at test time
// instead of at RViz startup.
constexpr char kBaseClassPackage[] = "rviz_common";
constexpr char kBaseClass[] = "rviz_common::Panel";
constexpr char kPackageName[] = "lio_sam_hesai";
constexpr char kPluginXmlResource[] = "share/lio_sam_hesai/plugin_description.xml";
constexpr char kPluginIndexResource[] = "rviz_common__pluginlib__plugin";

// RViz discovers the panel through pluginlib, which loads the built library and
// instantiates the class registered by PLUGINLIB_EXPORT_CLASS. Doing the same
// here catches a wrong class name, a missing registration, or an unresolved
// symbol at test time instead of at RViz startup.
class SaveMapPanelPluginTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
    static char application_name[] = "test_save_map_panel_plugin";
    static int argc = 1;
    static char* argv[] = {application_name, nullptr};
    application_ = new QApplication(argc, argv);
  }

  static void TearDownTestSuite()
  {
    delete application_;
    application_ = nullptr;
    std::error_code error;
    std::filesystem::remove_all(fixture_prefix_, error);
  }

  static void WriteTextFile(
    const std::filesystem::path& path, const std::string& text)
  {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    ASSERT_TRUE(stream.is_open()) << path;
    stream << text;
  }

  // Recreates the ament index layout that installing this package with
  // pluginlib_export_plugin_description_file(rviz_common ...) produces: the
  // package marker, the rviz_common__pluginlib__plugin resource pointing at the
  // plugin XML, and the plugin library under lib/. Pointing AMENT_PREFIX_PATH at
  // this prefix lets pluginlib resolve lio_sam_hesai/SaveMapPanel exactly like
  // an installed RViz session does.
  static std::filesystem::path CreateInstalledPrefixFixture()
  {
    const auto prefix =
      std::filesystem::temp_directory_path() /
      "lio_sam_hesai_save_map_panel_prefix";
    std::filesystem::remove_all(prefix);

    const auto package_share = prefix / "share" / kPackageName;
    std::filesystem::create_directories(package_share);
    // pluginlib reads <name> from the manifest next to the plugin XML to learn
    // which package exports the plugin.
    WriteTextFile(
      package_share / "package.xml",
      "<?xml version=\"1.0\"?>\n"
      "<package format=\"3\">\n"
      "  <name>lio_sam_hesai</name>\n"
      "  <version>0.0.0</version>\n"
      "</package>\n");
    std::filesystem::copy_file(
      kPluginXml, package_share / "plugin_description.xml",
      std::filesystem::copy_options::overwrite_existing);

    const auto index = prefix / "share" / "ament_index" / "resource_index";
    WriteTextFile(index / "packages" / kPackageName, "");
    WriteTextFile(
      index / kPluginIndexResource / kPackageName,
      std::string(kPluginXmlResource) + "\n");

    const auto library_dir = prefix / "lib";
    std::filesystem::create_directories(library_dir);
    std::filesystem::copy_file(
      kPanelLibrary,
      library_dir / std::filesystem::path(kPanelLibrary).filename(),
      std::filesystem::copy_options::overwrite_existing);
    return prefix;
  }

  static QApplication* application_;
  static std::filesystem::path fixture_prefix_;
};

QApplication* SaveMapPanelPluginTest::application_ = nullptr;
std::filesystem::path SaveMapPanelPluginTest::fixture_prefix_;

TEST_F(SaveMapPanelPluginTest, InstantiatesPanelFromBuiltLibrary)
{
  ASSERT_TRUE(std::filesystem::exists(kPanelLibrary)) << kPanelLibrary;

  class_loader::ClassLoader loader(kPanelLibrary);
  const auto available = loader.getAvailableClasses<rviz_common::Panel>();
  EXPECT_NE(
    std::find(available.begin(), available.end(), kPanelClass),
    available.end());

  std::shared_ptr<rviz_common::Panel> panel =
    loader.createInstance<rviz_common::Panel>(kPanelClass);
  ASSERT_NE(panel, nullptr);
  EXPECT_TRUE(loader.isLibraryLoaded());
}

TEST_F(SaveMapPanelPluginTest, LooksUpPanelThroughPluginlibWithRvizCommonBaseClass)
{
  ASSERT_TRUE(std::filesystem::exists(kPluginXml)) << kPluginXml;

  fixture_prefix_ = CreateInstalledPrefixFixture();
  std::string ament_prefix_path = fixture_prefix_.string();
  if (const char* existing = std::getenv("AMENT_PREFIX_PATH");
    existing != nullptr && *existing != '\0')
  {
    ament_prefix_path += ":";
    ament_prefix_path += existing;
  }
  ASSERT_EQ(setenv("AMENT_PREFIX_PATH", ament_prefix_path.c_str(), 1), 0);

  pluginlib::ClassLoader<rviz_common::Panel> loader(kBaseClassPackage, kBaseClass);
  const auto declared = loader.getDeclaredClasses();
  EXPECT_NE(
    std::find(declared.begin(), declared.end(), kPluginName), declared.end())
    << "pluginlib did not find " << kPluginName << " under base class package "
    << kBaseClassPackage;

  std::shared_ptr<rviz_common::Panel> panel =
    loader.createSharedInstance(kPluginName);
  ASSERT_NE(panel, nullptr)
    << "pluginlib failed to instantiate " << kPluginName << " from "
    << kPanelLibrary;
}

}  // namespace
