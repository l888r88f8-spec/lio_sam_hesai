#include "relocalization/ground_map_source.h"

#include <gtest/gtest.h>

namespace {

PCLPointCloudXYZI::Ptr MakeCloud(float z) {
  PCLPointCloudXYZI::Ptr cloud(new PCLPointCloudXYZI);
  PCLPointXYZI point;
  point.z = z;
  cloud->push_back(point);
  return cloud;
}

pcl::KdTreeFLANN<PCLPointXYZI>::Ptr MakeTree(
    const PCLPointCloudXYZI::Ptr& cloud) {
  auto tree = pcl::KdTreeFLANN<PCLPointXYZI>::Ptr(
      new pcl::KdTreeFLANN<PCLPointXYZI>());
  tree->setInputCloud(cloud);
  return tree;
}

}  // namespace

TEST(GroundMapSource, PrefersDedicatedCloudWithRealHeight) {
  const auto global_cloud = MakeCloud(7.0F);
  const auto global_tree = MakeTree(global_cloud);
  const auto ground_cloud = MakeCloud(1.25F);
  const auto flattened_ground = MakeCloud(0.0F);
  const auto ground_tree = MakeTree(flattened_ground);

  const auto source = SelectGroundMapSource(
      global_cloud, global_tree, ground_cloud, ground_tree);

  ASSERT_TRUE(source.has_value());
  EXPECT_EQ(source->height_cloud.get(), ground_cloud.get());
  EXPECT_EQ(source->xy_kdtree.get(), ground_tree.get());
  EXPECT_FLOAT_EQ(source->height_cloud->front().z, 1.25F);
}

TEST(GroundMapSource, FallsBackToGlobalMap) {
  const auto global_cloud = MakeCloud(7.0F);
  const auto global_tree = MakeTree(global_cloud);

  const auto source = SelectGroundMapSource(
      global_cloud, global_tree, nullptr, nullptr);

  ASSERT_TRUE(source.has_value());
  EXPECT_EQ(source->height_cloud.get(), global_cloud.get());
  EXPECT_EQ(source->xy_kdtree.get(), global_tree.get());
}
