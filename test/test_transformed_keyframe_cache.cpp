#include "common/transformed_keyframe_cache.h"

#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <type_traits>
#include <utility>
#include <vector>

namespace {

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;
using Cache = lio_sam_hesai::TransformedKeyframeCache<PointT>;

TEST(TransformedKeyframeCache, CannotCopyIteratorBackedCache) {
  EXPECT_FALSE(std::is_copy_constructible<Cache>::value);
  EXPECT_FALSE(std::is_copy_assignable<Cache>::value);
}

Cloud MakeCloud(std::size_t size, float offset) {
  Cloud cloud;
  cloud.points.reserve(size);
  for (std::size_t i = 0; i < size; ++i) {
    PointT point;
    point.x = offset + static_cast<float>(i);
    point.y = offset - static_cast<float>(i);
    point.z = 0.5F * static_cast<float>(i);
    point.intensity = offset;
    cloud.points.push_back(point);
  }
  cloud.width = static_cast<std::uint32_t>(cloud.points.size());
  cloud.height = 1;
  return cloud;
}

std::size_t PairBytes(const Cloud& corner, const Cloud& surf) {
  return Cache::BufferBytes(corner) + Cache::BufferBytes(surf);
}

Cloud TransformLikeMap(const Cloud& cloud, const Eigen::Affine3f& transform) {
  Cloud transformed;
  transformed.points.resize(cloud.points.size());
  for (std::size_t i = 0; i < cloud.points.size(); ++i) {
    const PointT& from = cloud.points[i];
    PointT& to = transformed.points[i];
    to.x = transform(0, 0) * from.x + transform(0, 1) * from.y +
           transform(0, 2) * from.z + transform(0, 3);
    to.y = transform(1, 0) * from.x + transform(1, 1) * from.y +
           transform(1, 2) * from.z + transform(1, 3);
    to.z = transform(2, 0) * from.x + transform(2, 1) * from.y +
           transform(2, 2) * from.z + transform(2, 3);
    to.intensity = from.intensity;
  }
  return transformed;
}

TEST(TransformedKeyframeCache, HitRefreshesLeastRecentlyUsedOrder) {
  const Cloud corner0 = MakeCloud(2, 0.0F);
  const Cloud surf0 = MakeCloud(2, 10.0F);
  const Cloud corner1 = MakeCloud(2, 1.0F);
  const Cloud surf1 = MakeCloud(2, 11.0F);
  const Cloud corner2 = MakeCloud(2, 2.0F);
  const Cloud surf2 = MakeCloud(2, 12.0F);
  Cache cache(PairBytes(corner0, surf0) + PairBytes(corner1, surf1));

  ASSERT_TRUE(cache.Insert(0, corner0, surf0));
  ASSERT_TRUE(cache.Insert(1, corner1, surf1));
  ASSERT_NE(cache.Find(0), nullptr);
  ASSERT_TRUE(cache.Insert(2, corner2, surf2));

  EXPECT_EQ(cache.size(), 2U);
  EXPECT_NE(cache.Find(0), nullptr);
  EXPECT_EQ(cache.Find(1), nullptr);
  EXPECT_NE(cache.Find(2), nullptr);
}

TEST(TransformedKeyframeCache, ReplacementUpdatesBytesAndRecency) {
  const Cloud corner0 = MakeCloud(4, 0.0F);
  const Cloud surf0 = MakeCloud(4, 10.0F);
  const Cloud corner1 = MakeCloud(4, 1.0F);
  const Cloud surf1 = MakeCloud(4, 11.0F);
  const Cloud replacement_corner = MakeCloud(2, 20.0F);
  const Cloud replacement_surf = MakeCloud(2, 30.0F);
  const Cloud corner2 = MakeCloud(4, 2.0F);
  const Cloud surf2 = MakeCloud(4, 12.0F);
  Cache cache(PairBytes(corner0, surf0) + PairBytes(corner1, surf1));

  ASSERT_TRUE(cache.Insert(0, corner0, surf0));
  ASSERT_TRUE(cache.Insert(1, corner1, surf1));
  ASSERT_TRUE(cache.Insert(0, replacement_corner, replacement_surf));
  EXPECT_EQ(cache.bytes(),
            PairBytes(replacement_corner, replacement_surf) +
                PairBytes(corner1, surf1));

  ASSERT_TRUE(cache.Insert(2, corner2, surf2));
  EXPECT_EQ(cache.size(), 2U);
  ASSERT_NE(cache.Find(0), nullptr);
  EXPECT_EQ(cache.Find(1), nullptr);
  ASSERT_NE(cache.Find(2), nullptr);
  ASSERT_EQ(cache.Find(0)->corner.size(), replacement_corner.size());
  EXPECT_FLOAT_EQ(cache.Find(0)->corner.points.front().x, 20.0F);
  EXPECT_EQ(cache.bytes(),
            PairBytes(replacement_corner, replacement_surf) +
                PairBytes(corner2, surf2));
}

TEST(TransformedKeyframeCache, ClearReleasesRetainedBytes) {
  const Cloud corner = MakeCloud(3, 0.0F);
  const Cloud surf = MakeCloud(3, 10.0F);
  Cache cache(PairBytes(corner, surf));

  ASSERT_TRUE(cache.Insert(7, corner, surf));
  ASSERT_GT(cache.bytes(), 0U);
  cache.Clear();

  EXPECT_EQ(cache.size(), 0U);
  EXPECT_EQ(cache.bytes(), 0U);
  EXPECT_EQ(cache.Find(7), nullptr);
}

TEST(TransformedKeyframeCache, ZeroBudgetDisablesRetention) {
  const Cloud corner = MakeCloud(3, 0.0F);
  const Cloud surf = MakeCloud(3, 10.0F);
  Cache cache(0);

  EXPECT_FALSE(cache.Insert(0, corner, surf));
  EXPECT_EQ(cache.size(), 0U);
  EXPECT_EQ(cache.bytes(), 0U);
  EXPECT_EQ(cache.Find(0), nullptr);
}

TEST(TransformedKeyframeCache, OversizedEntryBypassesRetentionWithoutEviction) {
  const Cloud corner0 = MakeCloud(2, 0.0F);
  const Cloud surf0 = MakeCloud(2, 10.0F);
  const Cloud corner1 = MakeCloud(5, 1.0F);
  const Cloud surf1 = MakeCloud(5, 11.0F);
  const std::size_t budget = PairBytes(corner0, surf0);
  Cache cache(budget);

  ASSERT_TRUE(cache.Insert(0, corner0, surf0));
  EXPECT_FALSE(cache.Insert(1, corner1, surf1));
  EXPECT_EQ(cache.size(), 1U);
  EXPECT_EQ(cache.bytes(), budget);
  EXPECT_NE(cache.Find(0), nullptr);
  EXPECT_EQ(cache.Find(1), nullptr);
}

TEST(TransformedKeyframeCache, AccountsForPointBufferCapacityNotSize) {
  Cloud sparse = MakeCloud(1, 0.0F);
  sparse.points.reserve(64);
  const std::size_t expected_corner_bytes =
      sparse.points.capacity() * sizeof(PointT);
  ASSERT_GT(sparse.points.capacity(), sparse.points.size());
  Cache cache(expected_corner_bytes);

  ASSERT_TRUE(cache.Insert(3, std::move(sparse), Cloud()));
  EXPECT_EQ(cache.bytes(), expected_corner_bytes);
  ASSERT_TRUE(cache.Insert(4, MakeCloud(1, 1.0F), Cloud()));
  EXPECT_EQ(cache.bytes(), sizeof(PointT));
  EXPECT_EQ(cache.Find(3), nullptr);
}

TEST(TransformedKeyframeCache, EvictionThenRegenerationMatchesMapCoordinates) {
  const Cloud raw_corner = MakeCloud(8, 0.0F);
  const Cloud raw_surf = MakeCloud(8, 20.0F);
  const Eigen::Affine3f pose =
      pcl::getTransformation(1.5F, -2.0F, 3.0F, 0.1F, -0.2F, 0.3F);
  const Cloud expected_corner = TransformLikeMap(raw_corner, pose);
  const Cloud expected_surf = TransformLikeMap(raw_surf, pose);
  const Cloud other_corner = MakeCloud(8, 5.0F);
  const Cloud other_surf = MakeCloud(8, 25.0F);
  Cache cache(PairBytes(expected_corner, expected_surf));

  ASSERT_TRUE(cache.Insert(0, expected_corner, expected_surf));
  ASSERT_TRUE(cache.Insert(1, other_corner, other_surf));
  ASSERT_EQ(cache.Find(0), nullptr);

  const Cloud regenerated_corner = TransformLikeMap(raw_corner, pose);
  const Cloud regenerated_surf = TransformLikeMap(raw_surf, pose);
  ASSERT_EQ(regenerated_corner.size(), expected_corner.size());
  ASSERT_EQ(regenerated_surf.size(), expected_surf.size());
  for (std::size_t i = 0; i < expected_corner.size(); ++i) {
    EXPECT_FLOAT_EQ(regenerated_corner.points[i].x,
                    expected_corner.points[i].x);
    EXPECT_FLOAT_EQ(regenerated_corner.points[i].y,
                    expected_corner.points[i].y);
    EXPECT_FLOAT_EQ(regenerated_corner.points[i].z,
                    expected_corner.points[i].z);
    EXPECT_FLOAT_EQ(regenerated_corner.points[i].intensity,
                    expected_corner.points[i].intensity);
  }
  for (std::size_t i = 0; i < expected_surf.size(); ++i) {
    EXPECT_FLOAT_EQ(regenerated_surf.points[i].x, expected_surf.points[i].x);
    EXPECT_FLOAT_EQ(regenerated_surf.points[i].y, expected_surf.points[i].y);
    EXPECT_FLOAT_EQ(regenerated_surf.points[i].z, expected_surf.points[i].z);
    EXPECT_FLOAT_EQ(regenerated_surf.points[i].intensity,
                    expected_surf.points[i].intensity);
  }
}

}  // namespace
