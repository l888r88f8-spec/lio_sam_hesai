#include "lio_sam/current_ground_segmenter.h"
#include "lio_sam/ground_segmentation_mode.h"
#include <gtest/gtest.h>
#include <stdexcept>

using namespace lio_sam_hesai;
namespace {
pcl::PointCloud<pcl::PointXYZI> Plane() {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int row=0;row<4;++row) for(int column=0;column<10;++column) {
    pcl::PointXYZI point; point.x=5.0F+row; point.y=0.3F*column;
    point.z=-1.0F;point.intensity=10*row+column;cloud.push_back(point);
  }
  return cloud;
}
GroundSegmentationConfig Config() {
  GroundSegmentationConfig config;config.scan_start_index=0;
  config.scan_end_index=1;config.plane_min_points=3;return config;
}
}
TEST(GroundStageSelection, ExactMatrixAndCurrentPriority) {
  for (const std::string current : {"off","online","export"})
    for (const std::string patchwork : {"off","online","export"})
      for (auto stage : {GroundStage::Online,GroundStage::Export}) {
        const std::string name=stage==GroundStage::Online?"online":"export";
        auto expected=current==name?GroundAlgorithm::Current:
            patchwork==name?GroundAlgorithm::Patchworkpp:GroundAlgorithm::None;
        EXPECT_EQ(SelectGroundAlgorithm(current,patchwork,stage),expected)
            << current << " " << patchwork << " " << name;
      }
}
TEST(CurrentGroundSegmenter, PlaneClassificationKeepsOriginalOutputRingPolicy) {
  auto cloud=Plane();auto result=SegmentCurrentGround(Config(),cloud,
      std::vector<std::uint8_t>(40,1),4,10);
  ASSERT_EQ(result.classification.size(),40U);
  for(auto label:result.classification) EXPECT_EQ(label,1);
  ASSERT_EQ(result.ground.size(),20U);
  // Legacy plane path visits each column, then the configured seed rows.
  EXPECT_FLOAT_EQ(result.ground[0].intensity,0);
  EXPECT_FLOAT_EQ(result.ground[1].intensity,10);
  EXPECT_FLOAT_EQ(result.ground[2].intensity,1);
  for(const auto& point:result.ground) EXPECT_LT(point.intensity,20);
}
TEST(CurrentGroundSegmenter, MissingCellsAndElevatedPointsStopConnectivity) {
  auto cloud=Plane();std::vector<std::uint8_t> valid(40,1);
  valid[23]=0;cloud[25].z=1;
  auto result=SegmentCurrentGround(Config(),cloud,valid,4,10);
  ASSERT_EQ(result.classification.size(),40U);
  EXPECT_NE(result.classification[33],1);
  EXPECT_NE(result.classification[25],1);
  EXPECT_NE(result.classification[35],1);
  EXPECT_EQ(result.classification[34],1);
}
TEST(CurrentGroundSegmenter, PlaneFitFailureKeepsMeasuredSeedFallbackAndOrder) {
  auto config=Config();config.plane_min_points=100000;
  auto result=SegmentCurrentGround(config,Plane(),std::vector<std::uint8_t>(40,1),4,10);
  ASSERT_EQ(result.ground.size(),20U);
  for(std::size_t index=0;index<20;++index)
    EXPECT_FLOAT_EQ(result.ground[index].intensity,index);
}
TEST(CurrentGroundSegmenter, CompactProjectionExactlyReplaysOrganization) {
  auto cloud=Plane();pcl::PointCloud<pcl::PointXYZI> compact;
  std::vector<std::uint32_t> indices;
  for (std::size_t index=0;index<cloud.size();++index) if(index!=23) {
    compact.push_back(cloud[index]);indices.push_back(index);
  }
  auto result=SegmentProjectedGround(Config(),compact,indices,4,10);
  EXPECT_EQ(result.ground.size(),20U);
  ASSERT_EQ(result.classification.size(),40U);
  EXPECT_NE(result.classification[33],1);
  EXPECT_THROW(SegmentProjectedGround(Config(),compact,{},4,10),std::invalid_argument);
  auto duplicate=indices;duplicate[1]=duplicate[0];
  EXPECT_THROW(SegmentProjectedGround(Config(),compact,duplicate,4,10),std::invalid_argument);
  indices[1]=40;
  EXPECT_THROW(SegmentProjectedGround(Config(),compact,indices,4,10),std::invalid_argument);
  EXPECT_THROW(SegmentProjectedGround(Config(),{}, {},-1,10),std::invalid_argument);
}

#include "lio_sam/ground_projection_cache.h"
#include <filesystem>
#include <fstream>
#include <unistd.h>
TEST(GroundProjectionCache, RoundTripAndRejectsMissingCorruptOrMismatchedInput) {
  const auto path=std::filesystem::temp_directory_path()/
      ("lio_sam_projection_"+std::to_string(::getpid())+" ; '.bin");
  struct Cleanup { std::filesystem::path path; ~Cleanup(){
    std::error_code error;std::filesystem::remove(path,error);} } cleanup{path};
  const std::vector<std::uint32_t> indices{0,3,11};
  EXPECT_THROW(ReadGroundProjectionFile(path,3,4,10),std::runtime_error);
  WriteGroundProjectionFile(path,3,indices,4,10);
  EXPECT_EQ(ReadGroundProjectionFile(path,3,4,10),indices);
  EXPECT_THROW(ReadGroundProjectionFile(path,4,4,10),std::runtime_error);
  EXPECT_THROW(ReadGroundProjectionFile(path,3,5,10),std::runtime_error);
  EXPECT_THROW(WriteGroundProjectionFile(path,3,{0,0,1},4,10),std::invalid_argument);
  EXPECT_THROW(WriteGroundProjectionFile(path,3,{0,1,40},4,10),std::invalid_argument);
  EXPECT_THROW(WriteGroundProjectionFile(path,3,indices,2147483647,2147483647),std::invalid_argument);
  { std::ofstream corrupt(path,std::ios::app|std::ios::binary);corrupt<<'x'; }
  EXPECT_THROW(ReadGroundProjectionFile(path,3,4,10),std::runtime_error);
}
