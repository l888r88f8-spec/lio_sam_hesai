#include "lio_sam/online_ground_pipeline.h"
#include "lio_sam/patchworkpp_ground_segmenter.h"
#include <memory>

namespace lio_sam_hesai {
OnlineGroundPipeline::OnlineGroundPipeline(const GroundSegmentationConfig& current,
    const PatchworkppConfig& patchwork,PatchworkFactory factory)
    :current_(current),algorithm_(SelectGroundAlgorithm(current.mode,patchwork.mode,GroundStage::Online)) {
  if(algorithm_!=GroundAlgorithm::Patchworkpp) return;
  if(factory) patchwork_=factory();
  else {
    auto backend=std::make_shared<PatchworkppGroundSegmenter>(patchwork);
    patchwork_=[backend](const auto& cloud){return backend->Segment(cloud);};
  }
  if(!patchwork_) throw std::invalid_argument("empty Patchwork++ online segmenter");
}
CurrentGroundResult OnlineGroundPipeline::Segment(
    const pcl::PointCloud<pcl::PointXYZI>& organized,
    const std::vector<std::uint8_t>& valid,int rows,int columns,
    const pcl::PointCloud<pcl::PointXYZI>& complete,
    const std::vector<int>& projection_cells) {
  if(algorithm_==GroundAlgorithm::Current)
    return SegmentCurrentGround(current_,organized,valid,rows,columns);
  CurrentGroundResult result;
  result.classification.assign(organized.size(),0);
  if(algorithm_==GroundAlgorithm::None) return result;
  if(complete.size()!=projection_cells.size())
    throw std::invalid_argument("Patchwork++ complete scan mapping size mismatch");
  for(const auto index:patchwork_(complete)) {
    if(index>=complete.size())
      throw std::runtime_error("Patchwork++ returned an invalid ground index");
    result.ground.push_back(complete[index]);
    const int cell=projection_cells[index];
    if(cell>=0) {
      if(static_cast<std::size_t>(cell)>=result.classification.size())
        throw std::invalid_argument("Patchwork++ projection cell out of range");
      result.classification[cell]=1;
    }
  }
  return result;
}
}
