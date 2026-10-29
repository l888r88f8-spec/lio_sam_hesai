#pragma once
#include <string>

namespace lio_sam_hesai {
enum class GroundStage { Online, Export };
enum class GroundAlgorithm { None, Current, Patchworkpp };
inline GroundAlgorithm SelectGroundAlgorithm(const std::string& current,
                                            const std::string& patchwork,
                                            GroundStage stage) {
  const char* requested=stage==GroundStage::Online ? "online" : "export";
  if(current==requested) return GroundAlgorithm::Current;
  if(patchwork==requested) return GroundAlgorithm::Patchworkpp;
  return GroundAlgorithm::None;
}
}
