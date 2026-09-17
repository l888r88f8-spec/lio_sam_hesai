#pragma once

namespace lio_sam_hesai
{

// Negative service values ask the mapping node to use its active parameters.
// Zero retains the existing meaning of no additional downsampling.
inline float ResolveMapSaveResolution(float requested, double configured)
{
  return requested < 0.0F ? static_cast<float>(configured) : requested;
}

}  // namespace lio_sam_hesai
