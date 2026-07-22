#ifndef LIO_SAM_HESAI_RELOCALIZATION_ACCEPTANCE_POLICY_H
#define LIO_SAM_HESAI_RELOCALIZATION_ACCEPTANCE_POLICY_H

#include <cmath>

inline bool ShouldEarlyAcceptRelocalization(float fitness,
                                            double accept_threshold) {
  return accept_threshold > 0.0 && std::isfinite(fitness) &&
         static_cast<double>(fitness) < accept_threshold;
}

#endif  // LIO_SAM_HESAI_RELOCALIZATION_ACCEPTANCE_POLICY_H
