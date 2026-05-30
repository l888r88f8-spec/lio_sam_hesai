#ifndef LIO_SAM_HESAI_GROUND_RING_POLICY_H_
#define LIO_SAM_HESAI_GROUND_RING_POLICY_H_

#include <algorithm>

namespace lio_sam_hesai
{

struct GroundRingRange
{
    int start;
    int end;
};

inline GroundRingRange normalizeGroundScanRange(
    int configuredStart, int configuredEnd, int scanCount)
{
    if (scanCount <= 0)
        return {0, -1};

    if (configuredEnd < 0)
        configuredEnd = scanCount - 1;

    int start = std::clamp(configuredStart, 0, scanCount - 1);
    int end = std::clamp(configuredEnd, 0, scanCount - 1);
    if (start > end)
        std::swap(start, end);

    return {start, end};
}

inline GroundRingRange fullGroundClassificationRange(int scanCount)
{
    if (scanCount <= 0)
        return {0, -1};
    return {0, scanCount - 1};
}

inline bool isGroundOutputRing(int ring, const GroundRingRange &range)
{
    return ring >= range.start && ring <= range.end;
}

}  // namespace lio_sam_hesai

#endif  // LIO_SAM_HESAI_GROUND_RING_POLICY_H_
