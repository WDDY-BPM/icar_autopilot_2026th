#include "ctrl/perception_geometry_builder.hpp"

#include <algorithm>
#include <cmath>

namespace
{
std::vector<PointX> dualCenter(const std::vector<PointX> &left,
                               const std::vector<PointX> &right)
{
    std::vector<PointX> center;
    std::size_t leftIndex = 0;
    std::size_t rightIndex = 0;
    while (leftIndex < left.size() && rightIndex < right.size())
    {
        if (left[leftIndex].x == right[rightIndex].x)
        {
            center.emplace_back(left[leftIndex].x,
                                (left[leftIndex].y + right[rightIndex].y) / 2);
            ++leftIndex;
            ++rightIndex;
        }
        else if (left[leftIndex].x < right[rightIndex].x)
            ++leftIndex;
        else
            ++rightIndex;
    }
    return center;
}

std::vector<PointX> singleCenter(const std::vector<PointX> &edge, int side,
                                 float width)
{
    std::vector<PointX> center;
    const int offset = side < 0 ? static_cast<int>(width / 2.0f)
                                : -static_cast<int>(width / 2.0f);
    center.reserve(edge.size());
    for (const auto &point : edge)
        center.emplace_back(point.x, point.y + offset);
    return center;
}
}

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const LaneWidthModel &laneWidthModel,
    const Config &config)
{
    PerceptionGeometryResult result;
    const auto &quality = track.quality;
    const float width = laneWidthModel.ready ? laneWidthModel.nominalWidth : 96.0f;
    if (quality.leftReliable && quality.rightReliable)
    {
        result.centerLine = dualCenter(track.pointsEdgeLeft, track.pointsEdgeRight);
        result.recoveryMode = quality.valid
            ? PerceptionRecoveryKind::STRICT_DUAL
            : PerceptionRecoveryKind::RELAXED_DUAL;
    }
    else if (quality.leftSingleUsable)
    {
        result.centerLine = singleCenter(track.pointsEdgeLeft, -1, width);
        result.singleSide = -1;
        result.recoveryMode = PerceptionRecoveryKind::LEFT_SINGLE;
    }
    else if (quality.rightSingleUsable)
    {
        result.centerLine = singleCenter(track.pointsEdgeRight, 1, width);
        result.singleSide = 1;
        result.recoveryMode = PerceptionRecoveryKind::RIGHT_SINGLE;
    }
    else if (!track.pointsEdgeLeft.empty() && !track.pointsEdgeRight.empty())
    {
        result.centerLine = dualCenter(track.pointsEdgeLeft, track.pointsEdgeRight);
        result.recoveryMode = PerceptionRecoveryKind::WEAK_HYBRID;
    }
    result.nearSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x >= 176 && point.x <= 220; }));
    result.farSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x < 176; }));
    result.candidateValid = result.centerLine.size() >=
        static_cast<std::size_t>(std::max(8, config.singleLaneInteriorPointsMin)) &&
        result.nearSamples >= 4;
    return result;
}
