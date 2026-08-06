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
        center.emplace_back(point.x, std::clamp(point.y + offset, 0, 319));
    return center;
}
}

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const PlannedLaneWidthModel &laneWidthModel,
    const Config &config)
{
    PerceptionGeometryResult result;
    const auto &quality = track.quality;
    const float width = laneWidthModel.ready ? laneWidthModel.fallbackWidth : 96.0f;
    if (quality.leftReliable && quality.rightReliable)
    {
        result.centerLine = dualCenter(track.pointsEdgeLeft, track.pointsEdgeRight);
        result.recoveryMode = quality.valid
            ? LaneRecoveryMode::STRICT_DUAL
            : LaneRecoveryMode::RELAXED_DUAL;
        result.widthConsistent = quality.widthVariation <= 0.30f;
    }
    else if (quality.leftSingleUsable)
    {
        result.centerLine = singleCenter(track.pointsEdgeLeft, -1, width);
        result.singleSide = -1;
        result.recoveryMode = LaneRecoveryMode::LEFT_SINGLE;
    }
    else if (quality.rightSingleUsable)
    {
        result.centerLine = singleCenter(track.pointsEdgeRight, 1, width);
        result.singleSide = 1;
        result.recoveryMode = LaneRecoveryMode::RIGHT_SINGLE;
    }
    else if (!track.pointsEdgeLeft.empty() && !track.pointsEdgeRight.empty())
    {
        result.centerLine = dualCenter(track.pointsEdgeLeft, track.pointsEdgeRight);
        result.recoveryMode = LaneRecoveryMode::WEAK_HYBRID;
    }
    result.nearSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x >= 176 && point.x <= 220; }));
    result.farSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x < 176; }));
    const bool continuous = std::adjacent_find(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &left, const PointX &right) {
            return std::abs(left.y - right.y) > 45;
        }) == result.centerLine.end();
    result.candidateValid = result.centerLine.size() >=
        static_cast<std::size_t>(std::max(8, config.singleLaneInteriorPointsMin)) &&
        result.nearSamples >= 4 && continuous &&
        (result.recoveryMode != LaneRecoveryMode::STRICT_DUAL ||
         result.widthConsistent);
    return result;
}
