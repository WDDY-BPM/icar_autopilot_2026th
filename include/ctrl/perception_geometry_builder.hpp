#pragma once

#include <vector>
#include "config/config.hpp"
#include "ctrl/track.hpp"
#include "ctrl/planned_geometry_builder.hpp"

enum class LaneRecoveryMode
{
    INVALID,
    STRICT_DUAL,
    RELAXED_DUAL,
    WEAK_HYBRID,
    LEFT_SINGLE,
    RIGHT_SINGLE
};

// 启动车道决策：严格双边或稳定单边都算有效（复用LaneRecoveryMode，
// 不是FSM状态）。单边必须真实覆盖近场且内部点足够。
inline LaneRecoveryMode assessStartupLaneMode(
    const Track::LaneQuality &quality,
    bool leftCoversNear, bool rightCoversNear,
    const Config &config)
{
    if (quality.leftReliable && quality.rightReliable &&
        quality.coversBottom &&
        quality.commonRows >= control_algorithms::kStartupCommonRows)
        return LaneRecoveryMode::STRICT_DUAL;
    const bool leftSingleStart =
        quality.leftSingleUsable && leftCoversNear &&
        quality.leftInteriorPoints >= config.singleLaneInteriorPointsMin;
    if (leftSingleStart)
        return LaneRecoveryMode::LEFT_SINGLE;
    const bool rightSingleStart =
        quality.rightSingleUsable && rightCoversNear &&
        quality.rightInteriorPoints >= config.singleLaneInteriorPointsMin;
    if (rightSingleStart)
        return LaneRecoveryMode::RIGHT_SINGLE;
    return LaneRecoveryMode::INVALID;
}

struct PerceptionGeometryResult
{
    std::vector<PointX> centerLine;
    LaneRecoveryMode recoveryMode{LaneRecoveryMode::INVALID};
    int singleSide{0};
    int nearSamples{0};
    int farSamples{0};
    bool widthConsistent{false};
    bool geometryContinuous{false};
    bool candidateValid{false};
};

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const PlannedLaneWidthModel &laneWidthModel,
    const Config &config);
