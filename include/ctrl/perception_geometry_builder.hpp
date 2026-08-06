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

// 统一的恢复模式名称（STARTUP日志、Center调试画面、遥测共用）。
inline const char *laneRecoveryModeName(LaneRecoveryMode mode)
{
    switch (mode)
    {
    case LaneRecoveryMode::STRICT_DUAL: return "STRICT_DUAL";
    case LaneRecoveryMode::RELAXED_DUAL: return "RELAXED_DUAL";
    case LaneRecoveryMode::WEAK_HYBRID: return "WEAK_HYBRID";
    case LaneRecoveryMode::LEFT_SINGLE: return "LEFT_SINGLE";
    case LaneRecoveryMode::RIGHT_SINGLE: return "RIGHT_SINGLE";
    default: return "INVALID";
    }
}

// 唯一的单边方向选择：裁剪侧排除；两侧都可用时按可靠性与内部点数裁决，
// 禁止因代码顺序固定优先LEFT_SINGLE。
inline LaneRecoveryMode selectSingleLaneMode(
    const Track::LaneQuality &quality)
{
    const bool leftCandidate =
        quality.leftSingleUsable && !quality.leftClipped;
    const bool rightCandidate =
        quality.rightSingleUsable && !quality.rightClipped;
    if (leftCandidate && !rightCandidate)
        return LaneRecoveryMode::LEFT_SINGLE;
    if (rightCandidate && !leftCandidate)
        return LaneRecoveryMode::RIGHT_SINGLE;
    if (leftCandidate && rightCandidate)
    {
        if (quality.leftReliable && !quality.rightReliable)
            return LaneRecoveryMode::LEFT_SINGLE;
        if (quality.rightReliable && !quality.leftReliable)
            return LaneRecoveryMode::RIGHT_SINGLE;
        if (quality.leftInteriorPoints > quality.rightInteriorPoints)
            return LaneRecoveryMode::LEFT_SINGLE;
        if (quality.rightInteriorPoints > quality.leftInteriorPoints)
            return LaneRecoveryMode::RIGHT_SINGLE;
    }
    return LaneRecoveryMode::INVALID;
}

// 启动车道决策：严格双边或稳定单边都算有效（复用LaneRecoveryMode，
// 不是FSM状态）。单边方向与Center共用selectSingleLaneMode。
inline LaneRecoveryMode assessStartupLaneMode(
    const Track::LaneQuality &quality,
    bool leftCoversNear, bool rightCoversNear,
    const Config &config)
{
    if (quality.leftReliable && quality.rightReliable &&
        quality.coversBottom &&
        quality.commonRows >= control_algorithms::kStartupCommonRows)
        return LaneRecoveryMode::STRICT_DUAL;
    const auto singleMode = selectSingleLaneMode(quality);
    const bool leftSingleStart =
        singleMode == LaneRecoveryMode::LEFT_SINGLE && leftCoversNear &&
        quality.leftInteriorPoints >= config.singleLaneInteriorPointsMin;
    if (leftSingleStart)
        return LaneRecoveryMode::LEFT_SINGLE;
    const bool rightSingleStart =
        singleMode == LaneRecoveryMode::RIGHT_SINGLE && rightCoversNear &&
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
