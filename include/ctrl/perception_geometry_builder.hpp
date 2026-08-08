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

// 单边重建中心线不能与真实双边中心同等信任：横向P/D按
// singleLaneHeadingConfidence缩放，航向保持原强度；其余模式不缩放。
inline float lateralScaleForMode(LaneRecoveryMode mode,
                                 const Config &config)
{
    if (mode == LaneRecoveryMode::LEFT_SINGLE ||
        mode == LaneRecoveryMode::RIGHT_SINGLE)
        return config.singleLaneHeadingConfidence;
    return 1.0f;
}

// oppositeSideRecovery 的 0.25 额外衰减只用于真实双边几何
// （STRICT_DUAL/RELAXED_DUAL）；单边与WEAK_HYBRID的跨中心是合法
// 退化几何，不应再被额外削弱航向。
inline bool recoveryDampingEnabledForMode(LaneRecoveryMode mode)
{
    return mode == LaneRecoveryMode::STRICT_DUAL ||
           mode == LaneRecoveryMode::RELAXED_DUAL;
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
