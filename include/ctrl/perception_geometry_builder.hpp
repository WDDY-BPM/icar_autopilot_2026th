#pragma once

#include <algorithm>
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

// 单边模式下横向P/D的动态权重及原因（用于遥测诊断）。
enum class LateralScaleReason
{
    FULL_DUAL = 0,
    SINGLE_PREVIEW = 1,
    SINGLE_RECOVERY = 2,
    SINGLE_NO_HEADING = 3,
    WEAK_HYBRID_CONFLICT = 4
};

struct LateralScaleResult
{
    float scale{1.0f};
    LateralScaleReason reason{LateralScaleReason::FULL_DUAL};
};

// 单边重建中心线在预瞄阶段（near/far 跨图像中心）保持低横向权重，
// 真正进入弯道（near/far 同侧）后随 |nearError| 恢复到 1.0；
// heading 失效时直接恢复 full lateral。仅影响 LEFT/RIGHT_SINGLE。
inline constexpr float kSingleLaneFullLateralError = 30.0f;
inline constexpr float kWeakHybridConflictScaleMin = 0.30f;

inline LateralScaleResult singleLaneLateralScale(
    LaneRecoveryMode mode, bool nearValid, bool farValid,
    float nearError, float farError, float headingConfidence,
    const Config &config)
{
    LateralScaleResult result;
    const bool singleMode =
        mode == LaneRecoveryMode::LEFT_SINGLE ||
        mode == LaneRecoveryMode::RIGHT_SINGLE;
    if (!singleMode)
        return result; // FULL_DUAL：lateral 不缩放
    const float baseScale = config.singleLaneHeadingConfidence;
    if (!nearValid)
    {
        // near 无效时不允许恢复逻辑产生异常 full-scale 控制。
        result.scale = baseScale;
        result.reason = LateralScaleReason::SINGLE_PREVIEW;
        return result;
    }
    if (!farValid || headingConfidence <= 0.0f)
    {
        // heading 已失效，唯一剩余恢复控制不能再被削弱。
        result.scale = 1.0f;
        result.reason = LateralScaleReason::SINGLE_NO_HEADING;
        return result;
    }
    const bool straddle =
        (nearError >= 0.0f && farError <= 0.0f) ||
        (nearError <= 0.0f && farError >= 0.0f);
    if (straddle)
    {
        result.scale = baseScale;
        result.reason = LateralScaleReason::SINGLE_PREVIEW;
        return result;
    }
    const float recoveryRatio = std::clamp(
        std::abs(nearError) / kSingleLaneFullLateralError, 0.0f, 1.0f);
    result.scale = baseScale + (1.0f - baseScale) * recoveryRatio;
    result.reason = LateralScaleReason::SINGLE_RECOVERY;
    return result;
}

// WEAK_HYBRID 弯道预瞄仲裁：near/far 跨中心且 lateral 与 heading 方向冲突
// 时，把控制权优先交给 heading（lateral 降为 headingConfidence 的钳制值）。
// 只有明确冲突场景才生效；其余情况返回 FULL_DUAL（不覆盖）。
inline LateralScaleResult weakHybridConflictLateralScale(
    LaneRecoveryMode mode, bool nearValid, bool farValid,
    float nearError, float farError, float headingConfidence,
    float lateralRaw, float headingApplied, const Config &config)
{
    LateralScaleResult result;
    if (mode != LaneRecoveryMode::WEAK_HYBRID)
        return result;
    if (!nearValid || !farValid || headingConfidence <= 0.0f)
        return result;
    const bool nearFarStraddle =
        (nearError > 0.0f && farError < 0.0f) ||
        (nearError < 0.0f && farError > 0.0f);
    if (!nearFarStraddle)
        return result;
    const bool lateralHeadingConflict =
        lateralRaw * headingApplied < 0.0f;
    if (!lateralHeadingConflict)
        return result;
    result.scale = std::clamp(
        headingConfidence, kWeakHybridConflictScaleMin, 1.0f);
    result.reason = LateralScaleReason::WEAK_HYBRID_CONFLICT;
    return result;
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
