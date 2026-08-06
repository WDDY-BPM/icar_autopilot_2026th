#pragma once

#include <vector>
#include "config/config.hpp"
#include "ctrl/track.hpp"

enum class PerceptionRecoveryKind
{
    INVALID,
    STRICT_DUAL,
    RELAXED_DUAL,
    WEAK_HYBRID,
    LEFT_SINGLE,
    RIGHT_SINGLE
};

struct LaneWidthModel
{
    bool ready{false};
    float nominalWidth{96.0f};
};

struct PerceptionGeometryResult
{
    std::vector<PointX> centerLine;
    PerceptionRecoveryKind recoveryMode{PerceptionRecoveryKind::INVALID};
    bool candidateValid{false};
    int singleSide{0};
    int nearSamples{0};
    int farSamples{0};
};

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const LaneWidthModel &laneWidthModel,
    const Config &config);
