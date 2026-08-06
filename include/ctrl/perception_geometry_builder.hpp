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

struct PerceptionGeometryResult
{
    std::vector<PointX> centerLine;
    LaneRecoveryMode recoveryMode{LaneRecoveryMode::INVALID};
    bool candidateValid{false};
    int singleSide{0};
    int nearSamples{0};
    int farSamples{0};
    bool widthConsistent{false};
};

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const PlannedLaneWidthModel &laneWidthModel,
    const Config &config);
