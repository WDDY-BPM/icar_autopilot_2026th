#pragma once

#include <array>
#include <vector>
#include "runtime/path_override.hpp"
#include "runtime/planned_path_validation.hpp"

struct PlannedGeometryResult
{
    std::vector<PointX> centerLine;
    PathSource source{PathSource::NONE};
    PlannedPathValidation validation;
    bool valid{false};
};

struct PlannedLaneWidthModel
{
    bool ready{false};
    std::array<float, 240> widthByRow{};
    float fallbackWidth{96.0f};
};

PlannedGeometryResult buildPlannedGeometry(const PathOverride &path,
                                           FsmMode mode,
                                           const PlannedLaneWidthModel &widthModel = {});
