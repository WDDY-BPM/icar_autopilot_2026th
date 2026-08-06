#pragma once

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

PlannedGeometryResult buildPlannedGeometry(const PathOverride &path,
                                           FsmMode mode);
