#pragma once

#include <cstddef>
#include "runtime/path_override.hpp"

enum class ControlGeometrySource
{
    NONE,
    PERCEPTION,
    PLANNED
};

enum class GeometryPolicy
{
    PERCEPTION_ALLOWED,
    PLANNED_REQUIRED,
    STOPPED
};

inline ControlGeometrySource selectGeometrySource(
    GeometryPolicy policy, bool plannedAvailable)
{
    if (policy == GeometryPolicy::STOPPED)
        return ControlGeometrySource::NONE;
    if (plannedAvailable)
        return ControlGeometrySource::PLANNED;
    return policy == GeometryPolicy::PERCEPTION_ALLOWED
        ? ControlGeometrySource::PERCEPTION
        : ControlGeometrySource::NONE;
}

inline const char *controlGeometrySourceName(ControlGeometrySource source)
{
    switch (source)
    {
    case ControlGeometrySource::PERCEPTION: return "PERCEPTION";
    case ControlGeometrySource::PLANNED: return "PLANNED";
    default: return "NONE";
    }
}

struct ControlGeometry
{
    ControlGeometrySource source{ControlGeometrySource::NONE};
    PathSource pathSource{PathSource::NONE};
    bool updated{false};
    bool valid{false};
    std::size_t pointCount{0};

    void reset()
    {
        source = ControlGeometrySource::NONE;
        pathSource = PathSource::NONE;
        updated = false;
        valid = false;
        pointCount = 0;
    }
};
