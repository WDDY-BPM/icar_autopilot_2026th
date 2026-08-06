#pragma once

#include <vector>
#include "runtime/path_override.hpp"

enum class ControlGeometrySource
{
    NONE,
    PERCEPTION,
    PLANNED
};

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
    std::vector<PointX> centerLine;
    bool updated{false};
    bool valid{false};

    void reset()
    {
        source = ControlGeometrySource::NONE;
        pathSource = PathSource::NONE;
        centerLine.clear();
        updated = false;
        valid = false;
    }
};
