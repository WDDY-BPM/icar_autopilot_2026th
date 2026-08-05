#pragma once

#include <vector>
#include "utils/point.hpp"

enum class PathSource { NONE, PARK, BUSY, FORK, YFORK, OBSTACLE };

inline const char *pathSourceName(PathSource source)
{
    switch (source)
    {
    case PathSource::PARK: return "PARK";
    case PathSource::BUSY: return "BUSY";
    case PathSource::FORK: return "FORK";
    case PathSource::YFORK: return "YFORK";
    case PathSource::OBSTACLE: return "OBSTACLE";
    default: return "TRACK";
    }
}

struct PathOverride
{
    bool active{false};
    PathSource source{PathSource::NONE};
    std::vector<PointX> leftEdge;
    std::vector<PointX> rightEdge;
    std::vector<PointX> centerLine;
    bool hasLeftEdge{false};
    bool hasRightEdge{false};
    bool hasCenterLine{false};
    float headingConfidence{0.0f};
    float speedLimit{-1.0f};

    void setEdges(PathSource owner, const std::vector<PointX> &left,
                  const std::vector<PointX> &right)
    {
        active = true;
        source = owner;
        leftEdge = left;
        rightEdge = right;
        centerLine.clear();
        hasLeftEdge = !leftEdge.empty();
        hasRightEdge = !rightEdge.empty();
        hasCenterLine = false;
    }

    void clear() { *this = PathOverride{}; }

    bool clear(PathSource owner)
    {
        if (!active || source != owner)
            return false;
        clear();
        return true;
    }
};

struct LaneInput
{
    const std::vector<PointX> *left{nullptr};
    const std::vector<PointX> *right{nullptr};
    const std::vector<PointX> *center{nullptr};
    PathSource source{PathSource::NONE};
    bool planned() const { return source != PathSource::NONE; }
};

inline LaneInput selectLaneInput(const std::vector<PointX> &trackLeft,
                                 const std::vector<PointX> &trackRight,
                                 const PathOverride &overridePath)
{
    if (overridePath.active && overridePath.hasCenterLine &&
        !overridePath.centerLine.empty())
        return {nullptr, nullptr, &overridePath.centerLine, overridePath.source};
    if (overridePath.active &&
        ((overridePath.hasLeftEdge && !overridePath.leftEdge.empty()) ||
         (overridePath.hasRightEdge && !overridePath.rightEdge.empty())))
        return {&overridePath.leftEdge, &overridePath.rightEdge, nullptr,
                overridePath.source};
    return {&trackLeft, &trackRight, nullptr, PathSource::NONE};
}
