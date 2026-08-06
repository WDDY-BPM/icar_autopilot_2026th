#pragma once

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>
#include "runtime/fsm_mode.hpp"
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

inline bool pathSourceAllowed(PathSource source, FsmMode mode)
{
    switch (source)
    {
    case PathSource::PARK:
        return mode == FsmMode::PARK;
    case PathSource::BUSY:
        return mode == FsmMode::BUSY;
    case PathSource::FORK:
        return mode == FsmMode::FORK;
    case PathSource::YFORK:
        return mode == FsmMode::YFORK;
    case PathSource::OBSTACLE:
        return mode == FsmMode::NORMAL || mode == FsmMode::SLOW ||
               mode == FsmMode::CROSS || mode == FsmMode::STOP ||
               mode == FsmMode::STATION;
    default:
        return false;
    }
}

struct PathOverride
{
    PathSource source{PathSource::NONE};
    std::vector<PointX> leftEdge;
    std::vector<PointX> rightEdge;
    std::vector<PointX> centerLine;
    float headingConfidence{0.0f};
    float speedLimit{-1.0f};
    int ttlFrames{0};
    uint64_t generatedFrameId{0};

    void setEdges(PathSource owner, std::vector<PointX> left,
                  std::vector<PointX> right,
                  float newHeadingConfidence = 0.0f,
                  float newSpeedLimit = -1.0f,
                  int newTtlFrames = 1)
    {
        resetPayload();
        source = owner;
        leftEdge = std::move(left);
        rightEdge = std::move(right);
        headingConfidence = newHeadingConfidence;
        speedLimit = newSpeedLimit;
        ttlFrames = std::max(1, newTtlFrames);
        generatedFrameId = observedFrameId;
    }

    void setCenterLine(PathSource owner, std::vector<PointX> center,
                       float newHeadingConfidence = 0.0f,
                       float newSpeedLimit = -1.0f,
                       int newTtlFrames = 1)
    {
        resetPayload();
        source = owner;
        centerLine = std::move(center);
        headingConfidence = newHeadingConfidence;
        speedLimit = newSpeedLimit;
        ttlFrames = std::max(1, newTtlFrames);
        generatedFrameId = observedFrameId;
    }

    void clear() { resetPayload(); }

    bool clear(PathSource owner)
    {
        if (!active() || source != owner)
            return false;
        clear();
        return true;
    }

    void tick(uint64_t currentFrameId)
    {
        observedFrameId = currentFrameId;
        if (!active())
            return;
        if (generatedFrameId == 0)
        {
            generatedFrameId = currentFrameId;
            return;
        }
        if (currentFrameId > generatedFrameId &&
            currentFrameId - generatedFrameId >=
                static_cast<uint64_t>(ttlFrames))
            clear();
    }

    bool hasLeft() const { return !leftEdge.empty(); }
    bool hasRight() const { return !rightEdge.empty(); }
    bool hasCenter() const { return !centerLine.empty(); }
    bool hasGeometry() const { return hasLeft() || hasRight() || hasCenter(); }
    bool hasValidGeometry() const
    {
        return source != PathSource::NONE && ttlFrames > 0 && hasGeometry();
    }
    bool active() const { return hasValidGeometry(); }

    bool validFor(PathSource expectedSource) const
    {
        return hasValidGeometry() && source == expectedSource;
    }

private:
    uint64_t observedFrameId{0};

    void resetPayload()
    {
        source = PathSource::NONE;
        leftEdge.clear();
        rightEdge.clear();
        centerLine.clear();
        headingConfidence = 0.0f;
        speedLimit = -1.0f;
        ttlFrames = 0;
        generatedFrameId = 0;
    }
};

inline float applyPathSpeedLimit(float speed, const PathOverride &overridePath)
{
    if (!overridePath.active() || overridePath.speedLimit < 0.0f)
        return speed;
    return std::min(speed, overridePath.speedLimit);
}

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
    if (overridePath.hasValidGeometry() && overridePath.hasCenter())
        return {nullptr, nullptr, &overridePath.centerLine, overridePath.source};
    if (overridePath.hasValidGeometry() &&
        (overridePath.hasLeft() || overridePath.hasRight()))
        return {&overridePath.leftEdge, &overridePath.rightEdge, nullptr,
                overridePath.source};
    return {&trackLeft, &trackRight, nullptr, PathSource::NONE};
}
