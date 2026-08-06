#pragma once

#include <vector>
#include "runtime/path_override.hpp"

class ParkPathPlanner
{
public:
    static PathOverride fromEdges(PathSource source,
                                  const std::vector<PointX> &left,
                                  const std::vector<PointX> &right,
                                  float headingConfidence = 0.0f,
                                  float speedLimit = -1.0f,
                                  int ttlFrames = 1);
    static PathOverride fromCenterLine(PathSource source,
                                       const std::vector<PointX> &center,
                                       float headingConfidence = 0.0f,
                                       float speedLimit = -1.0f,
                                       int ttlFrames = 1);
};
