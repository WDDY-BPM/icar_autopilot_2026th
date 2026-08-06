#pragma once

#include <vector>
#include "config/config.hpp"
#include "runtime/path_override.hpp"
#include "vision/predict_result.hpp"

class ParkPathPlanner
{
public:
    static PathOverride buildForkIn(const Config &config);
    static PathOverride buildTrackGuide(const PointX &start,
                                        const PredictResult &direction,
                                        bool rightSide,
                                        const Config &config);
    static PathOverride buildParkingTurn(bool turnLeft, const Config &config);
    static PathOverride buildInSpotStraight(const Config &config);
    static PathOverride buildReplay(const std::vector<PointX> &left,
                                    const std::vector<PointX> &right,
                                    const Config &config);
    static PathOverride buildForkOut(const Config &config);

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
