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
                                        int lateralBiasPixels,
                                        const Config &config);
    static PathOverride buildParkingTurn(bool turnLeft, const Config &config);
    static PathOverride buildInSpotStraight(const Config &config);
    static PathOverride buildReplay(const std::vector<PointX> &left,
                                    const std::vector<PointX> &right,
                                    const Config &config);
    static PathOverride buildForkOut(const Config &config);

};
