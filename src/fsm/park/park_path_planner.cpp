#include "fsm/park/park_path_planner.hpp"

#include <algorithm>
#include <cmath>

namespace
{
std::vector<PointX> bezier(const std::vector<PointX> &controlPoints)
{
    std::vector<PointX> output;
    if (controlPoints.empty())
        return output;
    output.reserve(51);
    for (int sample = 0; sample <= 50; ++sample)
    {
        const double t = sample / 50.0;
        const double inverse = 1.0 - t;
        PointX point;
        if (controlPoints.size() == 3)
        {
            point.x = inverse * inverse * controlPoints[0].x +
                      2.0 * inverse * t * controlPoints[1].x +
                      t * t * controlPoints[2].x;
            point.y = inverse * inverse * controlPoints[0].y +
                      2.0 * inverse * t * controlPoints[1].y +
                      t * t * controlPoints[2].y;
        }
        else
        {
            point = controlPoints.front();
        }
        output.push_back(point);
    }
    return output;
}

PathOverride edgePath(std::vector<PointX> left, std::vector<PointX> right,
                      const Config &config)
{
    PathOverride path;
    path.setEdgesForFrames(PathSource::PARK, std::move(left), std::move(right),
                           config.parkingHeadingConfidence, config.velPark, 2);
    return path;
}
}

PathOverride ParkPathPlanner::buildForkIn(const Config &config)
{
    return edgePath(
        bezier({{230, 1}, {93, 1}, {80, 1}}),
        bezier({{230, 256}, {155, 100}, {80, 30}}), config);
}

PathOverride ParkPathPlanner::buildTrackGuide(const PointX &start,
                                              const PredictResult &direction,
                                              int lateralBiasPixels,
                                              const Config &config)
{
    const int endRow = std::clamp(direction.y, 0, 239);
    const int endColumn = std::clamp(
        direction.x + direction.width / 2 + lateralBiasPixels, 1, 318);
    const PointX end(endRow, endColumn);
    const PointX mid((start.x + end.x) / 2, (start.y + end.y) / 2);
    PathOverride path;
    path.setCenterLineForTime(PathSource::PARK, bezier({start, mid, end}),
                              std::chrono::milliseconds(150),
                              config.parkingHeadingConfidence, config.velPark);
    return path;
}

PathOverride ParkPathPlanner::buildParkingTurn(bool turnLeft,
                                               const Config &config)
{
    if (turnLeft)
        return edgePath(
            bezier({{230, 1}, {93, 1}, {80, 1}}),
            bezier({{230, 256}, {93, 128}, {80, 1}}), config);
    return edgePath(
        bezier({{230, 64}, {78, 191}, {30, 319}}),
        bezier({{230, 319}, {78, 319}, {30, 319}}), config);
}

PathOverride ParkPathPlanner::buildInSpotStraight(const Config &config)
{
    return edgePath(bezier({{210, 64}, {130, 88}, {50, 112}}),
                    bezier({{210, 256}, {130, 232}, {50, 208}}), config);
}

PathOverride ParkPathPlanner::buildReplay(const std::vector<PointX> &left,
                                          const std::vector<PointX> &right,
                                          const Config &config)
{
    return edgePath(left, right, config);
}

PathOverride ParkPathPlanner::buildForkOut(const Config &config)
{
    return edgePath(
        bezier({{230, 1}, {93, 1}, {80, 1}}),
        bezier({{230, 256}, {155, 100}, {80, 30}}), config);
}
