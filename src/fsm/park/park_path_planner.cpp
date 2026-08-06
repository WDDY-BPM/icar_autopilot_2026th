#include "fsm/park/park_path_planner.hpp"

PathOverride ParkPathPlanner::fromEdges(PathSource source,
                                         const std::vector<PointX> &left,
                                         const std::vector<PointX> &right,
                                         float headingConfidence,
                                         float speedLimit, int ttlFrames)
{
    PathOverride path;
    path.setEdges(source, left, right, headingConfidence, speedLimit, ttlFrames);
    return path;
}

PathOverride ParkPathPlanner::fromCenterLine(PathSource source,
                                              const std::vector<PointX> &center,
                                              float headingConfidence,
                                              float speedLimit, int ttlFrames)
{
    PathOverride path;
    path.setCenterLine(source, center, headingConfidence, speedLimit, ttlFrames);
    return path;
}
