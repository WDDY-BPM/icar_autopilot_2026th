#include "ctrl/planned_geometry_builder.hpp"

#include <algorithm>

namespace
{
std::vector<PointX> centerFromEdges(const std::vector<PointX> &left,
                                    const std::vector<PointX> &right)
{
    std::vector<PointX> center;
    const std::size_t count = std::min(left.size(), right.size());
    center.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
        center.emplace_back(left[index].x,
                            (left[index].y + right[index].y) / 2);
    return center;
}

std::vector<PointX> centerFromSingle(const std::vector<PointX> &edge, int side)
{
    std::vector<PointX> center;
    center.reserve(edge.size());
    const int offset = side < 0 ? 48 : -48;
    for (const auto &point : edge)
        center.emplace_back(point.x, point.y + offset);
    return center;
}
}

PlannedGeometryResult buildPlannedGeometry(const PathOverride &path,
                                           FsmMode mode)
{
    PlannedGeometryResult result;
    if (!path.hasValidGeometry() || !pathSourceAllowed(path.source, mode))
        return result;
    result.source = path.source;
    if (path.hasCenter())
        result.centerLine = path.centerLine;
    else if (path.hasLeft() && path.hasRight())
        result.centerLine = centerFromEdges(path.leftEdge, path.rightEdge);
    else if (path.hasLeft())
        result.centerLine = centerFromSingle(path.leftEdge, -1);
    else if (path.hasRight())
        result.centerLine = centerFromSingle(path.rightEdge, 1);
    result.validation = validatePlannedPath(result.centerLine, 240, 320);
    result.valid = result.validation.valid;
    return result;
}
