#include "ctrl/planned_geometry_builder.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace
{
constexpr int kImageRows = 240;
constexpr int kImageColumns = 320;
constexpr int kMaximumInterpolationGap = 6;

using EdgeByRow = std::array<int, kImageRows>;

EdgeByRow indexEdgeByRow(const std::vector<PointX> &edge)
{
    EdgeByRow columns;
    columns.fill(-1);
    for (const auto &point : edge)
        if (point.x >= 0 && point.x < kImageRows &&
            point.y >= 0 && point.y < kImageColumns)
            columns[point.x] = point.y;
    return columns;
}

void interpolateShortGaps(EdgeByRow &columns)
{
    int previousRow = -1;
    for (int row = 0; row < kImageRows; ++row)
    {
        if (columns[row] < 0)
            continue;
        if (previousRow >= 0)
        {
            const int missingRows = row - previousRow - 1;
            if (missingRows > 0 && missingRows <= kMaximumInterpolationGap)
                for (int gap = 1; gap <= missingRows; ++gap)
                {
                    const float ratio = static_cast<float>(gap) /
                        static_cast<float>(missingRows + 1);
                    columns[previousRow + gap] = static_cast<int>(std::lround(
                        columns[previousRow] +
                        (columns[row] - columns[previousRow]) * ratio));
                }
        }
        previousRow = row;
    }
}

std::vector<PointX> centerFromEdges(const std::vector<PointX> &left,
                                    const std::vector<PointX> &right,
                                    bool &orderValid)
{
    EdgeByRow leftByRow = indexEdgeByRow(left);
    EdgeByRow rightByRow = indexEdgeByRow(right);
    interpolateShortGaps(leftByRow);
    interpolateShortGaps(rightByRow);
    std::vector<PointX> center;
    center.reserve(kImageRows);
    orderValid = true;
    int previousRow = -1;
    for (int row = kImageRows - 1; row >= 0; --row)
    {
        if (leftByRow[row] < 0 || rightByRow[row] < 0)
            continue;
        if (previousRow >= 0 && previousRow - row - 1 >
            kMaximumInterpolationGap)
        {
            orderValid = false;
            return {};
        }
        if (rightByRow[row] <= leftByRow[row])
        {
            orderValid = false;
            center.clear();
            return center;
        }
        center.emplace_back(row, (leftByRow[row] + rightByRow[row]) / 2);
        previousRow = row;
    }
    return center;
}

std::vector<PointX> centerFromSingle(
    const std::vector<PointX> &edge, int side,
    const PlannedLaneWidthModel &widthModel)
{
    EdgeByRow edgeByRow = indexEdgeByRow(edge);
    interpolateShortGaps(edgeByRow);
    std::vector<PointX> center;
    center.reserve(edge.size());
    int previousRow = -1;
    for (int row = kImageRows - 1; row >= 0; --row)
    {
        if (edgeByRow[row] < 0)
            continue;
        if (previousRow >= 0 && previousRow - row - 1 >
            kMaximumInterpolationGap)
            return {};
        const float laneWidth = widthModel.ready &&
                widthModel.widthByRow[row] > 1.0f
            ? widthModel.widthByRow[row] : widthModel.fallbackWidth;
        const float shifted = edgeByRow[row] +
            (side < 0 ? laneWidth * 0.5f : -laneWidth * 0.5f);
        center.emplace_back(row, std::clamp(
            static_cast<int>(std::lround(shifted)), 0, kImageColumns - 1));
        previousRow = row;
    }
    return center;
}
}

PlannedGeometryResult buildPlannedGeometry(
    const PathOverride &path, FsmMode mode,
    const PlannedLaneWidthModel &widthModel)
{
    PlannedGeometryResult result;
    if (!path.hasValidGeometry() || !pathSourceAllowed(path.source, mode))
        return result;
    result.source = path.source;
    bool orderValid = true;
    if (path.hasCenter())
        result.centerLine = path.centerLine;
    else if (path.hasLeft() && path.hasRight())
        result.centerLine = centerFromEdges(
            path.leftEdge, path.rightEdge, orderValid);
    else if (path.hasLeft())
        result.centerLine = centerFromSingle(path.leftEdge, -1, widthModel);
    else if (path.hasRight())
        result.centerLine = centerFromSingle(path.rightEdge, 1, widthModel);
    if (!orderValid)
        return result;
    result.validation = validatePlannedPath(
        result.centerLine, kImageRows, kImageColumns);
    result.valid = result.validation.valid;
    return result;
}
