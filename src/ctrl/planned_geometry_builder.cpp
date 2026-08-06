#include "ctrl/planned_geometry_builder.hpp"
#include "ctrl/row_aligned_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace
{
std::vector<PointX> centerFromEdges(const std::vector<PointX> &left,
                                    const std::vector<PointX> &right,
                                    const GeometryBuildOptions &options)
{
    return buildDualCenterByRow(
        left, right, options.rows, options.columns,
        options.maximumInterpolationGap, options.maximumAcceptedCenterGap,
        options.nearFieldStartRow);
}

std::vector<PointX> centerFromSingle(
    const std::vector<PointX> &edge, int side,
    const PlannedLaneWidthModel &widthModel,
    const GeometryBuildOptions &options)
{
    RowIndexedEdge edgeByRow = indexEdgeByRow(
        edge, options.rows, options.columns);
    interpolateShortGaps(edgeByRow, options.maximumInterpolationGap);
    std::vector<PointX> candidates;
    candidates.reserve(edge.size());
    for (int row = options.rows - 1; row >= 0; --row)
    {
        if (edgeByRow.columnByRow[row] < 0)
            continue;
        const float laneWidth = widthModel.ready &&
                widthModel.widthByRow[row] > 1.0f
            ? widthModel.widthByRow[row] : widthModel.fallbackWidth;
        const float shifted = edgeByRow.columnByRow[row] +
            (side < 0 ? laneWidth * 0.5f : -laneWidth * 0.5f);
        candidates.emplace_back(row, std::clamp(
            static_cast<int>(std::lround(shifted)), 0,
            options.columns - 1));
    }
    return selectBestNearFieldSegment(
        candidates, options.maximumAcceptedCenterGap,
        options.nearFieldStartRow);
}
}

PlannedGeometryResult buildPlannedGeometry(
    const PathOverride &path, FsmMode mode,
    const PlannedLaneWidthModel &widthModel)
{
    PlannedGeometryResult result;
    if (!path.hasValidGeometry() || !pathSourceAllowed(path.source, mode))
        return result;
    const GeometryBuildOptions options;
    result.source = path.source;
    if (path.hasCenter())
        result.centerLine = path.centerLine;
    else if (path.hasLeft() && path.hasRight())
        result.centerLine = centerFromEdges(
            path.leftEdge, path.rightEdge, options);
    else if (path.hasLeft())
        result.centerLine = centerFromSingle(
            path.leftEdge, -1, widthModel, options);
    else if (path.hasRight())
        result.centerLine = centerFromSingle(
            path.rightEdge, 1, widthModel, options);
    result.validation = validatePlannedPath(
        result.centerLine, options.rows, options.columns);
    result.valid = result.validation.valid;
    return result;
}
