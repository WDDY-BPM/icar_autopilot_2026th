#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include "utils/point.hpp"

/**
 * Row-aligned edge indexing shared by the perception and planned geometry
 * builders. Edge points may arrive in any row order; every builder indexes
 * them by image row first, so the resulting centerline no longer depends on
 * the caller's sort direction.
 */

struct RowIndexedEdge
{
    std::vector<int> columnByRow;

    explicit RowIndexedEdge(int rows = 240) : columnByRow(rows, -1) {}
    int rows() const { return static_cast<int>(columnByRow.size()); }
};

inline RowIndexedEdge indexEdgeByRow(const std::vector<PointX> &edge,
                                     int rows, int columns)
{
    RowIndexedEdge indexed(rows);
    for (const auto &point : edge)
    {
        if (point.x >= 0 && point.x < rows &&
            point.y >= 0 && point.y < columns)
            indexed.columnByRow[point.x] = point.y;
    }
    return indexed;
}

inline void interpolateShortGaps(RowIndexedEdge &edge, int maximumGap)
{
    if (maximumGap <= 0)
        return;
    int previousRow = -1;
    for (int row = 0; row < edge.rows(); ++row)
    {
        if (edge.columnByRow[row] < 0)
            continue;
        if (previousRow >= 0)
        {
            const int missingRows = row - previousRow - 1;
            if (missingRows > 0 && missingRows <= maximumGap)
            {
                for (int gap = 1; gap <= missingRows; ++gap)
                {
                    const float ratio =
                        static_cast<float>(gap) /
                        static_cast<float>(missingRows + 1);
                    edge.columnByRow[previousRow + gap] =
                        static_cast<int>(std::lround(
                            edge.columnByRow[previousRow] +
                            (edge.columnByRow[row] -
                             edge.columnByRow[previousRow]) * ratio));
                }
            }
        }
        previousRow = row;
    }
}

/**
 * From an already row-sorted (descending) candidate list, keep the longest
 * continuous segment that contains at least one near-field row. Large holes
 * are not interpolated, but they must not reject the near-side control path
 * just because the far side of the image has a gap.
 */
inline std::vector<PointX> selectBestNearFieldSegment(
    const std::vector<PointX> &orderedCenter,
    int maximumAcceptedCenterGap, int nearFieldStartRow)
{
    if (orderedCenter.empty())
        return {};
    std::vector<std::vector<PointX>> segments;
    for (const auto &point : orderedCenter)
    {
        if (segments.empty() ||
            segments.back().back().x - point.x > maximumAcceptedCenterGap)
            segments.emplace_back();
        segments.back().push_back(point);
    }
    const std::vector<PointX> *best = nullptr;
    for (const auto &segment : segments)
    {
        const bool hasNear = std::any_of(
            segment.begin(), segment.end(),
            [&](const PointX &point) { return point.x >= nearFieldStartRow; });
        if (!hasNear)
            continue;
        if (!best || segment.size() > best->size())
            best = &segment;
    }
    if (!best)
        return {};
    return *best;
}

inline std::vector<PointX> buildDualCenterByRow(
    const std::vector<PointX> &left, const std::vector<PointX> &right,
    int rows, int columns, int maximumInterpolationGap,
    int maximumAcceptedCenterGap, int nearFieldStartRow)
{
    RowIndexedEdge leftByRow = indexEdgeByRow(left, rows, columns);
    RowIndexedEdge rightByRow = indexEdgeByRow(right, rows, columns);
    interpolateShortGaps(leftByRow, maximumInterpolationGap);
    interpolateShortGaps(rightByRow, maximumInterpolationGap);

    std::vector<PointX> candidates;
    candidates.reserve(static_cast<std::size_t>(rows));
    for (int row = rows - 1; row >= 0; --row)
    {
        const int leftColumn = leftByRow.columnByRow[row];
        const int rightColumn = rightByRow.columnByRow[row];
        if (leftColumn < 0 || rightColumn <= leftColumn)
            continue;
        candidates.emplace_back(row, (leftColumn + rightColumn) / 2);
    }
    return selectBestNearFieldSegment(
        candidates, maximumAcceptedCenterGap, nearFieldStartRow);
}

/**
 * Build options shared by every production geometry builder. The default
 * image geometry matches the production camera (ROWSIMAGE x COLSIMAGE =
 * 240 x 320); callers may pass explicit values for future image sizes.
 */
struct GeometryBuildOptions
{
    int rows{240};
    int columns{320};
    int maximumInterpolationGap{6};
    int maximumAcceptedCenterGap{8};
    int nearFieldStartRow{176};
};
