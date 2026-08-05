#pragma once

#include <algorithm>
#include <cstdlib>
#include <vector>
#include "utils/point.hpp"

struct PlannedPathValidation
{
    bool valid{false};
    int validPoints{0};
    int nearSamples{0};
    int maxColumnJump{0};
};

inline PlannedPathValidation validatePlannedPath(
    const std::vector<PointX> &path, int rows, int cols)
{
    PlannedPathValidation result;
    if (rows <= 0 || cols <= 0 || path.size() < 8)
        return result;

    int direction = 0;
    int rowReversals = 0;
    const int maximumColumnJump = std::max(20, cols / 4);
    for (size_t index = 0; index < path.size(); ++index)
    {
        const auto &point = path[index];
        if (point.x < 0 || point.x >= rows || point.y < 0 || point.y >= cols)
            return result;
        result.validPoints++;
        if (point.x >= rows * 2 / 3)
            result.nearSamples++;
        if (index == 0)
            continue;

        result.maxColumnJump = std::max(
            result.maxColumnJump, std::abs(point.y - path[index - 1].y));
        const int rowDelta = point.x - path[index - 1].x;
        if (rowDelta != 0)
        {
            const int currentDirection = rowDelta > 0 ? 1 : -1;
            if (direction != 0 && currentDirection != direction)
                rowReversals++;
            direction = currentDirection;
        }
    }

    result.valid = result.nearSamples >= 3 &&
                   result.maxColumnJump <= maximumColumnJump &&
                   rowReversals <= std::max(1, static_cast<int>(path.size() / 10));
    return result;
}
