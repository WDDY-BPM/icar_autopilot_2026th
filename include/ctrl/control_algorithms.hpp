#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace control_algorithms
{
struct LaneRecoveryState
{
    int invalidFrames = 0;
    int recoveryFrames = 0;
    bool recovering = false;
    bool controlValid = true;
};

inline bool updateLaneRecovery(LaneRecoveryState &state, bool candidateValid,
                               int requiredRecoveryFrames = 5)
{
    if (candidateValid)
    {
        state.invalidFrames = 0;
        if (state.recovering)
        {
            state.recoveryFrames++;
            state.controlValid = state.recoveryFrames >= requiredRecoveryFrames;
            if (state.controlValid)
                state.recovering = false;
        }
        else
        {
            state.controlValid = true;
        }
    }
    else
    {
        state.invalidFrames++;
        state.recoveryFrames = 0;
        state.recovering = true;
        state.controlValid = false;
    }
    return state.controlValid;
}

inline float applyStartupSpeed(float desiredSpeed, int &count,
                               int rampFrames, float startupSpeed,
                               float normalLowSpeed)
{
    rampFrames = std::max(1, rampFrames);
    if (count >= rampFrames)
        return desiredSpeed;
    count++;
    const float ratio = static_cast<float>(count) / rampFrames;
    const float rampSpeed = startupSpeed +
        ratio * (normalLowSpeed - startupSpeed);
    return std::min(desiredSpeed, rampSpeed);
}

inline int applyStartupServoLimit(int targetServo, int neutral,
                                  int startupLimit, int startupCount,
                                  int startupFrames)
{
    if (startupCount >= startupFrames)
        return targetServo;
    return std::clamp(targetServo, neutral - startupLimit,
                      neutral + startupLimit);
}

template <typename Point>
inline bool fillAlignedLaneGaps(std::vector<Point> &left,
                                std::vector<Point> &right,
                                std::vector<Point> &width,
                                int maxGapRows)
{
    if (left.size() < 2 || left.size() != right.size() ||
        left.size() != width.size())
        return false;
    std::vector<Point> leftFilled{left.front()};
    std::vector<Point> rightFilled{right.front()};
    std::vector<Point> widthFilled{Point(left.front().x,
        right.front().y - left.front().y)};
    for (std::size_t i = 1; i < left.size(); ++i)
    {
        const int rowGap = left[i - 1].x - left[i].x;
        if (right[i - 1].x == left[i - 1].x && right[i].x == left[i].x &&
            rowGap > 1 && rowGap <= maxGapRows)
        {
            for (int row = left[i - 1].x - 1; row > left[i].x; --row)
            {
                const float ratio = static_cast<float>(left[i - 1].x - row) / rowGap;
                const int leftColumn = static_cast<int>(std::lround(
                    left[i - 1].y + ratio * (left[i].y - left[i - 1].y)));
                const int rightColumn = static_cast<int>(std::lround(
                    right[i - 1].y + ratio * (right[i].y - right[i - 1].y)));
                leftFilled.emplace_back(row, leftColumn);
                rightFilled.emplace_back(row, rightColumn);
                widthFilled.emplace_back(row, rightColumn - leftColumn);
            }
        }
        leftFilled.push_back(left[i]);
        rightFilled.push_back(right[i]);
        widthFilled.emplace_back(left[i].x, right[i].y - left[i].y);
    }
    left.swap(leftFilled);
    right.swap(rightFilled);
    width.swap(widthFilled);
    return true;
}
} // namespace control_algorithms