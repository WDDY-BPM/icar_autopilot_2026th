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

struct SingleLaneSpeedLimitState
{
    bool active = false;
    int dualLaneRecoveryFrames = 0;
};

struct CenterlineSpeedResult
{
    bool valid = false;
    float curveStrength = 0.0f;
    float speed = 0.0f;
};

struct CenterWindowResult
{
    float column = 0.0f;
    int samples = 0;
    bool valid = false;
};

struct LaneControlCenters
{
    float nearCenter = 0.0f;
    float farCenter = 0.0f;
    float controlCenter = 0.0f;
    float headingError = 0.0f;
    float headingCorrection = 0.0f;
    int nearSamples = 0;
    int farSamples = 0;
    bool nearValid = false;
    bool farValid = false;
};

struct EdgeReliability
{
    bool reliable = false;
    int pointCount = 0;
    int longestBorderRun = 0;
    float borderRatio = 1.0f;
    float maximumJump = 0.0f;
    bool coversBottom = false;
};

template <typename Point>
inline EdgeReliability assessEdgeReliability(const std::vector<Point> &edge,
                                             bool leftEdge, int imageWidth,
                                             int imageHeight, int rowCutBottom)
{
    EdgeReliability result;
    result.pointCount = static_cast<int>(edge.size());
    if (edge.empty()) return result;
    int borderPoints = 0, currentBorderRun = 0, nearestRow = 0;
    int previousColumn = edge.front().y;
    for (std::size_t i = 0; i < edge.size(); ++i)
    {
        const auto &point = edge[i];
        nearestRow = std::max(nearestRow, point.x);
        if (i > 0)
            result.maximumJump = std::max(result.maximumJump,
                static_cast<float>(std::abs(point.y - previousColumn)));
        previousColumn = point.y;
        const bool onBorder = leftEdge ? point.y <= 2 : point.y >= imageWidth - 3;
        if (onBorder)
        {
            borderPoints++;
            currentBorderRun++;
            result.longestBorderRun = std::max(result.longestBorderRun, currentBorderRun);
        }
        else currentBorderRun = 0;
    }
    result.borderRatio = static_cast<float>(borderPoints) / edge.size();
    result.coversBottom = nearestRow >= imageHeight - rowCutBottom - 4;
    const bool borderFailure = result.borderRatio > 0.25f && result.longestBorderRun >= 8;
    result.reliable = result.pointCount >= 20 && result.coversBottom &&
                      result.maximumJump <= 30.0f && !borderFailure;
    return result;
}

inline bool isSingleLaneCenterContinuous(int currentCenter,
                                         int lastValidLaneCenter,
                                         int maximumJump = 15)
{
    return std::abs(currentCenter - lastValidLaneCenter) <= maximumJump;
}

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

inline bool updateSingleLaneSpeedLimit(SingleLaneSpeedLimitState &state,
                                       bool strictLaneMode, bool controlValid,
                                       bool leftReliable, bool rightReliable,
                                       int requiredDualLaneFrames = 5)
{
    if (!strictLaneMode)
    {
        state = SingleLaneSpeedLimitState{};
        return false;
    }
    const bool singleLaneControl = controlValid &&
        (leftReliable != rightReliable);
    const bool dualLaneControl = controlValid && leftReliable && rightReliable;
    if (singleLaneControl)
    {
        state.active = true;
        state.dualLaneRecoveryFrames = 0;
    }
    else if (state.active)
    {
        state.dualLaneRecoveryFrames = dualLaneControl
            ? state.dualLaneRecoveryFrames + 1 : 0;
        if (state.dualLaneRecoveryFrames >= std::max(1, requiredDualLaneFrames))
            state = SingleLaneSpeedLimitState{};
    }
    return state.active;
}

template <typename Point>
inline CenterWindowResult calculateCenterWindow(
    const std::vector<Point> &centerline, float defaultCenter,
    int rowBegin, int rowEnd, int peakRow, int peakWeight,
    int minimumSamples)
{
    CenterWindowResult result;
    result.column = defaultCenter;
    if (rowBegin > rowEnd)
        return result;

    double weightedSum = 0.0;
    double weightSum = 0.0;
    for (const auto &point : centerline)
    {
        if (point.x < rowBegin || point.x > rowEnd)
            continue;
        const int weight = std::max(
            1, peakWeight - std::abs(point.x - peakRow));
        weightedSum += static_cast<double>(point.y) * weight;
        weightSum += weight;
        result.samples++;
    }

    minimumSamples = std::max(1, minimumSamples);
    if (result.samples >= minimumSamples && weightSum > 0.0)
    {
        result.column = static_cast<float>(weightedSum / weightSum);
        result.valid = true;
    }
    return result;
}

template <typename Point>
inline LaneControlCenters calculateLaneControlCenters(
    const std::vector<Point> &centerline, float defaultCenter,
    float nearBlend = 0.65f, int minimumSamples = 8,
    bool enableHeadingCorrection = false, float headingGain = 300.0f,
    float maximumHeadingCorrection = 60.0f,
    float headingFadeError = 40.0f)
{
    const auto near = calculateCenterWindow(
        centerline, defaultCenter, 176, 220, 205, 26, minimumSamples);
    const auto far = calculateCenterWindow(
        centerline, defaultCenter, 90, 155, 120, 31, minimumSamples);

    LaneControlCenters result;
    result.nearCenter = near.column;
    result.farCenter = far.column;
    result.controlCenter = defaultCenter;
    result.nearSamples = near.samples;
    result.farSamples = far.samples;
    result.nearValid = near.valid;
    result.farValid = far.valid;
    if (!near.valid)
        return result;

    result.controlCenter = near.column;
    if (far.valid)
    {
        nearBlend = std::clamp(nearBlend, 0.0f, 1.0f);
        result.controlCenter = nearBlend * near.column +
            (1.0f - nearBlend) * far.column;
        // Lane-relative vehicle heading in camera coordinates. Unlike the old
        // column delta this is an angle and is independent of window spacing.
        constexpr float rowSeparation = 85.0f; // near peak 205 - far peak 120
        result.headingError = std::atan2(
            far.column - near.column, rowSeparation);

        if (enableHeadingCorrection)
        {
            const float nearError = near.column - defaultCenter;
            const float farError = far.column - defaultCenter;
            const bool oppositeSideRecovery =
                (nearError > 4.0f && farError < -8.0f) ||
                (nearError < -4.0f && farError > 8.0f);

            // Keep heading feedback active during lateral recovery. Completely
            // fading it out lets the car point away from the lane while the
            // lateral controller is trying to return to the center.
            const float fadeRange = std::max(1.0f, headingFadeError);
            const float headingWeight = std::max(0.35f, 1.0f - std::clamp(
                std::abs(nearError) / fadeRange, 0.0f, 1.0f));
            const float correctionLimit =
                std::max(0.0f, maximumHeadingCorrection);
            if (!oppositeSideRecovery)
            {
                result.headingCorrection = std::clamp(
                    std::max(0.0f, headingGain) * result.headingError *
                        headingWeight,
                    -correctionLimit, correctionLimit);
                // Applied independently by Motion::poseControl as a PWM term.
            }
        }
    }
    return result;
}

template <typename Point>
inline CenterlineSpeedResult calculateCenterlineSpeed(
    const std::vector<Point> &centerline, float highSpeed, float curveSpeed,
    int farRowBegin = 90, int farRowEnd = 130,
    int nearRowBegin = 170, int nearRowEnd = 210,
    float straightStrength = 8.0f, float fullCurveStrength = 35.0f,
    int minimumBandPoints = 3)
{
    CenterlineSpeedResult result;
    // Missing geometry must not silently restore full speed.
    const float safeCurveSpeed = std::min(highSpeed, curveSpeed);
    result.speed = safeCurveSpeed;
    if (farRowBegin > farRowEnd || nearRowBegin > nearRowEnd ||
        fullCurveStrength <= straightStrength)
        return result;

    double farSum = 0.0;
    double nearSum = 0.0;
    int farCount = 0;
    int nearCount = 0;
    for (const auto &point : centerline)
    {
        if (point.x >= farRowBegin && point.x <= farRowEnd)
        {
            farSum += point.y;
            farCount++;
        }
        if (point.x >= nearRowBegin && point.x <= nearRowEnd)
        {
            nearSum += point.y;
            nearCount++;
        }
    }

    minimumBandPoints = std::max(1, minimumBandPoints);
    if (farCount < minimumBandPoints || nearCount < minimumBandPoints)
        return result;

    const float farCenter = static_cast<float>(farSum / farCount);
    const float nearCenter = static_cast<float>(nearSum / nearCount);
    result.curveStrength = std::abs(farCenter - nearCenter);
    const float curveRatio = std::clamp(
        (result.curveStrength - straightStrength) /
            (fullCurveStrength - straightStrength),
        0.0f, 1.0f);
    result.speed = highSpeed + curveRatio * (safeCurveSpeed - highSpeed);
    result.valid = true;
    return result;
}

inline float applyStartupSpeed(float desiredSpeed, int &count,
                               int rampFrames, float startupSpeed)
{
    rampFrames = std::max(1, rampFrames);
    if (count >= rampFrames)
        return desiredSpeed;
    count++;
    const float ratio = static_cast<float>(count) / rampFrames;
    const float rampSpeed = startupSpeed +
        ratio * (desiredSpeed - startupSpeed);
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
