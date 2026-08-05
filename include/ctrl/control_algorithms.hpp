#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <cstdint>
#include <string>

namespace control_algorithms
{
enum class StopReason : std::uint32_t
{
    STARTUP = 1u << 0, CAMERA = 1u << 1, EMERGENCY = 1u << 2,
    LANE = 1u << 3, GATE = 1u << 4, PARK = 1u << 5,
    BUSY = 1u << 6, STATION = 1u << 7, CROSS = 1u << 8,
    MANUAL = 1u << 9, AI_STALE = 1u << 10
};

class StopReasonState
{
public:
    void set(StopReason reason, bool active)
    {
        const auto bit = static_cast<std::uint32_t>(reason);
        bits = active ? bits | bit : bits & ~bit;
    }
    bool has(StopReason reason) const
    {
        return (bits & static_cast<std::uint32_t>(reason)) != 0;
    }
    bool mustStop() const { return bits != 0; }
    std::uint32_t value() const { return bits; }
    std::string string() const
    {
        if (!bits) return "NONE";
        std::string result;
        const auto append = [&](StopReason reason, const char *name) {
            if (!has(reason)) return;
            if (!result.empty()) result += "|";
            result += name;
        };
        append(StopReason::STARTUP, "STARTUP"); append(StopReason::CAMERA, "CAMERA");
        append(StopReason::EMERGENCY, "EMERGENCY"); append(StopReason::LANE, "LANE");
        append(StopReason::GATE, "GATE"); append(StopReason::PARK, "PARK");
        append(StopReason::BUSY, "BUSY"); append(StopReason::STATION, "STATION");
        append(StopReason::CROSS, "CROSS"); append(StopReason::MANUAL, "MANUAL");
        append(StopReason::AI_STALE, "AI_STALE");
        return result;
    }
private:
    std::uint32_t bits = 0;
};

inline bool advanceAlertCountdown(int &countdown, int beepEvery = 10)
{
    if (countdown <= 0)
        return false;
    const bool shouldBeep = beepEvery > 0 && countdown % beepEvery == 0;
    countdown--;
    return shouldBeep;
}

inline int updateAlertDecelCountdown(int countdown, bool targetDetected,
                                     int holdFrames = 5)
{
    if (targetDetected)
        return holdFrames;
    return countdown > 0 ? countdown - 1 : 0;
}

constexpr int BUSY_CONFIRM_POSITIVE_FRAMES = 4;
constexpr int BUSY_CLEAR_NEGATIVE_FRAMES = 5;
constexpr std::int64_t AI_STALE_TIMEOUT_MS = 500;
constexpr int AI_RECOVERY_FRESH_RESULTS = 3;
constexpr int CROSS_DISAPPEAR_FRAMES = 15;

struct BusyConfirmationState
{
    int positiveFrames = 0;
    int negativeFrames = 0;
    bool waiting = false;
    bool confirmed = false;
};

enum class BusyConfirmationEvent
{
    NONE,
    WAITING,
    CONFIRMED,
    CLEARED
};

inline BusyConfirmationEvent updateBusyConfirmation(
    BusyConfirmationState &state, bool freshResult, bool busyDetected)
{
    if (!freshResult || state.confirmed)
        return BusyConfirmationEvent::NONE;
    if (busyDetected)
    {
        state.negativeFrames = 0;
        state.positiveFrames++;
        if (state.positiveFrames >= BUSY_CONFIRM_POSITIVE_FRAMES)
        {
            state.confirmed = true;
            state.waiting = false;
            return BusyConfirmationEvent::CONFIRMED;
        }
        if (state.positiveFrames == BUSY_CONFIRM_POSITIVE_FRAMES - 1)
        {
            state.waiting = true;
            return BusyConfirmationEvent::WAITING;
        }
        return BusyConfirmationEvent::NONE;
    }
    if (state.waiting)
    {
        state.negativeFrames++;
        if (state.negativeFrames >= BUSY_CLEAR_NEGATIVE_FRAMES)
        {
            state = BusyConfirmationState{};
            return BusyConfirmationEvent::CLEARED;
        }
    }
    else
    {
        state.positiveFrames = 0;
    }
    return BusyConfirmationEvent::NONE;
}

struct AiFreshnessState
{
    bool automaticActive = false;
    bool automaticSeen = false;
    bool stale = false;
    bool hasSuccessfulResult = false;
    int recoveryFreshResults = 0;
    std::int64_t automaticStartedMs = 0;
    std::int64_t lastSuccessfulResultMs = 0;
};

inline bool updateAiFreshness(AiFreshnessState &state, bool automaticMode,
                              bool successfulFreshResult, std::int64_t nowMs,
                              std::int64_t timeoutMs = AI_STALE_TIMEOUT_MS,
                              int recoveryResults = AI_RECOVERY_FRESH_RESULTS,
                              std::int64_t successfulResultMs = -1)
{
    if (!automaticMode)
    {
        state.automaticActive = false;
        state.stale = false;
        state.recoveryFreshResults = 0;
        return false;
    }
    if (!state.automaticActive)
    {
        const bool returningFromManual = state.automaticSeen;
        state.automaticActive = true;
        state.automaticSeen = true;
        state.automaticStartedMs = nowMs;
        state.recoveryFreshResults = 0;
        state.stale = returningFromManual;
    }
    if (successfulFreshResult)
    {
        const std::int64_t publishedMs = successfulResultMs >= 0
            ? successfulResultMs : nowMs;
        state.hasSuccessfulResult = true;
        state.lastSuccessfulResultMs = publishedMs;
        if (nowMs - publishedMs > timeoutMs)
        {
            state.stale = true;
            state.recoveryFreshResults = 0;
        }
        else if (state.stale && ++state.recoveryFreshResults >= recoveryResults)
        {
            state.stale = false;
            state.recoveryFreshResults = 0;
        }
    }
    else
    {
        const std::int64_t reference = state.hasSuccessfulResult
            ? state.lastSuccessfulResultMs : state.automaticStartedMs;
        if (nowMs - reference > timeoutMs)
        {
            state.stale = true;
            state.recoveryFreshResults = 0;
        }
    }
    return state.stale;
}

struct CrossConfirmationState
{
    bool armed = false;
    bool linePassed = false;
    int passFrames = 0;
    int missingFrames = 0;
};

enum class CrossConfirmationEvent
{
    NONE,
    LAP_PASSED,
    FINAL_STOP
};

inline CrossConfirmationEvent updateCrossConfirmation(
    CrossConfirmationState &state, bool freshResult, bool crossDetected,
    bool passCandidate, bool finalLap, bool lapTaskComplete,
    int passConfirmFrames = 2,
    int disappearFrames = CROSS_DISAPPEAR_FRAMES)
{
    if (!freshResult)
        return CrossConfirmationEvent::NONE;
    if (!state.armed)
    {
        state.missingFrames = crossDetected ? 0 : state.missingFrames + 1;
        if (state.missingFrames >= disappearFrames)
        {
            state.armed = true;
            state.missingFrames = 0;
        }
        return CrossConfirmationEvent::NONE;
    }
    if (!state.linePassed)
    {
        state.passFrames = passCandidate ? state.passFrames + 1 : 0;
        if (state.passFrames >= passConfirmFrames)
        {
            // Only credit a crossing when the lap task was already complete
            // while the line was actually being passed. Disarm until this
            // same line disappears to prevent retroactive lap completion.
            if (!lapTaskComplete)
            {
                state = CrossConfirmationState{};
                return CrossConfirmationEvent::NONE;
            }
            state.linePassed = true;
            state.missingFrames = 0;
        }
        return CrossConfirmationEvent::NONE;
    }
    state.missingFrames = crossDetected ? 0 : state.missingFrames + 1;
    if (state.missingFrames < disappearFrames)
        return CrossConfirmationEvent::NONE;
    const auto event = finalLap
        ? CrossConfirmationEvent::FINAL_STOP
        : CrossConfirmationEvent::LAP_PASSED;
    state = CrossConfirmationState{};
    return event;
}
struct LaneUnconfirmedState
{
    int frames = 0;
    int stableFrames = 0;
};

inline int updateLaneUnconfirmed(LaneUnconfirmedState &state,
                                 bool confirmedValid, int clearFrames = 5)
{
    if (confirmedValid)
    {
        state.stableFrames++;
        if (state.stableFrames >= clearFrames) state.frames = 0;
    }
    else
    {
        state.frames++;
        state.stableFrames = 0;
    }
    return state.frames;
}
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

inline bool updateLaneSafetyStop(bool latched, bool safetyLaneMode,
                                 bool controlValid, int invalidFrames,
                                 int recoveryFrames,
                                 int stopAfterInvalidFrames = 7,
                                 int releaseAfterRecoveryFrames = 5,
                                 int unconfirmedFrames = 0,
                                 int unconfirmedLimit = 30)
{
    if (!safetyLaneMode) return false;
    if (!latched)
        return !controlValid && (invalidFrames >= stopAfterInvalidFrames ||
                                 unconfirmedFrames >= unconfirmedLimit);
    if (controlValid && recoveryFrames >= releaseAfterRecoveryFrames)
        return false;
    return true;
}

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
    float headingConfidence = 0.0f;
    int nearSamples = 0;
    int farSamples = 0;
    bool nearValid = false;
    bool farValid = false;
};

struct EdgeReliability
{
    bool reliable = false;
    bool singleEdgeUsable = false;
    int pointCount = 0;
    int interiorPointCount = 0;
    int longestBorderRun = 0;
    int leftBorderRun = 0;
    int rightBorderRun = 0;
    int expectedBorderRun = 0;
    int oppositeBorderRun = 0;
    float borderRatio = 1.0f;
    float maximumJump = 0.0f;
    bool coversBottom = false;
};

template <typename Point>
inline EdgeReliability assessEdgeReliability(const std::vector<Point> &edge,
                                             bool leftEdge, int imageWidth,
                                             int imageHeight, int rowCutBottom,
                                             int interiorPointsMinimum = 12)
{
    EdgeReliability result;
    result.pointCount = static_cast<int>(edge.size());
    if (edge.empty()) return result;
    int borderPoints = 0, currentBorderRun = 0, nearestRow = 0;
    int leftRun = 0, rightRun = 0;
    int previousColumn = edge.front().y;
    for (std::size_t i = 0; i < edge.size(); ++i)
    {
        const auto &point = edge[i];
        nearestRow = std::max(nearestRow, point.x);
        if (i > 0)
            result.maximumJump = std::max(result.maximumJump,
                static_cast<float>(std::abs(point.y - previousColumn)));
        previousColumn = point.y;
        const bool onLeftBorder = point.y <= 2;
        const bool onRightBorder = point.y >= imageWidth - 3;
        const bool onBorder = leftEdge ? onLeftBorder : onRightBorder;
        leftRun = onLeftBorder ? leftRun + 1 : 0;
        rightRun = onRightBorder ? rightRun + 1 : 0;
        result.leftBorderRun = std::max(result.leftBorderRun, leftRun);
        result.rightBorderRun = std::max(result.rightBorderRun, rightRun);
        if (onBorder)
        {
            borderPoints++;
            currentBorderRun++;
            result.longestBorderRun = std::max(result.longestBorderRun, currentBorderRun);
        }
        else currentBorderRun = 0;
        if (!onLeftBorder && !onRightBorder) result.interiorPointCount++;
    }
    result.borderRatio = static_cast<float>(borderPoints) / edge.size();
    result.coversBottom = nearestRow >= imageHeight - rowCutBottom - 4;
    const bool borderFailure = result.borderRatio > 0.25f && result.longestBorderRun >= 8;
    result.expectedBorderRun = leftEdge ? result.leftBorderRun : result.rightBorderRun;
    result.oppositeBorderRun = leftEdge ? result.rightBorderRun : result.leftBorderRun;
    const bool oppositeBorderFailure = result.oppositeBorderRun >= 3;
    result.reliable = result.pointCount >= 20 && result.coversBottom &&
                      result.maximumJump <= 30.0f && !borderFailure &&
                      !oppositeBorderFailure;
    result.singleEdgeUsable = result.pointCount >= 20 && result.coversBottom &&
        result.maximumJump <= 30.0f &&
        !oppositeBorderFailure && result.interiorPointCount >= interiorPointsMinimum &&
        (!borderFailure || result.interiorPointCount >= interiorPointsMinimum);
    return result;
}

struct SingleLaneCenterResult
{
    bool valid = false;
    int rawJump = 0;
    int appliedCenter = 0;
    int appliedStep = 0;
};

inline SingleLaneCenterResult limitSingleLaneCenter(int candidateCenter,
                                                    int previousCenter,
                                                    int maximumJump = 45,
                                                    int maximumStep = 8)
{
    SingleLaneCenterResult result;
    result.rawJump = candidateCenter - previousCenter;
    result.appliedCenter = previousCenter;
    if (std::abs(result.rawJump) > std::max(0, maximumJump)) return result;
    result.valid = true;
    result.appliedStep = std::clamp(result.rawJump,
        -std::max(0, maximumStep), std::max(0, maximumStep));
    result.appliedCenter = previousCenter + result.appliedStep;
    return result;
}

inline int reconstructSingleLaneCenterColumn(int edgeColumn, float laneWidth,
                                             bool leftEdge)
{
    const float halfWidth = std::max(0.0f, laneWidth) * 0.5f;
    return static_cast<int>(std::lround(leftEdge
        ? edgeColumn + halfWidth : edgeColumn - halfWidth));
}

template <typename Point, typename WidthProfile>
inline std::vector<Point> reconstructSingleLaneCenter(
    const std::vector<Point> &edge, const WidthProfile &laneWidthProfile,
    bool leftEdge, int imageRows, int imageColumns)
{
    std::vector<Point> center;
    center.reserve(edge.size());
    for (std::size_t i = 0; i < edge.size(); ++i)
    {
        const int row = edge[i].x;
        if (row < 0 || row >= imageRows || laneWidthProfile[row] <= 1.0f)
            continue;
        const int column = reconstructSingleLaneCenterColumn(
            edge[i].y, laneWidthProfile[row], leftEdge);
        if (column > 0 && column < imageColumns)
            center.emplace_back(row, column);
    }
    return center;
}

template <typename Point, typename WidthProfile>
inline std::vector<Point> buildDegradedLaneCenter(
    const std::vector<Point> &left, const std::vector<Point> &right,
    const WidthProfile &laneWidthProfile, int imageRows, int imageColumns,
    float maximumWidthError = 0.35f)
{
    std::vector<int> leftByRow(imageRows, -1), rightByRow(imageRows, -1);
    for (const auto &point : left)
        if (point.x >= 0 && point.x < imageRows) leftByRow[point.x] = point.y;
    for (const auto &point : right)
        if (point.x >= 0 && point.x < imageRows) rightByRow[point.x] = point.y;
    std::vector<Point> center;
    for (int row = 0; row < imageRows; ++row)
    {
        const float learnedWidth = laneWidthProfile[row];
        if (learnedWidth <= 1.0f) continue;
        const bool leftInterior = leftByRow[row] > 2 &&
                                  leftByRow[row] < imageColumns - 3;
        const bool rightInterior = rightByRow[row] > 2 &&
                                   rightByRow[row] < imageColumns - 3;
        int column = -1;
        if (leftInterior && rightInterior && rightByRow[row] > leftByRow[row])
        {
            const float width = rightByRow[row] - leftByRow[row];
            if (std::abs(width - learnedWidth) / learnedWidth <= maximumWidthError)
                column = (leftByRow[row] + rightByRow[row]) / 2;
        }
        else if (leftInterior)
            column = reconstructSingleLaneCenterColumn(
                leftByRow[row], learnedWidth, true);
        else if (rightInterior)
            column = reconstructSingleLaneCenterColumn(
                rightByRow[row], learnedWidth, false);
        if (column > 0 && column < imageColumns) center.emplace_back(row, column);
    }
    return center;
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
                                       int requiredDualLaneFrames = 5,
                                       bool recoveredSingleEdge = false)
{
    if (!strictLaneMode)
    {
        state = SingleLaneSpeedLimitState{};
        return false;
    }
    const bool singleLaneControl = controlValid &&
        ((leftReliable != rightReliable) || recoveredSingleEdge);
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
    float headingFadeError = 40.0f, float headingConfidence = 1.0f)
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
            result.headingConfidence = std::clamp(
                headingConfidence, 0.0f, 1.0f);
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
            const float recoveryDamping = oppositeSideRecovery ? 0.25f : 1.0f;
            result.headingCorrection = std::clamp(
                std::max(0.0f, headingGain) * result.headingError *
                    headingWeight * result.headingConfidence * recoveryDamping,
                -correctionLimit, correctionLimit);
            // Applied independently by Motion::poseControl as a PWM term.
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
