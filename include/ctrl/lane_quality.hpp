#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace control_algorithms
{
// 车道判定共用具名常量（集中定义，禁止散落魔法数字）。
inline constexpr float kClippedBorderRatio = 0.55f; // 边缘贴图像边界的比例阈值
inline constexpr int kClippedBorderRun = 20;        // 边缘贴边的最长连续点数
inline constexpr int kStartupCommonRows = 20;       // 启动严格双边最小公共行数
inline constexpr int kStartupBottomTolerance = 6;   // 启动近场覆盖容差（行）
inline constexpr int kEdgeBottomTolerance = 4;      // 质量评估近场覆盖容差（行）
inline constexpr int kImageBorderMargin = 2;        // 图像边框判定裕量（像素）
inline constexpr int kWeakHybridMinCommonInteriorRows = 20; // 弱双边真实观测公共行下限

// 图像边框坐标只能表示“这一侧车道线出视野/clipped”，不能作为真实边线。
inline bool isImageBorderColumn(int column, int imageWidth)
{
    return column <= kImageBorderMargin ||
           column >= imageWidth - 1 - kImageBorderMargin;
}

inline bool isInteriorLanePoint(int column, int imageWidth)
{
    return !isImageBorderColumn(column, imageWidth);
}

// 权威的近场覆盖判定：边缘是否延伸到图像底部（近场）。
template <typename Point>
inline bool edgeCoversNearField(const std::vector<Point> &edge,
                                int bottomRequiredRow)
{
    for (const auto &point : edge)
        if (point.x >= bottomRequiredRow)
            return true;
    return false;
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
    bool oppositeSideRecovery = false;
    float recoveryDamping = 1.0f;
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
    bool clipped = false; // 本边缘被图像边界裁剪（另一侧出画面）
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
    int borderPoints = 0, currentBorderRun = 0;
    int leftRun = 0, rightRun = 0;
    int previousColumn = edge.front().y;
    for (std::size_t i = 0; i < edge.size(); ++i)
    {
        const auto &point = edge[i];
        if (i > 0)
            result.maximumJump = std::max(result.maximumJump,
                static_cast<float>(std::abs(point.y - previousColumn)));
        previousColumn = point.y;
        const bool onLeftBorder = point.y <= kImageBorderMargin;
        const bool onRightBorder =
            point.y >= imageWidth - 1 - kImageBorderMargin;
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
    result.coversBottom = edgeCoversNearField(
        edge, imageHeight - rowCutBottom - kEdgeBottomTolerance);
    result.clipped = result.borderRatio > kClippedBorderRatio ||
                     result.longestBorderRun > kClippedBorderRun;
    const bool borderFailure = result.borderRatio > 0.25f && result.longestBorderRun >= 8;
    result.expectedBorderRun = leftEdge ? result.leftBorderRun : result.rightBorderRun;
    result.oppositeBorderRun = leftEdge ? result.rightBorderRun : result.leftBorderRun;
    const bool oppositeBorderFailure = result.oppositeBorderRun >= 3;
    result.reliable = result.pointCount >= 20 && result.coversBottom &&
                      result.maximumJump <= 30.0f && !borderFailure &&
                      !oppositeBorderFailure;
    // 本边缘严重贴着其预期图像边界（borderFailure）时视为被裁剪，
    // 不能作为真实单边控制边缘；只有可靠可见边缘才能单边重建。
    result.singleEdgeUsable = result.pointCount >= 20 && result.coversBottom &&
        result.maximumJump <= 30.0f &&
        !borderFailure && !oppositeBorderFailure &&
        result.interiorPointCount >= interiorPointsMinimum;
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

} // namespace control_algorithms
