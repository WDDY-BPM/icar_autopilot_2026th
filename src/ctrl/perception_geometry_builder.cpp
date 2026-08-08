#include "ctrl/perception_geometry_builder.hpp"
#include "ctrl/row_aligned_geometry.hpp"

#include <algorithm>
#include <cmath>

namespace
{
struct WidthProfileWithFallback
{
    const PlannedLaneWidthModel &model;

    float operator[](int row) const
    {
        return model.ready && model.widthByRow[row] > 1.0f
            ? model.widthByRow[row] : model.fallbackWidth;
    }
};

std::vector<PointX> singleCenter(const std::vector<PointX> &edge,
                                 bool leftEdge,
                                 const PlannedLaneWidthModel &widthModel)
{
    // Per-row learned width when available; fallback only for rows without a
    // learned value. Input order (near -> far) is preserved by the shared
    // algorithm, and the output is clamped to the image columns.
    WidthProfileWithFallback profile{widthModel};
    return control_algorithms::reconstructSingleLaneCenter(
        edge, profile, leftEdge, ROWSIMAGE, COLSIMAGE);
}
}

PerceptionGeometryResult buildPerceptionGeometry(
    const Track &track,
    const PlannedLaneWidthModel &laneWidthModel,
    const Config &config)
{
    PerceptionGeometryResult result;
    const auto &quality = track.quality;
    const GeometryBuildOptions options{
        ROWSIMAGE, COLSIMAGE,
        /*maximumInterpolationGap=*/6,
        /*maximumAcceptedCenterGap=*/8,
        /*nearFieldStartRow=*/ROWSIMAGE - 64};
    const auto selectedSingleMode = selectSingleLaneMode(quality);
    if (quality.leftReliable && quality.rightReliable)
    {
        result.centerLine = buildDualCenterByRow(
            track.pointsEdgeLeft, track.pointsEdgeRight,
            options.rows, options.columns,
            options.maximumInterpolationGap,
            options.maximumAcceptedCenterGap,
            options.nearFieldStartRow);
        result.recoveryMode = quality.valid
            ? LaneRecoveryMode::STRICT_DUAL
            : LaneRecoveryMode::RELAXED_DUAL;
        result.widthConsistent = quality.widthVariation <= 0.30f;
    }
    else if (selectedSingleMode == LaneRecoveryMode::LEFT_SINGLE)
    {
        result.centerLine = singleCenter(
            track.pointsEdgeLeft, true, laneWidthModel);
        result.singleSide = -1;
        result.recoveryMode = LaneRecoveryMode::LEFT_SINGLE;
    }
    else if (selectedSingleMode == LaneRecoveryMode::RIGHT_SINGLE)
    {
        result.centerLine = singleCenter(
            track.pointsEdgeRight, false, laneWidthModel);
        result.singleSide = 1;
        result.recoveryMode = LaneRecoveryMode::RIGHT_SINGLE;
    }
    else if (!track.pointsEdgeLeft.empty() && !track.pointsEdgeRight.empty())
    {
        // 图像边框不是真实车道边线：只有两侧都存在足够真实内部点
        // （公共行内左右都不是边框）才允许 WEAK_HYBRID 双边构造；
        // 否则保持 INVALID，禁止用假边线拼凑中心线。
        if (quality.commonInteriorRows >=
            control_algorithms::kWeakHybridMinCommonInteriorRows)
        {
            std::vector<PointX> interiorLeft;
            std::vector<PointX> interiorRight;
            interiorLeft.reserve(track.pointsEdgeLeft.size());
            interiorRight.reserve(track.pointsEdgeRight.size());
            for (const auto &point : track.pointsEdgeLeft)
                if (control_algorithms::isInteriorLanePoint(
                        point.y, options.columns))
                    interiorLeft.push_back(point);
            for (const auto &point : track.pointsEdgeRight)
                if (control_algorithms::isInteriorLanePoint(
                        point.y, options.columns))
                    interiorRight.push_back(point);
            result.centerLine = buildDualCenterByRow(
                interiorLeft, interiorRight,
                options.rows, options.columns,
                options.maximumInterpolationGap,
                options.maximumAcceptedCenterGap,
                options.nearFieldStartRow);
            result.recoveryMode = LaneRecoveryMode::WEAK_HYBRID;
        }
    }
    result.nearSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x >= 176 && point.x <= 220; }));
    result.farSamples = static_cast<int>(std::count_if(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &point) { return point.x < 176; }));
    const bool continuous = std::adjacent_find(
        result.centerLine.begin(), result.centerLine.end(),
        [](const PointX &left, const PointX &right) {
            return std::abs(left.y - right.y) > 45;
        }) == result.centerLine.end();
    result.geometryContinuous = continuous;
    const bool weakHybrid =
        result.recoveryMode == LaneRecoveryMode::WEAK_HYBRID;
    const bool singleMode =
        result.recoveryMode == LaneRecoveryMode::LEFT_SINGLE ||
        result.recoveryMode == LaneRecoveryMode::RIGHT_SINGLE;
    const int minimumCenterPoints = weakHybrid
        ? std::max(8, config.singleLaneInteriorPointsMin)
        : singleMode
        ? std::max(12, config.singleLaneInteriorPointsMin)
        : 20;
    const int minimumNearSamples =
        weakHybrid ? 8 : singleMode ? 6 : 4;
    const bool strictDual =
        result.recoveryMode == LaneRecoveryMode::STRICT_DUAL;
    const bool strictDualAcceptable =
        !strictDual || (result.widthConsistent && quality.valid);
    result.candidateValid =
        static_cast<int>(result.centerLine.size()) >= minimumCenterPoints &&
        result.nearSamples >= minimumNearSamples && continuous &&
        strictDualAcceptable;
    return result;
}
