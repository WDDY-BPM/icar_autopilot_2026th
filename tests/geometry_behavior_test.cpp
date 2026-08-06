#include "ctrl/perception_geometry_builder.hpp"
#include "ctrl/planned_geometry_builder.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <algorithm>
#include <vector>

namespace
{
void setQualityDual(Track::LaneQuality &quality, bool valid)
{
    quality = Track::LaneQuality{};
    quality.valid = valid;
    quality.leftReliable = true;
    quality.rightReliable = true;
    quality.widthVariation = 0.05f;
}

std::vector<PointX> straightEdge(int startRow, int endRow, int column,
                                 int missingFrom = -1, int missingTo = -1)
{
    std::vector<PointX> edge;
    for (int row = startRow; row >= endRow; --row)
    {
        if (missingFrom >= 0 && row >= missingTo && row <= missingFrom)
            continue;
        edge.emplace_back(row, column);
    }
    return edge;
}

bool containsRow(const std::vector<PointX> &points, int row)
{
    return std::any_of(points.begin(), points.end(),
                       [&](const PointX &point) { return point.x == row; });
}
} // namespace

int main()
{
    // 1. Perfectly aligned left/right edges: row-aligned center, independent
    //    of whether the caller provides descending or ascending input order.
    {
        auto params = makeTestParams();
        Track track;
        setQualityDual(track.quality, true);
        track.pointsEdgeLeft = straightEdge(220, 40, 80);
        track.pointsEdgeRight = straightEdge(220, 40, 240);
        PlannedLaneWidthModel widthModel;
        const auto descending = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(descending.centerLine.size() == 181);
        CHECK(descending.centerLine.front().x == 220);
        CHECK(descending.centerLine.back().x == 40);
        CHECK(descending.centerLine.front().y == 160);
        CHECK(descending.recoveryMode == LaneRecoveryMode::STRICT_DUAL);
        CHECK(descending.nearSamples == 45);
        CHECK(descending.geometryContinuous);
        CHECK(descending.candidateValid);

        std::reverse(track.pointsEdgeLeft.begin(), track.pointsEdgeLeft.end());
        std::reverse(track.pointsEdgeRight.begin(), track.pointsEdgeRight.end());
        const auto ascending = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(ascending.centerLine.size() == descending.centerLine.size());
        CHECK(ascending.centerLine.front().x ==
              descending.centerLine.front().x);
        CHECK(ascending.candidateValid);
    }
    // 2/3/4. A missing row on either side (or different rows on each side)
    //         is interpolated; the center stays complete.
    {
        auto params = makeTestParams();
        Track track;
        setQualityDual(track.quality, true);
        track.pointsEdgeLeft = straightEdge(220, 40, 80, 150, 150);
        track.pointsEdgeRight = straightEdge(220, 40, 240);
        PlannedLaneWidthModel widthModel;
        auto result = buildPerceptionGeometry(track, widthModel, params->config);
        CHECK(result.centerLine.size() == 181);
        CHECK(containsRow(result.centerLine, 150));

        track.pointsEdgeLeft = straightEdge(220, 40, 80);
        track.pointsEdgeRight = straightEdge(220, 40, 240, 120, 120);
        result = buildPerceptionGeometry(track, widthModel, params->config);
        CHECK(result.centerLine.size() == 181);
        CHECK(containsRow(result.centerLine, 120));

        track.pointsEdgeLeft = straightEdge(220, 40, 80, 150, 150);
        track.pointsEdgeRight = straightEdge(220, 40, 240, 120, 120);
        result = buildPerceptionGeometry(track, widthModel, params->config);
        CHECK(result.centerLine.size() == 181);
        CHECK(containsRow(result.centerLine, 150));
        CHECK(containsRow(result.centerLine, 120));
    }
    // 5. Short gaps (3 missing rows) are interpolated without splitting.
    {
        auto params = makeTestParams();
        Track track;
        setQualityDual(track.quality, true);
        track.pointsEdgeLeft = straightEdge(220, 40, 80, 150, 148);
        track.pointsEdgeRight = straightEdge(220, 40, 240, 150, 148);
        PlannedLaneWidthModel widthModel;
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(result.centerLine.size() == 181);
        CHECK(containsRow(result.centerLine, 149));
        CHECK(result.candidateValid);
    }
    // 6. A large far-end gap is not interpolated and must not reject the
    //    near-side control segment.
    {
        auto params = makeTestParams();
        Track track;
        setQualityDual(track.quality, true);
        track.pointsEdgeLeft = straightEdge(220, 40, 80, 109, 100);
        track.pointsEdgeRight = straightEdge(220, 40, 240, 109, 100);
        PlannedLaneWidthModel widthModel;
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(containsRow(result.centerLine, 220));
        CHECK(containsRow(result.centerLine, 110));
        CHECK(!containsRow(result.centerLine, 100));
        CHECK(!containsRow(result.centerLine, 99));
        CHECK(result.nearSamples == 45);
        CHECK(result.candidateValid);
    }
    // 7. Planned geometry builder applies the same far-gap rule.
    {
        auto params = makeTestParams();
        PathOverride path;
        path.setEdgesForFrames(
            PathSource::PARK,
            straightEdge(220, 40, 80, 109, 100),
            straightEdge(220, 40, 240, 109, 100),
            0.7f, 0.15f, 2);
        const auto result = buildPlannedGeometry(path, FsmMode::PARK);
        CHECK(result.valid);
        CHECK(containsRow(result.centerLine, 220));
        CHECK(!containsRow(result.centerLine, 100));
        CHECK(result.centerLine.front().y == 160);
    }
    // 8. Planned geometry with a large near-field-only gap picks the longest
    //    near-field segment instead of rejecting the whole path.
    {
        auto params = makeTestParams();
        PathOverride path;
        path.setEdgesForFrames(
            PathSource::PARK,
            straightEdge(220, 40, 80, 110, 101),
            straightEdge(220, 40, 240, 110, 101),
            0.7f, 0.15f, 2);
        const auto result = buildPlannedGeometry(path, FsmMode::PARK);
        CHECK(result.valid);
        CHECK(!result.centerLine.empty());
        CHECK(!containsRow(result.centerLine, 110));
        CHECK(!containsRow(result.centerLine, 101));
    }
    // 9. No near-field rows at all -> no usable segment -> invalid.
    {
        auto params = makeTestParams();
        PathOverride path;
        path.setEdgesForFrames(
            PathSource::PARK,
            straightEdge(150, 40, 80),
            straightEdge(150, 40, 240),
            0.7f, 0.15f, 2);
        CHECK(!buildPlannedGeometry(path, FsmMode::PARK).valid);
    }
    // 10. Single-edge recovery uses per-row learned width and only falls back
    //     to the flat width for rows without a learned value.
    {
        auto params = makeTestParams();
        Track track;
        track.quality = Track::LaneQuality{};
        track.quality.leftSingleUsable = true;
        track.pointsEdgeLeft = straightEdge(220, 40, 80);
        PlannedLaneWidthModel widthModel;
        widthModel.ready = true;
        widthModel.fallbackWidth = 96.0f;
        for (int row = 176; row <= 220; ++row)
            widthModel.widthByRow[row] = 100.0f; // learned width for near rows
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(result.recoveryMode == LaneRecoveryMode::LEFT_SINGLE);
        CHECK(result.singleSide == -1);
        CHECK(result.centerLine.size() == 181);
        // Near rows use the learned width: 80 + 50 = 130.
        CHECK(containsRow(result.centerLine, 200));
        const auto nearPoint = std::find_if(
            result.centerLine.begin(), result.centerLine.end(),
            [](const PointX &point) { return point.x == 200; });
        CHECK(nearPoint->y == 130);
        // Far rows without a learned width use the fallback: 80 + 48 = 128.
        const auto farPoint = std::find_if(
            result.centerLine.begin(), result.centerLine.end(),
            [](const PointX &point) { return point.x == 100; });
        CHECK(farPoint->y == 128);
        CHECK(result.candidateValid);
    }
    // 11. Single-edge recovery excludes rows whose reconstructed column would
    //     fall outside the image (boundary limiting).
    {
        auto params = makeTestParams();
        Track track;
        track.quality = Track::LaneQuality{};
        track.quality.leftSingleUsable = true;
        track.pointsEdgeLeft = straightEdge(220, 40, 310);
        PlannedLaneWidthModel widthModel;
        widthModel.ready = true;
        widthModel.fallbackWidth = 100.0f;
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(result.centerLine.empty());
        CHECK(!result.candidateValid);
    }
    // 12. Reliable right single edge -> RIGHT_SINGLE with near-field samples.
    {
        auto params = makeTestParams();
        Track track;
        track.quality = Track::LaneQuality{};
        track.quality.rightSingleUsable = true;
        track.pointsEdgeRight = straightEdge(220, 40, 240);
        PlannedLaneWidthModel widthModel;
        widthModel.ready = true;
        widthModel.fallbackWidth = 96.0f;
        for (int row = 176; row <= 220; ++row)
            widthModel.widthByRow[row] = 100.0f;
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(result.recoveryMode == LaneRecoveryMode::RIGHT_SINGLE);
        CHECK(result.singleSide == 1);
        CHECK(!result.centerLine.empty());
        CHECK(result.nearSamples >= 6);
        CHECK(result.candidateValid);
    }
    // 13. Reconstructed single center with an excessive jump must be invalid.
    {
        auto params = makeTestParams();
        Track track;
        track.quality = Track::LaneQuality{};
        track.quality.leftSingleUsable = true;
        track.pointsEdgeLeft = straightEdge(220, 120, 80);
        const auto far = straightEdge(119, 40, 200);
        track.pointsEdgeLeft.insert(
            track.pointsEdgeLeft.end(), far.begin(), far.end());
        const auto result = buildPerceptionGeometry(
            track, PlannedLaneWidthModel{}, params->config);
        CHECK(result.recoveryMode == LaneRecoveryMode::LEFT_SINGLE);
        CHECK(!result.geometryContinuous);
        CHECK(!result.candidateValid);
    }
    // 14. Width model not ready: safe fallback, bounded output, no crash.
    {
        auto params = makeTestParams();
        Track track;
        track.quality = Track::LaneQuality{};
        track.quality.leftSingleUsable = true;
        track.pointsEdgeLeft = straightEdge(220, 40, 60);
        PlannedLaneWidthModel widthModel; // ready=false, fallbackWidth=96
        const auto result = buildPerceptionGeometry(
            track, widthModel, params->config);
        CHECK(result.recoveryMode == LaneRecoveryMode::LEFT_SINGLE);
        CHECK(!result.centerLine.empty());
        bool inRange = true;
        for (const auto &point : result.centerLine)
            if (point.y < 0 || point.y >= COLSIMAGE)
                inRange = false;
        CHECK(inRange);
        CHECK(result.candidateValid);
    }
    return 0;
}
