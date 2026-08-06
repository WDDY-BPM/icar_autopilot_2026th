#include "ctrl/lane_quality.hpp"
#include "ctrl/perception_geometry_builder.hpp"
#include "ctrl/track.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <vector>

namespace
{
using LaneQuality = Track::LaneQuality;

void fillWhite(cv::Mat &img, int row, int left, int right)
{
    if (right < left)
        return;
    for (int col = left; col <= right; ++col)
        img.at<uchar>(row, col) = 255;
}

cv::Mat leftCurveImage()
{
    cv::Mat img = cv::Mat::zeros(cv::Size(COLSIMAGE, ROWSIMAGE), CV_8U);
    // 近场：道路左边界被图像左边界裁剪（left==0），右边缘完整且平滑。
    for (int row = 220; row >= 100; --row)
        fillWhite(img, row, 0, 120 + (row - 100) / 2);
    // 远场：左右边界都可见。
    for (int row = 99; row >= 40; --row)
        fillWhite(img, row, 4 + (99 - row) * 3 / 5,
                  120 - (99 - row) / 2);
    return img;
}

cv::Mat rightCurveImage()
{
    cv::Mat img = cv::Mat::zeros(cv::Size(COLSIMAGE, ROWSIMAGE), CV_8U);
    // 近场：道路右边界被图像右边界裁剪（right==COLSIMAGE-1）。
    for (int row = 220; row >= 100; --row)
        fillWhite(img, row, 130 + (220 - row), COLSIMAGE - 1);
    for (int row = 99; row >= 40; --row)
        fillWhite(img, row, 250 - (99 - row) / 2,
                  315 - (99 - row) * 3 / 5);
    return img;
}

cv::Mat narrowRoadImage()
{
    cv::Mat img = cv::Mat::zeros(cv::Size(COLSIMAGE, ROWSIMAGE), CV_8U);
    // 中间窄道路：不贴边、宽度不足，初始色块应被拒绝。
    for (int row = 220; row >= 40; --row)
        fillWhite(img, row, 130, 190);
    return img;
}

Track makeTrack()
{
    Track track;
    track.rowCutUp = 40;
    track.rowCutBottom = 20;
    track.maxGapRows = 8;
    track.singleLaneInteriorPointsMin = 12;
    return track;
}
} // namespace

int main()
{
    // 1. 左弯起点：左边界贴边裁剪，右边缘可靠 -> RIGHT_SINGLE 可用，
    //    不得伪装成双边可靠。
    {
        Track track = makeTrack();
        track.handle(leftCurveImage());
        CHECK(!track.quality.leftReliable);
        CHECK(track.quality.leftClipped);
        CHECK(track.quality.rightReliable);
        CHECK(track.quality.rightSingleUsable);
        CHECK(track.quality.rightInteriorPoints >= 12);
        CHECK(!(track.quality.leftReliable && track.quality.rightReliable));
    }
    // 2. 右弯镜像：右边界贴边裁剪，左边缘可靠 -> LEFT_SINGLE 可用。
    {
        Track track = makeTrack();
        track.handle(rightCurveImage());
        CHECK(!track.quality.rightReliable);
        CHECK(track.quality.rightClipped);
        CHECK(track.quality.leftReliable);
        CHECK(track.quality.leftSingleUsable);
        CHECK(!(track.quality.leftReliable && track.quality.rightReliable));
    }
    // 3. 两边都不足：窄道路不贴边 -> 单边不可用。
    {
        Track track = makeTrack();
        track.handle(narrowRoadImage());
        CHECK(!track.quality.leftSingleUsable);
        CHECK(!track.quality.rightSingleUsable);
        CHECK(!track.quality.leftReliable);
        CHECK(!track.quality.rightReliable);
    }
    // 4. 边缘只出现在远场、不覆盖近场 -> 不能作为启动单边车道。
    {
        const std::vector<PointX> farEdge = {
            PointX(120, 150), PointX(110, 152), PointX(100, 154)};
        CHECK(!control_algorithms::edgeCoversNearField(
            farEdge, ROWSIMAGE - 20 - control_algorithms::kStartupBottomTolerance));
        LaneQuality quality;
        quality.rightSingleUsable = true;
        quality.rightInteriorPoints = 40;
        const auto mode = assessStartupLaneMode(
            quality, false, false, makeTestParams()->config);
        CHECK(mode == LaneRecoveryMode::INVALID);
    }
    // 5. 边缘可靠性：贴边边缘标记 clipped 且不可靠；内部边缘可用。
    {
        std::vector<PointX> leftBorderEdge;
        std::vector<PointX> rightInteriorEdge;
        for (int row = 220; row >= 40; --row)
        {
            leftBorderEdge.emplace_back(row, 0);
            rightInteriorEdge.emplace_back(row, 140 + (220 - row) / 4);
        }
        const auto leftReliability = control_algorithms::assessEdgeReliability(
            leftBorderEdge, true, COLSIMAGE, ROWSIMAGE, 20, 12);
        const auto rightReliability = control_algorithms::assessEdgeReliability(
            rightInteriorEdge, false, COLSIMAGE, ROWSIMAGE, 20, 12);
        CHECK(leftReliability.clipped);
        CHECK(!leftReliability.reliable);
        CHECK(!rightReliability.clipped);
        CHECK(rightReliability.singleEdgeUsable);
    }
    // 6. 启动车道决策：严格双边 / 稳定左单边 / 稳定右单边 / 内部点不足。
    {
        auto params = makeTestParams();
        LaneQuality dual;
        dual.leftReliable = true;
        dual.rightReliable = true;
        dual.coversBottom = true;
        dual.commonRows = 25;
        CHECK(assessStartupLaneMode(
            dual, true, true, params->config) == LaneRecoveryMode::STRICT_DUAL);

        LaneQuality rightSingle;
        rightSingle.rightSingleUsable = true;
        rightSingle.rightInteriorPoints = 30;
        CHECK(assessStartupLaneMode(
            rightSingle, false, true, params->config) ==
            LaneRecoveryMode::RIGHT_SINGLE);

        LaneQuality leftSingle;
        leftSingle.leftSingleUsable = true;
        leftSingle.leftInteriorPoints = 30;
        CHECK(assessStartupLaneMode(
            leftSingle, true, false, params->config) ==
            LaneRecoveryMode::LEFT_SINGLE);

        LaneQuality tooFew;
        tooFew.rightSingleUsable = true;
        tooFew.rightInteriorPoints = 5;
        CHECK(assessStartupLaneMode(
            tooFew, false, true, params->config) == LaneRecoveryMode::INVALID);
    }
    // 7. 启动稳定计数：连续12帧放行；任一无效帧归零。
    {
        auto params = makeTestParams();
        int stableCount = 0;
        for (int frame = 0; frame < 12; ++frame)
            stableCount = control_algorithms::advanceStartupLaneCount(
                stableCount, true);
        CHECK(stableCount == params->config.startupStableFrames);
        stableCount = control_algorithms::advanceStartupLaneCount(
            stableCount, false);
        CHECK(stableCount == 0);
    }
    return 0;
}
