#include "ctrl/control_algorithms.hpp"
#include <cassert>
#include <cmath>
#include <vector>

struct TestPoint
{
    int x;
    int y;
    TestPoint(int row, int column) : x(row), y(column) {}
};

int main()
{
    control_algorithms::StopReasonState stopReasons;
    stopReasons.set(control_algorithms::StopReason::CAMERA, true);
    stopReasons.set(control_algorithms::StopReason::EMERGENCY, true);
    assert(stopReasons.mustStop());
    stopReasons.set(control_algorithms::StopReason::CAMERA, false);
    assert(stopReasons.mustStop());
    assert(stopReasons.has(control_algorithms::StopReason::EMERGENCY));
    assert(stopReasons.string() == "EMERGENCY");
    stopReasons.set(control_algorithms::StopReason::EMERGENCY, false);
    assert(!stopReasons.mustStop());

    int count = 0;
    const float crossStart = control_algorithms::applyStartupSpeed(
        0.35f, count, 60, 0.10f);
    assert(crossStart > 0.10f && crossStart < 0.105f);
    float startupEnd = crossStart;
    for (int i = 1; i < 60; ++i)
        startupEnd = control_algorithms::applyStartupSpeed(
            0.35f, count, 60, 0.10f);
    assert(std::abs(startupEnd - 0.35f) < 0.001f);
    const float afterStartup = control_algorithms::applyStartupSpeed(
        0.35f, count, 60, 0.10f);
    assert(std::abs(afterStartup - startupEnd) < 0.001f);

    assert(control_algorithms::applyStartupServoLimit(
        1880, 1500, 160, 0, 60) == 1660);

    assert(control_algorithms::isSingleLaneCenterContinuous(175, 160));
    assert(!control_algorithms::isSingleLaneCenterContinuous(176, 160));

    control_algorithms::LaneRecoveryState lane;
    for (int i = 0; i < 7; ++i)
        assert(!control_algorithms::updateLaneRecovery(lane, false));
    assert(lane.invalidFrames == 7);
    for (int i = 0; i < 4; ++i)
        assert(!control_algorithms::updateLaneRecovery(lane, true));
    assert(control_algorithms::updateLaneRecovery(lane, true));
    assert(lane.invalidFrames == 0);
    assert(!lane.recovering);

    bool safetyStop = false;
    safetyStop = control_algorithms::updateLaneSafetyStop(
        safetyStop, true, false, 1, 0);
    assert(!safetyStop);
    control_algorithms::LaneUnconfirmedState unconfirmed;
    for (int i = 0; i < 30; ++i)
        control_algorithms::updateLaneUnconfirmed(unconfirmed, false, 5);
    assert(unconfirmed.frames == 30);
    assert(control_algorithms::updateLaneSafetyStop(
        false, true, false, 1, 0, 7, 5, unconfirmed.frames, 30));
    for (int recoveryFrame = 1; recoveryFrame <= 4; ++recoveryFrame)
    {
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, 0, recoveryFrame);
        assert(!safetyStop);
    }
    safetyStop = false;
    for (int invalidFrame = 1; invalidFrame <= 7; ++invalidFrame)
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, invalidFrame, 0);
    assert(safetyStop);
    for (int recoveryFrame = 1; recoveryFrame <= 4; ++recoveryFrame)
    {
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, 0, recoveryFrame);
        assert(safetyStop);
    }
    safetyStop = control_algorithms::updateLaneSafetyStop(
        safetyStop, true, true, 0, 5);
    assert(!safetyStop);

    control_algorithms::SingleLaneSpeedLimitState speedLimit;
    assert(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, true, false));
    for (int i = 0; i < 4; ++i)
        assert(control_algorithms::updateSingleLaneSpeedLimit(
            speedLimit, true, true, true, true));
    assert(speedLimit.dualLaneRecoveryFrames == 4);
    assert(!control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, true, true));
    assert(!speedLimit.active);
    assert(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, false, true));
    assert(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, false, false, false));
    assert(speedLimit.dualLaneRecoveryFrames == 0);
    assert(!control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, false, true, false, true));

    std::vector<TestPoint> laneControlCenter;
    for (int row = 120; row <= 175; ++row)
        laneControlCenter.emplace_back(row, 125);
    for (int row = 180; row <= 220; ++row)
        laneControlCenter.emplace_back(row, 185);
    const auto laneCenters =
        control_algorithms::calculateLaneControlCenters(
            laneControlCenter, 160.0f);
    assert(laneCenters.nearValid && laneCenters.farValid);
    assert(laneCenters.nearSamples == 41);
    assert(laneCenters.farSamples == 36);
    assert(std::abs(laneCenters.nearCenter - 185.0f) < 0.001f);
    assert(std::abs(laneCenters.farCenter - 125.0f) < 0.001f);
    assert(std::abs(laneCenters.controlCenter - 164.0f) < 0.001f);
    assert(std::abs(laneCenters.headingError - std::atan2(-60.0f, 85.0f)) < 0.001f);
    assert(std::abs(laneCenters.headingCorrection) < 0.001f);

    std::vector<TestPoint> earlyRightHeading;
    for (int row = 120; row <= 175; ++row)
        earlyRightHeading.emplace_back(row, 162);
    for (int row = 180; row <= 220; ++row)
        earlyRightHeading.emplace_back(row, 157);
    const auto headingAssisted =
        control_algorithms::calculateLaneControlCenters(
            earlyRightHeading, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f);
    const float expectedHeading = std::atan2(5.0f, 85.0f);
    assert(std::abs(headingAssisted.headingError - expectedHeading) < 0.001f);
    assert(std::abs(headingAssisted.headingCorrection -
                    300.0f * expectedHeading * 0.925f) < 0.001f);
    assert(std::abs(headingAssisted.controlCenter - 158.75f) < 0.001f);

    const auto oppositeSideRecovery =
        control_algorithms::calculateLaneControlCenters(
            laneControlCenter, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f);
    const float recoveryHeading = std::atan2(-60.0f, 85.0f);
    assert(std::abs(oppositeSideRecovery.headingCorrection -
                    300.0f * recoveryHeading * 0.375f * 0.25f) < 0.001f);
    assert(std::abs(oppositeSideRecovery.controlCenter - 164.0f) < 0.001f);

    std::vector<TestPoint> cappedHeading;
    for (int row = 120; row <= 175; ++row)
        cappedHeading.emplace_back(row, 200);
    for (int row = 180; row <= 220; ++row)
        cappedHeading.emplace_back(row, 160);
    const auto cappedHeadingAssist =
        control_algorithms::calculateLaneControlCenters(
            cappedHeading, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f);
    assert(std::abs(cappedHeadingAssist.headingCorrection - 60.0f) < 0.001f);
    assert(std::abs(cappedHeadingAssist.controlCenter - 174.0f) < 0.001f);

    const auto confidenceLimitedHeading =
        control_algorithms::calculateLaneControlCenters(
            earlyRightHeading, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f, 0.45f);
    assert(std::abs(confidenceLimitedHeading.headingConfidence - 0.45f) < 0.001f);
    assert(std::abs(confidenceLimitedHeading.headingCorrection -
                    headingAssisted.headingCorrection * 0.45f) < 0.001f);

    std::vector<TestPoint> missingNearCenter;
    for (int row = 120; row <= 175; ++row)
        missingNearCenter.emplace_back(row, 125);
    for (int row = 180; row < 187; ++row)
        missingNearCenter.emplace_back(row, 185);
    const auto missingNear =
        control_algorithms::calculateLaneControlCenters(
            missingNearCenter, 160.0f);
    assert(!missingNear.nearValid && missingNear.farValid);
    assert(std::abs(missingNear.farCenter - 125.0f) < 0.001f);
    assert(std::abs(missingNear.controlCenter - 160.0f) < 0.001f);

    std::vector<TestPoint> straightCenter;
    for (int row = 90; row <= 210; ++row)
        straightCenter.emplace_back(row, 160);
    const auto straightSpeed = control_algorithms::calculateCenterlineSpeed(
        straightCenter, 0.35f, 0.25f);
    assert(straightSpeed.valid);
    assert(std::abs(straightSpeed.curveStrength) < 0.001f);
    assert(std::abs(straightSpeed.speed - 0.35f) < 0.001f);

    std::vector<TestPoint> curvedCenter;
    for (int row = 90; row <= 130; ++row)
        curvedCenter.emplace_back(row, 120);
    for (int row = 170; row <= 210; ++row)
        curvedCenter.emplace_back(row, 160);
    const auto curvedSpeed = control_algorithms::calculateCenterlineSpeed(
        curvedCenter, 0.35f, 0.25f);
    assert(curvedSpeed.valid);
    assert(std::abs(curvedSpeed.curveStrength - 40.0f) < 0.001f);
    assert(std::abs(curvedSpeed.speed - 0.25f) < 0.001f);

    std::vector<TestPoint> mediumCurve;
    for (int row = 90; row <= 130; ++row)
        mediumCurve.emplace_back(row, 140);
    for (int row = 170; row <= 210; ++row)
        mediumCurve.emplace_back(row, 160);
    const auto mediumSpeed = control_algorithms::calculateCenterlineSpeed(
        mediumCurve, 0.35f, 0.25f);
    assert(mediumSpeed.valid);
    assert(mediumSpeed.speed > 0.25f && mediumSpeed.speed < 0.35f);

    std::vector<TestPoint> partialCenter{{175, 160}, {180, 160}, {185, 160}};
    const auto partialSpeed = control_algorithms::calculateCenterlineSpeed(
        partialCenter, 0.35f, 0.25f);
    assert(!partialSpeed.valid);
    assert(std::abs(partialSpeed.speed - 0.25f) < 0.001f);

    std::vector<TestPoint> goodLeft;
    std::vector<TestPoint> stuckLeft;
    for (int row = 220; row >= 190; --row)
    {
        goodLeft.emplace_back(row, 40 + (220 - row) / 3);
        stuckLeft.emplace_back(row, 0);
    }
    const auto goodReliability = control_algorithms::assessEdgeReliability(
        goodLeft, true, 320, 240, 20);
    const auto stuckReliability = control_algorithms::assessEdgeReliability(
        stuckLeft, true, 320, 240, 20);
    assert(goodReliability.reliable);
    assert(!stuckReliability.reliable);
    assert(stuckReliability.borderRatio > 0.99f);
    assert(stuckReliability.longestBorderRun == 31);

    std::vector<TestPoint> clippedRight;
    for (int row = 220; row >= 190; --row)
    {
        const int index = 220 - row;
        clippedRight.emplace_back(row, index < 19 ? 319 : 300);
    }
    const auto clippedReliability = control_algorithms::assessEdgeReliability(
        clippedRight, false, 320, 240, 20, 12);
    assert(!clippedReliability.reliable);
    assert(clippedReliability.singleEdgeUsable);
    assert(clippedReliability.interiorPointCount == 12);
    std::vector<float> learnedWidths(240, 240.0f);
    const auto clippedCenter = control_algorithms::reconstructSingleLaneCenter(
        clippedRight, learnedWidths, false, 240, 320);
    assert(clippedCenter.size() == clippedRight.size());
    assert(clippedCenter.size() >= 20);
    const auto clippedControl = control_algorithms::limitSingleLaneCenter(
        clippedCenter.front().y, 160, 45, 8);
    assert(clippedControl.valid);
    control_algorithms::SingleLaneSpeedLimitState clippedSpeedLimit;
    assert(control_algorithms::updateSingleLaneSpeedLimit(
        clippedSpeedLimit, true, true, false, false, 5, true));

    std::vector<TestPoint> mostlyBorderRight;
    for (int row = 220; row >= 190; --row)
    {
        const int index = 220 - row;
        mostlyBorderRight.emplace_back(row, index < 22 ? 319 : 300);
    }
    const auto mostlyBorderReliability = control_algorithms::assessEdgeReliability(
        mostlyBorderRight, false, 320, 240, 20, 12);
    assert(!mostlyBorderReliability.reliable);
    assert(!mostlyBorderReliability.singleEdgeUsable);
    assert(mostlyBorderReliability.interiorPointCount == 9);

    std::vector<TestPoint> wrongLeft, wrongRight;
    for (int row = 220; row >= 190; --row)
    {
        wrongLeft.emplace_back(row, 319);
        wrongRight.emplace_back(row, 0);
    }
    const auto wrongLeftReliability = control_algorithms::assessEdgeReliability(
        wrongLeft, true, 320, 240, 20, 12);
    const auto wrongRightReliability = control_algorithms::assessEdgeReliability(
        wrongRight, false, 320, 240, 20, 12);
    assert(!wrongLeftReliability.reliable && !wrongLeftReliability.singleEdgeUsable);
    assert(!wrongRightReliability.reliable && !wrongRightReliability.singleEdgeUsable);
    assert(wrongLeftReliability.oppositeBorderRun == 31);
    assert(wrongRightReliability.oppositeBorderRun == 31);

    std::vector<TestPoint> hybridLeft, hybridRight;
    for (int row = 176; row <= 220; ++row)
    {
        hybridLeft.emplace_back(row, row % 3 == 0 ? 0 : 40);
        hybridRight.emplace_back(row, row % 3 == 1 ? 319 : 280);
    }
    const auto hybridCenter = control_algorithms::buildDegradedLaneCenter(
        hybridLeft, hybridRight, learnedWidths, 240, 320);
    assert(hybridCenter.size() >= 30);
    const auto hybridNear = control_algorithms::calculateCenterWindow(
        hybridCenter, 160.0f, 176, 220, 205, 26, 8);
    assert(hybridNear.valid && hybridNear.samples >= 8);
    std::vector<TestPoint> bothBorders{{200, 0}};
    std::vector<TestPoint> bothBordersRight{{200, 319}};
    assert(control_algorithms::buildDegradedLaneCenter(
        bothBorders, bothBordersRight, learnedWidths, 240, 320).empty());

    assert(control_algorithms::reconstructSingleLaneCenterColumn(
        40, 240.0f, true) == 160);
    assert(control_algorithms::reconstructSingleLaneCenterColumn(
        280, 240.0f, false) == 160);
    const auto acceptedCenter = control_algorithms::limitSingleLaneCenter(
        190, 160, 45, 8);
    assert(acceptedCenter.valid && acceptedCenter.rawJump == 30);
    assert(acceptedCenter.appliedCenter == 168 && acceptedCenter.appliedStep == 8);
    const auto rejectedCenter = control_algorithms::limitSingleLaneCenter(
        206, 160, 45, 8);
    assert(!rejectedCenter.valid && rejectedCenter.appliedCenter == 160);

    std::vector<TestPoint> left{{220, 40}, {216, 44}};
    std::vector<TestPoint> right{{220, 280}, {216, 276}};
    std::vector<TestPoint> width{{220, 240}, {216, 232}};
    assert(control_algorithms::fillAlignedLaneGaps(left, right, width, 8));
    assert(left.size() == 5 && right.size() == 5 && width.size() == 5);
    for (std::size_t i = 0; i < left.size(); ++i)
        assert(width[i].y == right[i].y - left[i].y);
    return 0;
}
