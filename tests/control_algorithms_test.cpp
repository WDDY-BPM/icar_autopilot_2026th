#include "ctrl/control_algorithms.hpp"
#include "com/control_watchdog.hpp"
#include "test_check.hpp"
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
    {
        ControlWatchdogState watchdog;
        watchdog.onConnected(0);
        CHECK(!watchdog.armed());
        CHECK(!watchdog.expired(30000, 500));

        constexpr std::uint8_t controlAddress = 1;
        CHECK(!watchdog.onValidFrame(2, controlAddress, 30000));
        CHECK(!watchdog.armed());
        CHECK(!watchdog.onValidFrame(3, controlAddress, 30100));
        CHECK(!watchdog.armed());

        CHECK(watchdog.onValidFrame(controlAddress, controlAddress, 31000));
        CHECK(watchdog.armed());
        CHECK(!watchdog.expired(31499, 500));
        CHECK(!watchdog.expired(31500, 500));
        CHECK(watchdog.expired(31501, 500));

        watchdog.onValidControlFrame(32000);
        CHECK(!watchdog.expired(32500, 500));
        CHECK(watchdog.expired(32501, 500));

        watchdog.reset();
        CHECK(!watchdog.armed());
        CHECK(!watchdog.expired(90000, 500));
        watchdog.onConnected(120000);
        CHECK(!watchdog.armed());
        CHECK(!watchdog.expired(120000, 500));
    }
    {
        int busyCountdown = 40;
        int alertCountdown = 0;
        int decelCountdown = 0;
        std::vector<int> busyBeeps;
        for (int frame = 0; frame < 40; ++frame)
        {
            const auto events = control_algorithms::advanceAlertTimers(
                busyCountdown, alertCountdown, decelCountdown, false);
            if (events.busyBeep)
                busyBeeps.push_back(40 - frame);
        }
        CHECK((busyBeeps == std::vector<int>{40, 30, 20, 10}));
        CHECK(busyCountdown == 0);

        control_algorithms::refreshAlertEvidence(
            true, true, alertCountdown);
        CHECK(alertCountdown == 30);
        for (int frame = 0; frame < 30; ++frame)
            control_algorithms::advanceAlertTimers(
                busyCountdown, alertCountdown, decelCountdown, false);
        CHECK(alertCountdown == 0);

        control_algorithms::refreshAlertEvidence(
            false, true, alertCountdown);
        CHECK(alertCountdown == 0);
        control_algorithms::refreshAlertEvidence(
            true, false, alertCountdown);
        CHECK(alertCountdown == 0);
        control_algorithms::refreshAlertEvidence(
            true, true, alertCountdown);
        CHECK(alertCountdown == 30);

        auto events = control_algorithms::advanceAlertTimers(
            busyCountdown, alertCountdown, decelCountdown, true);
        CHECK(decelCountdown == 5);
        CHECK(!events.busyBeep);
        events = control_algorithms::advanceAlertTimers(
            busyCountdown, alertCountdown, decelCountdown, false);
        CHECK(decelCountdown == 4);
    }
    control_algorithms::StopReasonState stopReasons;
    stopReasons.set(control_algorithms::StopReason::CAMERA, true);
    stopReasons.set(control_algorithms::StopReason::EMERGENCY, true);
    CHECK(stopReasons.mustStop());
    stopReasons.set(control_algorithms::StopReason::CAMERA, false);
    CHECK(stopReasons.mustStop());
    CHECK(stopReasons.has(control_algorithms::StopReason::EMERGENCY));
    CHECK(stopReasons.string() == "EMERGENCY");
    stopReasons.set(control_algorithms::StopReason::EMERGENCY, false);
    CHECK(!stopReasons.mustStop());

    int alertCountdown = 21;
    CHECK(!control_algorithms::advanceAlertCountdown(alertCountdown));
    CHECK(alertCountdown == 20);
    CHECK(control_algorithms::advanceAlertCountdown(alertCountdown));
    CHECK(alertCountdown == 19);
    for (int i = 0; i < 19; ++i)
        control_algorithms::advanceAlertCountdown(alertCountdown);
    CHECK(alertCountdown == 0);
    CHECK(!control_algorithms::advanceAlertCountdown(alertCountdown));
    CHECK(control_algorithms::updateAlertDecelCountdown(0, true) == 5);
    CHECK(control_algorithms::updateAlertDecelCountdown(5, false) == 4);
    CHECK(control_algorithms::updateAlertDecelCountdown(0, false) == 0);
    int count = 0;
    const float crossStart = control_algorithms::applyStartupSpeed(
        0.35f, count, 60, 0.10f);
    CHECK(crossStart > 0.10f && crossStart < 0.105f);
    float startupEnd = crossStart;
    for (int i = 1; i < 60; ++i)
        startupEnd = control_algorithms::applyStartupSpeed(
            0.35f, count, 60, 0.10f);
    CHECK(std::abs(startupEnd - 0.35f) < 0.001f);
    const float afterStartup = control_algorithms::applyStartupSpeed(
        0.35f, count, 60, 0.10f);
    CHECK(std::abs(afterStartup - startupEnd) < 0.001f);

    CHECK(control_algorithms::applyStartupServoLimit(
        1880, 1500, 160, 0, 60) == 1660);

    CHECK(control_algorithms::isSingleLaneCenterContinuous(175, 160));
    CHECK(!control_algorithms::isSingleLaneCenterContinuous(176, 160));

    control_algorithms::LaneRecoveryState lane;
    for (int i = 0; i < 7; ++i)
        CHECK(!control_algorithms::updateLaneRecovery(lane, false));
    CHECK(lane.invalidFrames == 7);
    for (int i = 0; i < 4; ++i)
        CHECK(!control_algorithms::updateLaneRecovery(lane, true));
    CHECK(control_algorithms::updateLaneRecovery(lane, true));
    CHECK(lane.invalidFrames == 0);
    CHECK(!lane.recovering);

    bool safetyStop = false;
    safetyStop = control_algorithms::updateLaneSafetyStop(
        safetyStop, true, false, 1, 0);
    CHECK(!safetyStop);
    control_algorithms::LaneUnconfirmedState unconfirmed;
    for (int i = 0; i < 30; ++i)
        control_algorithms::updateLaneUnconfirmed(unconfirmed, false, 5);
    CHECK(unconfirmed.frames == 30);
    CHECK(control_algorithms::updateLaneSafetyStop(
        false, true, false, 1, 0, 7, 5, unconfirmed.frames, 30));
    for (int recoveryFrame = 1; recoveryFrame <= 4; ++recoveryFrame)
    {
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, 0, recoveryFrame);
        CHECK(!safetyStop);
    }
    safetyStop = false;
    for (int invalidFrame = 1; invalidFrame <= 7; ++invalidFrame)
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, invalidFrame, 0);
    CHECK(safetyStop);
    for (int recoveryFrame = 1; recoveryFrame <= 4; ++recoveryFrame)
    {
        safetyStop = control_algorithms::updateLaneSafetyStop(
            safetyStop, true, false, 0, recoveryFrame);
        CHECK(safetyStop);
    }
    safetyStop = control_algorithms::updateLaneSafetyStop(
        safetyStop, true, true, 0, 5);
    CHECK(!safetyStop);

    control_algorithms::SingleLaneSpeedLimitState speedLimit;
    CHECK(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, true, false));
    for (int i = 0; i < 4; ++i)
        CHECK(control_algorithms::updateSingleLaneSpeedLimit(
            speedLimit, true, true, true, true));
    CHECK(speedLimit.dualLaneRecoveryFrames == 4);
    CHECK(!control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, true, true));
    CHECK(!speedLimit.active);
    CHECK(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, true, false, true));
    CHECK(control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, true, false, false, false));
    CHECK(speedLimit.dualLaneRecoveryFrames == 0);
    CHECK(!control_algorithms::updateSingleLaneSpeedLimit(
        speedLimit, false, true, false, true));

    std::vector<TestPoint> laneControlCenter;
    for (int row = 120; row <= 175; ++row)
        laneControlCenter.emplace_back(row, 125);
    for (int row = 180; row <= 220; ++row)
        laneControlCenter.emplace_back(row, 185);
    const auto laneCenters =
        control_algorithms::calculateLaneControlCenters(
            laneControlCenter, 160.0f);
    CHECK(laneCenters.nearValid && laneCenters.farValid);
    CHECK(laneCenters.nearSamples == 41);
    CHECK(laneCenters.farSamples == 36);
    CHECK(std::abs(laneCenters.nearCenter - 185.0f) < 0.001f);
    CHECK(std::abs(laneCenters.farCenter - 125.0f) < 0.001f);
    CHECK(std::abs(laneCenters.controlCenter - 164.0f) < 0.001f);
    CHECK(std::abs(laneCenters.headingError - std::atan2(-60.0f, 85.0f)) < 0.001f);
    CHECK(std::abs(laneCenters.headingCorrection) < 0.001f);

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
    CHECK(std::abs(headingAssisted.headingError - expectedHeading) < 0.001f);
    CHECK(std::abs(headingAssisted.headingCorrection -
                    300.0f * expectedHeading * 0.925f) < 0.001f);
    CHECK(std::abs(headingAssisted.controlCenter - 158.75f) < 0.001f);

    const auto oppositeSideRecovery =
        control_algorithms::calculateLaneControlCenters(
            laneControlCenter, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f);
    const float recoveryHeading = std::atan2(-60.0f, 85.0f);
    CHECK(std::abs(oppositeSideRecovery.headingCorrection -
                    300.0f * recoveryHeading * 0.375f * 0.25f) < 0.001f);
    CHECK(std::abs(oppositeSideRecovery.controlCenter - 164.0f) < 0.001f);

    std::vector<TestPoint> cappedHeading;
    for (int row = 120; row <= 175; ++row)
        cappedHeading.emplace_back(row, 200);
    for (int row = 180; row <= 220; ++row)
        cappedHeading.emplace_back(row, 160);
    const auto cappedHeadingAssist =
        control_algorithms::calculateLaneControlCenters(
            cappedHeading, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f);
    CHECK(std::abs(cappedHeadingAssist.headingCorrection - 60.0f) < 0.001f);
    CHECK(std::abs(cappedHeadingAssist.controlCenter - 174.0f) < 0.001f);

    const auto confidenceLimitedHeading =
        control_algorithms::calculateLaneControlCenters(
            earlyRightHeading, 160.0f, 0.65f, 8, true,
            300.0f, 60.0f, 40.0f, 0.45f);
    CHECK(std::abs(confidenceLimitedHeading.headingConfidence - 0.45f) < 0.001f);
    CHECK(std::abs(confidenceLimitedHeading.headingCorrection -
                    headingAssisted.headingCorrection * 0.45f) < 0.001f);

    std::vector<TestPoint> missingNearCenter;
    for (int row = 120; row <= 175; ++row)
        missingNearCenter.emplace_back(row, 125);
    for (int row = 180; row < 187; ++row)
        missingNearCenter.emplace_back(row, 185);
    const auto missingNear =
        control_algorithms::calculateLaneControlCenters(
            missingNearCenter, 160.0f);
    CHECK(!missingNear.nearValid && missingNear.farValid);
    CHECK(std::abs(missingNear.farCenter - 125.0f) < 0.001f);
    CHECK(std::abs(missingNear.controlCenter - 160.0f) < 0.001f);

    std::vector<TestPoint> straightCenter;
    for (int row = 90; row <= 210; ++row)
        straightCenter.emplace_back(row, 160);
    const auto straightSpeed = control_algorithms::calculateCenterlineSpeed(
        straightCenter, 0.35f, 0.25f);
    CHECK(straightSpeed.valid);
    CHECK(std::abs(straightSpeed.curveStrength) < 0.001f);
    CHECK(std::abs(straightSpeed.speed - 0.35f) < 0.001f);

    std::vector<TestPoint> curvedCenter;
    for (int row = 90; row <= 130; ++row)
        curvedCenter.emplace_back(row, 120);
    for (int row = 170; row <= 210; ++row)
        curvedCenter.emplace_back(row, 160);
    const auto curvedSpeed = control_algorithms::calculateCenterlineSpeed(
        curvedCenter, 0.35f, 0.25f);
    CHECK(curvedSpeed.valid);
    CHECK(std::abs(curvedSpeed.curveStrength - 40.0f) < 0.001f);
    CHECK(std::abs(curvedSpeed.speed - 0.25f) < 0.001f);

    std::vector<TestPoint> mediumCurve;
    for (int row = 90; row <= 130; ++row)
        mediumCurve.emplace_back(row, 140);
    for (int row = 170; row <= 210; ++row)
        mediumCurve.emplace_back(row, 160);
    const auto mediumSpeed = control_algorithms::calculateCenterlineSpeed(
        mediumCurve, 0.35f, 0.25f);
    CHECK(mediumSpeed.valid);
    CHECK(mediumSpeed.speed > 0.25f && mediumSpeed.speed < 0.35f);

    std::vector<TestPoint> partialCenter{{175, 160}, {180, 160}, {185, 160}};
    const auto partialSpeed = control_algorithms::calculateCenterlineSpeed(
        partialCenter, 0.35f, 0.25f);
    CHECK(!partialSpeed.valid);
    CHECK(std::abs(partialSpeed.speed - 0.25f) < 0.001f);

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
    CHECK(goodReliability.reliable);
    CHECK(!stuckReliability.reliable);
    CHECK(stuckReliability.borderRatio > 0.99f);
    CHECK(stuckReliability.longestBorderRun == 31);

    std::vector<TestPoint> clippedRight;
    for (int row = 220; row >= 190; --row)
    {
        const int index = 220 - row;
        clippedRight.emplace_back(row, index < 19 ? 319 : 300);
    }
    const auto clippedReliability = control_algorithms::assessEdgeReliability(
        clippedRight, false, 320, 240, 20, 12);
    CHECK(!clippedReliability.reliable);
    CHECK(clippedReliability.singleEdgeUsable);
    CHECK(clippedReliability.interiorPointCount == 12);
    std::vector<float> learnedWidths(240, 240.0f);
    const auto clippedCenter = control_algorithms::reconstructSingleLaneCenter(
        clippedRight, learnedWidths, false, 240, 320);
    CHECK(clippedCenter.size() == clippedRight.size());
    CHECK(clippedCenter.size() >= 20);
    const auto clippedControl = control_algorithms::limitSingleLaneCenter(
        clippedCenter.front().y, 160, 45, 8);
    CHECK(clippedControl.valid);
    control_algorithms::SingleLaneSpeedLimitState clippedSpeedLimit;
    CHECK(control_algorithms::updateSingleLaneSpeedLimit(
        clippedSpeedLimit, true, true, false, false, 5, true));

    std::vector<TestPoint> mostlyBorderRight;
    for (int row = 220; row >= 190; --row)
    {
        const int index = 220 - row;
        mostlyBorderRight.emplace_back(row, index < 22 ? 319 : 300);
    }
    const auto mostlyBorderReliability = control_algorithms::assessEdgeReliability(
        mostlyBorderRight, false, 320, 240, 20, 12);
    CHECK(!mostlyBorderReliability.reliable);
    CHECK(!mostlyBorderReliability.singleEdgeUsable);
    CHECK(mostlyBorderReliability.interiorPointCount == 9);

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
    CHECK(!wrongLeftReliability.reliable && !wrongLeftReliability.singleEdgeUsable);
    CHECK(!wrongRightReliability.reliable && !wrongRightReliability.singleEdgeUsable);
    CHECK(wrongLeftReliability.oppositeBorderRun == 31);
    CHECK(wrongRightReliability.oppositeBorderRun == 31);

    std::vector<TestPoint> hybridLeft, hybridRight;
    for (int row = 176; row <= 220; ++row)
    {
        hybridLeft.emplace_back(row, row % 3 == 0 ? 0 : 40);
        hybridRight.emplace_back(row, row % 3 == 1 ? 319 : 280);
    }
    const auto hybridCenter = control_algorithms::buildDegradedLaneCenter(
        hybridLeft, hybridRight, learnedWidths, 240, 320);
    CHECK(hybridCenter.size() >= 30);
    const auto hybridNear = control_algorithms::calculateCenterWindow(
        hybridCenter, 160.0f, 176, 220, 205, 26, 8);
    CHECK(hybridNear.valid && hybridNear.samples >= 8);
    std::vector<TestPoint> bothBorders{{200, 0}};
    std::vector<TestPoint> bothBordersRight{{200, 319}};
    CHECK(control_algorithms::buildDegradedLaneCenter(
        bothBorders, bothBordersRight, learnedWidths, 240, 320).empty());

    CHECK(control_algorithms::reconstructSingleLaneCenterColumn(
        40, 240.0f, true) == 160);
    CHECK(control_algorithms::reconstructSingleLaneCenterColumn(
        280, 240.0f, false) == 160);
    const auto acceptedCenter = control_algorithms::limitSingleLaneCenter(
        190, 160, 45, 8);
    CHECK(acceptedCenter.valid && acceptedCenter.rawJump == 30);
    CHECK(acceptedCenter.appliedCenter == 168 && acceptedCenter.appliedStep == 8);
    const auto rejectedCenter = control_algorithms::limitSingleLaneCenter(
        206, 160, 45, 8);
    CHECK(!rejectedCenter.valid && rejectedCenter.appliedCenter == 160);

    std::vector<TestPoint> left{{220, 40}, {216, 44}};
    std::vector<TestPoint> right{{220, 280}, {216, 276}};
    std::vector<TestPoint> width{{220, 240}, {216, 232}};
    CHECK(control_algorithms::fillAlignedLaneGaps(left, right, width, 8));
    CHECK(left.size() == 5 && right.size() == 5 && width.size() == 5);
    for (std::size_t i = 0; i < left.size(); ++i)
        CHECK(width[i].y == right[i].y - left[i].y);
    control_algorithms::BusyConfirmationState busy;
    CHECK(control_algorithms::updateBusyConfirmation(busy, true, true) ==
           control_algorithms::BusyConfirmationEvent::NONE);
    CHECK(control_algorithms::updateBusyConfirmation(busy, true, true) ==
           control_algorithms::BusyConfirmationEvent::NONE);
    CHECK(control_algorithms::updateBusyConfirmation(busy, true, true) ==
           control_algorithms::BusyConfirmationEvent::WAITING);
    for (int i = 0; i < 100; ++i)
        CHECK(control_algorithms::updateBusyConfirmation(busy, false, false) ==
               control_algorithms::BusyConfirmationEvent::NONE);
    for (int i = 0; i < control_algorithms::BUSY_CLEAR_NEGATIVE_FRAMES - 1; ++i)
        CHECK(control_algorithms::updateBusyConfirmation(busy, true, false) ==
               control_algorithms::BusyConfirmationEvent::NONE);
    CHECK(control_algorithms::updateBusyConfirmation(busy, true, false) ==
           control_algorithms::BusyConfirmationEvent::CLEARED);
    for (int i = 0; i < 3; ++i)
        control_algorithms::updateBusyConfirmation(busy, true, true);
    CHECK(control_algorithms::updateBusyConfirmation(busy, true, true) ==
           control_algorithms::BusyConfirmationEvent::CONFIRMED);

    control_algorithms::AiFreshnessState ai;
    CHECK(!control_algorithms::updateAiFreshness(ai, true, false, 0));
    CHECK(control_algorithms::updateAiFreshness(ai, true, false, 501));
    CHECK(control_algorithms::updateAiFreshness(ai, true, true, 510));
    CHECK(control_algorithms::updateAiFreshness(ai, true, true, 520));
    CHECK(!control_algorithms::updateAiFreshness(ai, true, true, 530));
    CHECK(!control_algorithms::updateAiFreshness(ai, false, false, 1000));
    CHECK(control_algorithms::updateAiFreshness(ai, true, false, 1001));
    CHECK(control_algorithms::updateAiFreshness(ai, true, true, 1010));
    CHECK(control_algorithms::updateAiFreshness(ai, true, true, 1020));
    CHECK(!control_algorithms::updateAiFreshness(ai, true, true, 1030));
    control_algorithms::AiFreshnessState delayedAi;
    CHECK(control_algorithms::updateAiFreshness(
        delayedAi, true, true, 600,
        control_algorithms::AI_STALE_TIMEOUT_MS,
        control_algorithms::AI_RECOVERY_FRESH_RESULTS, 0));
    stopReasons.set(control_algorithms::StopReason::AI_STALE, true);
    CHECK(stopReasons.string() == "AI_STALE");
    stopReasons.set(control_algorithms::StopReason::AI_STALE, false);

    control_algorithms::CrossConfirmationState cross;
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            cross, true, false, false, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    CHECK(cross.armed);
    for (int i = 0; i < 2; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            cross, true, true, true, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    for (int i = 0; i < 100; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            cross, false, false, false, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES - 1; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            cross, true, false, false, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    CHECK(control_algorithms::updateCrossConfirmation(
        cross, true, false, false, false, true) ==
        control_algorithms::CrossConfirmationEvent::LAP_PASSED);

    control_algorithms::CrossConfirmationState finalCross;
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES; ++i)
        control_algorithms::updateCrossConfirmation(
            finalCross, true, false, false, true, true);
    for (int i = 0; i < 2; ++i)
        control_algorithms::updateCrossConfirmation(
            finalCross, true, true, true, true, true);
    for (int i = 0; i < 3; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            finalCross, true, false, false, true, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    for (int i = 3; i < control_algorithms::CROSS_DISAPPEAR_FRAMES - 1; ++i)
        control_algorithms::updateCrossConfirmation(
            finalCross, true, false, false, true, true);
    CHECK(control_algorithms::updateCrossConfirmation(
        finalCross, true, false, false, true, true) ==
        control_algorithms::CrossConfirmationEvent::FINAL_STOP);

    control_algorithms::CrossConfirmationState taskBlockedCross;
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES; ++i)
        control_algorithms::updateCrossConfirmation(
            taskBlockedCross, true, false, false, false, false);
    for (int i = 0; i < 2; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            taskBlockedCross, true, true, true, false, false) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    CHECK(!taskBlockedCross.armed);
    CHECK(!taskBlockedCross.linePassed);
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            taskBlockedCross, true, false, false, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    CHECK(taskBlockedCross.armed);
    CHECK(control_algorithms::updateCrossConfirmation(
        taskBlockedCross, true, false, false, false, true) ==
        control_algorithms::CrossConfirmationEvent::NONE);
    for (int i = 0; i < 2; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            taskBlockedCross, true, true, true, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    for (int i = 0; i < control_algorithms::CROSS_DISAPPEAR_FRAMES - 1; ++i)
        CHECK(control_algorithms::updateCrossConfirmation(
            taskBlockedCross, true, false, false, false, true) ==
            control_algorithms::CrossConfirmationEvent::NONE);
    CHECK(control_algorithms::updateCrossConfirmation(
        taskBlockedCross, true, false, false, false, true) ==
        control_algorithms::CrossConfirmationEvent::LAP_PASSED);
    return 0;
}
