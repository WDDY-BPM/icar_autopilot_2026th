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
    for (int i = 0; i < 4; ++i)
        assert(!control_algorithms::updateLaneRecovery(lane, false));
    assert(lane.invalidFrames == 4);
    for (int i = 0; i < 4; ++i)
        assert(!control_algorithms::updateLaneRecovery(lane, true));
    assert(control_algorithms::updateLaneRecovery(lane, true));

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
    assert(laneCenters.farSamples == 56);
    assert(std::abs(laneCenters.nearCenter - 185.0f) < 0.001f);
    assert(std::abs(laneCenters.farCenter - 125.0f) < 0.001f);
    assert(std::abs(laneCenters.controlCenter - 173.0f) < 0.001f);

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

    std::vector<TestPoint> left{{220, 40}, {216, 44}};
    std::vector<TestPoint> right{{220, 280}, {216, 276}};
    std::vector<TestPoint> width{{220, 240}, {216, 232}};
    assert(control_algorithms::fillAlignedLaneGaps(left, right, width, 8));
    assert(left.size() == 5 && right.size() == 5 && width.size() == 5);
    for (std::size_t i = 0; i < left.size(); ++i)
        assert(width[i].y == right[i].y - left[i].y);
    return 0;
}
