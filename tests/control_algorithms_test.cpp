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
        0.25f, count, 60, 0.10f, 0.30f);
    assert(crossStart > 0.10f && crossStart < 0.104f);

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