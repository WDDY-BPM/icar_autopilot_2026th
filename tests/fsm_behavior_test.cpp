#include "fsm/obstacle.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace
{
void seedObstaclePath(std::shared_ptr<Params> &params,
                      std::chrono::milliseconds ttl)
{
    std::vector<PointX> left;
    std::vector<PointX> right;
    for (int row = 220; row >= 40; --row)
    {
        left.emplace_back(row, 60);
        right.emplace_back(row, 260);
    }
    params->pathOverride.setEdgesForTime(
        PathSource::OBSTACLE, std::move(left), std::move(right),
        ttl, 0.45f, 0.15f);
}
} // namespace

int main()
{
    // A valid OBSTACLE plan survives frames without a fresh AI result.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->obstacle = true;
        FsmObstacle obstacle(params);
        seedObstaclePath(params, std::chrono::milliseconds(200));
        params->aiResultFresh = false;
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::NO_FRESH_RESULT);
        CHECK(params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(params->ctrl.obstacleSlow);
    }
    // An expired obstacle plan must not pretend the corridor is safe.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        seedObstaclePath(params, std::chrono::milliseconds(1));
        obstacle.unresolvedHazard = true;
        obstacle.lastHazardResult =
            PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30};
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        params->aiResultFresh = false;
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(!params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(params->ctrl.obstacleSlow);
        CHECK(params->plannerSafety.latched);
        CHECK(params->plannerSafety.rejectedSource == PathSource::OBSTACLE);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PLANNER));
    }
    // An expired plan without a recorded unresolved hazard does not stop;
    // the vehicle simply falls back to normal lane control.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        seedObstaclePath(params, std::chrono::milliseconds(1));
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        params->aiResultFresh = false;
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::NO_FRESH_RESULT);
        CHECK(!params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(!params->ctrl.obstacleSlow);
        CHECK(!params->plannerSafety.latched);
    }
    // Fresh AI without obstacles clears the path and releases planner safety.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        seedObstaclePath(params, std::chrono::milliseconds(200));
        params->plannerSafety.reject(PathSource::OBSTACLE);
        obstacle.unresolvedHazard = true;
        params->aiResultFresh = true;
        params->results.clear();
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::NOT_APPLICABLE);
        CHECK(!params->pathOverride.active());
        CHECK(!params->plannerSafety.latched);
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(!params->ctrl.obstacleSlow);
        CHECK(!obstacle.unresolvedHazard);
    }
    // Obstacle completely outside the lane clears path and releases safety.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 60, 250);
        obstacle.unresolvedHazard = true;
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 300, 100, 30, 30}};
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::NOT_APPLICABLE);
        CHECK(!params->pathOverride.active());
        CHECK(!params->plannerSafety.latched);
        CHECK(!params->ctrl.obstacleSlow);
        CHECK(!obstacle.unresolvedHazard);
    }
    // In-lane obstacle with enough lane data produces a time-TTL plan.
    {
        auto params = makeTestParams();
        params->mode = FsmMode::NORMAL;
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 60, 260);
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::VALID_PLAN);
        CHECK(params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(params->pathOverride.freshnessMode ==
              PathFreshnessMode::TIME_TTL);
        CHECK(params->pathOverride.validForTime ==
              std::chrono::milliseconds(120));
        CHECK(params->pathOverride.speedLimit == 0.15f);
        CHECK(params->ctrl.obstacleSlow);
        CHECK(obstacle.unresolvedHazard);
        CHECK(obstacle.lastHazardResult.type == LABEL_CONE);
    }
    // In-lane obstacle with insufficient lane data must stop (not NOT_APPLICABLE).
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 130, 60, 260); // 91 rows < ROWSIMAGE/2
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_PERSON, "", 0.9f, 150, 100, 30, 40}};
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(params->plannerSafety.latched);
        CHECK(params->plannerSafety.rejectedSource == PathSource::OBSTACLE);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(params->ctrl.obstacleSlow);
        CHECK(!params->pathOverride.active());
        CHECK(obstacle.unresolvedHazard);
    }
    // Abnormal left/right edge order is also treated as unprovable safety.
    {
        auto params = makeTestParams();
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 260, 60); // left > right columns
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(params->ctrl.obstacleSlow);
    }
    return 0;
}
