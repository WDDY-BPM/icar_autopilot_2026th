#include "ctrl/center.hpp"
#include "fsm/obstacle.hpp"
#include "fsm/park.hpp"
#include "park_fsm_test_fixture.hpp"
#include "runtime/control_decision.hpp"
#include "runtime/final_command.hpp"
#include "test_check.hpp"
#include "test_params.hpp"

#include <chrono>
#include <thread>
#include <vector>

namespace
{
using ParkStep = ParkFsmTestFixture::Step;
using MissionProgress = ParkFsmTestFixture::MissionProgress;

PredictResult closeGate()
{
    return PredictResult{LABEL_GATE, "", 0.9f, 150, 200, 120, 100};
}

PredictResult farGate()
{
    return PredictResult{LABEL_GATE, "", 0.9f, 150, 20, 120, 30};
}

PredictResult leftMarker()
{
    return PredictResult{LABEL_LEFT, "", 0.9f, 150, 100, 30, 30};
}

ControlGeometry validGeometry()
{
    ControlGeometry geometry;
    geometry.updated = true;
    geometry.valid = true;
    geometry.source = ControlGeometrySource::PLANNED;
    geometry.pathSource = PathSource::PARK;
    geometry.pointCount = 20;
    return geometry;
}

FinalCommand finalCommandFor(const std::shared_ptr<Params> &params)
{
    const bool geometrySafetyHold = params->stopReasons.hasOnly(
        control_algorithms::StopReason::LANE,
        control_algorithms::StopReason::PLANNER);
    FinalCommand command = resolveFinalCommand(
        {0.6f, 1500},
        {params->mustStop() && !geometrySafetyHold,
         false, true, true},
        1500);
    // 与 applyFinalStopArbitration 一致：LANE/PLANNER 几何安全锁存
    // 由仲裁另行强制速度0。
    if (geometrySafetyHold)
        command.speed = 0.0f;
    return command;
}
} // namespace

int main()
{
    // Test 1: PARK gate too close -> PARK_GATE through stop arbitration,
    // final command speed is zero, and PARK_GATE clears once the gate is far.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::TRACKIN);
        ParkFsmTestFixture::run(park, img, {closeGate()}, true);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        CHECK(params->stopReasonString().find("PARK_GATE") !=
              std::string::npos);
        CHECK(!evaluateRuntimeControl(
            validGeometry(), params->stopReasons, true).allowMotion);
        CHECK(finalCommandFor(params).speed == 0.0f);

        // 无新AI帧时道闸停车必须保持锁存，不能被空观察错误解除。
        ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        CHECK(params->mustStop());

        ParkFsmTestFixture::run(park, img, {farGate()}, true);
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        CHECK(!params->mustStop());
        CHECK(evaluateRuntimeControl(
            validGeometry(), params->stopReasons, true).allowMotion);
    }

    // Test 2: TRACKIN target lost (parkSpot=4) must never skip to FORKOUT;
    // the vehicle holds PARK_TARGET_LOST and the lap task stays incomplete.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->config.currentLapConfig->parkSpot = 4;
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::TRACKIN);
        for (int frame = 0; frame < 101; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::TARGET_LOST_STOP);
        CHECK(!params->lapTaskCompleted);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_TARGET_LOST));
        CHECK(params->geometryPolicy == GeometryPolicy::STOPPED);
        CHECK(params->mustStop());
        CHECK(!evaluateRuntimeControl(
            validGeometry(), params->stopReasons, true).allowMotion);
        CHECK(finalCommandFor(params).speed == 0.0f);
    }

    // Test 3: FORKOUT without an armed exit sign must not complete the lap;
    // a 2s timeout holds EXIT_UNCONFIRMED_STOP with zero speed.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::FORKOUT);
        for (int frame = 0; frame < 5; ++frame)
            ParkFsmTestFixture::run(park, img, {}, true);
        CHECK(!ParkFsmTestFixture::exitSignArmed(park));
        CHECK(!params->lapTaskCompleted);
        ParkFsmTestFixture::setForkOutStartedAt(
            park, std::chrono::steady_clock::now() -
                std::chrono::milliseconds(2100));
        ParkFsmTestFixture::run(park, img, {}, true);
        CHECK(ParkFsmTestFixture::stage(park) ==
              ParkStep::EXIT_UNCONFIRMED_STOP);
        CHECK(!params->lapTaskCompleted);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_EXIT_UNCONFIRMED));
        CHECK(params->mustStop());
        CHECK(finalCommandFor(params).speed == 0.0f);
    }

    // Test 4: only after a completed parking mission (EXIT_REPLAY_COMPLETED),
    // armed exit sign + consecutive missing frames complete the lap.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setMissionProgress(
            park, MissionProgress::EXIT_REPLAY_COMPLETED);
        ParkFsmTestFixture::setStage(park, ParkStep::FORKOUT);
        ParkFsmTestFixture::run(park, img, {leftMarker()}, true);
        CHECK(!ParkFsmTestFixture::exitSignArmed(park));
        ParkFsmTestFixture::run(park, img, {leftMarker()}, true);
        CHECK(ParkFsmTestFixture::exitSignArmed(park));
        for (int frame = 0; frame < 3; ++frame)
            ParkFsmTestFixture::run(park, img, {}, true);
        CHECK(ParkFsmTestFixture::stage(park) == ParkStep::NONE);
        CHECK(params->lapTaskCompleted);
    }

    // Test 5: obstacle plan expiry with an unresolved hazard must stop
    // immediately (PLANNER), closing the stale-AI safety gap.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->obstacle = true;
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 60, 260);
        params->mode = FsmMode::NORMAL;
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        cv::Mat img;
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::VALID_PLAN);
        CHECK(obstacle.unresolvedHazard);
        std::this_thread::sleep_for(std::chrono::milliseconds(130));
        params->aiResultFresh = false;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(obstacle.unresolvedHazard);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(params->mustStop());
        CHECK(!evaluateRuntimeControl(
            validGeometry(), params->stopReasons, true).allowMotion);
        CHECK(finalCommandFor(params).speed == 0.0f);
    }

    // Test 6: a fresh AI result without an obstacle clears the hazard and
    // releases the PLANNER stop.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->obstacle = true;
        FsmObstacle obstacle(params);
        obstacle.unresolvedHazard = true;
        params->plannerSafety.reject(PathSource::OBSTACLE);
        params->aiResultFresh = true;
        params->results.clear();
        cv::Mat img;
        obstacle.run(img);
        CHECK(!obstacle.unresolvedHazard);
        CHECK(!params->plannerSafety.latched);
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(!params->mustStop());
    }

    // Test 7: PLANNED -> PERCEPTION requires 5 consecutive valid perception
    // frames before control is released; an invalid middle frame resets the
    // recovery counter.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->mode = FsmMode::PARK;
        params->geometryPolicy = GeometryPolicy::PLANNED_REQUIRED;
        std::vector<PointX> left;
        std::vector<PointX> right;
        for (int row = 220; row >= 40; --row)
        {
            left.emplace_back(row, 60);
            right.emplace_back(row, 260);
        }
        params->pathOverride.setEdgesForFrames(
            PathSource::PARK, std::move(left), std::move(right),
            0.65f, 0.5f, 2);
        Center center;
        center.fitting(params);
        CHECK(center.geometry.source == ControlGeometrySource::PLANNED);
        CHECK(center.controlValid);

        params->clearPathOverride(PathSource::PARK);
        params->geometryPolicy = GeometryPolicy::PERCEPTION_ALLOWED;
        setStraightTrack(params, 220, 40, 80, 240);
        params->track->quality.valid = true;
        params->track->quality.leftReliable = true;
        params->track->quality.rightReliable = true;
        params->track->quality.widthVariation = 0.05f;
        for (int frame = 1; frame <= 5; ++frame)
        {
            center.fitting(params);
            if (frame < 5)
                CHECK(!center.controlValid);
            else
                CHECK(center.controlValid);
        }
        // 恢复中间插入一个无效帧：恢复计数归零。
        params->track->pointsEdgeLeft.clear();
        params->track->pointsEdgeRight.clear();
        center.fitting(params);
        CHECK(!center.controlValid);
        CHECK(center.laneRecoveryFrames == 0);
        setStraightTrack(params, 220, 40, 80, 240);
        for (int frame = 1; frame <= 5; ++frame)
        {
            center.fitting(params);
            if (frame < 5)
                CHECK(!center.controlValid);
            else
                CHECK(center.controlValid);
        }
    }

    // StopReason lifecycle: setStep clears PARK_GATE/PARK_TARGET_LOST on
    // stage transitions; reset clears every park hold.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::TRACKIN);
        ParkFsmTestFixture::run(park, img, {closeGate()}, true);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        ParkFsmTestFixture::setStage(park, ParkStep::ENTER);
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        params->setStopReason(
            control_algorithms::StopReason::PARK_TARGET_LOST, true);
        ParkFsmTestFixture::reset(park);
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_TARGET_LOST));
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PARK));
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_ENTER_UNCONFIRMED));
        CHECK(!params->hasStopReason(
            control_algorithms::StopReason::PARK_EXIT_UNCONFIRMED));
    }

    // ENTER timeout without real entry evidence holds PARK_ENTER_UNCONFIRMED
    // with zero speed; the mission must not be marked PARKED.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->config.currentLapConfig->parkSpot = 4;
        params->lapTaskRequired = true;
        params->lapTaskCompleted = false;
        FsmPark park(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::TRACKIN);
        ParkFsmTestFixture::setStage(park, ParkStep::ENTER);
        for (int frame = 0; frame < 31; ++frame)
            ParkFsmTestFixture::run(park, img, {}, false);
        CHECK(ParkFsmTestFixture::stage(park) ==
              ParkStep::ENTER_UNCONFIRMED_STOP);
        CHECK(ParkFsmTestFixture::missionProgress(park) ==
              MissionProgress::ENTERING);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_ENTER_UNCONFIRMED));
        CHECK(params->mustStop());
        CHECK(finalCommandFor(params).speed == 0.0f);
    }

    // Obstacle replan recovery: expired hazard latches PLANNER; a fresh
    // valid replan uses a longer recovery TTL, and a second consecutive
    // control-geometry validation releases the latch and resumes motion
    // at the obstacle speed limit.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->obstacle = true;
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 60, 260);
        params->mode = FsmMode::NORMAL;
        cv::Mat img;
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::VALID_PLAN);
        CHECK(!params->plannerSafety.latched);
        std::this_thread::sleep_for(std::chrono::milliseconds(130));
        params->aiResultFresh = false;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(params->plannerSafety.latched);
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::VALID_PLAN);
        CHECK(params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(params->pathOverride.validForTime ==
              std::chrono::milliseconds(220));
        CHECK(params->plannerSafety.latched); // 仍需第二次验证
        Center center;
        center.fitting(params);
        CHECK(center.geometry.source == ControlGeometrySource::PLANNED);
        CHECK(center.geometry.pathSource == PathSource::OBSTACLE);
        CHECK(center.geometry.valid);
        params->plannerSafety.observeFrame(true);
        params->setStopReason(control_algorithms::StopReason::PLANNER,
                              params->plannerSafety.latched);
        CHECK(!params->plannerSafety.latched);
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PLANNER));
        CHECK(params->pathOverride.validFor(PathSource::OBSTACLE));
        CHECK(params->ctrl.obstacleSlow);
        const float limitedSpeed =
            applyPathSpeedLimit(0.2f, params->pathOverride);
        FinalCommand command = resolveFinalCommand(
            {limitedSpeed, 1500}, {false, false, true, true}, 1500);
        CHECK(command.speed > 0.0f);
        CHECK(command.speed <= params->pathOverride.speedLimit);
    }

    // Obstacle cross-mode lifecycle: suspending for PARK/BUSY clears the
    // hazard and forces a fresh-AI reevaluation before any new plan; old
    // obstacle coordinates must never be reused.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->obstacle = true;
        FsmObstacle obstacle(params);
        setStraightTrack(params, 220, 40, 60, 260);
        params->mode = FsmMode::NORMAL;
        cv::Mat img;
        params->aiResultFresh = true;
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        obstacle.run(img);
        CHECK(obstacle.unresolvedHazard);
        params->mode = FsmMode::PARK;
        obstacle.suspend();
        CHECK(!obstacle.unresolvedHazard);
        CHECK(obstacle.needsFreshReevaluation);
        CHECK(!params->pathOverride.active());
        params->mode = FsmMode::NORMAL;
        params->aiResultFresh = false;
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::BLOCKED_WITHOUT_SAFE_PLAN);
        CHECK(params->hasStopReason(control_algorithms::StopReason::PLANNER));
        params->aiResultFresh = true;
        params->results.clear();
        obstacle.run(img);
        CHECK(!obstacle.needsFreshReevaluation);
        CHECK(!params->plannerSafety.latched);
        CHECK(!params->hasStopReason(control_algorithms::StopReason::PLANNER));
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        obstacle.run(img);
        CHECK(obstacle.planningResult == ObstaclePlanningResult::VALID_PLAN);
        CHECK(params->pathOverride.validFor(PathSource::OBSTACLE));
    }

    // PARK safety hold owns the path: obstacle must not overwrite a PARK
    // path while PARK_GATE is latched.
    {
        auto params = makeTestParams();
        params->config.currentLapConfig->park = true;
        params->config.currentLapConfig->obstacle = true;
        FsmPark park(params);
        FsmObstacle obstacle(params);
        cv::Mat img;
        ParkFsmTestFixture::setStage(park, ParkStep::TRACKIN);
        std::vector<PointX> left;
        std::vector<PointX> right;
        for (int row = 220; row >= 40; --row)
        {
            left.emplace_back(row, 60);
            right.emplace_back(row, 260);
        }
        params->pathOverride.setEdgesForFrames(
            PathSource::PARK, std::move(left), std::move(right),
            0.65f, 0.5f, 2);
        params->aiResultFresh = true;
        params->results = {closeGate()};
        park.run(img);
        CHECK(params->hasStopReason(
            control_algorithms::StopReason::PARK_GATE));
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        params->results = {PredictResult{LABEL_CONE, "", 0.9f, 150, 100, 30, 30}};
        obstacle.run(img);
        CHECK(obstacle.planningResult ==
              ObstaclePlanningResult::NOT_APPLICABLE);
        CHECK(params->pathOverride.source == PathSource::PARK);
        CHECK(params->pathOverride.validFor(PathSource::PARK));
        CHECK(params->mustStop());
    }

    return 0;
}
