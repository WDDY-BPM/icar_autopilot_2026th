#include "com/control_watchdog.hpp"
#include "config/config_loader.hpp"
#include "ctrl/control_algorithms.hpp"
#include "ctrl/planned_geometry_builder.hpp"
#include "fsm/busy_exit_state.hpp"
#include "fsm/park/park_observation.hpp"
#include "fsm/park/park_path_planner.hpp"
#include "fsm/yfork_guide_hold.hpp"
#include "runtime/camera_recovery.hpp"
#include "runtime/control_decision.hpp"
#include "runtime/final_command.hpp"
#include "runtime/path_override.hpp"
#include "runtime/planned_path_validation.hpp"
#include "runtime/planner_safety.hpp"
#include "utils/config_validation.hpp"
#include "vision/predict_result.hpp"
#include "test_check.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>
#include <vector>

struct TestLapConfig
{
    bool park{false};
    bool busy{false};
    bool yfork{false};
    bool fork{false};
    bool station{false};
    bool busyStopEnable{false};
    bool manualTakeover{false};
    bool yforkLeft{false};
    int parkSpot{0};
    int busyStopPoint{0};
};

struct TestConfig
{
    float velLow{0.18f}, velHigh{0.20f}, velSlow{0.2f}, velPark{0.25f};
    float velCurve{0.18f}, velBusy{0.25f}, velStop{0.25f};
    float velCross{0.25f}, velYfork{0.25f}, startupSpeed{0.1f};
    float servoRate{600.0f}, startupServoRate{550.0f}, score{0.2f};
    int startupRampFrames{60}, startupStableFrames{12};
    int singleLaneCenterStep{8}, singleLaneMaxCenterJump{45};
    int rowCutUp{40}, rowCutBottom{20}, totalLaps{3};
    TestLapConfig lap1, lap2, lap3;
};

template <typename Function>
bool throwsInvalid(Function function)
{
    try
    {
        function();
    }
    catch (const std::invalid_argument &)
    {
        return true;
    }
    return false;
}

int main()
{
    {
        CHECK(selectGeometrySource(GeometryPolicy::PERCEPTION_ALLOWED, false) ==
              ControlGeometrySource::PERCEPTION);
        CHECK(selectGeometrySource(GeometryPolicy::PERCEPTION_ALLOWED, true) ==
              ControlGeometrySource::PLANNED);
        CHECK(selectGeometrySource(GeometryPolicy::PLANNED_REQUIRED, false) ==
              ControlGeometrySource::NONE);
        CHECK(selectGeometrySource(GeometryPolicy::STOPPED, true) ==
              ControlGeometrySource::NONE);
    }
    {
        nlohmann::json lapJson = {
            {"park", true}, {"parkSpot", 4}, {"yfork", true},
            {"yforkLeft", false}, {"busyStopEnable", false}};
        const auto lap = parseLapConfig(lapJson);
        CHECK(lap.park && lap.parkSpot == 4);
        CHECK(lap.yfork && !lap.yforkLeft);
    }
    {
        const std::vector<PredictResult> detections{
            PredictResult{LABEL_FORK, "", 0.4f, 0, 30, 20, 20},
            PredictResult{LABEL_FORK, "", 0.9f, 0, 32, 20, 20},
            PredictResult{LABEL_FORK, "", 0.7f, 0, 100, 20, 20},
            PredictResult{LABEL_CHOICE, "", 0.8f, 0, 0, 10, 10}};
        const auto observation = scanParkObservation(detections);
        CHECK(observation.forkMarkers.size() == 3);
        CHECK(observation.hasChoice && observation.bestChoice.score == 0.8f);
        auto stations = selectParkStations(detections);
        CHECK(stations.size() == 2);
        CHECK(stations[0].score == 0.9f && stations[1].score == 0.7f);
        const auto planned = ParkPathPlanner::fromEdges(
            PathSource::PARK, {{220, 100}}, {{220, 220}}, 0.7f, 0.18f, 2);
        CHECK(planned.hasValidGeometry() && planned.source == PathSource::PARK);
        const Config config;
        const auto leftTurn = ParkPathPlanner::buildParkingTurn(true, config);
        CHECK(leftTurn.leftEdge.size() == 51 && leftTurn.rightEdge.size() == 51);
        CHECK(leftTurn.leftEdge.front().x == 230 &&
              leftTurn.leftEdge.back().x == 80);
        const auto rightTurn = ParkPathPlanner::buildParkingTurn(false, config);
        CHECK(rightTurn.leftEdge.front().y == 64 &&
              rightTurn.leftEdge.back().y == 319);
        const auto guide = ParkPathPlanner::buildTrackGuide(
            {220, 160}, PredictResult{LABEL_FORK, "", 0.9f, 120, 90, 20, 20},
            true, config);
        CHECK(guide.freshnessMode == PathFreshnessMode::TIME_TTL);
        CHECK(guide.validForTime == std::chrono::milliseconds(150));
        const auto inSpot = ParkPathPlanner::buildInSpotStraight(config);
        const auto plannedGeometry = buildPlannedGeometry(inSpot, FsmMode::PARK);
        CHECK(plannedGeometry.source == PathSource::PARK);
        CHECK(plannedGeometry.valid && !plannedGeometry.centerLine.empty());
        CHECK(!buildPlannedGeometry(inSpot, FsmMode::NORMAL).valid);
    }
    {
        control_algorithms::StopReasonState reasons;
        const RequestedCommand requested{0.2f, 1700};
        auto command = resolveFinalCommand(requested, {}, 1500);
        CHECK(command.speed == requested.speed && command.servo == requested.servo);
        reasons.set(control_algorithms::StopReason::CAMERA, true);
        command = resolveFinalCommand(requested, {reasons.mustStop(), false, false, true}, 1500);
        CHECK(command.speed == 0.0f && command.servo == 1500);
        reasons.set(control_algorithms::StopReason::EMERGENCY, true);
        reasons.set(control_algorithms::StopReason::CAMERA, false);
        command = resolveFinalCommand(requested, {reasons.mustStop(), true, true, true}, 1500);
        CHECK(command.speed == 0.0f && command.servo == 1500);
        reasons.set(control_algorithms::StopReason::EMERGENCY, false);
        command = resolveFinalCommand(requested, {reasons.mustStop(), false, true, true}, 1500);
        CHECK(command.speed == requested.speed);
    }
    {
        CameraRecoveryState camera;
        CHECK(camera.onFreshFrame().controlReady);
        CHECK(camera.onTimeout().cameraStopActive);
        CHECK(!camera.onFreshFrame().controlReady);
        CHECK(!camera.onFreshFrame().controlReady);
        auto third = camera.onFreshFrame();
        CHECK(!third.cameraStopActive && !third.controlReady && third.holdFrames == 2);
        CHECK(!camera.onFreshFrame().controlReady);
        CHECK(!camera.onFreshFrame().controlReady);
        CHECK(camera.onFreshFrame().controlReady);
    }
    {
        using Clock = BusyExitState::Clock;
        const auto start = Clock::time_point{};
        BusyExitState state;
        state.startDriving(start);
        CHECK(state.update(false, false, false, false,
                            start + std::chrono::seconds(7)) ==
               BusyExitEvent::NONE);
        CHECK(state.update(false, false, false, false,
                            start + std::chrono::milliseconds(8001)) ==
               BusyExitEvent::SIGN_WAIT_TIMEOUT);
        CHECK(state.stopped);

        state.startDriving(start);
        CHECK(state.update(false, false, true, true, start) ==
               BusyExitEvent::EXIT_STARTED);
        CHECK(state.update(false, false, false, false,
                            start + std::chrono::seconds(1)) ==
               BusyExitEvent::NONE);
        CHECK(state.update(false, false, true, false,
                            start + std::chrono::milliseconds(1100)) ==
               BusyExitEvent::NONE);
        CHECK(state.update(false, false, true, false,
                            start + std::chrono::milliseconds(1200)) ==
               BusyExitEvent::NONE);
        CHECK(state.update(false, false, true, false,
                            start + std::chrono::milliseconds(1300)) ==
               BusyExitEvent::COMPLETED);

        state.startDriving(start);
        state.update(false, false, true, true, start);
        CHECK(state.update(false, false, false, false,
                            start + std::chrono::milliseconds(2001)) ==
               BusyExitEvent::EXIT_GUIDE_TIMEOUT);

        state.startDriving(start, true);
        CHECK(state.phase == BusyTraversalPhase::WAIT_STATION);
        CHECK(state.update(true, false, false, false,
                            start + std::chrono::milliseconds(9999)) ==
               BusyExitEvent::NONE);
        CHECK(state.update(true, false, false, false,
                            start + std::chrono::milliseconds(10001)) ==
               BusyExitEvent::STATION_WAIT_TIMEOUT);

        state.startDriving(start);
        CHECK(state.update(false, false, false, false,
                            start + std::chrono::seconds(15)) ==
               BusyExitEvent::TRAVERSAL_TIMEOUT);
    }
    {
        using Clock = YforkGuideHold::Clock;
        const auto start = Clock::time_point{};
        YforkGuideHold hold;
        auto guide = hold.update(120, 150, start);
        CHECK(guide.valid && guide.row == 120 && guide.column == 150);
        for (int frame = 1; frame <= 18; ++frame)
        {
            guide = hold.update(0, 0, start + std::chrono::milliseconds(frame * 33));
            CHECK(guide.valid);
        }
        guide = hold.update(0, 0, start + std::chrono::milliseconds(19 * 33));
        CHECK(!guide.valid && guide.expired);
    }
    {
        std::vector<PointX> trackLeft{{1, 10}, {2, 11}};
        std::vector<PointX> trackRight{{1, 100}, {2, 101}};
        PathOverride overridePath;
        auto input = selectLaneInput(trackLeft, trackRight, overridePath);
        CHECK(input.source == PathSource::NONE && input.left == &trackLeft);
        overridePath.tick(100);
        overridePath.setCenterLine(
            PathSource::FORK, {{230, 50}, {220, 51}, {210, 52}, {200, 53},
                               {190, 54}, {180, 55}, {170, 56}, {160, 57}},
            0.8f, 0.12f, 3);
        overridePath.setEdges(PathSource::YFORK, {{1, 20}}, {{1, 80}},
                              0.65f, 0.15f, 2);
        input = selectLaneInput(trackLeft, trackRight, overridePath);
        CHECK(input.source == PathSource::YFORK && input.left != &trackLeft);
        CHECK(overridePath.centerLine.empty() && !overridePath.hasCenter());
        CHECK(overridePath.headingConfidence == 0.65f);
        CHECK(overridePath.speedLimit == 0.15f && overridePath.ttlFrames == 2);
        CHECK(applyPathSpeedLimit(0.20f, overridePath) == 0.15f);
        CHECK(applyPathSpeedLimit(0.10f, overridePath) == 0.10f);
        CHECK(overridePath.generatedFrameId == 100);
        overridePath.leftEdge[0].y = 30;
        CHECK(trackLeft[0].y == 10);
        CHECK(!overridePath.clear(PathSource::BUSY));
        CHECK(overridePath.active());
        overridePath.tick(101);
        CHECK(overridePath.validFor(PathSource::YFORK));
        overridePath.tick(102);
        CHECK(!overridePath.active());
        overridePath.setCenterLineForTime(
            PathSource::PARK, {{230, 160}, {220, 161}},
            std::chrono::milliseconds(20));
        CHECK(overridePath.freshnessMode == PathFreshnessMode::TIME_TTL);
        CHECK(overridePath.hasValidGeometry());
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        CHECK(!overridePath.hasValidGeometry());

        overridePath.setEdges(PathSource::FORK, {{1, 20}}, {{1, 80}});
        overridePath.setEdges(PathSource::OBSTACLE, {{2, 30}}, {{2, 70}});
        CHECK(overridePath.source == PathSource::OBSTACLE);
        CHECK(overridePath.leftEdge.size() == 1 &&
               overridePath.leftEdge[0].y == 30);
        CHECK(overridePath.clear(PathSource::OBSTACLE));
        CHECK(!overridePath.active());
        CHECK(pathSourceAllowed(PathSource::PARK, FsmMode::PARK));
        CHECK(!pathSourceAllowed(PathSource::PARK, FsmMode::NORMAL));
        CHECK(pathSourceAllowed(PathSource::OBSTACLE, FsmMode::CROSS));
        CHECK(!pathSourceAllowed(PathSource::OBSTACLE, FsmMode::YFORK));
    }
    {
        control_algorithms::StopReasonState reasons;
        ControlGeometry perception{ControlGeometrySource::PERCEPTION,
                                   PathSource::NONE, true, true, 1};
        CHECK(evaluateRuntimeControl(perception, reasons, true).allowMotion);
        perception.updated = false;
        CHECK(!evaluateRuntimeControl(perception, reasons, true).allowMotion);
        perception.updated = true;
        perception.valid = false;
        CHECK(!evaluateRuntimeControl(perception, reasons, true).allowMotion);
        perception.valid = true;
        perception.pointCount = 0;
        CHECK(!evaluateRuntimeControl(perception, reasons, true).allowMotion);
        perception.pointCount = 1;
        reasons.set(control_algorithms::StopReason::LANE, true);
        CHECK(!evaluateRuntimeControl(perception, reasons, true).allowMotion);
        reasons.set(control_algorithms::StopReason::LANE, false);
        reasons.set(control_algorithms::StopReason::PLANNER, true);
        CHECK(reasons.string() == "PLANNER");
        CHECK(!evaluateRuntimeControl(perception, reasons, true).allowMotion);
        reasons.set(control_algorithms::StopReason::PLANNER, false);
        CHECK(!evaluateRuntimeControl(perception, reasons, false).allowMotion);

        ControlGeometry planned{ControlGeometrySource::PLANNED,
                                PathSource::PARK, true, true, 1};
        CHECK(evaluateRuntimeControl(planned, reasons, true).allowMotion);
        planned.source = ControlGeometrySource::NONE;
        CHECK(!evaluateRuntimeControl(planned, reasons, true).allowMotion);
        planned.source = ControlGeometrySource::PLANNED;
        planned.valid = false;
        CHECK(evaluateRuntimeControl(planned, reasons, true).centerSteering);
    }
    {
        PlannerSafetyState safety;
        safety.reject(PathSource::YFORK);
        CHECK(safety.latched && safety.validRecoveryFrames == 0);
        CHECK(!safety.observe(PathSource::YFORK, false));
        CHECK(!safety.observe(PathSource::YFORK, true));
        CHECK(safety.validRecoveryFrames == 1);
        CHECK(!safety.observe(PathSource::YFORK, false));
        CHECK(safety.validRecoveryFrames == 0);
        CHECK(!safety.observe(PathSource::YFORK, true));
        CHECK(safety.observe(PathSource::YFORK, true));
        CHECK(!safety.latched);
        safety.reject(PathSource::YFORK);
        CHECK(!safety.observeFrame(true));
        CHECK(!safety.observeFrame(false));
        CHECK(!safety.observeFrame(true));
        CHECK(safety.latched);
        CHECK(!safety.observeFrame(false));
        CHECK(!safety.observeFrame(true));
        CHECK(safety.observeFrame(true));
        CHECK(!safety.latched);
        safety.reject(PathSource::PARK);
        safety.clear(PathSource::BUSY);
        CHECK(safety.latched);
        safety.clear(PathSource::PARK);
        CHECK(!safety.latched);
    }
    {
        std::vector<PointX> valid;
        for (int index = 0; index < 12; ++index)
            valid.emplace_back(230 - index * 10, 150 + index);
        CHECK(validatePlannedPath(valid, 240, 320).valid);
        CHECK(!validatePlannedPath({}, 240, 320).valid);
        CHECK(!validatePlannedPath({{230, 150}}, 240, 320).valid);
        std::vector<PointX> noNear;
        for (int index = 0; index < 8; ++index)
            noNear.emplace_back(100 - index * 5, 150);
        CHECK(!validatePlannedPath(noNear, 240, 320).valid);
        auto outOfBounds = valid;
        outOfBounds[4].y = 320;
        CHECK(!validatePlannedPath(outOfBounds, 240, 320).valid);
        auto largeJump = valid;
        largeJump[4].y = 300;
        CHECK(!validatePlannedPath(largeJump, 240, 320).valid);
    }
    {
        ControlWatchdogState watchdog;
        watchdog.onConnected(1000);
        CHECK(!watchdog.expired(2000, 500));
        CHECK(!watchdog.startupExpired(31000, 30000));
        CHECK(watchdog.startupExpired(31001, 30000));
        CHECK(!watchdog.onValidFrame(2, 1, 32000));
        CHECK(!watchdog.armed());
        CHECK(watchdog.onValidFrame(1, 1, 32000));
        CHECK(watchdog.armed() && !watchdog.startupExpired(70000, 30000));
        CHECK(watchdog.expired(32501, 500));
        watchdog.onConnected(40000);
        CHECK(!watchdog.armed() && !watchdog.startupExpired(40000, 30000));
    }
    {
        TestConfig config;
        validateIcarConfig(config, 240);
        config.velLow = -0.1f;
        CHECK(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap2.busy = true;
        config.lap2.yfork = true;
        CHECK(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap2.busyStopEnable = true;
        CHECK(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap3.yforkLeft = true;
        CHECK(throwsInvalid([&] { validateIcarConfig(config, 240); }));
    }
}
