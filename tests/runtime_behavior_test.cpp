#include "com/control_watchdog.hpp"
#include "ctrl/control_algorithms.hpp"
#include "fsm/busy_exit_state.hpp"
#include "fsm/yfork_guide_hold.hpp"
#include "runtime/camera_recovery.hpp"
#include "runtime/final_command.hpp"
#include "runtime/path_override.hpp"
#include "runtime/planned_path_validation.hpp"
#include "utils/config_validation.hpp"
#include "test_check.hpp"

#include <chrono>
#include <stdexcept>
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
        CHECK(overridePath.centerLine.empty() && !overridePath.hasCenterLine);
        CHECK(overridePath.headingConfidence == 0.65f);
        CHECK(overridePath.speedLimit == 0.15f && overridePath.ttlFrames == 2);
        CHECK(applyPathSpeedLimit(0.20f, overridePath) == 0.15f);
        CHECK(applyPathSpeedLimit(0.10f, overridePath) == 0.10f);
        CHECK(overridePath.generatedFrameId == 100);
        overridePath.leftEdge[0].y = 30;
        CHECK(trackLeft[0].y == 10);
        CHECK(!overridePath.clear(PathSource::BUSY));
        CHECK(overridePath.active);
        overridePath.tick(101);
        CHECK(overridePath.validFor(PathSource::YFORK));
        overridePath.tick(102);
        CHECK(!overridePath.active);

        overridePath.setEdges(PathSource::FORK, {{1, 20}}, {{1, 80}});
        overridePath.setEdges(PathSource::OBSTACLE, {{2, 30}}, {{2, 70}});
        CHECK(overridePath.source == PathSource::OBSTACLE);
        CHECK(overridePath.leftEdge.size() == 1 &&
               overridePath.leftEdge[0].y == 30);
        CHECK(overridePath.clear(PathSource::OBSTACLE));
        CHECK(!overridePath.active);
        CHECK(pathSourceAllowed(PathSource::PARK, FsmMode::PARK));
        CHECK(!pathSourceAllowed(PathSource::PARK, FsmMode::NORMAL));
        CHECK(pathSourceAllowed(PathSource::OBSTACLE, FsmMode::CROSS));
        CHECK(!pathSourceAllowed(PathSource::OBSTACLE, FsmMode::YFORK));
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
