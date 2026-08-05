#include "com/control_watchdog.hpp"
#include "ctrl/control_algorithms.hpp"
#include "fsm/busy_exit_state.hpp"
#include "runtime/camera_recovery.hpp"
#include "runtime/final_command.hpp"
#include "runtime/path_override.hpp"
#include "utils/config_validation.hpp"

#include <cassert>
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
        assert(command.speed == requested.speed && command.servo == requested.servo);
        reasons.set(control_algorithms::StopReason::CAMERA, true);
        command = resolveFinalCommand(requested, {reasons.mustStop(), false, false, true}, 1500);
        assert(command.speed == 0.0f && command.servo == 1500);
        reasons.set(control_algorithms::StopReason::EMERGENCY, true);
        reasons.set(control_algorithms::StopReason::CAMERA, false);
        command = resolveFinalCommand(requested, {reasons.mustStop(), true, true, true}, 1500);
        assert(command.speed == 0.0f && command.servo == 1500);
        reasons.set(control_algorithms::StopReason::EMERGENCY, false);
        command = resolveFinalCommand(requested, {reasons.mustStop(), false, true, true}, 1500);
        assert(command.speed == requested.speed);
    }
    {
        CameraRecoveryState camera;
        assert(camera.onFreshFrame().controlReady);
        assert(camera.onTimeout().cameraStopActive);
        assert(!camera.onFreshFrame().controlReady);
        assert(!camera.onFreshFrame().controlReady);
        auto third = camera.onFreshFrame();
        assert(!third.cameraStopActive && !third.controlReady && third.holdFrames == 2);
        assert(!camera.onFreshFrame().controlReady);
        assert(!camera.onFreshFrame().controlReady);
        assert(camera.onFreshFrame().controlReady);
    }
    {
        using Clock = BusyExitState::Clock;
        const auto start = Clock::time_point{};
        BusyExitState state;
        state.startDriving(start);
        assert(state.update(true, false, false, start + std::chrono::seconds(7)) ==
               BusyExitEvent::NONE);
        assert(state.update(true, false, false, start + std::chrono::milliseconds(15001)) ==
               BusyExitEvent::SIGN_WAIT_TIMEOUT);
        assert(state.stopped);

        state.startDriving(start);
        assert(state.update(true, true, true, start) == BusyExitEvent::EXIT_STARTED);
        assert(state.update(true, false, false, start + std::chrono::seconds(1)) ==
               BusyExitEvent::NONE);
        assert(state.update(true, true, false, start + std::chrono::milliseconds(1100)) ==
               BusyExitEvent::NONE);
        assert(state.update(true, true, false, start + std::chrono::milliseconds(1200)) ==
               BusyExitEvent::NONE);
        assert(state.update(true, true, false, start + std::chrono::milliseconds(1300)) ==
               BusyExitEvent::COMPLETED);

        state.startDriving(start);
        state.update(true, true, true, start);
        assert(state.update(true, false, false, start + std::chrono::milliseconds(2001)) ==
               BusyExitEvent::EXIT_GUIDE_TIMEOUT);
    }
    {
        std::vector<PointX> trackLeft{{1, 10}, {2, 11}};
        std::vector<PointX> trackRight{{1, 100}, {2, 101}};
        PathOverride overridePath;
        auto input = selectLaneInput(trackLeft, trackRight, overridePath);
        assert(input.source == PathSource::NONE && input.left == &trackLeft);
        overridePath.setEdges(PathSource::YFORK, {{1, 20}}, {{1, 80}});
        input = selectLaneInput(trackLeft, trackRight, overridePath);
        assert(input.source == PathSource::YFORK && input.left != &trackLeft);
        overridePath.leftEdge[0].y = 30;
        assert(trackLeft[0].y == 10);
        assert(!overridePath.clear(PathSource::BUSY));
        assert(overridePath.active);
        assert(overridePath.clear(PathSource::YFORK));
        assert(!overridePath.active);
    }
    {
        ControlWatchdogState watchdog;
        watchdog.onConnected(1000);
        assert(!watchdog.expired(2000, 500));
        assert(!watchdog.startupExpired(31000, 30000));
        assert(watchdog.startupExpired(31001, 30000));
        assert(!watchdog.onValidFrame(2, 1, 32000));
        assert(!watchdog.armed());
        assert(watchdog.onValidFrame(1, 1, 32000));
        assert(watchdog.armed() && !watchdog.startupExpired(70000, 30000));
        assert(watchdog.expired(32501, 500));
        watchdog.onConnected(40000);
        assert(!watchdog.armed() && !watchdog.startupExpired(40000, 30000));
    }
    {
        TestConfig config;
        validateIcarConfig(config, 240);
        config.velLow = -0.1f;
        assert(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap2.busy = true;
        config.lap2.yfork = true;
        assert(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap2.busyStopEnable = true;
        assert(throwsInvalid([&] { validateIcarConfig(config, 240); }));
        config = TestConfig{};
        config.lap3.yforkLeft = true;
        assert(throwsInvalid([&] { validateIcarConfig(config, 240); }));
    }
}
