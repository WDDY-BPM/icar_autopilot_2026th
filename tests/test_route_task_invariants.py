from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]

def read_runtime_sources():
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((ROOT / "src" / "runtime").glob("*.cpp"))
    )


def read_algorithm_headers():
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted((ROOT / "include" / "ctrl").glob("*.hpp"))
    )


class RouteTaskInvariantTests(unittest.TestCase):
    def test_runtime_orchestration_stays_split(self):
        header = (ROOT / "include" / "icar.hpp").read_text(encoding="utf-8")
        running = (ROOT / "src" / "runtime" / "running.cpp").read_text(encoding="utf-8")
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertLessEqual(len(header.splitlines()), 300)
        self.assertLessEqual(len(running.splitlines()), 120)
        self.assertNotIn("GLOB_RECURSE SRC_LIST", cmake)
        for source in (ROOT / "src" / "runtime").glob("*.cpp"):
            self.assertIn(f"src/runtime/{source.name}", cmake)

    def test_control_algorithm_compatibility_header_is_thin(self):
        compatibility = (ROOT / "include" / "ctrl" / "control_algorithms.hpp").read_text(encoding="utf-8")
        self.assertLessEqual(len(compatibility.splitlines()), 10)
        for header in (
            "stop_reasons.hpp", "alert_logic.hpp", "scene_confirmation.hpp",
            "lane_quality.hpp", "lane_control.hpp",
        ):
            self.assertIn(f'#include "{header}"', compatibility)

    def test_active_lap_configuration_is_read_only(self):
        params = (ROOT / "include" / "utils" / "params.hpp").read_text(encoding="utf-8")
        fsm = (ROOT / "src" / "runtime" / "fsm.cpp").read_text(encoding="utf-8")
        self.assertIn("const Config::LapConfig &activeLapConfig() const", params)
        self.assertIn("bool featureEnabled(Feature feature) const", params)
        self.assertNotRegex(fsm, r"params->config\.(?:fork|park|busy|slow|stop|cross|yfork|station|obstacle)\s*=")

    def test_latest_frame_capture_keeps_shutdown_safe(self):
        capture = (ROOT / "src" / "runtime" / "latest_frame_capture.cpp").read_text(encoding="utf-8")
        risk = (ROOT / "docs" / "camera_shutdown_risk.md").read_text(encoding="utf-8")
        self.assertNotIn("detach()", capture)
        self.assertNotIn("release()", capture)
        self.assertIn("thread_.join()", capture)
        self.assertIn("poll()", risk)

    def test_steering_polarity_matches_vehicle(self):
        motion = (ROOT / "include" / "ctrl" / "motion.hpp").read_text(encoding="utf-8")
        manual = (ROOT / "src" / "fsm" / "manualControl.cpp").read_text(encoding="utf-8")
        self.assertIn("PWMSERVOMID - pwmDiff", motion)
        self.assertNotIn("PWMSERVOMID + pwmDiff", motion)
        self.assertIn("std::clamp(filteredError", motion)
        self.assertIn("std::clamp(PWMSERVOMID - pwmDiff", motion)
        self.assertIn("lastServo - maxServoStep", motion)
        self.assertIn("void reset()", motion)
        icar = read_runtime_sources()
        self.assertIn("motion->reset();", icar)
        self.assertIn("(error - errorLast) / dt", motion)
        self.assertIn("servoRatePerSecond * dt", motion)
        self.assertIn("limitServoCommand(int targetServo, float dtSeconds,", motion)
        self.assertIn("motion->poseControl(params, steeringDt)", icar)
        self.assertIn("params->ctrl.servo = motion->limitServoCommand(", icar)
        self.assertIn("*steering = PWMSERVOMID + 300; // 左转", manual)
        self.assertIn("*steering = PWMSERVOMID - 300; // 右转", manual)

    def test_lane_quality_and_loss_protection_are_wired(self):
        track_h = (ROOT / "include" / "ctrl" / "track.hpp").read_text(encoding="utf-8")
        track = (ROOT / "src" / "ctrl" / "track.cpp").read_text(encoding="utf-8")
        center = (ROOT / "src" / "ctrl" / "center.cpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        predeal = (ROOT / "src" / "ctrl" / "predeal.cpp").read_text(encoding="utf-8")
        self.assertIn("struct LaneQuality", track_h)
        self.assertIn("candidateAcceptable", track)
        algorithms = read_algorithm_headers()
        self.assertIn("rowGap > 1 && rowGap <= maxGapRows", algorithms)
        self.assertIn("evaluateQuality();", track)
        self.assertIn("buildRowAlignedCenter", center)
        self.assertIn("laneWidthProfile[row]", center)
        self.assertIn("laneWidthProfileReady()", center)
        self.assertIn("laneUnconfirmedState.frames > 0", core)
        self.assertIn("control_algorithms::updateLaneRecovery", center)
        self.assertIn("params->track->quality.leftReliable", core)
        self.assertIn("params->track->quality.rightReliable", core)
        self.assertIn("0.85f * filteredThreshold", predeal)
        self.assertIn("MORPH_CLOSE", predeal)
    def test_cross_requires_current_lap_task(self):
        source = (ROOT / "src" / "fsm" / "cross.cpp").read_text(encoding="utf-8")
        self.assertIn("!params->lapTaskRequired || params->lapTaskCompleted", source)
        self.assertIn("updateCrossConfirmation", source)
        event = source.index("CrossConfirmationEvent::LAP_PASSED")
        next_lap = source.index("params->nextLap()", event)
        self.assertLess(event, next_lap)
    def test_construction_boxes_are_edge_counted(self):
        source = (ROOT / "src" / "fsm" / "station.cpp").read_text(encoding="utf-8")
        self.assertNotIn("busyEntryDelay", source)
        self.assertIn("stationVisible && boxAtThreshold && boxArmed", source)
        self.assertIn("if (!params->aiResultFresh)", source)
        self.assertIn("if (++boxMissingFrames >= 8)", source)
        self.assertIn("detectedBoxIndex++", source)
        self.assertIn("detectedBoxIndex == targetBox", source)

    def test_only_confirmed_exits_complete_tasks(self):
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        yfork = (ROOT / "src" / "fsm" / "yfork.cpp").read_text(encoding="utf-8")
        self.assertIn('const bool exitConfirmed = countRes > 2;', park)
        self.assertIn('params->completeLapTask("park-exit")', park)
        self.assertIn('const bool exitConfirmed = countRes > 2;', busy)
        self.assertIn('params->completeLapTask("construction-exit")', busy)
        self.assertIn('params->completeLapTask("yfork-right-exit")', yfork)
        timeout_end = re.search(r"case Step::END:.*?return true;", yfork, re.DOTALL)
        self.assertIsNotNone(timeout_end)
        self.assertNotIn("completeLapTask", timeout_end.group(0))
        self.assertIn("completed = false", timeout_end.group(0))

    def test_yfork_exit_uses_camera_edge_before_bezier_replan(self):
        source = (ROOT / "src" / "fsm" / "yfork.cpp").read_text(encoding="utf-8")
        capture = source.index("exitEdgeColumn = params->track->pointsEdgeLeft.back().y")
        replan = source.index("replanTracking(selectLeft, img)", capture)
        exit_check = source.index("int cur = exitEdgeColumn", replan)
        self.assertLess(capture, replan)
        self.assertLess(replan, exit_check)

    def test_slow_mode_has_timeout_and_lap_reset(self):
        header = (ROOT / "include" / "fsm" / "slow.hpp").read_text(encoding="utf-8")
        source = (ROOT / "src" / "fsm" / "slow.cpp").read_text(encoding="utf-8")
        self.assertIn("ENABLE_TIMEOUT_FRAMES = 450", header)
        self.assertIn("timeout >= ENABLE_TIMEOUT_FRAMES", source)
        enable_start = source.index("case Step::ENABLE:")
        enable_end = source.index("default:", enable_start)
        self.assertNotIn("timeout = 0;", source[enable_start:enable_end])
        self.assertGreaterEqual(source.count("if (!params->aiResultFresh)"), 2)
        reset = re.search(r"void FsmSlow::resetLap\(\).*?\n\}", source, re.DOTALL)
        self.assertIsNotNone(reset)
        self.assertIn("setStep(Step::NONE)", reset.group(0))
        self.assertIn("params->ctrl.slow = false", reset.group(0))

    def test_obstacle_edits_only_in_path_and_slowdown_is_frame_scoped(self):
        obstacle = (ROOT / "src" / "fsm" / "obstacle.cpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        motion = (ROOT / "include" / "ctrl" / "motion.hpp").read_text(encoding="utf-8")
        guard = obstacle.index("if (!obstacleInDrivingPath)")
        truncation = obstacle.index("pointsEdgeLeft.resize", guard)
        self.assertLess(guard, truncation)
        self.assertIn("params->ctrl.obstacleSlow = false", core)
        self.assertIn("params->ctrl.obstacleSlow = true", obstacle)
        self.assertIn("params->ctrl.slow || params->ctrl.obstacleSlow", motion)
    def test_stop_ai_evidence_uses_only_fresh_results(self):
        source = (ROOT / "src" / "fsm" / "stop.cpp").read_text(encoding="utf-8")
        none = re.search(r"case Step::NONE:.*?case Step::ENABLE:", source, re.DOTALL)
        enable = re.search(r"case Step::ENABLE:.*?case Step::STOP:", source, re.DOTALL)
        stopped = re.search(r"case Step::STOP:.*?\n    \}", source, re.DOTALL)
        self.assertIsNotNone(none)
        self.assertIsNotNone(enable)
        self.assertIsNotNone(stopped)
        self.assertIn("if (!params->aiResultFresh)", none.group(0))
        self.assertIn("if (params->aiResultFresh)", enable.group(0))
        self.assertIn("else if (countSes >= 10)", enable.group(0))
        stale_timeout = enable.group(0).index("else if (timeout > 50)")
        stale_branch = enable.group(0)[stale_timeout:]
        self.assertIn("setStep(Step::STOP)", stale_branch)
        self.assertIn("StopReason::GATE, true", stale_branch)
        self.assertIn("params->ctrl.speed = 0.0f", stale_branch)
        self.assertNotIn("setStep(Step::NONE)", stale_branch)
        self.assertIn("if (!params->aiResultFresh)", stopped.group(0))
        self.assertIn("StopReason::GATE, true", stopped.group(0))
        self.assertNotIn("timeout", stopped.group(0))
        self.assertIn("countSes >= 30", stopped.group(0))

    def test_fork_computed_sizes_and_gradients_are_guarded(self):
        source = (ROOT / "src" / "fsm" / "fork.cpp").read_text(encoding="utf-8")
        self.assertIn("edgeCount <= 0 || edgeCount > ROWSIMAGE", source)
        self.assertIn("targetEdgeCount <= 0 || targetEdgeCount > ROWSIMAGE", source)
        self.assertIn("if (deltaPoint > 0)", source)
        self.assertIn("gradientStart.x != gradientEnd.x", source)
        self.assertIn("params->track->pointsEdgeRight[0].x == lastForkR.x", source)
        self.assertNotIn(
            "resize(params->track->pointsEdgeRight[0].x - lastForkR.x + 1)",
            source,
        )
        self.assertNotIn(
            "resize(params->track->pointsEdgeRight[0].x - TempPoint.x + 1)",
            source,
        )
        exit_repair_start = source.index("vector<PointX> repair0{lastForkR}")
        handle = source.index("params->track->handle(true, edgeCount)", exit_repair_start)
        collect = source.index("repair0.push_back(params->track->pointsEdgeRight[i])", handle)
        truncate = source.index(
            "pointsEdgeRight.resize(static_cast<size_t>(edgeCount))", collect)
        smooth = source.index("repair0 = smoothLine(repair0)", truncate)
        append = source.index("pointsEdgeRight.push_back(point)", smooth)
        self.assertLess(handle, collect)
        self.assertLess(collect, truncate)
        self.assertLess(truncate, smooth)
        self.assertLess(smooth, append)
    def test_passenger_wait_states_stop_the_vehicle(self):
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        station = (ROOT / "src" / "fsm" / "station.cpp").read_text(encoding="utf-8")
        pickup = re.search(r"case Step::WAIT_PICKUP:.*?break;", park, re.DOTALL)
        self.assertIsNotNone(pickup)
        self.assertIn("StopReason::PARK, true", pickup.group(0))
        self.assertIn("timeout > 90", pickup.group(0))
        self.assertIn("stopCounter > 90", station)

    def test_emergency_stop_freezes_all_fsm_progress(self):
        source = (ROOT / "src" / "runtime" / "state_machines.cpp").read_text(encoding="utf-8")
        self.assertIn(
            "params->autoRecoveryFrames <= 0 && !params->laneSafetyStop &&\n            (frame.manualBeforeFsm || !frame.aiStale))\n            runFsm(frame.binary);",
            source,
        )

    def test_live_video_is_published_before_control_processing(self):
        preprocess = (ROOT / "src" / "runtime" / "preprocess.cpp").read_text(encoding="utf-8")
        snapshot = (ROOT / "src" / "runtime" / "ai_snapshot.cpp").read_text(encoding="utf-8")
        running = (ROOT / "src" / "runtime" / "running.cpp").read_text(encoding="utf-8")
        self.assertLess(preprocess.index("predeal->correction(frame.image);"),
                        preprocess.index("fsmFactory.manual->sendImage("))
        self.assertLess(running.index("preprocessFrame(frame);"),
                        running.index("consumeAiSnapshot(frame);"))
        self.assertLess(running.index("consumeAiSnapshot(frame);"),
                        running.index("runStateMachines(frame);"))

    def test_manual_control_requires_fresh_video_and_serializes_sends(self):
        source = (ROOT / "tools" / "manual_control_client.py").read_text(encoding="utf-8")
        self.assertIn("self.send_lock = threading.Lock()", source)
        self.assertIn("with self.send_lock:", source)
        self.assertIn("now - reference_time > 0.75", source)
        self.assertIn("self.fresh_frame_streak >= 3", source)
        self.assertIn('self.send("STOP")', source)
        self.assertIn('text="画面已失效" if self.video_stale else ""', source)
        self.assertIn('self.root.bind("<FocusOut>", self.focus_lost)', source)
        self.assertNotIn("key_deadlines", source)
        self.assertIn("now - self.gui_heartbeat_time <= 0.3", source)
        self.assertIn("self.gui_heartbeat_time = time.monotonic()", source)
        self.assertIn("and self.drive_enabled", source)
        self.assertIn('self.root.bind("<KeyPress-Shift_L>", self.enable_down)', source)
        self.assertIn("1500.0 - self.current_steering", source)
        self.assertIn("cv2.polylines", source)
        self.assertIn("Dashed red planned center line", source)
        self.assertIn("真实车道线（L）", source)
        self.assertIn("AI框和运行信息（B）", source)
        self.assertIn("OVERLAY:", source)
        self.assertIn("abs(self.last_frame_id - int(overlay.get", source)

    def test_overlay_protocol_is_latest_only_and_frame_correlated(self):
        header = (ROOT / "include" / "fsm" / "manualControl.hpp").read_text(
            encoding="utf-8")
        server = (ROOT / "src" / "fsm" / "manualControl.cpp").read_text(
            encoding="utf-8")
        core = read_runtime_sources()
        client = (ROOT / "tools" / "manual_control_client.py").read_text(
            encoding="utf-8")
        self.assertIn("std::atomic<bool> hasOverlay", header)
        self.assertIn("std::chrono::milliseconds(80)", server)
        self.assertIn("IMAGE:", server)
        self.assertIn("OVERLAY:", server)
        self.assertIn("frame_id", core)
        self.assertIn("frame_timestamp_ms", core)
        self.assertIn("constexpr size_t stride = 4", core)
        self.assertIn("center_line", core)
        self.assertIn("detections", core)
        self.assertIn("detections_frame_id", core)
        self.assertIn("valid_left", core)
        self.assertIn("lanes_valid", core)
        self.assertIn("center_valid", core)
        self.assertIn("!params->manualTakeover", core)
        self.assertIn("overlayNow - lastOverlayBuilt", core)
        self.assertIn("<= 4", client)
        self.assertIn("overlay_matches and not self.manual_mode", client)

    def test_scene_entry_counts_only_fresh_ai_results(self):
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        self.assertIn("busyConfirmation, params->aiResultFresh, busyDetected", busy)
        park_none = re.search(r"case Step::NONE:.*?break;", park, re.DOTALL)
        self.assertIsNotNone(park_none)
        self.assertIn("if (!params->aiResultFresh)", park_none.group(0))
    def test_park_internal_ai_evidence_and_exit_timeouts(self):
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        enable = re.search(r"case Step::ENABLE:.*?case Step::FORKIN:", park, re.DOTALL)
        forkin = re.search(r"case Step::FORKIN:.*?case Step::TRACKIN:", park, re.DOTALL)
        self.assertIsNotNone(enable)
        self.assertIsNotNone(forkin)
        self.assertIn("if (params->aiResultFresh)", enable.group(0))
        self.assertIn("params->aiResultFresh && findSymbols", forkin.group(0))
        self.assertIn("count() >= 2000", park)
        self.assertIn("count() >= 2000", busy)

        trackin = re.search(r"case Step::TRACKIN:.*?case Step::ENTER:", park, re.DOTALL)
        trackout = re.search(r"case Step::TRACKOUT:.*?case Step::FORKOUT:", park, re.DOTALL)
        self.assertIsNotNone(trackin)
        self.assertIsNotNone(trackout)
        self.assertIn("params->aiResultFresh", trackin.group(0))
        self.assertIn("params->aiResultFresh", trackout.group(0))
        self.assertNotIn("if (timeout > 45)\n            countRes++", trackout.group(0))

    def test_scene_entry_counts_at_most_once_per_ai_frame(self):
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        self.assertIn("bool parkDetectedThisFrame = false;", park)
        self.assertIn("if (parkDetectedThisFrame)\n            countRes++;", park)
        self.assertIn("std::any_of(params->results.begin(), params->results.end()", busy)
        self.assertIn("updateBusyConfirmation", busy)


    def test_hardware_independent_safety_states_are_wired(self):
        algorithms = read_algorithm_headers()
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        cross = (ROOT / "src" / "fsm" / "cross.cpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        self.assertIn("BUSY_CLEAR_NEGATIVE_FRAMES", algorithms)
        self.assertIn("updateBusyConfirmation", busy)
        self.assertNotIn("Fourth confirmation timed out", busy)
        self.assertIn("updateCrossConfirmation", cross)
        self.assertIn("StopReason::AI_STALE", core)
    def test_automatic_busy_path_completes_and_resets_confirmation(self):
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        confirmed = busy.index("BusyConfirmationEvent::CONFIRMED")
        automatic = busy.index("drivingThrough = true;", confirmed)
        self.assertGreater(automatic, confirmed)
        exit_complete = busy.index('params->completeLapTask("construction-exit")')
        reset = busy.rindex(
            "busyConfirmation = control_algorithms::BusyConfirmationState{};",
            0,
            exit_complete,
        )
        self.assertLess(reset, exit_complete)
    def test_stop_mode_survives_inactive_cross_and_alerts_run_outside_fsm(self):
        fsm = (ROOT / "src" / "runtime" / "fsm.cpp").read_text(encoding="utf-8")
        alerts = (ROOT / "src" / "runtime" / "alerts.cpp").read_text(encoding="utf-8")
        running = (ROOT / "src" / "runtime" / "running.cpp").read_text(encoding="utf-8")
        self.assertIn("if (crossMode != FsmMode::NORMAL)", fsm)
        self.assertIn("void Icar::updateAlerts()", alerts)
        self.assertIn("updateAlerts();", (ROOT / "src" / "runtime" / "ai_snapshot.cpp").read_text(encoding="utf-8"))
        self.assertLess(running.index("consumeAiSnapshot(frame);"), running.index("runStateMachines(frame);"))
        self.assertNotIn("advanceAlertCountdown", fsm)
        self.assertNotIn("updateAlertDecelCountdown", fsm)

    def test_boot_watchdog_is_half_second(self):
        boot = (ROOT / "src" / "tool" / "boot.cpp").read_text(encoding="utf-8")
        self.assertIn("constexpr int64_t WATCHDOG_TIMEOUT_MS = 500;", boot)
        self.assertIn("server.watchdogExpired(WATCHDOG_TIMEOUT_MS)", boot)

if __name__ == "__main__":
    unittest.main()
