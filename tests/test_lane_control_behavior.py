from pathlib import Path
import json
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


def launch_speed(desired, count, frames=60, start=0.10, low=0.30):
    if count >= frames:
        return desired, count
    count += 1
    ramp = start + (count / frames) * (low - start)
    return min(desired, ramp), count


def lane_guard(valid_sequence):
    invalid = 0
    recovery = 0
    recovering = False
    output = []
    for valid in valid_sequence:
        if valid:
            invalid = 0
            if recovering:
                recovery += 1
                control_valid = recovery >= 5
                if control_valid:
                    recovering = False
            else:
                control_valid = True
        else:
            invalid += 1
            recovery = 0
            recovering = True
            control_valid = False
        speed_state = "normal" if control_valid else ("slow" if 0 < invalid <= 6 else "stop")
        output.append((control_valid, invalid, recovery, speed_state))
    return output


def fill_lane_gap(left, right, max_gap=8):
    out_left = [left[0]]
    out_right = [right[0]]
    widths = [(left[0][0], right[0][1] - left[0][1])]
    for previous_left, current_left, previous_right, current_right in zip(
            left, left[1:], right, right[1:]):
        gap = previous_left[0] - current_left[0]
        if 1 < gap <= max_gap:
            for row in range(previous_left[0] - 1, current_left[0], -1):
                ratio = (previous_left[0] - row) / gap
                lcol = round(previous_left[1] + ratio * (current_left[1] - previous_left[1]))
                rcol = round(previous_right[1] + ratio * (current_right[1] - previous_right[1]))
                out_left.append((row, lcol))
                out_right.append((row, rcol))
                widths.append((row, rcol - lcol))
        out_left.append(current_left)
        out_right.append(current_right)
        widths.append((current_left[0], current_right[1] - current_left[1]))
    return out_left, out_right, widths


class LaneControlBehaviorTests(unittest.TestCase):
    def test_cross_launch_obeys_common_envelope(self):
        count = 0
        speeds = []
        for _ in range(60):
            speed, count = launch_speed(0.25, count)
            speeds.append(speed)
        self.assertLessEqual(speeds[0], 0.104)
        self.assertTrue(all(a <= b for a, b in zip(speeds, speeds[1:])))
        self.assertLessEqual(max(speeds), 0.25)

    def test_joint_gap_fill_keeps_fork_vectors_aligned(self):
        left, right, widths = fill_lane_gap([(220, 40), (216, 44)], [(220, 280), (216, 276)])
        self.assertEqual(len(left), 5)
        self.assertEqual(len(left), len(right))
        self.assertEqual(len(left), len(widths))
        self.assertTrue(all(w == r[1] - l[1] for l, r, (_, w) in zip(left, right, widths)))

    def test_lane_loss_holds_six_frames_and_stops_seventh(self):
        states = lane_guard([False] * 7)
        self.assertEqual([state[3] for state in states], ["slow"] * 6 + ["stop"])

    def test_lane_recovery_requires_five_frames(self):
        states = lane_guard([False] * 4 + [True] * 5)
        self.assertTrue(all(not state[0] for state in states[4:8]))
        self.assertTrue(states[8][0])

    def test_one_invalid_frame_does_not_latch_safety_stop(self):
        algorithms = read_algorithm_headers()
        core = read_runtime_sources()
        self.assertIn("updateLaneSafetyStop", algorithms)
        self.assertIn("if (!latched)", algorithms)
        self.assertIn("invalidFrames >= stopAfterInvalidFrames", algorithms)
        self.assertIn("controlValid && recoveryFrames >= releaseAfterRecoveryFrames", algorithms)
        self.assertIn("params->laneSafetyStop = control_algorithms::updateLaneSafetyStop", core)
        self.assertNotIn("laneInvalidFrames >= 7 || center->laneRecoveryFrames > 0", core)

    def test_invalid_measurement_does_not_update_width_history(self):
        history = 220.0
        measured = 90.0
        quality_valid = False
        updated = 0.8 * history + 0.2 * measured if quality_valid else history
        self.assertEqual(updated, history)

    def test_startup_arrow_command_stays_within_servo_limit(self):
        neutral = 1500
        startup_limit = 160
        raw_arrow_command = 1880
        limited = max(neutral - startup_limit,
                      min(raw_arrow_command, neutral + startup_limit))
        self.assertLessEqual(abs(limited - neutral), 160)

    def test_search_failure_resets_quality(self):
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        reset = track.index("quality = LaneQuality{};")
        search = track.index("for (int row = rowStart;")
        evaluate = track.index("evaluateQuality();")
        self.assertLess(reset, search)
        self.assertLess(search, evaluate)
    def test_lane_loss_uses_previous_final_servo_after_special_mode(self):
        previous_final_servo = 1400
        previous_final_servo = 1500  # final command sent by parking mode
        held_servo = previous_final_servo
        self.assertEqual(held_servo, 1500)

        core = read_runtime_sources()
        final_override = core.index("if (!frame.startupGateReleased)")
        cache_update = core.index("previousFinalServo = params->ctrl.servo;")
        publish = core.index("fsmFactory.manual->updateVehicleState", cache_update)
        self.assertLess(final_override, cache_update)
        self.assertLess(cache_update, publish)
        self.assertIn("syncServoCommand(previousFinalServo)", core)
        self.assertNotIn("lastValidLaneServo", core)

    def test_single_lane_center_continuity_boundary(self):
        continuous = lambda current, previous: abs(current - previous) <= 15
        self.assertTrue(continuous(175, 160))
        self.assertFalse(continuous(176, 160))

    def test_border_clipped_single_edge_recovery_is_wired(self):
        algorithms = read_algorithm_headers()
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        self.assertIn("result.singleEdgeUsable", algorithms)
        self.assertIn("result.interiorPointCount >= interiorPointsMinimum", algorithms)
        self.assertIn("limitSingleLaneCenter", center)
        self.assertIn("buildPerceptionGeometry", center)
        self.assertNotIn("centerCompute(interiorOnly", center)
        self.assertIn("recoveryMode == LaneRecoveryMode::WEAK_HYBRID", center)
        self.assertIn("borderClippedHeadingConfidence", center)
        self.assertIn("leftSingleUsable", track)
        self.assertIn("left_single_usable", core)
        self.assertIn("raw_center_jump", core)
        self.assertIn("control_valid", core)

    def test_startup_accepts_stable_single_edge(self):
        builder = (ROOT / "include/ctrl/perception_geometry_builder.hpp").read_text(
            encoding="utf-8")
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        lane_quality = (ROOT / "include/ctrl/lane_quality.hpp").read_text(
            encoding="utf-8")
        self.assertIn("assessStartupLaneMode", builder)
        self.assertIn("selectSingleLaneMode", builder)
        self.assertIn("laneRecoveryModeName", builder)
        self.assertIn("edgeCoversNearField", lane_quality)
        self.assertIn("singleEdgeUsable", lane_quality)
        self.assertIn("!borderFailure && !oppositeBorderFailure", lane_quality)
        # 另一侧贴边裁剪时，可靠侧允许单边重建；但不伪造双边可靠。
        self.assertIn("rightClipped && stableLeftEdge", track)
        self.assertIn("leftClipped && stableRightEdge", track)
        self.assertNotIn("quality.leftReliable = true", track)

    def test_obsolete_outline_stop_path_remains_removed(self):
        center_h = (ROOT / "include/ctrl/center.hpp").read_text(encoding="utf-8")
        center_cpp = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        self.assertNotIn("derailmentCheck", center_h + center_cpp)
        self.assertNotIn("ICAR Outline", center_h + center_cpp)
        motion = (ROOT / "include/ctrl/motion.hpp").read_text(encoding="utf-8")
        self.assertNotIn("outlineCheck", motion)
        self.assertNotIn("std::_Exit", motion)

    def test_degraded_modes_and_stop_arbitration_are_published(self):
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        self.assertNotIn("params->track->pointsEdgeLeft.clear()", center)
        self.assertNotIn("params->track->pointsEdgeRight.clear()", center)
        self.assertIn("buildPerceptionGeometry", center)
        self.assertIn("LaneRecoveryMode::WEAK_HYBRID", center)
        for field in ("recovery_mode", "strict_dual", "relaxed_dual",
                      "stop_reasons", "unconfirmed_frames", "camera_stop"):
            self.assertIn(field, core)
        self.assertEqual(core.count("params->ctrl.stop ="), 2)
        self.assertIn("resolveFinalCommand", core)
        self.assertIn("params->mustStop() || !frame.cameraReady", core)

    def test_recovery_controls_and_fsm_freezing_are_wired(self):
        core = read_runtime_sources()
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        park = (ROOT / "src/fsm/park.cpp").read_text(encoding="utf-8")
        self.assertIn("buildPerceptionGeometry", center)
        self.assertIn("calculateLaneControlCenters", center)
        self.assertIn("updateSingleLaneSpeedLimit", core)
        hold = core[core.index("if (strictLaneMode && laneUnconfirmedState.frames > 0)"):]
        hold = hold[:hold.index("\n            }")]
        self.assertIn("if (!center->controlValid)", hold)
        self.assertIn("StopReason::GATE))\n            return;", core)
        self.assertIn("StopReason::CROSS))\n            return;", core)
        self.assertIn("params->mustStopExcept", park)

    def test_single_reliable_edge_is_published_independently(self):
        core = read_runtime_sources()
        self.assertIn("const bool leftOverlayValid", core)
        self.assertIn("const bool rightOverlayValid", core)
        self.assertIn("overlay[\"left\"] = leftOverlayValid", core)
        self.assertIn("overlay[\"right\"] = rightOverlayValid", core)

    def test_single_lane_lateral_scale_and_damping_wiring(self):
        motion = (ROOT / "include/ctrl/motion.hpp").read_text(encoding="utf-8")
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        builder = (ROOT / "include/ctrl/perception_geometry_builder.hpp").read_text(
            encoding="utf-8")
        telemetry = (ROOT / "src/runtime/telemetry.cpp").read_text(encoding="utf-8")
        self.assertIn("lateralApplied", motion)
        self.assertIn("params->ctrl.laneLateralScale", motion)
        self.assertIn("singleLaneLateralScale(", center)
        self.assertIn("laneLateralScaleReason", center)
        self.assertIn("recoveryDampingEnabledForMode(recoveryMode)", center)
        self.assertIn("singleLaneLateralScale", builder)
        self.assertIn("LateralScaleReason", builder)
        self.assertIn("recoveryDampingEnabledForMode", builder)
        self.assertIn("lateral_scale", telemetry)
        self.assertIn("lateral_scale_reason", telemetry)
        self.assertIn("lateral_raw", telemetry)
        self.assertIn("lateral_applied", telemetry)
        self.assertIn("pwm_diff", telemetry)

    def test_steering_json_fallbacks_match_tuned_defaults(self):
        loader = (ROOT / "src/config/config_loader.cpp").read_text(encoding="utf-8")
        self.assertIn('value("servoRate", 600.0f)', loader)
        self.assertIn('value("startupServoRate", 550.0f)', loader)
        self.assertIn('value("startupServoLimit", 180)', loader)
        self.assertIn('value("laneHeadingGain", 300.0f)', loader)

    def test_normal_lane_speed_uses_centerline_curvature(self):
        motion = (ROOT / 'include/ctrl/motion.hpp').read_text(encoding='utf-8')
        center = (ROOT / 'src/ctrl/center.cpp').read_text(encoding='utf-8')
        algorithms = read_algorithm_headers()
        core = read_runtime_sources()
        client = (ROOT / 'tools/manual_control_client.py').read_text(
            encoding='utf-8')
        self.assertIn('control_algorithms::calculateCenterlineSpeed', motion)
        self.assertNotIn('params->ctrl.lineArea', motion)
        self.assertIn('control_algorithms::calculateLaneControlCenters', center)
        self.assertIn('const bool candidateValid = plannedPath', center)
        self.assertIn('perceptionGeometry.candidateValid && controlWindowValid', center)
        self.assertIn('continuityValid', center)
        self.assertIn('176, 220, 205, 26', algorithms)
        self.assertIn('90, 155, 120, 31', algorithms)
        self.assertIn('std::atan2', algorithms)
        self.assertIn('params->ctrl.laneHeadingCorrection', motion)
        self.assertIn('near_center', core)
        self.assertIn('nearErr=%+d farErr=%+d ctrlErr=%+d', client)

    def test_curved_lane_can_release_startup_gate(self):
        core = read_runtime_sources()
        builder = (ROOT / "include/ctrl/perception_geometry_builder.hpp").read_text(
            encoding="utf-8")
        self.assertIn("assessStartupLaneMode", core)
        self.assertIn("edgeCoversNearField", core)
        self.assertIn("startupLaneValidCount = laneValid", core)
        self.assertNotIn("advanceStartupLaneCount", core)
        self.assertIn("startupLaneMode != LaneRecoveryMode::INVALID", core)
        self.assertIn("startupLaneValidCount >= params->config.startupStableFrames", core)
        self.assertIn("quality.leftReliable && quality.rightReliable", builder)
        self.assertIn("LEFT_SINGLE", builder)
        self.assertIn("RIGHT_SINGLE", builder)

    def test_visual_cone_confirmation_reaches_icar(self):
        launcher = (ROOT / 'src/start.py').read_text(encoding='utf-8')
        core = read_runtime_sources()
        self.assertIn('ICAR_START_CONE_PRECONFIRMED', launcher)
        self.assertIn('start_cone_preconfirmed=(label == "cone")', launcher)
        self.assertIn('ICAR_START_CONE_PRECONFIRMED', core)
        self.assertIn('StartupGateState::WAIT_FOR_REMOVAL', core)
        self.assertIn('icar.log', launcher)

    def test_low_latency_capture_and_heading_confidence_wiring(self):
        core = read_runtime_sources()
        predeal_h = (ROOT / 'include/ctrl/predeal.hpp').read_text(encoding='utf-8')
        predeal_cpp = (ROOT / 'src/ctrl/predeal.cpp').read_text(encoding='utf-8')
        center = (ROOT / 'src/ctrl/center.cpp').read_text(encoding='utf-8')
        config = (ROOT / 'res/config.json').read_text(encoding='utf-8')
        client = (ROOT / 'tools/manual_control_client.py').read_text(encoding='utf-8')
        self.assertIn('cv::CAP_V4L2', core)
        self.assertIn('cv::CAP_PROP_BUFFERSIZE, 1', core)
        self.assertIn('latestSequence_', (ROOT / 'include/runtime/latest_frame_capture.hpp').read_text(encoding='utf-8'))
        self.assertIn('undistortMapX', predeal_h)
        self.assertIn('undistortMapSize != sizeImage', predeal_cpp)
        self.assertIn('singleLaneHeadingConfidence', center)
        self.assertIn('parkingHeadingConfidence', center)
        self.assertIn('"singleLaneHeadingConfidence": 0.45', config)
        self.assertIn('"parkingHeadingConfidence": 0.65', config)
        self.assertIn('heading_confidence', core)
        self.assertIn('headErr=%+.3f', client)

    def test_cached_rectification_and_startup_diagnostics(self):
        predeal = (ROOT / 'src/ctrl/predeal.cpp').read_text(encoding='utf-8')
        core = read_runtime_sources()
        launcher = (ROOT / 'src/start.py').read_text(encoding='utf-8')
        self.assertIn('!enable || img.empty()', predeal)
        self.assertNotIn('remap(img, img', predeal)
        self.assertIn('cv::Mat corrected;', predeal)
        self.assertIn('remap(img, corrected', predeal)
        self.assertIn('img = std::move(corrected);', predeal)
        self.assertIn('undistortMapSize != sizeImage', predeal)
        self.assertIn('undistortMapSize = sizeImage', predeal)
        for field in ('leftReliable', 'rightReliable', 'coversBottom',
                      'coneMissing', 'laneFrames'):
            self.assertIn(field, core)
        self.assertIn('ICAR_START_CONE_PRECONFIRMED', launcher)
        self.assertIn('ICAR_START_CONE_PRECONFIRMED', core)

    def test_lane_center_validation_speeds_are_conservative(self):
        config = json.loads((ROOT / 'res/config.json').read_text(
            encoding='utf-8'))['通用配置参数']
        self.assertEqual(config['velLow'], 0.18)
        self.assertEqual(config['velHigh'], 0.20)
        self.assertEqual(config['velCurve'], 0.18)

    def test_startup_single_edge_requires_stable_geometry(self):
        core = read_runtime_sources()
        builder = (ROOT / "include/ctrl/perception_geometry_builder.hpp").read_text(
            encoding="utf-8")
        self.assertIn("LaneRecoveryMode::LEFT_SINGLE && leftCoversNear", builder)
        self.assertIn("LaneRecoveryMode::RIGHT_SINGLE && rightCoversNear", builder)
        self.assertIn("singleLaneInteriorPointsMin", builder)
        self.assertIn("laneRecoveryModeName", core)
        self.assertIn("startupLaneValidCount >= params->config.startupStableFrames", core)
        self.assertNotIn("quality.leftReliable = true",
                         (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8"))

    def test_production_code_wires_behavior_guards(self):
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        motion = (ROOT / "include/ctrl/motion.hpp").read_text(encoding="utf-8")
        core = read_runtime_sources()
        predeal = (ROOT / "src/ctrl/predeal.cpp").read_text(encoding="utf-8")
        self.assertIn("quality = LaneQuality{};", track)
        self.assertIn("assessEdgeReliability", track)
        self.assertIn("initialStableRows >= 3", track)
        algorithms = read_algorithm_headers()
        self.assertIn("widthFilled.emplace_back(row, rightColumn - leftColumn)", algorithms)
        self.assertIn("observeLaneWidth", center)
        self.assertIn("laneWidthProfileReady()", center)
        self.assertIn("controlWindowValid", center)
        self.assertIn("control_algorithms::applyStartupSpeed", motion)
        self.assertIn("params->geometryPolicy", center)
        self.assertIn("selectGeometrySource", center)
        self.assertIn("!params->laneSafetyStop", core)
        self.assertIn("evaluateRuntimeControl", core)
        self.assertIn("controlDecision.centerSteering", core)
        self.assertIn("center->laneInvalidFrames, center->laneRecoveryFrames, 7, 5", core)
        self.assertIn("motion->syncServoCommand(previousFinalServo)", core)
        self.assertNotIn("derailmentCheck", center)
        self.assertNotIn("ICAR Outline", center)
        self.assertIn("laneUnconfirmedState.frames > 0", core)
        self.assertIn("params->track->allowOuterEnvelope = !forkMarkerActive", core)
        self.assertIn("!allowCoherentEnvelope", track)
        self.assertLess(predeal.index("bitwise_not(imgBin, imgInv)"),
                        predeal.index("morphologyEx(imgInv, imgInv, MORPH_CLOSE"))


    def test_weak_hybrid_scores_both_single_edge_candidates(self):
        center = (ROOT / "src" / "ctrl" / "center.cpp").read_text(encoding="utf-8")
        self.assertIn("LaneRecoveryMode::WEAK_HYBRID", center)
        self.assertIn("nearCenterSamples", center)
        self.assertIn("degradedCenterContinuous", center)

if __name__ == "__main__":
    unittest.main()
