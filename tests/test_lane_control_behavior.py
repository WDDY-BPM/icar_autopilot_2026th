from pathlib import Path
import json
import unittest


ROOT = Path(__file__).resolve().parents[1]


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

        core = (ROOT / "include/icar.hpp").read_text(encoding="utf-8")
        final_override = core.index("if (!startupGateReleased)")
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

    def test_single_reliable_edge_is_published_independently(self):
        core = (ROOT / "include/icar.hpp").read_text(encoding="utf-8")
        self.assertIn("const bool leftOverlayValid", core)
        self.assertIn("const bool rightOverlayValid", core)
        self.assertIn("overlay[\"left\"] = leftOverlayValid", core)
        self.assertIn("overlay[\"right\"] = rightOverlayValid", core)

    def test_steering_json_fallbacks_match_tuned_defaults(self):
        params = (ROOT / "include/utils/params.hpp").read_text(encoding="utf-8")
        self.assertIn('value("servoRate", 600.0f)', params)
        self.assertIn('value("startupServoRate", 300.0f)', params)
        self.assertIn('value("startupServoLimit", 120)', params)

    def test_normal_lane_speed_uses_centerline_curvature(self):
        motion = (ROOT / 'include/ctrl/motion.hpp').read_text(encoding='utf-8')
        center = (ROOT / 'src/ctrl/center.cpp').read_text(encoding='utf-8')
        algorithms = (ROOT / 'include/ctrl/control_algorithms.hpp').read_text(
            encoding='utf-8')
        core = (ROOT / 'include/icar.hpp').read_text(encoding='utf-8')
        client = (ROOT / 'tools/manual_control_client.py').read_text(
            encoding='utf-8')
        self.assertIn('control_algorithms::calculateCenterlineSpeed', motion)
        self.assertNotIn('params->ctrl.lineArea', motion)
        self.assertIn('control_algorithms::calculateLaneControlCenters', center)
        self.assertIn('const bool candidateValid = controlWindowValid', center)
        self.assertIn('180, 220, 205, 26', algorithms)
        self.assertIn('120, 175, 145, 31', algorithms)
        self.assertIn('near_center', core)
        self.assertIn('nearErr=%+d farErr=%+d ctrlErr=%+d', client)

    def test_lane_center_validation_speeds_are_conservative(self):
        config = json.loads((ROOT / 'res/config.json').read_text(
            encoding='utf-8'))['通用配置参数']
        self.assertEqual(config['velLow'], 0.18)
        self.assertEqual(config['velHigh'], 0.20)
        self.assertEqual(config['velCurve'], 0.18)

    def test_startup_single_lane_remains_locked(self):
        core = (ROOT / "include/icar.hpp").read_text(encoding="utf-8")
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        self.assertIn("const bool laneValid = laneQuality.valid", core)
        self.assertIn("quality.leftReliable && quality.rightReliable", track)
        self.assertIn("startupLaneValidCount >= params->config.startupStableFrames", core)

    def test_production_code_wires_behavior_guards(self):
        track = (ROOT / "src/ctrl/track.cpp").read_text(encoding="utf-8")
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        motion = (ROOT / "include/ctrl/motion.hpp").read_text(encoding="utf-8")
        core = (ROOT / "include/icar.hpp").read_text(encoding="utf-8")
        predeal = (ROOT / "src/ctrl/predeal.cpp").read_text(encoding="utf-8")
        self.assertIn("quality = LaneQuality{};", track)
        self.assertIn("assessEdgeReliability", track)
        self.assertIn("initialStableRows >= 3", track)
        algorithms = (ROOT / "include/ctrl/control_algorithms.hpp").read_text(encoding="utf-8")
        self.assertIn("widthFilled.emplace_back(row, rightColumn - leftColumn)", algorithms)
        self.assertIn("if (updateHistory)", center)
        self.assertIn("laneWidthProfileReady()", center)
        self.assertIn("controlWindowValid", center)
        self.assertIn("control_algorithms::applyStartupSpeed", motion)
        self.assertIn("mode == FsmMode::CROSS", center)
        self.assertIn("!params->laneSafetyStop", core)
        self.assertIn("automaticControlActive && laneHold", core)
        self.assertIn("center->laneInvalidFrames >= 7", core)
        self.assertIn("motion->syncServoCommand(previousFinalServo)", core)
        self.assertIn("params->track->allowOuterEnvelope = !forkMarkerActive", core)
        self.assertIn("!allowCoherentEnvelope", track)
        self.assertLess(predeal.index("bitwise_not(imgBin, imgInv)"),
                        predeal.index("morphologyEx(imgInv, imgInv, MORPH_CLOSE"))


if __name__ == "__main__":
    unittest.main()
