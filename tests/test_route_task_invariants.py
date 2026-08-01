from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class RouteTaskInvariantTests(unittest.TestCase):
    def test_cross_requires_current_lap_task(self):
        source = (ROOT / "src" / "fsm" / "cross.cpp").read_text(encoding="utf-8")
        gate = source.index("params->lapTaskRequired && !params->lapTaskCompleted")
        count = source.index("crossCount++", gate)
        next_lap = source.index("params->nextLap()", count)
        self.assertLess(gate, count)
        self.assertLess(count, next_lap)
        self.assertIn("if (!params->aiResultFresh)", source)
        self.assertIn("++crossPassConfirmCount >= 2", source)

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

    def test_passenger_wait_states_stop_the_vehicle(self):
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        station = (ROOT / "src" / "fsm" / "station.cpp").read_text(encoding="utf-8")
        pickup = re.search(r"case Step::WAIT_PICKUP:.*?break;", park, re.DOTALL)
        self.assertIsNotNone(pickup)
        self.assertIn("params->ctrl.stop = true;", pickup.group(0))
        self.assertIn("timeout > 90", pickup.group(0))
        self.assertIn("stopCounter > 90", station)

    def test_emergency_stop_freezes_all_fsm_progress(self):
        source = (ROOT / "include" / "icar.hpp").read_text(encoding="utf-8")
        self.assertIn(
            "if (startupGateReleased && !emergencyStopRequested && params->autoRecoveryFrames <= 0)\n            runFsm(imgBin);",
            source,
        )

    def test_live_video_is_published_before_control_processing(self):
        source = (ROOT / "include" / "icar.hpp").read_text(encoding="utf-8")
        publish = source.index("fsmFactory.manual->sendImage(img);")
        predeal = source.index("predeal->correction(img);")
        startup_gate = source.index("updateStartupGate(receivedNewAiResult)")
        run_fsm = source.index("runFsm(imgBin);")
        self.assertLess(publish, predeal)
        self.assertLess(publish, startup_gate)
        self.assertLess(publish, run_fsm)

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
        self.assertIn("self.current_steering - 1500.0", source)
        self.assertIn("cv2.polylines", source)
        self.assertIn("Dashed red planned center line", source)

    def test_scene_entry_counts_only_fresh_ai_results(self):
        busy = (ROOT / "src" / "fsm" / "busy.cpp").read_text(encoding="utf-8")
        park = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        self.assertRegex(
            busy,
            r"if \(params->aiResultFresh\)[\s\S]*?LABEL_BUSY[\s\S]*?countRec\+\+;",
        )
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
        self.assertIn("bool countedBusyThisFrame = false;", busy)
        self.assertIn("!countedBusyThisFrame", busy)


if __name__ == "__main__":
    unittest.main()
