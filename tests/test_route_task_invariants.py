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

    def test_construction_boxes_are_edge_counted(self):
        source = (ROOT / "src" / "fsm" / "station.cpp").read_text(encoding="utf-8")
        self.assertNotIn("busyEntryDelay", source)
        self.assertIn("stationVisible && boxAtThreshold && boxArmed", source)
        self.assertIn("if (++boxMissingFrames >= 5)", source)
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


if __name__ == "__main__":
    unittest.main()
