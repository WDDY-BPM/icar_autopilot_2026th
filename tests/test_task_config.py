import unittest

from src.speech.speech import normalizeTasks, parsedToLapConfig


class TaskConfigTests(unittest.TestCase):
    def test_tasks_are_limited_to_three_laps(self):
        parsed = {
            "tasks": [
                {"type": "park", "spot": 1},
                {"type": "construction", "stop": 2},
                {"type": "fork", "direction": "left"},
                {"type": "park", "spot": 4},
            ]
        }
        normalizeTasks(parsed)
        laps = parsedToLapConfig(parsed)
        self.assertEqual(3, len(parsed["tasks"]))
        self.assertEqual({"lap1", "lap2", "lap3"}, set(laps))

    def test_construction_lap_enables_manual_takeover(self):
        parsed = {"tasks": [{"type": "construction", "stop": 1}]}
        normalizeTasks(parsed)
        lap = parsedToLapConfig(parsed)["lap1"]
        self.assertTrue(lap["busy"])
        self.assertTrue(lap["manualTakeover"])
        self.assertEqual(1, lap["busyStopPoint"])


if __name__ == "__main__":
    unittest.main()
