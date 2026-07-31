import json
import os
import tempfile
import unittest

from src.speech.speech import normalizeTasks, parsedToLapConfig, updateConfigJson


class TaskConfigTests(unittest.TestCase):
    def test_more_than_three_tasks_are_rejected(self):
        parsed = {"tasks": [
            {"type": "park", "spot": 1},
            {"type": "construction", "stop": 2},
            {"type": "fork", "direction": "left"},
            {"type": "park", "spot": 4},
        ]}
        with self.assertRaises(ValueError):
            normalizeTasks(parsed)

    def test_construction_lap_enables_manual_takeover(self):
        parsed = {"tasks": [{"type": "construction", "stop": 1}]}
        lap = parsedToLapConfig(parsed)["lap1"]
        self.assertTrue(lap["busy"])
        self.assertTrue(lap["manualTakeover"])
        self.assertEqual(1, lap["busyStopPoint"])

    def test_unknown_and_out_of_range_tasks_are_rejected(self):
        invalid = [
            {"tasks": [{"type": "unknown"}]},
            {"tasks": [{"type": "park", "spot": 5}]},
            {"tasks": [{"type": "construction", "stop": 0}]},
            {"tasks": [{"type": "fork", "direction": "straight"}]},
        ]
        for parsed in invalid:
            with self.subTest(parsed=parsed), self.assertRaises(ValueError):
                normalizeTasks(parsed)

    def test_update_fully_replaces_all_three_laps(self):
        stale = {"manualTakeover": True, "busy": True, "park": False}
        config = {
            "圈数配置": {"totalLaps": 3},
            "每圈功能使能配置": {f"lap{i}": dict(stale) for i in range(1, 4)},
        }
        parsed = {"tasks": [{"type": "park", "spot": 2}]}
        laps = parsedToLapConfig(parsed)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "config.json")
            with open(path, "w", encoding="utf-8") as stream:
                json.dump(config, stream, ensure_ascii=False)
            updateConfigJson(laps, 1, path)
            with open(path, "r", encoding="utf-8") as stream:
                updated = json.load(stream)
        self.assertEqual(1, updated["圈数配置"]["totalLaps"])
        self.assertTrue(updated["每圈功能使能配置"]["lap1"]["park"])
        for lap_name in ("lap2", "lap3"):
            lap = updated["每圈功能使能配置"][lap_name]
            self.assertFalse(lap["manualTakeover"])
            self.assertFalse(lap["busy"])
            self.assertFalse(lap["park"])


if __name__ == "__main__":
    unittest.main()