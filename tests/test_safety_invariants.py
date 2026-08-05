from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]


class SafetyInvariantTests(unittest.TestCase):
    def test_emergency_recovery_preserves_fsm_state(self):
        source = (ROOT / "include" / "icar.hpp").read_text(encoding="utf-8")
        self.assertNotIn("resetSpecialElementsAfterEmergency", source)
        self.assertIn(
            "params->autoRecoveryFrames <= 0 && !params->laneSafetyStop &&",
            source,
        )
        self.assertIn("params->autoRecoveryFrames = 15;", source)

    def test_park_reset_clears_reverse_control_flags(self):
        source = (ROOT / "src" / "fsm" / "park.cpp").read_text(encoding="utf-8")
        match = re.search(
            r"void FsmPark::reset\(\)\s*\{(?P<body>.*?)\n\}",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")
        self.assertIn("params->ctrl.back = false;", body)
        self.assertIn("params->ctrl.parking = false;", body)
        self.assertIn("params->ctrl.fitting = false;", body)

    def test_legacy_uart_key_does_not_stop_receiver(self):
        source = (ROOT / "include" / "com" / "uart.hpp").read_text(
            encoding="utf-8"
        )
        match = re.search(
            r"case USB_ADDR_KEY:(?P<body>.*?)break;",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(match)
        body = match.group("body")
        self.assertIn("keypress = true;", body)
        self.assertNotIn("running =", body)


if __name__ == "__main__":
    unittest.main()
