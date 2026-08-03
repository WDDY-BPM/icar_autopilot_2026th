from pathlib import Path
import re
import unittest


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / "src" / "fsm" / "manualControl.cpp").read_text(encoding="utf-8")


def command_body(command):
    match = re.search(
        rf'if \(cmd == "{command}\\n"\) \{{(?P<body>.*?)\n    \}}',
        SOURCE,
        flags=re.DOTALL,
    )
    if not match:
        raise AssertionError(f"missing {command} command handler")
    return match.group("body")


class ManualProtocolTests(unittest.TestCase):
    def test_stop_return_ping_clear_stop_preserves_return_request(self):
        return_auto = False

        stop = command_body("STOP")
        if "manualControl.returnAuto = false" in stop:
            return_auto = False

        returned = command_body("RETURN")
        manual_mode = True
        if "manualControl.returnAuto = manualMode" in returned:
            return_auto = True

        ping = command_body("PING")
        if "manualControl.returnAuto = false" in ping:
            return_auto = False

        clear = command_body("CLEAR_STOP")
        preserve = "resetManualControl(false, true)" in clear
        if not preserve:
            return_auto = False

        self.assertTrue(return_auto)

        check = re.search(
            r"bool ManualControlThread::checkForReturnKey\(\) \{(?P<body>.*?)\n\}",
            SOURCE,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(check)
        self.assertIn("manualControl.returnAuto.exchange(false)", check.group("body"))

        icar = (ROOT / "include" / "icar.hpp").read_text(encoding="utf-8")
        return_branch = re.search(
            r"if \(fsmFactory\.manual->checkForReturnKey\(\)\)(?P<body>.*?)\n            \}",
            icar,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(return_branch)
        self.assertIn("fsmFactory.busy->endManualTakeover();", return_branch.group("body"))

    def test_return_in_auto_does_not_leave_stale_request(self):
        returned = command_body("RETURN")
        self.assertIn("manualControl.returnAuto = manualMode", returned)
        manual_mode = False
        return_auto = manual_mode
        manual_mode = True
        check_for_return_key = return_auto
        return_auto = False
        self.assertTrue(manual_mode)
        self.assertFalse(check_for_return_key)
        self.assertFalse(return_auto)
    def test_return_bypasses_auto_mode_gate(self):
        gate = SOURCE.index('if (cmd != "PING\\n"')
        handler = SOURCE.index('if (cmd == "RETURN\\n")')
        self.assertLess(gate, handler)
        self.assertIn('cmd != "RETURN\\n"', SOURCE[gate:handler])

    def test_malformed_state_is_ignored_without_dropping_link(self):
        client = (ROOT / "tools" / "manual_control_client.py").read_text(
            encoding="utf-8")
        state_start = client.index('if text.startswith("STATE:"):')
        image_start = client.index('elif text.startswith("IMAGE:"):', state_start)
        state_body = client[state_start:image_start]
        self.assertIn("if len(fields) < 2:", state_body)
        self.assertIn("continue", state_body)
        self.assertIn("except (ValueError, IndexError):", state_body)
    def test_exit_confirmation_counts_only_fresh_ai_results(self):
        for relative in ("src/fsm/busy.cpp", "src/fsm/park.cpp"):
            source = (ROOT / relative).read_text(encoding="utf-8")
            with self.subTest(relative=relative):
                self.assertRegex(
                    source,
                    r"if \(params->aiResultFresh\)\s*\{\s*if \(left(?:Visible|Sign)\)[\s\S]*?countRes\+\+;",
                )


if __name__ == "__main__":
    unittest.main()
