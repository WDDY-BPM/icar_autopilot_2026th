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
        if "manualControl.returnAuto = true" in returned:
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
