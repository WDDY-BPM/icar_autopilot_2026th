import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class ArchitectureInvariantTests(unittest.TestCase):
    def test_vehicle_command_has_one_runtime_outlet(self):
        runtime = {
            path.relative_to(ROOT).as_posix(): path.read_text(encoding="utf-8")
            for path in (ROOT / "src" / "runtime").glob("*.cpp")
        }
        calls = [name for name, source in runtime.items()
                 for _ in re.finditer(r"client->carControl\(", source)]
        self.assertEqual(calls, ["src/runtime/vehicle_output.cpp"])

    def test_fsms_do_not_mutate_perception_edges(self):
        forbidden = re.compile(
            r"params->track->pointsEdge(?:Left|Right)(?:\s*=|"
            r"\.(?:clear|resize|push_back|pop_back|emplace_back|insert|erase)\(|"
            r"\[[^]]+\]\.[xy]\s*=(?!=))")
        for path in (ROOT / "src" / "fsm").glob("*.cpp"):
            with self.subTest(path=path.name):
                self.assertIsNone(forbidden.search(path.read_text(encoding="utf-8")))

    def test_mouse_callback_passes_instance_userdata(self):
        source = (ROOT / "src/runtime/icar.cpp").read_text(encoding="utf-8")
        self.assertIn('cv::setMouseCallback("ICAR", &Icar::callbackMouse, this);', source)

    def test_cmake_uses_target_level_flags(self):
        source = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertNotIn("CMAKE_CXX_FLAGS", source)
        self.assertNotIn("GLOB_RECURSE", source)
        self.assertIn("target_compile_options", source)
        self.assertIn("ICAR_BUILD_LOGIC_TESTS \"Build hardware-independent control tests\" ON", source)


if __name__ == "__main__":
    unittest.main()
