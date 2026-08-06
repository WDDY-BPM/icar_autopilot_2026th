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
        self.assertIn("ICAR_BUILD_LOGIC_TESTS \"Build hardware-independent control tests\" OFF", source)

    def test_logic_test_cmake_isolation(self):
        cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
        self.assertIn("target_include_directories(icar_logic_core PUBLIC ${OpenCV_INCLUDE_DIRS})", cmake)
        self.assertIn("target_link_libraries(icar_logic_core PUBLIC ${OpenCV_LIBS})", cmake)
        # OpenCV stub 只能出现在 ICAR_LOGIC_TESTS_ONLY 分支内。
        stub = cmake.index("tests/stubs")
        cutoff = cmake.index("if(ICAR_LOGIC_TESTS_ONLY)\n  return()")
        self.assertLess(stub, cutoff)
        self.assertNotIn("tests/stubs", cmake[cutoff:])
        workflow = (ROOT / ".github" / "workflows" / "logic-tests.yml").read_text(
            encoding="utf-8")
        self.assertNotIn("continue-on-error", workflow)
        self.assertIn("ICAR_BUILD_LOGIC_TESTS=ON", workflow)
        self.assertIn("PIPESTATUS[0]", workflow)

    def test_planned_path_lifecycle_is_explicit(self):
        path = (ROOT / "include/runtime/path_override.hpp").read_text(encoding="utf-8")
        park = (ROOT / "src/fsm/park.cpp").read_text(encoding="utf-8")
        center = (ROOT / "src/ctrl/center.cpp").read_text(encoding="utf-8")
        self.assertIn("void setCenterLine", path)
        self.assertIn("int ttlFrames{0}", path)
        self.assertIn("uint64_t generatedFrameId{0}", path)
        self.assertIn("void tick(uint64_t currentFrameId)", path)
        self.assertNotRegex(park, r"params->ctrl\.centerEdge\s*=")
        self.assertNotIn("params->ctrl.fitting = true", park)
        set_step = park[park.index("void FsmPark::setStep"):]
        self.assertLess(set_step.index("clearPathOverride(PathSource::PARK)"),
                        set_step.index("state.stage = st"))
        self.assertIn("controlValid = candidateValid", center)
        self.assertNotIn("controlValid = !params->ctrl.centerEdge.empty()", center)

    def test_cpp_tests_do_not_use_assert(self):
        cpp_tests = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (ROOT / "tests").glob("*.cpp"))
        self.assertNotIn("assert(", cpp_tests)
        self.assertIn("logic_check_failure_test", (ROOT / "CMakeLists.txt").read_text(encoding="utf-8"))

    def test_busy_and_yfork_use_physical_time(self):
        busy = (ROOT / "include/fsm/busy_exit_state.hpp").read_text(encoding="utf-8")
        yfork = (ROOT / "include/fsm/yfork_guide_hold.hpp").read_text(encoding="utf-8")
        for duration in ("seconds(10)", "seconds(8)", "seconds(2)", "seconds(15)"):
            self.assertIn(duration, busy)
        self.assertIn("milliseconds(600)", yfork)


if __name__ == "__main__":
    unittest.main()
