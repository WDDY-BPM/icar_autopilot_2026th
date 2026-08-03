import sys
import types
import unittest

# Parser tests do not open images or make HTTP requests. Keep them runnable in
# lightweight environments without deployment-only dependencies.
if "PIL" not in sys.modules:
    pil = types.ModuleType("PIL")
    pil.Image = object()
    sys.modules["PIL"] = pil
if "requests" not in sys.modules:
    sys.modules["requests"] = types.ModuleType("requests")

from src.visual.visual import VisualLLM


class VisualParserTests(unittest.TestCase):
    def setUp(self):
        self.visual = VisualLLM(api_key="test-token")

    def test_parses_label_from_nonstandard_output_name(self):
        response = {
            "result": {
                "outputs": [
                    {
                        "name": "result_text",
                        "value": "识别结果为 park",
                    }
                ]
            }
        }
        self.assertEqual("park", self.visual._parse_result(response))

    def test_parses_label_from_nested_json_string(self):
        response = {
            "data": {
                "answer": '{"description": "The detected scene is busy."}'
            }
        }
        self.assertEqual("busy", self.visual._parse_result(response))

    def test_negated_unlimit_description_is_not_unlimit(self):
        self.assertNotEqual("unlimit", self.visual._match_label("不是解除限速标志"))
        self.assertNotEqual("unlimit", self.visual._match_label("not unlimit"))

    def test_negative_visual_trait_keeps_affirmed_limit(self):
        self.assertEqual(
            "limit",
            self.visual._match_label("没有斜线，这是限速标志"),
        )
    def test_parses_direct_nested_label(self):
        self.assertEqual(
            "cone",
            self.visual._parse_result({"data": {"prediction": "cone"}}),
        )


if __name__ == "__main__":
    unittest.main()
