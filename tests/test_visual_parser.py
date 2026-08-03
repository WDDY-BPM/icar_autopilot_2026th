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
            "outputs": [
                {"text": '{"description": "The detected scene is busy."}'},
            ]
        }
        self.assertEqual("busy", self.visual._parse_result(response))

    def test_negated_unlimit_description_is_not_unlimit(self):
        cases = (
            "不是解除限速标志",
            "这不是一个解除限速标志",
            "没有看到解除限速标志",
            "未见到解除限速标志",
            "not unlimit",
            "not an unlimit sign",
        )
        for text in cases:
            with self.subTest(text=text):
                self.assertIsNone(self.visual._match_label(text))

    def test_negative_visual_trait_keeps_affirmed_limit(self):
        self.assertEqual(
            "limit",
            self.visual._match_label("没有斜线，这是限速标志"),
        )
    def test_ignores_echoed_prompt_outside_output_fields(self):
        response = {
            "query": "请区分限速(limit)和解除限速(unlimit)",
            "result": {
                "outputs": [
                    {"name": "result_text", "value": "limit"},
                ]
            },
        }
        self.assertEqual("limit", self.visual._parse_result(response))
    def test_parses_direct_nested_label(self):
        self.assertEqual(
            "cone",
            self.visual._parse_result({"outputs": [{"result": "cone"}]}),
        )


if __name__ == "__main__":
    unittest.main()
