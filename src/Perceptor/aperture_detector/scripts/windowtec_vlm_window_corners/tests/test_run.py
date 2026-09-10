import json
import unittest

from run import build_messages, parse_windows, response_format


class WindowCornerParsingTests(unittest.TestCase):
    def test_1000_scale_points_are_converted_to_pixels(self) -> None:
        raw = json.dumps(
            {
                "windows": [
                    {
                        "points": [[100, 200], [300, 200], [300, 400], [100, 400]],
                    }
                ]
            }
        )
        _, windows, errors, error, valid = parse_windows(raw, 640, 480)
        self.assertIsNone(error)
        self.assertEqual(errors, [])
        self.assertTrue(valid)
        self.assertEqual(windows[0]["pixel_points"], [[64.0, 96.0], [192.0, 96.0], [192.0, 192.0], [64.0, 192.0]])
        self.assertEqual(windows[0]["points_1000"], [[100, 200], [300, 200], [300, 400], [100, 400]])
        self.assertEqual(windows[0]["window_id"], 1)
        self.assertNotIn("confidence", windows[0])

    def test_coordinates_are_not_guessed_as_pixels(self) -> None:
        raw = '{"windows":[{"points":[[100,200],[300,200],[300,400],[100,400]]}]}'
        _, windows, errors, error, valid = parse_windows(raw, 640, 480)
        self.assertIsNone(error)
        self.assertEqual(errors, [])
        self.assertTrue(valid)
        self.assertEqual(windows[0]["coordinate_mode"], "normalized_1000")
        self.assertEqual(windows[0]["points"][0], [0.1, 0.2])

    def test_self_intersecting_points_are_rejected(self) -> None:
        raw = '{"windows":[{"points":[[100,100],[400,400],[400,100],[100,400]]}]}'
        _, windows, errors, error, valid = parse_windows(raw, 640, 480)
        self.assertIsNone(error)
        self.assertTrue(errors)
        self.assertFalse(valid)
        self.assertEqual(windows, [])

    def test_empty_window_list_is_valid(self) -> None:
        _, windows, errors, error, valid = parse_windows('{"windows":[]}', 640, 480)
        self.assertIsNone(error)
        self.assertEqual(errors, [])
        self.assertEqual(windows, [])
        self.assertTrue(valid)

    def test_response_schema_uses_four_points(self) -> None:
        schema = response_format(1000)
        self.assertEqual(schema["type"], "json_schema")
        self.assertTrue(schema["json_schema"]["strict"])
        item = schema["json_schema"]["schema"]["properties"]["windows"]["items"]
        self.assertEqual(set(item["properties"]), {"points"})
        self.assertEqual(item["required"], ["points"])
        self.assertEqual(item["properties"]["points"]["minItems"], 4)
        self.assertEqual(item["properties"]["points"]["maxItems"], 4)
        coordinate = item["properties"]["points"]["items"]["items"]
        self.assertEqual(coordinate["type"], "integer")
        self.assertEqual(coordinate["minimum"], 0)
        self.assertEqual(coordinate["maximum"], 1000)

    def test_out_of_range_coordinates_are_rejected(self) -> None:
        raw = '{"windows":[{"points":[[100,100],[1001,100],[400,400],[100,400]]}]}'
        _, windows, errors, error, valid = parse_windows(raw, 640, 480)
        self.assertIsNone(error)
        self.assertEqual(windows, [])
        self.assertTrue(errors)
        self.assertFalse(valid)

    def test_user_message_contains_scale_protocol_once(self) -> None:
        messages = build_messages("system", "window rules", "image.png", 640, 480, "data:image/png;base64,x")
        user_text = messages[1]["content"][0]["text"]
        self.assertIn("右下角为 [1000, 1000]", user_text)
        self.assertNotIn("0 到 1 的归一化坐标", user_text)
        self.assertIn("每个元素只能包含 points 字段", user_text)


if __name__ == "__main__":
    unittest.main()
