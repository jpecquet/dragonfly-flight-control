import unittest

from post.blender.render_dragonfly import _resolve_material_color


class TestBlenderMaterialColors(unittest.TestCase):
    def test_light_theme_defaults_use_dark_material_tokens(self):
        body = _resolve_material_color(
            {"theme": "light", "body_color": "#111111"},
            "body_color",
            (0.0, 0.0, 0.0, 1.0),
        )
        wing = _resolve_material_color(
            {"theme": "light", "wing_color": "#d3d3d3"},
            "wing_color",
            (0.0, 0.0, 0.0, 1.0),
        )

        self.assertEqual(body, (0xF2 / 255.0, 0xF5 / 255.0, 0xF7 / 255.0, 1.0))
        self.assertEqual(wing, (0x8F / 255.0, 0x9A / 255.0, 0xA6 / 255.0, 1.0))

    def test_light_theme_custom_color_is_preserved(self):
        body = _resolve_material_color(
            {"theme": "light", "body_color": "#224466"},
            "body_color",
            (0.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(body, (0x22 / 255.0, 0x44 / 255.0, 0x66 / 255.0, 1.0))

    def test_dark_theme_uses_provided_color(self):
        wing = _resolve_material_color(
            {"theme": "dark", "wing_color": "#102030"},
            "wing_color",
            (0.0, 0.0, 0.0, 1.0),
        )
        self.assertEqual(wing, (0x10 / 255.0, 0x20 / 255.0, 0x30 / 255.0, 1.0))


if __name__ == "__main__":
    unittest.main()
