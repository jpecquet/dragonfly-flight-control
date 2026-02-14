import unittest

from post.constants import (
    FW_X0,
    HW_X0,
    LEFT_WING_Y_OFFSET,
    RIGHT_WING_Y_OFFSET,
    get_wing_info,
    get_wing_offsets,
)


class TestWingOffsets(unittest.TestCase):
    def test_get_wing_offsets_by_name(self):
        self.assertEqual(get_wing_offsets("fore_left"), (FW_X0, LEFT_WING_Y_OFFSET))
        self.assertEqual(get_wing_offsets("fore_right"), (FW_X0, RIGHT_WING_Y_OFFSET))
        self.assertEqual(get_wing_offsets("hind_left"), (HW_X0, LEFT_WING_Y_OFFSET))
        self.assertEqual(get_wing_offsets("hind_right"), (HW_X0, RIGHT_WING_Y_OFFSET))

    def test_get_wing_info_uses_offsets(self):
        wing_vectors = {"fore_left": {}, "hind_right": {}}
        wing_params = {"fore_left": 0.8, "hind_right": 0.9}
        info = get_wing_info(wing_vectors, wing_params)
        self.assertEqual(info[0], ("fore_left", FW_X0, LEFT_WING_Y_OFFSET, 0.8))
        self.assertEqual(info[1], ("hind_right", HW_X0, RIGHT_WING_Y_OFFSET, 0.9))


if __name__ == "__main__":
    unittest.main()
