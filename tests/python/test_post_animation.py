import unittest
from unittest.mock import patch

from post.animation import save_animation


class FakeAnimation:
    def __init__(self):
        self.calls = []

    def save(self, outfile, writer=None, fps=None):
        self.calls.append((outfile, writer, fps))


class TestSaveAnimation(unittest.TestCase):
    def test_save_mp4_uses_ffmpeg_writer(self):
        anim = FakeAnimation()
        sentinel_writer = object()
        with patch("post.animation.ani.FFMpegWriter", return_value=sentinel_writer) as writer_ctor:
            save_animation(anim, "out.mp4", fps=24)
        writer_ctor.assert_called_once_with(fps=24, bitrate=2000)
        self.assertEqual(anim.calls, [("out.mp4", sentinel_writer, None)])

    def test_save_gif_uses_pillow_writer(self):
        anim = FakeAnimation()
        save_animation(anim, "out.gif", fps=20)
        self.assertEqual(anim.calls, [("out.gif", "pillow", 20)])

    def test_unsupported_extension_raises(self):
        anim = FakeAnimation()
        with self.assertRaises(ValueError) as ctx:
            save_animation(anim, "out.avi", fps=20)
        self.assertIn("Unsupported animation format", str(ctx.exception))
        self.assertEqual(anim.calls, [])


if __name__ == "__main__":
    unittest.main()
