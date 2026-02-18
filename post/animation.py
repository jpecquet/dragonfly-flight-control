"""Shared animation save helpers."""

from pathlib import Path

import matplotlib.animation as ani


def save_animation(anim, outfile, fps, bitrate=2000, dpi=None):
    """Save a matplotlib animation to mp4 or gif based on file extension."""
    suffix = Path(outfile).suffix.lower()
    if suffix == ".mp4":
        writer = ani.FFMpegWriter(fps=fps, bitrate=bitrate)
        kwargs = {"writer": writer}
        if dpi is not None:
            kwargs["dpi"] = dpi
        anim.save(outfile, **kwargs)
        return
    if suffix == ".gif":
        kwargs = {"writer": "pillow", "fps": fps}
        if dpi is not None:
            kwargs["dpi"] = dpi
        anim.save(outfile, **kwargs)
        return
    raise ValueError(f"Unsupported animation format for '{outfile}'. Use .mp4 or .gif.")
