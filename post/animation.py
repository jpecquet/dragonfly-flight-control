"""Shared animation save helpers."""

from pathlib import Path

import matplotlib.animation as ani


def save_animation(anim, outfile, fps, bitrate=2000):
    """Save a matplotlib animation to mp4 or gif based on file extension."""
    suffix = Path(outfile).suffix.lower()
    if suffix == ".mp4":
        writer = ani.FFMpegWriter(fps=fps, bitrate=bitrate)
        anim.save(outfile, writer=writer)
        return
    if suffix == ".gif":
        anim.save(outfile, writer="pillow", fps=fps)
        return
    raise ValueError(f"Unsupported animation format for '{outfile}'. Use .mp4 or .gif.")
