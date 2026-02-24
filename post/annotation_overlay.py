"""
Additional Matplotlib annotation overlays for hybrid-rendered animations.

These overlays are rendered as transparent PNG sequences and composited on top of
the Blender + primary Matplotlib overlay output.
"""

from __future__ import annotations

import os
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d

from .constants import DEFAULT_LB0, get_wing_offsets
from .hybrid_config import HybridConfig
from .kinematics import eval_series
from .mpl_overlay import create_overlay_figure, setup_axes
from .progress import pip_progress
from .style import apply_matplotlib_style

MAX_PARALLEL_WORKERS = 4
_ANN_STATES = None
_ANN_CONFIG_DICT = None
_ANN_OUTPUT_PATH = None
_ANN_KIND = None
_ANN_PAYLOAD = None


def _normalize_name(name: str) -> str:
    return str(name).strip().lower().replace("-", "_")


def _resolve_right_wings(wing_names: List[str]) -> tuple[Optional[str], Optional[str]]:
    fore = None
    hind = None
    names = list(wing_names)
    for name in names:
        n = _normalize_name(name)
        if "fore" in n and "right" in n and fore is None:
            fore = name
        if "hind" in n and "right" in n and hind is None:
            hind = name
    if fore is not None and hind is not None:
        return fore, hind
    for name in names:
        n = _normalize_name(name)
        if "fore" in n and fore is None:
            fore = name
        if "hind" in n and hind is None:
            hind = name
    return fore, hind


def _resolve_left_wings(wing_names: List[str]) -> tuple[Optional[str], Optional[str]]:
    fore = None
    hind = None
    names = list(wing_names)
    for name in names:
        n = _normalize_name(name)
        if "fore" in n and "left" in n and fore is None:
            fore = name
        if "hind" in n and "left" in n and hind is None:
            hind = name
    if fore is not None and hind is not None:
        return fore, hind
    for name in names:
        n = _normalize_name(name)
        if "fore" in n and fore is None:
            fore = name
        if "hind" in n and hind is None:
            hind = name
    return fore, hind


def _resolve_left_fore_wing(wing_names: List[str]) -> Optional[str]:
    names = list(wing_names)
    for name in names:
        n = _normalize_name(name)
        if "fore" in n and "left" in n:
            return name
    for name in names:
        n = _normalize_name(name)
        if "fore" in n:
            return name
    return None


def _phi_series(time_values: np.ndarray, params: Dict, wing_name: str) -> Optional[np.ndarray]:
    return _angle_series(time_values, params, wing_name, "phi")


def _angle_series(time_values: np.ndarray, params: Dict, wing_name: str, angle: str) -> Optional[np.ndarray]:
    try:
        omega = float(params["wing_omega"][wing_name])
        period = float(params.get("wing_harmonic_period_wingbeats", {}).get(wing_name, 1.0))
        phase_offset = float(params.get("wing_phase_offset", {}).get(wing_name, 0.0))
        angle_mean = float(params[f"wing_{angle}_mean"][wing_name])
        angle_amp = np.asarray(params.get(f"wing_{angle}_amp", {}).get(wing_name, []), dtype=float)
        angle_phase = np.asarray(params.get(f"wing_{angle}_phase", {}).get(wing_name, []), dtype=float)
    except Exception:
        return None

    if period == 0.0:
        return None
    basis_omega = omega / period
    phase = basis_omega * np.asarray(time_values, dtype=float) + phase_offset
    return angle_mean + eval_series(angle_amp, angle_phase, phase)


def _angle_eval_at_time(t: float, params: Dict, wing_name: str, angle: str) -> Optional[float]:
    arr = _angle_series(np.asarray([t], dtype=float), params, wing_name, angle)
    if arr is None or len(arr) == 0:
        return None
    return float(arr[0])


def _phi_derivative(time_values: np.ndarray, params: Dict, wing_name: str) -> Optional[np.ndarray]:
    try:
        omega = float(params["wing_omega"][wing_name])
        period = float(params.get("wing_harmonic_period_wingbeats", {}).get(wing_name, 1.0))
        phase_offset = float(params.get("wing_phase_offset", {}).get(wing_name, 0.0))
        phi_amp = np.asarray(params.get("wing_phi_amp", {}).get(wing_name, []), dtype=float)
        phi_phase = np.asarray(params.get("wing_phi_phase", {}).get(wing_name, []), dtype=float)
    except Exception:
        return None
    if period == 0.0:
        return None
    basis_omega = omega / period
    phase = basis_omega * np.asarray(time_values, dtype=float) + phase_offset
    deriv = np.zeros_like(phase, dtype=float)
    for k, (amp, phase_k) in enumerate(zip(phi_amp, phi_phase), 1):
        deriv += -k * basis_omega * amp * np.sin((k * phase) + phase_k)
    return deriv


def _refine_root_bisection(fun, a: float, b: float, *, max_iter: int = 60, tol: float = 1e-12) -> float:
    fa = float(fun(a))
    fb = float(fun(b))
    if fa == 0.0:
        return a
    if fb == 0.0:
        return b
    if fa * fb > 0.0:
        return 0.5 * (a + b)
    lo, hi = a, b
    flo, fhi = fa, fb
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        fm = float(fun(mid))
        if abs(fm) < tol or abs(hi - lo) < tol:
            return mid
        if flo * fm <= 0.0:
            hi, fhi = mid, fm
        else:
            lo, flo = mid, fm
    return 0.5 * (lo + hi)


def _phi_extrema_times(time_values: np.ndarray, params: Dict, wing_name: str) -> Optional[tuple[float, float]]:
    t = np.asarray(time_values, dtype=float).reshape(-1)
    if t.size == 0:
        return None
    if t.size == 1:
        return float(t[0]), float(t[0])

    t0 = float(t[0])
    t1 = float(t[-1])
    # Dense continuous scan + bisection on dphi/dt = 0.
    td = np.linspace(t0, t1, 2049, dtype=float)
    dphi = _phi_derivative(td, params, wing_name)
    phi = _phi_series(td, params, wing_name)
    if dphi is None or phi is None:
        return None

    candidate_times: List[float] = [t0, t1]
    eps = 1e-14
    for i in range(len(td) - 1):
        a, b = float(td[i]), float(td[i + 1])
        fa, fb = float(dphi[i]), float(dphi[i + 1])
        if abs(fa) < eps:
            candidate_times.append(a)
        if fa == 0.0 and fb == 0.0:
            continue
        if fa * fb < 0.0:
            root = _refine_root_bisection(
                lambda x: float(_phi_derivative(np.asarray([x]), params, wing_name)[0]),
                a,
                b,
            )
            candidate_times.append(float(root))

    # Deduplicate roots from neighboring brackets.
    candidate_times = sorted(candidate_times)
    deduped: List[float] = []
    for tt in candidate_times:
        if not deduped or abs(tt - deduped[-1]) > 1e-9 * max(1.0, abs(tt)):
            deduped.append(tt)

    phi_vals = []
    for tt in deduped:
        val = _angle_eval_at_time(tt, params, wing_name, "phi")
        if val is None:
            return None
        phi_vals.append(float(val))
    if not phi_vals:
        return None
    i_min = int(np.argmin(phi_vals))
    i_max = int(np.argmax(phi_vals))
    return deduped[i_min], deduped[i_max]


def _rot_x(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=float)


def _rot_y(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=float)


def _rot_z(angle: float) -> np.ndarray:
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def _wing_e_r_from_angles(gam_raw: float, phi: float, cone: float, is_left: bool) -> np.ndarray:
    # Simulator convention: interpreted stroke-plane angle is pi - gamma_raw.
    gam = np.pi - float(gam_raw)
    ex = np.array([1.0, 0.0, 0.0], dtype=float)
    if is_left:
        rs = _rot_y(-gam) @ _rot_z(-float(phi)) @ _rot_x(-float(cone))
    else:
        rs = _rot_x(-np.pi) @ _rot_y(gam) @ _rot_z(-float(phi)) @ _rot_x(float(cone))
    return rs @ ex


def _stroke_tip_segment(
    time_values: np.ndarray,
    params: Dict,
    wing_name: str,
) -> Optional[tuple[np.ndarray, np.ndarray]]:
    try:
        gamma = float(params.get("wing_gamma_mean", {}).get(wing_name))
        cone = float(params.get("wing_cone_angle", {}).get(wing_name, 0.0))
        lb0 = float(params.get("wing_lb0", {}).get(wing_name, DEFAULT_LB0))
    except Exception:
        return None

    xoff, yoff, zoff = get_wing_offsets(wing_name)
    root_offset = np.array([xoff, yoff, zoff], dtype=float)

    # User-specified geometric construction in side-view XZ:
    # 1) Base stroke-plane line through root with direction d = (-cos(gamma), sin(gamma))
    # 2) Offset line by lambda0*sin(beta) along the in-plane normal n = (sin(gamma), cos(gamma))
    d = np.array([-np.cos(gamma), 0.0, np.sin(gamma)], dtype=float)
    n = np.array([np.sin(gamma), 0.0, np.cos(gamma)], dtype=float)
    offset = lb0 * np.sin(cone) * n
    span = lb0 * d

    rel_p0 = root_offset + offset - span
    rel_p1 = root_offset + offset + span
    return rel_p0, rel_p1


def _cone_edge_segment(params: Dict, wing_name: str) -> Optional[tuple[np.ndarray, np.ndarray]]:
    try:
        gamma = float(params.get("wing_gamma_mean", {}).get(wing_name))
        cone = float(params.get("wing_cone_angle", {}).get(wing_name, 0.0))
        lb0 = float(params.get("wing_lb0", {}).get(wing_name, DEFAULT_LB0))
    except Exception:
        return None

    xoff, yoff, zoff = get_wing_offsets(wing_name)
    root_offset = np.array([xoff, yoff, zoff], dtype=float)
    d = np.array([-np.cos(gamma), 0.0, np.sin(gamma)], dtype=float)
    d_norm = float(np.linalg.norm(d))
    if d_norm <= 1e-12:
        return None
    d = d / d_norm
    if d[2] < 0.0:
        d = -d
    n = np.array([np.sin(gamma), 0.0, np.cos(gamma)], dtype=float)
    tip = root_offset + (lb0 * np.sin(cone)) * n + (lb0 * np.cos(cone)) * d
    return root_offset, tip


def _gamma_deg_text(params: Dict, wing_name: str) -> Optional[float]:
    gamma_map = params.get("wing_gamma_mean", {})
    if not isinstance(gamma_map, dict) or wing_name not in gamma_map:
        return None
    try:
        return float(np.degrees(float(gamma_map[wing_name])))
    except Exception:
        return None


def _wing_gamma_subscript(wing_name: str) -> str:
    n = _normalize_name(wing_name)
    if "fore" in n:
        return "f"
    if "hind" in n:
        return "h"
    return ""


def _split_frame_ranges(n_frames: int, n_workers: int) -> List[tuple[int, int]]:
    chunk = max(1, n_frames // max(1, n_workers * 4))
    ranges = []
    start = 0
    while start < n_frames:
        end = min(start + chunk, n_frames)
        ranges.append((start, end))
        start = end
    return ranges


def _canonical_dash_segments(
    n_dashes: int = 8,
    duty_cycle: float = 0.55,
    margin: float = 0.02,
) -> np.ndarray:
    """
    Precompute dash segments on the unit interval [0, 1].

    These normalized segments can be mapped onto any line segment via
    `p = p0 + s * (p1 - p0)` to keep a fixed dash count over time.
    """
    n = max(1, int(n_dashes))
    duty = float(np.clip(duty_cycle, 0.05, 0.95))
    m = float(np.clip(margin, 0.0, 0.2))
    usable = max(1e-6, 1.0 - 2.0 * m)
    step = usable / n
    dash = duty * step
    start_offset = 0.5 * (step - dash)
    segs = []
    for i in range(n):
        s0 = m + (i * step) + start_offset
        s1 = s0 + dash
        segs.append((s0, s1))
    return np.asarray(segs, dtype=float)


def _plot_transformed_dashes(
    ax,
    p0: np.ndarray,
    p1: np.ndarray,
    dash_segments_s: np.ndarray,
    *,
    color: str,
    alpha: float,
    line_width: float,
) -> None:
    """Draw precomputed normalized dash segments transformed onto line p0->p1."""
    a = np.asarray(p0, dtype=float)
    b = np.asarray(p1, dtype=float)
    v = b - a
    if float(np.linalg.norm(v)) <= 1e-12:
        return
    segs = np.asarray(dash_segments_s, dtype=float)
    if segs.size == 0:
        return
    for s0, s1 in segs:
        q0 = a + float(s0) * v
        q1 = a + float(s1) * v
        ax.plot(
            [q0[0], q1[0]],
            [q0[1], q1[1]],
            [q0[2], q1[2]],
            linestyle="-",
            linewidth=line_width,
            color=color,
            alpha=alpha,
        )


def _build_stroke_plane_angles_payload(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
) -> Dict:
    wing_names = list(wing_vectors.keys())
    selected = spec.get("wings")
    if isinstance(selected, list) and selected:
        draw_wings = [str(w) for w in selected if str(w) in wing_vectors]
    else:
        fore, hind = _resolve_right_wings(wing_names)
        draw_wings = [w for w in (fore, hind) if w is not None]
    if not draw_wings:
        raise ValueError("annotation_overlay 'stroke_plane_angles' could not resolve fore/hind wings")

    if time_values is None:
        time_values = np.arange(len(states), dtype=float)
    else:
        time_values = np.asarray(time_values, dtype=float).reshape(-1)

    tip_segments = {
        wname: _stroke_tip_segment(time_values, params, wname)
        for wname in draw_wings
    }
    tip_segments = {k: v for k, v in tip_segments.items() if v is not None}
    if not tip_segments:
        raise ValueError("annotation_overlay 'stroke_plane_angles' could not compute tip stroke-plane segments")

    gamma_deg = {wname: _gamma_deg_text(params, wname) for wname in draw_wings}
    style = config.style
    theme_name = str(getattr(style, "theme", "")).strip().lower()
    mono_color = str(spec.get("line_color", "#ffffff" if theme_name == "dark" else "#000000"))
    stroke_line_color = str(spec.get("stroke_line_color", mono_color))

    return {
        "draw_wings": draw_wings,
        "tip_segments": {
            k: (np.asarray(v[0], dtype=float), np.asarray(v[1], dtype=float))
            for k, v in tip_segments.items()
        },
        "gamma_deg": gamma_deg,
        "line_width": float(spec.get("line_width", 1.0)),
        "line_alpha": float(spec.get("line_alpha", 0.9)),
        "text_fontsize": int(spec.get("font_size", max(10, int(style.font_size)))),
        "mono_color": mono_color,
        "stroke_line_color": stroke_line_color,
    }


def _build_stroke_plane_cone_angles_payload(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
) -> Dict:
    wing_names = list(wing_vectors.keys())
    selected = spec.get("wings")
    if isinstance(selected, list) and selected:
        draw_wings = [str(w) for w in selected if str(w) in wing_vectors]
    else:
        fore, hind = _resolve_left_wings(wing_names)
        draw_wings = [w for w in (fore, hind) if w is not None]
    if not draw_wings:
        raise ValueError("annotation_overlay 'stroke_plane_cone_angles' could not resolve fore/hind wings")

    if time_values is None:
        time_values = np.arange(len(states), dtype=float)
    else:
        time_values = np.asarray(time_values, dtype=float).reshape(-1)

    tip_segments = {wname: _stroke_tip_segment(time_values, params, wname) for wname in draw_wings}
    tip_segments = {k: v for k, v in tip_segments.items() if v is not None}
    cone_edges = {wname: _cone_edge_segment(params, wname) for wname in draw_wings}
    cone_edges = {k: v for k, v in cone_edges.items() if v is not None}
    wing_tip_offsets = {}
    for wname in draw_wings:
        try:
            lb0 = float(params.get("wing_lb0", {}).get(wname, DEFAULT_LB0))
            xoff, yoff, zoff = get_wing_offsets(wname)
            root_offset = np.array([xoff, yoff, zoff], dtype=float)
            e_r = np.asarray(wing_vectors[wname]["e_r"], dtype=float)
            if e_r.ndim != 2 or e_r.shape[1] != 3:
                continue
            wing_tip_offsets[wname] = root_offset[None, :] + lb0 * e_r
        except Exception:
            continue
    if not tip_segments or not cone_edges:
        raise ValueError("annotation_overlay 'stroke_plane_cone_angles' could not compute geometry")

    style = config.style
    theme_name = str(getattr(style, "theme", "")).strip().lower()
    mono_color = str(spec.get("line_color", "#ffffff" if theme_name == "dark" else "#000000"))

    stroke_line_color = str(spec.get("stroke_line_color", mono_color))

    return {
        "draw_wings": draw_wings,
        "tip_segments": {
            k: (np.asarray(v[0], dtype=float), np.asarray(v[1], dtype=float))
            for k, v in tip_segments.items()
        },
        "cone_edges": {
            k: (np.asarray(v[0], dtype=float), np.asarray(v[1], dtype=float))
            for k, v in cone_edges.items()
        },
        "wing_tip_offsets": {
            k: np.asarray(v, dtype=float) for k, v in wing_tip_offsets.items()
        },
        "line_width": float(spec.get("line_width", 1.0)),
        "line_alpha": float(spec.get("line_alpha", 0.9)),
        "text_fontsize": int(spec.get("font_size", max(10, int(style.font_size)))),
        "mono_color": mono_color,
        "stroke_line_color": stroke_line_color,
        "beta_arrow_radius": float(spec.get("beta_arrow_radius", 0.6)),
        "wing_dash_segments_s": _canonical_dash_segments(
            n_dashes=int(spec.get("wing_dash_count", 8)),
            duty_cycle=float(spec.get("wing_dash_duty", 0.55)),
            margin=float(spec.get("wing_dash_margin", 0.02)),
        ),
    }


def _render_stroke_plane_angles_range(
    start_frame: int,
    end_frame: int,
    states: np.ndarray,
    config: HybridConfig,
    output_path: Path,
    payload: Dict,
) -> List[str]:
    style = config.style
    apply_matplotlib_style(style)
    fig, ax = create_overlay_figure(config.camera, style)
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.0)

    draw_wings = payload["draw_wings"]
    tip_segments = payload["tip_segments"]
    gamma_deg = payload["gamma_deg"]
    line_width = float(payload["line_width"])
    line_alpha = float(payload["line_alpha"])
    text_fontsize = int(payload["text_fontsize"])
    mono_color = str(payload["mono_color"])
    stroke_line_color = str(payload.get("stroke_line_color", mono_color))

    files: List[str] = []
    try:
        for frame_idx in range(start_frame, end_frame):
            ax.cla()
            setup_axes(ax, config.viewport, config.camera, style, show_axes=False)
            ax.patch.set_alpha(0.0)

            xb = np.asarray(states[frame_idx][0:3], dtype=float)
            for wname in draw_wings:
                seg = tip_segments.get(wname)
                if seg is None:
                    continue
                rel_p0, rel_p1 = seg
                p0 = xb + rel_p0
                p1 = xb + rel_p1
                ax.plot(
                    [p0[0], p1[0]],
                    [p0[1], p1[1]],
                    [p0[2], p1[2]],
                    linestyle="-",
                    linewidth=line_width,
                    color=stroke_line_color,
                    alpha=line_alpha,
                )
                top_point = p1 if p1[2] >= p0[2] else p0
                stroke_dir = (p1 - p0) if p1[2] >= p0[2] else (p0 - p1)
                _draw_stroke_plane_angle_glyph(
                    ax,
                    top_point,
                    stroke_dir,
                    gamma_deg.get(wname),
                    wing_name=wname,
                    color=mono_color,
                    alpha=line_alpha,
                    line_width=line_width,
                    text_fontsize=text_fontsize,
                )

            output_file = output_path / f"ann_{frame_idx:06d}.png"
            fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0, transparent=True)
            files.append(str(output_file))
    finally:
        plt.close(fig)
    return files


def _intersect_circle_with_line_xz(
    center: np.ndarray,
    radius: float,
    line_p0: np.ndarray,
    line_p1: np.ndarray,
) -> Optional[np.ndarray]:
    c = np.asarray(center, dtype=float)
    a = np.asarray(line_p0, dtype=float)
    b = np.asarray(line_p1, dtype=float)
    cx, cz = float(c[0]), float(c[2])
    ax, az = float(a[0]), float(a[2])
    bx, bz = float(b[0]), float(b[2])
    dx, dz = bx - ax, bz - az
    A = dx * dx + dz * dz
    if A <= 1e-16:
        return None
    rx, rz = ax - cx, az - cz
    B = 2.0 * (rx * dx + rz * dz)
    C = (rx * rx + rz * rz) - float(radius) * float(radius)
    disc = B * B - 4.0 * A * C
    if disc < 0.0:
        return None
    sqrt_disc = float(np.sqrt(max(0.0, disc)))
    ts = [(-B - sqrt_disc) / (2.0 * A), (-B + sqrt_disc) / (2.0 * A)]
    in_seg = [t for t in ts if -1e-6 <= t <= 1.0 + 1e-6]
    if in_seg:
        t_sel = max(in_seg)
    else:
        t_sel = max(ts)
    point = a + t_sel * (b - a)
    return point


def _draw_coning_angle_glyph(
    ax,
    *,
    top_point: np.ndarray,
    stroke_dir: np.ndarray,
    cone_edge_apex: np.ndarray,
    cone_edge_tip: np.ndarray,
    wing_name: Optional[str],
    color: str,
    alpha: float,
    line_width: float,
    text_fontsize: int,
    radius: float = 0.4,
) -> None:
    c = np.asarray(top_point, dtype=float)
    d = np.asarray(stroke_dir, dtype=float)
    d[1] = 0.0
    d_norm = float(np.linalg.norm(d))
    if d_norm <= 1e-12:
        return
    d = d / d_norm
    if d[2] < 0.0:
        d = -d
    start_dir = -d

    r = float(radius)
    if r <= 1e-8:
        return
    start = c + r * start_dir
    p_edge = _intersect_circle_with_line_xz(c, r, cone_edge_apex, cone_edge_tip)
    if p_edge is None:
        p_edge = np.asarray(cone_edge_tip, dtype=float)
        v = p_edge - c
        v[1] = 0.0
        v_norm = float(np.linalg.norm(v))
        if v_norm <= 1e-12:
            # Still place label near the base if geometry degenerates.
            label_pos = start + 0.1 * d
            sub = _wing_gamma_subscript(wing_name or "")
            beta_sym = rf"\beta_{{{sub}}}" if sub else r"\beta"
            ax.text(float(label_pos[0]), float(label_pos[1]), float(label_pos[2]),
                    rf"${beta_sym}$", color=color, fontsize=text_fontsize, ha="center", va="center")
            return
        p_edge = c + r * (v / v_norm)

    u0 = start_dir
    u1 = np.asarray(p_edge - c, dtype=float)
    u1[1] = 0.0
    u1_norm = float(np.linalg.norm(u1))
    if u1_norm <= 1e-12:
        return
    u1 = u1 / u1_norm
    e1 = np.array([-u0[2], 0.0, u0[0]], dtype=float)
    theta_full = float(np.arctan2(np.dot(u1, e1), np.dot(u1, u0)))

    # Label anchored near the arrow base on the stroke-plane side.
    sub = _wing_gamma_subscript(wing_name or "")
    beta_sym = rf"\beta_{{{sub}}}" if sub else r"\beta"
    sign = 1.0 if theta_full >= 0.0 else -1.0
    label_pos = start - sign * (0.1 * e1)

    if abs(theta_full) <= 1e-6:
        ax.text(float(label_pos[0]), float(label_pos[1]), float(label_pos[2]),
                rf"${beta_sym}$", color=color, fontsize=text_fontsize, ha="center", va="center")
        return

    max_head_len = 0.056
    theta_head = min(0.35, 0.45 * abs(theta_full), max_head_len / max(r, 1e-12))
    theta_arc_end = theta_full - sign * theta_head
    if sign * theta_arc_end <= 0.0:
        theta_arc_end = 0.6 * theta_full

    theta = np.linspace(0.0, theta_arc_end, 72, dtype=float)
    arc = c[None, :] + r * (np.cos(theta)[:, None] * u0[None, :] + np.sin(theta)[:, None] * e1[None, :])
    ax.plot(
        arc[:, 0], arc[:, 1], arc[:, 2],
        linestyle="-", linewidth=line_width, color=color, alpha=alpha, solid_capstyle="butt",
    )

    head_tip = c + r * (np.cos(theta_full) * u0 + np.sin(theta_full) * e1)
    head_base = arc[-1]
    head_vec = np.asarray(head_tip - head_base, dtype=float)
    head_gap = float(np.linalg.norm(head_vec))
    if head_gap > 1e-12:
        t_hat = head_vec / head_gap
        head_len = max_head_len
        n_hat = np.array([-t_hat[2], 0.0, t_hat[0]], dtype=float)
        n_hat_norm = float(np.linalg.norm(n_hat))
        if n_hat_norm > 1e-12:
            n_hat = n_hat / n_hat_norm
            head_base_draw = head_tip - head_len * t_hat
            head_w = 0.25 * head_len
            head_l = head_base_draw + head_w * n_hat
            head_r = head_base_draw - head_w * n_hat
            head_tri = art3d.Poly3DCollection(
                [[
                    (float(head_tip[0]), float(head_tip[1]), float(head_tip[2])),
                    (float(head_l[0]), float(head_l[1]), float(head_l[2])),
                    (float(head_r[0]), float(head_r[1]), float(head_r[2])),
                ]],
                facecolors=[color],
                edgecolors=[color],
                linewidths=[0.0],
                alpha=alpha,
            )
            ax.add_collection3d(head_tri)

    ax.text(float(label_pos[0]), float(label_pos[1]), float(label_pos[2]),
            rf"${beta_sym}$", color=color, fontsize=text_fontsize, ha="center", va="center")


def _render_stroke_plane_cone_angles_range(
    start_frame: int,
    end_frame: int,
    states: np.ndarray,
    config: HybridConfig,
    output_path: Path,
    payload: Dict,
) -> List[str]:
    style = config.style
    apply_matplotlib_style(style)
    fig, ax = create_overlay_figure(config.camera, style)
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.0)

    draw_wings = payload["draw_wings"]
    tip_segments = payload["tip_segments"]
    cone_edges = payload["cone_edges"]
    wing_tip_offsets = payload.get("wing_tip_offsets", {})
    line_width = float(payload["line_width"])
    line_alpha = float(payload["line_alpha"])
    text_fontsize = int(payload["text_fontsize"])
    mono_color = str(payload["mono_color"])
    stroke_line_color = str(payload.get("stroke_line_color", mono_color))
    beta_arrow_radius = float(payload["beta_arrow_radius"])
    wing_dash_segments_s = np.asarray(payload.get("wing_dash_segments_s", []), dtype=float)

    files: List[str] = []
    try:
        for frame_idx in range(start_frame, end_frame):
            ax.cla()
            setup_axes(ax, config.viewport, config.camera, style, show_axes=False)
            ax.patch.set_alpha(0.0)

            xb = np.asarray(states[frame_idx][0:3], dtype=float)
            for wname in draw_wings:
                seg = tip_segments.get(wname)
                cone_edge = cone_edges.get(wname)
                if seg is None or cone_edge is None:
                    continue
                rel_p0, rel_p1 = seg
                rel_apex, rel_cone_tip = cone_edge
                p0 = xb + rel_p0
                p1 = xb + rel_p1
                p_root = xb + rel_apex
                ax.plot(
                    [p0[0], p1[0]],
                    [p0[1], p1[1]],
                    [p0[2], p1[2]],
                    linestyle="-",
                    linewidth=line_width,
                    color=stroke_line_color,
                    alpha=line_alpha,
                )
                tip_series = wing_tip_offsets.get(wname)
                if tip_series is not None and frame_idx < len(tip_series):
                    p_tip = xb + np.asarray(tip_series[frame_idx], dtype=float)
                    _plot_transformed_dashes(
                        ax,
                        p_root,
                        p_tip,
                        wing_dash_segments_s,
                        color=mono_color,
                        alpha=line_alpha,
                        line_width=line_width,
                    )
                top_point = p1 if p1[2] >= p0[2] else p0
                stroke_dir = (p1 - p0) if p1[2] >= p0[2] else (p0 - p1)
                _draw_coning_angle_glyph(
                    ax,
                    top_point=top_point,
                    stroke_dir=stroke_dir,
                    cone_edge_apex=xb + rel_apex,
                    cone_edge_tip=xb + rel_cone_tip,
                    wing_name=wname,
                    color=mono_color,
                    alpha=line_alpha,
                    line_width=line_width,
                    text_fontsize=text_fontsize,
                    radius=beta_arrow_radius,
                )

            output_file = output_path / f"ann_{frame_idx:06d}.png"
            fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0, transparent=True)
            files.append(str(output_file))
    finally:
        plt.close(fig)
    return files


def _build_fore_left_y_reference_payload(
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    spec: Dict,
) -> Dict:
    wing_names = list(wing_vectors.keys())
    wing_name = str(spec.get("wing", "")).strip() or _resolve_left_fore_wing(wing_names)
    if not wing_name or wing_name not in wing_vectors:
        raise ValueError("annotation_overlay 'fore_left_y_reference' could not resolve left forewing")

    try:
        lb0 = float(params.get("wing_lb0", {}).get(wing_name, DEFAULT_LB0))
        gamma = float(params.get("wing_gamma_mean", {}).get(wing_name))
        cone = float(params.get("wing_cone_angle", {}).get(wing_name, 0.0))
    except Exception as exc:
        raise ValueError("annotation_overlay 'fore_left_y_reference' missing wing geometry params") from exc

    xoff, yoff, zoff = get_wing_offsets(wing_name)
    root_offset = np.array([xoff, yoff, zoff], dtype=float)
    endpoint_offset = root_offset + np.array([0.0, lb0, 0.0], dtype=float)
    e_r = np.asarray(wing_vectors[wing_name]["e_r"], dtype=float)
    if e_r.ndim != 2 or e_r.shape[1] != 3:
        raise ValueError("annotation_overlay 'fore_left_y_reference' expected wing e_r with shape (N,3)")
    tip_offsets = root_offset[None, :] + lb0 * e_r

    axis_normal = np.array([np.sin(gamma), 0.0, np.cos(gamma)], dtype=float)
    axis_norm = float(np.linalg.norm(axis_normal))
    if axis_norm <= 1e-12:
        axis_normal = np.array([0.0, 0.0, 1.0], dtype=float)
    else:
        axis_normal = axis_normal / axis_norm
    cone_center_offset = root_offset + (lb0 * np.sin(cone)) * axis_normal
    cone_radius = abs(lb0 * np.cos(cone))

    style = config.style
    theme_name = str(getattr(style, "theme", "")).strip().lower()
    mono_color = str(spec.get("line_color", "#ffffff" if theme_name == "dark" else "#000000"))

    return {
        "wing_name": wing_name,
        "root_offset": root_offset,
        "endpoint_offset": endpoint_offset,
        "tip_offsets": tip_offsets,
        "cone_center_offset": cone_center_offset,
        "cone_axis_normal": axis_normal,
        "cone_radius": float(cone_radius),
        "line_width": float(spec.get("line_width", 1.0)),
        "line_alpha": float(spec.get("line_alpha", 0.9)),
        "mono_color": mono_color,
    }


def _render_fore_left_y_reference_range(
    start_frame: int,
    end_frame: int,
    states: np.ndarray,
    config: HybridConfig,
    output_path: Path,
    payload: Dict,
) -> List[str]:
    style = config.style
    apply_matplotlib_style(style)
    fig, ax = create_overlay_figure(config.camera, style)
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.0)

    root_offset = np.asarray(payload["root_offset"], dtype=float)
    endpoint_offset = np.asarray(payload["endpoint_offset"], dtype=float)
    tip_offsets = np.asarray(payload["tip_offsets"], dtype=float)
    cone_center_offset = np.asarray(payload["cone_center_offset"], dtype=float)
    cone_axis_normal = np.asarray(payload["cone_axis_normal"], dtype=float)
    cone_radius = float(payload["cone_radius"])
    line_width = float(payload["line_width"])
    line_alpha = float(payload["line_alpha"])
    mono_color = str(payload["mono_color"])

    files: List[str] = []
    try:
        for frame_idx in range(start_frame, end_frame):
            ax.cla()
            setup_axes(ax, config.viewport, config.camera, style, show_axes=False)
            ax.patch.set_alpha(0.0)

            xb = np.asarray(states[frame_idx][0:3], dtype=float)
            p0 = xb + root_offset
            p_ref = xb + endpoint_offset
            p_tip = xb + tip_offsets[frame_idx]
            ax.plot(
                [p0[0], p_ref[0]],
                [p0[1], p_ref[1]],
                [p0[2], p_ref[2]],
                linestyle="--",
                linewidth=line_width,
                color=mono_color,
                alpha=line_alpha,
            )
            ax.plot(
                [p0[0], p_tip[0]],
                [p0[1], p_tip[1]],
                [p0[2], p_tip[2]],
                linestyle="--",
                linewidth=line_width,
                color=mono_color,
                alpha=line_alpha,
            )

            _draw_phi_cone_edge_glyph(
                ax,
                root_point=p0,
                ref_line_end=p_ref,
                tip_point=p_tip,
                cone_center=xb + cone_center_offset,
                cone_axis_normal=cone_axis_normal,
                cone_radius=cone_radius,
                color=mono_color,
                alpha=line_alpha,
                line_width=line_width,
                text_fontsize=max(10, int(style.font_size)),
            )

            output_file = output_path / f"ann_{frame_idx:06d}.png"
            fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0, transparent=True)
            files.append(str(output_file))
    finally:
        plt.close(fig)
    return files


def _draw_phi_cone_edge_glyph(
    ax,
    *,
    root_point: np.ndarray,
    ref_line_end: np.ndarray,
    tip_point: np.ndarray,
    cone_center: np.ndarray,
    cone_axis_normal: np.ndarray,
    cone_radius: float,
    color: str,
    alpha: float,
    line_width: float,
    text_fontsize: int,
) -> None:
    if cone_radius <= 1e-8:
        return

    c = np.asarray(cone_center, dtype=float)
    n = np.asarray(cone_axis_normal, dtype=float)
    n_norm = float(np.linalg.norm(n))
    if n_norm <= 1e-12:
        return
    n = n / n_norm

    ey = np.array([0.0, 1.0, 0.0], dtype=float)
    v_ref = ey - np.dot(ey, n) * n
    v_ref_norm = float(np.linalg.norm(v_ref))
    if v_ref_norm <= 1e-12:
        return
    v_ref = v_ref / v_ref_norm

    v_tip = np.asarray(tip_point, dtype=float) - c
    v_tip = v_tip - np.dot(v_tip, n) * n
    v_tip_norm = float(np.linalg.norm(v_tip))
    if v_tip_norm <= 1e-12:
        return
    v_tip = v_tip / v_tip_norm

    v_perp = np.cross(n, v_ref)
    v_perp_norm = float(np.linalg.norm(v_perp))
    if v_perp_norm <= 1e-12:
        return
    v_perp = v_perp / v_perp_norm

    dot_val = float(np.clip(np.dot(v_ref, v_tip), -1.0, 1.0))
    signed = float(np.arctan2(np.dot(n, np.cross(v_ref, v_tip)), dot_val))
    # Avoid a blink at phi ~= 0: keep the label visible at the reference position
    # even when the arc/head collapses.
    if abs(signed) <= 1e-6:
        label_pos = c + (cone_radius + 0.12) * v_ref
        ax.text(
            float(label_pos[0]),
            float(label_pos[1]),
            float(label_pos[2]),
            r"$\phi_f$",
            color=color,
            fontsize=text_fontsize,
            ha="center",
            va="center",
        )
        return

    sign = 1.0 if signed >= 0.0 else -1.0
    # Cap head size to roughly the stroke-plane arrow head scale.
    max_head_len = 0.056
    theta_head = min(0.35, 0.45 * abs(signed), max_head_len / max(cone_radius, 1e-12))
    theta_arc_end = signed - sign * theta_head
    if sign * theta_arc_end <= 0.0:
        theta_arc_end = 0.6 * signed

    theta = np.linspace(0.0, theta_arc_end, 72, dtype=float)
    arc = c[None, :] + cone_radius * (
        np.cos(theta)[:, None] * v_ref[None, :] +
        np.sin(theta)[:, None] * v_perp[None, :]
    )
    ax.plot(
        arc[:, 0], arc[:, 1], arc[:, 2],
        linestyle="-", linewidth=line_width, color=color, alpha=alpha, solid_capstyle="butt",
    )

    head_tip = c + cone_radius * (
        np.cos(signed) * v_ref + np.sin(signed) * v_perp
    )
    head_base = arc[-1]
    head_vec = head_tip - head_base
    head_vec = np.asarray(head_vec, dtype=float)
    head_len = float(np.linalg.norm(head_vec))
    if head_len <= 1e-12:
        return
    t_hat = head_vec / head_len
    n_hat = np.cross(n, t_hat)
    n_hat[0:3] = n_hat[0:3]
    n_hat_norm = float(np.linalg.norm(n_hat))
    if n_hat_norm <= 1e-12:
        return
    n_hat = n_hat / n_hat_norm
    head_w = 0.25 * head_len  # full width = 0.5 * length
    head_l = head_base + head_w * n_hat
    head_r = head_base - head_w * n_hat
    head_tri = art3d.Poly3DCollection(
        [[
            (float(head_tip[0]), float(head_tip[1]), float(head_tip[2])),
            (float(head_l[0]), float(head_l[1]), float(head_l[2])),
            (float(head_r[0]), float(head_r[1]), float(head_r[2])),
        ]],
        facecolors=[color],
        edgecolors=[color],
        linewidths=[0.0],
        alpha=alpha,
    )
    ax.add_collection3d(head_tri)

    tm = 0.5 * signed
    label_pos = c + (cone_radius + 0.12) * (
        np.cos(tm) * v_ref + np.sin(tm) * v_perp
    )
    ax.text(
        float(label_pos[0]),
        float(label_pos[1]),
        float(label_pos[2]),
        r"$\phi_f$",
        color=color,
        fontsize=text_fontsize,
        ha="center",
        va="center",
    )


def _draw_stroke_plane_angle_glyph(
    ax,
    top_point: np.ndarray,
    stroke_dir: np.ndarray,
    gamma_deg: Optional[float],
    wing_name: Optional[str] = None,
    *,
    color: str,
    alpha: float,
    line_width: float,
    text_fontsize: int,
) -> None:
    if gamma_deg is None:
        return

    p = np.asarray(top_point, dtype=float)
    d = np.asarray(stroke_dir, dtype=float)
    d[1] = 0.0
    d_norm = np.linalg.norm(d)
    if d_norm < 1e-12:
        return
    d = d / d_norm
    if d[2] < 0.0:
        d = -d

    # Horizontal reference chosen so the annotated angle equals gamma in the side-view XZ plane.
    h = np.array([-1.0, 0.0, 0.0], dtype=float)

    dash_len = 0.2
    arrow_scale = 1.0
    arc_r = arrow_scale * (0.9 * dash_len)
    zhat = np.array([0.0, 0.0, 1.0], dtype=float)

    # The stroke-plane line itself serves as the angled reference line.
    # Shift the original angle cue geometry along the stroke-plane line by one
    # former angled-guide length so it now anchors on the actual stroke-plane line.
    p_ref = p - dash_len * d
    p_stroke = p_ref + dash_len * d
    p_horiz = p_ref + dash_len * h
    ax.plot([p_ref[0], p_horiz[0]], [p_ref[1], p_horiz[1]], [p_ref[2], p_horiz[2]],
            linestyle="--", linewidth=line_width, color=color, alpha=alpha)

    # Circular arc kept inside the wedge between the dashed horizontal guide and stroke-plane line.
    cos_g = float(np.clip(np.dot(h, d), -1.0, 1.0))
    gamma_rad = float(np.arccos(cos_g))
    if gamma_rad > 1e-6:
        # Stop the arc early and let the arrowhead bridge to the angled dashed guide.
        # This avoids the visual mismatch caused by using a tangent-aligned head at the
        # very end of the arc while the head base sits back from the tip.
        theta_head = min(0.35, 0.45 * gamma_rad)
        theta_arc_end = max(0.0, gamma_rad - theta_head)
        theta = np.linspace(0.0, theta_arc_end, 72, dtype=float)
        arc = p_ref[None, :] + arc_r * (
            np.cos(theta)[:, None] * h[None, :] +
            np.sin(theta)[:, None] * zhat[None, :]
        )
        ax.plot(
            arc[:, 0], arc[:, 1], arc[:, 2],
            linestyle="-", linewidth=line_width, color=color, alpha=alpha, solid_capstyle="butt",
        )
        # Explicit filled arrowhead bridging from arc end to the angled dashed line.
        end = arc[-1]
        head_tip = p_ref + (0.97 * dash_len) * d
        t_hat = np.asarray(head_tip - end, dtype=float)
        t_hat[1] = 0.0
        head_len = float(np.linalg.norm(t_hat))
        if head_len <= 1e-12:
            head_len = arrow_scale * 0.056
            t_hat = d.copy()
        else:
            t_hat = t_hat / head_len
        n_hat = np.array([-t_hat[2], 0.0, t_hat[0]], dtype=float)
        # Full base width = 0.5 * head_len (twice as thin as long).
        head_w = 0.25 * head_len  # half-width
        head_base = end
        head_l = head_base + head_w * n_hat
        head_r = head_base - head_w * n_hat
        head_tri = art3d.Poly3DCollection(
            [[
                (float(head_tip[0]), float(head_tip[1]), float(head_tip[2])),
                (float(head_l[0]), float(head_l[1]), float(head_l[2])),
                (float(head_r[0]), float(head_r[1]), float(head_r[2])),
            ]],
            facecolors=[color],
            edgecolors=[color],
            linewidths=[0.0],
            alpha=alpha,
        )
        ax.add_collection3d(head_tri)

    # Math-text label placed to the right of the midpoint between dashed endpoints.
    mid_endpoints = 0.5 * (p_stroke + p_horiz)
    label_x = float(mid_endpoints[0]) - 0.06
    label_z = float(mid_endpoints[2])
    sub = _wing_gamma_subscript(wing_name or "")
    gamma_sym = rf"\gamma_{{{sub}}}" if sub else r"\gamma"
    ax.text(
        label_x,
        float(p[1]),
        label_z,
        rf"${gamma_sym}$",
        color=color,
        fontsize=text_fontsize,
        ha="left",
        va="center",
    )


def _extrema_frame_slice(
    time_values: np.ndarray,
    params: Dict,
    wing_name: str,
) -> Optional[tuple[int, int]]:
    """Return (lo, hi) frame indices spanning the phi extrema half-stroke."""
    extrema = _phi_extrema_times(time_values, params, wing_name)
    if extrema is None:
        return None
    t_min, t_max = extrema
    t = np.asarray(time_values, dtype=float).reshape(-1)
    idx_min = int(np.argmin(np.abs(t - t_min)))
    idx_max = int(np.argmin(np.abs(t - t_max)))
    lo = min(idx_min, idx_max)
    hi = max(idx_min, idx_max)
    if lo == hi:
        return None
    return lo, hi + 1


def _build_wingtip_paths_payload(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
) -> Dict:
    wing_names = list(wing_vectors.keys())
    selected = spec.get("wings")
    if isinstance(selected, list) and selected:
        draw_wings = [str(w) for w in selected if str(w) in wing_vectors]
    else:
        draw_wings = list(wing_names)

    # Remove wings destined for the bottom (mpl) layer.
    bottom = spec.get("bottom_layer_wings")
    if isinstance(bottom, list) and bottom:
        bottom_set = {_normalize_name(w) for w in bottom}
        draw_wings = [w for w in draw_wings if _normalize_name(w) not in bottom_set]

    if not draw_wings:
        raise ValueError("annotation_overlay 'wingtip_paths' could not resolve wings")

    wing_tip_offsets = {}
    for wname in draw_wings:
        try:
            lb0 = float(params.get("wing_lb0", {}).get(wname, DEFAULT_LB0))
            xoff, yoff, zoff = get_wing_offsets(wname)
            root_offset = np.array([xoff, yoff, zoff], dtype=float)
            e_r = np.asarray(wing_vectors[wname]["e_r"], dtype=float)
            if e_r.ndim != 2 or e_r.shape[1] != 3:
                continue
            offsets = root_offset[None, :] + lb0 * e_r
            if time_values is not None:
                sl = _extrema_frame_slice(time_values, params, wname)
                if sl is not None:
                    offsets = offsets[sl[0]:sl[1]]
            wing_tip_offsets[wname] = offsets
        except Exception:
            continue
    if not wing_tip_offsets:
        raise ValueError("annotation_overlay 'wingtip_paths' could not compute wingtip trajectories")

    style = config.style
    theme_name = str(getattr(style, "theme", "")).strip().lower()
    mono_color = str(spec.get("line_color", "#ffffff" if theme_name == "dark" else "#000000"))

    return {
        "draw_wings": draw_wings,
        "wing_tip_offsets": {
            k: np.asarray(v, dtype=float) for k, v in wing_tip_offsets.items()
        },
        "line_width": float(spec.get("line_width", 1.0)),
        "line_alpha": float(spec.get("line_alpha", 0.9)),
        "mono_color": mono_color,
    }


def build_wingtip_paths_bottom_payload(
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
) -> Optional[Dict]:
    """Build payload for wingtip paths that render on the bottom (mpl) layer."""
    bottom = spec.get("bottom_layer_wings")
    if not isinstance(bottom, list) or not bottom:
        return None

    bottom_set = {_normalize_name(w) for w in bottom}
    wing_names = list(wing_vectors.keys())
    draw_wings = [w for w in wing_names if _normalize_name(w) in bottom_set]
    if not draw_wings:
        return None

    wing_tip_offsets = {}
    for wname in draw_wings:
        try:
            lb0 = float(params.get("wing_lb0", {}).get(wname, DEFAULT_LB0))
            xoff, yoff, zoff = get_wing_offsets(wname)
            root_offset = np.array([xoff, yoff, zoff], dtype=float)
            e_r = np.asarray(wing_vectors[wname]["e_r"], dtype=float)
            if e_r.ndim != 2 or e_r.shape[1] != 3:
                continue
            offsets = root_offset[None, :] + lb0 * e_r
            if time_values is not None:
                sl = _extrema_frame_slice(time_values, params, wname)
                if sl is not None:
                    offsets = offsets[sl[0]:sl[1]]
            wing_tip_offsets[wname] = offsets
        except Exception:
            continue
    if not wing_tip_offsets:
        return None

    style = config.style
    theme_name = str(getattr(style, "theme", "")).strip().lower()
    mono_color = str(spec.get("line_color", "#ffffff" if theme_name == "dark" else "#000000"))

    return {
        "wing_tip_offsets": {
            k: np.asarray(v, dtype=float) for k, v in wing_tip_offsets.items()
        },
        "line_width": float(spec.get("line_width", 1.0)),
        "line_alpha": float(spec.get("line_alpha", 0.9)),
        "mono_color": mono_color,
    }


def _render_wingtip_paths_range(
    start_frame: int,
    end_frame: int,
    states: np.ndarray,
    config: HybridConfig,
    output_path: Path,
    payload: Dict,
) -> List[str]:
    style = config.style
    apply_matplotlib_style(style)
    fig, ax = create_overlay_figure(config.camera, style)
    fig.patch.set_alpha(0.0)
    ax.patch.set_alpha(0.0)

    draw_wings = payload["draw_wings"]
    wing_tip_offsets = payload.get("wing_tip_offsets", {})
    line_width = float(payload["line_width"])
    line_alpha = float(payload["line_alpha"])
    mono_color = str(payload["mono_color"])

    files: List[str] = []
    try:
        for frame_idx in range(start_frame, end_frame):
            ax.cla()
            setup_axes(ax, config.viewport, config.camera, style, show_axes=False)
            ax.patch.set_alpha(0.0)

            xb = np.asarray(states[frame_idx][0:3], dtype=float)
            for wname in draw_wings:
                tip_offsets = wing_tip_offsets.get(wname)
                if tip_offsets is None:
                    continue
                path = xb[None, :] + np.asarray(tip_offsets, dtype=float)
                ax.plot(
                    path[:, 0],
                    path[:, 1],
                    path[:, 2],
                    linestyle="--",
                    linewidth=line_width,
                    color=mono_color,
                    alpha=line_alpha,
                )

            output_file = output_path / f"ann_{frame_idx:06d}.png"
            fig.savefig(output_file, dpi=config.camera.dpi, pad_inches=0, transparent=True)
            files.append(str(output_file))
    finally:
        plt.close(fig)
    return files


def _annotation_worker_init(
    states: np.ndarray,
    config_dict: Dict,
    output_path_str: str,
    kind: str,
    payload: Dict,
):
    global _ANN_STATES, _ANN_CONFIG_DICT, _ANN_OUTPUT_PATH, _ANN_KIND, _ANN_PAYLOAD
    _ANN_STATES = states
    _ANN_CONFIG_DICT = config_dict
    _ANN_OUTPUT_PATH = output_path_str
    _ANN_KIND = kind
    _ANN_PAYLOAD = payload


def _annotation_chunk_worker(frame_range: tuple[int, int]) -> tuple[tuple[int, int], List[str]]:
    start, end = frame_range
    config = HybridConfig.from_dict(_ANN_CONFIG_DICT)
    output_path = Path(_ANN_OUTPUT_PATH)
    if _ANN_KIND == "stroke_plane_angles":
        files = _render_stroke_plane_angles_range(
            start, end, _ANN_STATES, config, output_path, _ANN_PAYLOAD
        )
        return frame_range, files
    if _ANN_KIND == "stroke_plane_cone_angles":
        files = _render_stroke_plane_cone_angles_range(
            start, end, _ANN_STATES, config, output_path, _ANN_PAYLOAD
        )
        return frame_range, files
    if _ANN_KIND == "fore_left_y_reference":
        files = _render_fore_left_y_reference_range(
            start, end, _ANN_STATES, config, output_path, _ANN_PAYLOAD
        )
        return frame_range, files
    if _ANN_KIND == "wingtip_paths":
        files = _render_wingtip_paths_range(
            start, end, _ANN_STATES, config, output_path, _ANN_PAYLOAD
        )
        return frame_range, files
    raise ValueError(f"Unsupported annotation worker kind '{_ANN_KIND}'")
    try:
        return float(np.degrees(float(gamma_map[wing_name])))
    except Exception:
        return None


def _render_stroke_plane_angles(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
    n_workers: Optional[int] = None,
) -> List[str]:
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))
    n_frames = len(states)
    payload = _build_stroke_plane_angles_payload(
        states, wing_vectors, params, config, spec, time_values=time_values
    )
    if n_workers == 1:
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_stroke_plane_angles_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    frame_ranges = _split_frame_ranges(n_frames, n_workers)
    config_dict = config.to_dict()
    chunk_results: Dict[tuple[int, int], List[str]] = {}
    try:
        with ProcessPoolExecutor(
            max_workers=n_workers,
            initializer=_annotation_worker_init,
            initargs=(states, config_dict, str(output_path), "stroke_plane_angles", payload),
        ) as executor:
            futures = {executor.submit(_annotation_chunk_worker, fr): fr for fr in frame_ranges}
            with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
                for future in as_completed(futures):
                    frame_range, files = future.result()
                    chunk_results[frame_range] = files
                    progress.update(frame_range[1] - frame_range[0])
    except (OSError, PermissionError) as exc:
        print(f"  Matplotlib: annotation parallel render unavailable ({exc}); falling back to 1 worker")
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_stroke_plane_angles_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    ordered: List[str] = []
    for frame_range in sorted(chunk_results.keys()):
        ordered.extend(chunk_results[frame_range])
    return ordered


def _render_stroke_plane_cone_angles(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
    n_workers: Optional[int] = None,
) -> List[str]:
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))
    n_frames = len(states)
    payload = _build_stroke_plane_cone_angles_payload(
        states, wing_vectors, params, config, spec, time_values=time_values
    )
    if n_workers == 1:
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_stroke_plane_cone_angles_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    frame_ranges = _split_frame_ranges(n_frames, n_workers)
    config_dict = config.to_dict()
    chunk_results: Dict[tuple[int, int], List[str]] = {}
    try:
        with ProcessPoolExecutor(
            max_workers=n_workers,
            initializer=_annotation_worker_init,
            initargs=(states, config_dict, str(output_path), "stroke_plane_cone_angles", payload),
        ) as executor:
            futures = {executor.submit(_annotation_chunk_worker, fr): fr for fr in frame_ranges}
            with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
                for future in as_completed(futures):
                    frame_range, files = future.result()
                    chunk_results[frame_range] = files
                    progress.update(frame_range[1] - frame_range[0])
    except (OSError, PermissionError) as exc:
        print(f"  Matplotlib: annotation parallel render unavailable ({exc}); falling back to 1 worker")
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_stroke_plane_cone_angles_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    ordered: List[str] = []
    for frame_range in sorted(chunk_results.keys()):
        ordered.extend(chunk_results[frame_range])
    return ordered


def _render_fore_left_y_reference(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    spec: Dict,
    n_workers: Optional[int] = None,
) -> List[str]:
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))
    n_frames = len(states)
    payload = _build_fore_left_y_reference_payload(wing_vectors, params, config, spec)
    if n_workers == 1:
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_fore_left_y_reference_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    frame_ranges = _split_frame_ranges(n_frames, n_workers)
    config_dict = config.to_dict()
    chunk_results: Dict[tuple[int, int], List[str]] = {}
    try:
        with ProcessPoolExecutor(
            max_workers=n_workers,
            initializer=_annotation_worker_init,
            initargs=(states, config_dict, str(output_path), "fore_left_y_reference", payload),
        ) as executor:
            futures = {executor.submit(_annotation_chunk_worker, fr): fr for fr in frame_ranges}
            with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
                for future in as_completed(futures):
                    frame_range, files = future.result()
                    chunk_results[frame_range] = files
                    progress.update(frame_range[1] - frame_range[0])
    except (OSError, PermissionError) as exc:
        print(f"  Matplotlib: annotation parallel render unavailable ({exc}); falling back to 1 worker")
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_fore_left_y_reference_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    ordered: List[str] = []
    for frame_range in sorted(chunk_results.keys()):
        ordered.extend(chunk_results[frame_range])
    return ordered


def _render_wingtip_paths(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    spec: Dict,
    time_values: Optional[np.ndarray] = None,
    n_workers: Optional[int] = None,
) -> List[str]:
    if n_workers is None or n_workers <= 0:
        n_workers = os.cpu_count() or MAX_PARALLEL_WORKERS
    n_workers = max(1, min(int(n_workers), MAX_PARALLEL_WORKERS))
    n_frames = len(states)
    payload = _build_wingtip_paths_payload(
        states, wing_vectors, params, config, spec, time_values=time_values,
    )
    if n_workers == 1:
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_wingtip_paths_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    frame_ranges = _split_frame_ranges(n_frames, n_workers)
    config_dict = config.to_dict()
    chunk_results: Dict[tuple[int, int], List[str]] = {}
    try:
        with ProcessPoolExecutor(
            max_workers=n_workers,
            initializer=_annotation_worker_init,
            initargs=(states, config_dict, str(output_path), "wingtip_paths", payload),
        ) as executor:
            futures = {executor.submit(_annotation_chunk_worker, fr): fr for fr in frame_ranges}
            with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
                for future in as_completed(futures):
                    frame_range, files = future.result()
                    chunk_results[frame_range] = files
                    progress.update(frame_range[1] - frame_range[0])
    except (OSError, PermissionError) as exc:
        print(f"  Matplotlib: annotation parallel render unavailable ({exc}); falling back to 1 worker")
        with pip_progress(n_frames, "Matplotlib", unit="frame") as progress:
            files = _render_wingtip_paths_range(
                0, n_frames, states, config, output_path, payload
            )
            progress.update(len(files))
        return files

    ordered: List[str] = []
    for frame_range in sorted(chunk_results.keys()):
        ordered.extend(chunk_results[frame_range])
    return ordered


def render_annotation_frames(
    states: np.ndarray,
    wing_vectors: Dict,
    params: Dict,
    config: HybridConfig,
    output_path: Path,
    annotation_spec: Optional[Dict],
    time_values: Optional[np.ndarray] = None,
    n_workers: Optional[int] = None,
) -> List[str]:
    """
    Render an additional transparent annotation overlay sequence.

    Parameters
    ----------
    annotation_spec
        Mapping with a ``kind`` key. Current supported values:
        ``stroke_plane_angles``, ``stroke_plane_cone_angles``,
        ``fore_left_y_reference``.
    """
    if not annotation_spec:
        return []
    if not isinstance(annotation_spec, dict):
        raise ValueError("annotation_overlay must be a mapping")

    kind = str(annotation_spec.get("kind", "")).strip()
    if not kind:
        raise ValueError("annotation_overlay requires a non-empty 'kind'")

    output_path = Path(output_path)
    output_path.mkdir(parents=True, exist_ok=True)

    if kind == "stroke_plane_angles":
        return _render_stroke_plane_angles(
            states, wing_vectors, params, config, output_path, annotation_spec,
            time_values=time_values, n_workers=n_workers,
        )
    if kind == "stroke_plane_cone_angles":
        return _render_stroke_plane_cone_angles(
            states, wing_vectors, params, config, output_path, annotation_spec,
            time_values=time_values, n_workers=n_workers,
        )
    if kind == "fore_left_y_reference":
        return _render_fore_left_y_reference(
            states, wing_vectors, params, config, output_path, annotation_spec,
            n_workers=n_workers,
        )
    if kind == "wingtip_paths":
        return _render_wingtip_paths(
            states, wing_vectors, params, config, output_path, annotation_spec,
            time_values=time_values, n_workers=n_workers,
        )

    raise ValueError(f"Unsupported annotation_overlay kind '{kind}'")
