"""Stick plot animation of wing kinematics from Wang 2007."""

from matplotlib.animation import FuncAnimation
from common import (
    bf,
    chord,
    delta,
    fourier_smooth,
    gamma_fore,
    gamma_hind,
    model_kinematics,
    np,
    plt,
    sf,
    sorted_xy,
    t_bf,
    t_sf,
)


def wing_endpoints(s, d, beta, gamma, offset=(0, 0)):
    """Compute leading edge and trailing edge in lab frame (x, z)."""
    es = np.array([-np.cos(gamma), np.sin(gamma)])
    en = np.array([np.sin(gamma), np.cos(gamma)])
    le = s * es + d * en + np.array(offset)
    chord_dir = np.cos(beta) * es + np.sin(beta) * en
    te = le - chord * chord_dir
    return le, te


def main():
    # Smooth experimental data (7 wingbeat harmonics)
    n = 7
    t_sf_s, sf_s = fourier_smooth(t_sf, sf, n)
    t_sh_s, sh_s = fourier_smooth(*sorted_xy("s_hind"), n)
    t_df_s, df_s = fourier_smooth(*sorted_xy("d_fore"), n)
    t_dh_s, dh_s = fourier_smooth(*sorted_xy("d_hind"), n)
    t_bf_s, bf_s = fourier_smooth(t_bf, bf, n)
    t_bh_s, bh_s = fourier_smooth(*sorted_xy("beta_hind"), n)

    # Common time grid
    t_min = max(t_sf_s[0], t_sh_s[0], t_df_s[0], t_dh_s[0], t_bf_s[0], t_bh_s[0])
    t_max = min(t_sf_s[-1], t_sh_s[-1], t_df_s[-1], t_dh_s[-1], t_bf_s[-1], t_bh_s[-1])
    t = np.linspace(t_min, t_max, 500)

    # Interpolate to common time (beta in radians)
    sf_i = np.interp(t, t_sf_s, sf_s)
    sh_i = np.interp(t, t_sh_s, sh_s)
    df_i = np.interp(t, t_df_s, df_s)
    dh_i = np.interp(t, t_dh_s, dh_s)
    bf_i = np.interp(t, t_bf_s, np.radians(bf_s))
    bh_i = np.interp(t, t_bh_s, np.radians(bh_s))

    # Model time series (d=0, s derived from phi)
    _, sf_m, beta_fore_m = model_kinematics(t, phase=0)
    _, sh_m, beta_hind_m = model_kinematics(t, phase=delta)
    bf_m = np.radians(beta_fore_m)
    bh_m = np.radians(beta_hind_m)

    # Set up figure
    fig, ax = plt.subplots(figsize=(4, 3.8))
    ax.set_aspect("equal")
    ax.set_xlabel("x (mm)")
    ax.set_ylabel("z (mm)")

    le_style_exp = dict(marker="o", markersize=4, markerfacecolor="white",
                        markeredgecolor="grey", markeredgewidth=2, linestyle="none")
    le_style_mod = dict(marker="o", markersize=4, markerfacecolor="white",
                        markeredgecolor="black", markeredgewidth=2, linestyle="none")
    fore_line, = ax.plot([], [], ":", color="grey", linewidth=2, label="Experiment")
    hind_line, = ax.plot([], [], ":", color="grey", linewidth=2)
    fore_dot, = ax.plot([], [], **le_style_exp)
    hind_dot, = ax.plot([], [], **le_style_exp)
    fore_line_m, = ax.plot([], [], "k-", linewidth=2, label="Model")
    hind_line_m, = ax.plot([], [], "k-", linewidth=2)
    fore_dot_m, = ax.plot([], [], **le_style_mod)
    hind_dot_m, = ax.plot([], [], **le_style_mod)
    time_text = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                        va="top", ha="left")

    # Draw stroke planes as faint reference lines
    sp_len = 45
    fore_offset = (1.5 * chord, 0)
    hind_offset = (-1.5 * chord, 0)
    for gam_w, off in [(gamma_fore, fore_offset), (gamma_hind, hind_offset)]:
        xs = np.array([-sp_len, sp_len]) * (-np.cos(gam_w)) + off[0]
        zs = np.array([-sp_len, sp_len]) * np.sin(gam_w) + off[1]
        ax.plot(xs, zs, "k-", linewidth=0.5, alpha=0.3)

    ax.legend(loc="upper right", fontsize=10)
    ax.set_xlim(-40, 40)
    ax.set_ylim(-40, 40)

    def update(i):
        # Experimental
        le_f, te_f = wing_endpoints(sf_i[i], df_i[i], bf_i[i], gamma_fore, offset=fore_offset)
        le_h, te_h = wing_endpoints(sh_i[i], dh_i[i], bh_i[i], gamma_hind, offset=hind_offset)
        # Model (d=0)
        le_fm, te_fm = wing_endpoints(sf_m[i], 0, bf_m[i], gamma_fore, offset=fore_offset)
        le_hm, te_hm = wing_endpoints(sh_m[i], 0, bh_m[i], gamma_hind, offset=hind_offset)

        fore_line.set_data([le_f[0], te_f[0]], [le_f[1], te_f[1]])
        hind_line.set_data([le_h[0], te_h[0]], [le_h[1], te_h[1]])
        fore_dot.set_data([le_f[0]], [le_f[1]])
        hind_dot.set_data([le_h[0]], [le_h[1]])

        fore_line_m.set_data([le_fm[0], te_fm[0]], [le_fm[1], te_fm[1]])
        hind_line_m.set_data([le_hm[0], te_hm[0]], [le_hm[1], te_hm[1]])
        fore_dot_m.set_data([le_fm[0]], [le_fm[1]])
        hind_dot_m.set_data([le_hm[0]], [le_hm[1]])

        time_text.set_text(f"$t/T_{{wb}}$ = {t[i]:.2f}")

        return (fore_line, hind_line, fore_dot, hind_dot,
                fore_line_m, hind_line_m, fore_dot_m, hind_dot_m, time_text)

    fig.tight_layout()
    anim = FuncAnimation(fig, update, frames=len(t), interval=20, blit=False)
    anim.save("stick_animation.mp4", writer="ffmpeg", fps=30, dpi=300)
    print(f"Saved stick_animation.mp4 ({len(t)} frames)")


if __name__ == "__main__":
    main()
