"""Compare experimental alpha with alpha computed from s, d, and beta."""

from common import *

n = 7  # number of Fourier components to keep

# Smooth s, d, beta
t_sf, sf = fourier_smooth(*sorted_xy("s_fore"), n)
t_df, df = fourier_smooth(*sorted_xy("d_fore"), n)
t_bf, bf = fourier_smooth(*sorted_xy("beta_fore"), n)

t_sh, sh = fourier_smooth(*sorted_xy("s_hind"), n)
t_dh, dh = fourier_smooth(*sorted_xy("d_hind"), n)
t_bh, bh = fourier_smooth(*sorted_xy("beta_hind"), n)

# Raw alpha (not filtered)
t_af, af = sorted_xy("alpha_fore")
t_ah, ah = sorted_xy("alpha_hind")


def compute_alpha(t_s, s, t_d, d, t_b, beta_deg):
    """Compute angle of attack from s, d, and beta.

    alpha = atan2(d_dot, s_dot) - beta

    This naturally handles stroke reversal: when s_dot flips sign,
    atan2 transitions through 180 degrees.
    """
    t_min = max(t_s[0], t_d[0], t_b[0])
    t_max = min(t_s[-1], t_d[-1], t_b[-1])
    t = np.linspace(t_min, t_max, 2000)

    s_interp = np.interp(t, t_s, s)
    d_interp = np.interp(t, t_d, d)
    b_interp = np.interp(t, t_b, beta_deg)

    s_dot = np.gradient(s_interp, t)
    d_dot = np.gradient(d_interp, t)

    alpha = np.degrees(np.arctan2(d_dot, s_dot)) - b_interp
    return t, alpha


t_comp_f, alpha_comp_f = compute_alpha(t_sf, sf, t_df, df, t_bf, bf)
t_comp_h, alpha_comp_h = compute_alpha(t_sh, sh, t_dh, dh, t_bh, bh)

# Break discontinuities for plotting
t_af_b, af_b = break_discontinuities(t_af, af)
t_ah_b, ah_b = break_discontinuities(t_ah, ah)
t_comp_f_b, alpha_comp_f_b = break_discontinuities(t_comp_f, alpha_comp_f)
t_comp_h_b, alpha_comp_h_b = break_discontinuities(t_comp_h, alpha_comp_h)

# Plot
fig, axes = plt.subplots(2, 1, figsize=(6.5, 4), sharex=True)

for ax, (t_exp, a_exp, t_comp, a_comp, label) in zip(axes, [
    (t_af_b, af_b, t_comp_f_b, alpha_comp_f_b, "Forewing"),
    (t_ah_b, ah_b, t_comp_h_b, alpha_comp_h_b, "Hindwing"),
]):
    ax.plot(t_exp, a_exp, "k.", markersize=2, alpha=0.5, label=r"$\alpha$ (data)")
    ax.plot(t_comp, a_comp, "-b", linewidth=1, label=r"$\mathrm{atan2}(\dot d, \dot s) - \beta$")
    ax.set_ylabel(r"$\alpha$ (deg)")
    ax.set_ylim(-80, 180)
    ax.legend(loc="upper right", fontsize=9)
    ax.set_title(label, fontsize=11)

axes[-1].set_xlabel(r"$t/T_{wb}$")
axes[0].set_xlim(0, 5)
fig.tight_layout()
plt.savefig("compare_alpha.pdf", dpi=300)
