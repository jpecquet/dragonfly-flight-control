"""Fit sinusoidal models to forewing kinematics, hindwing leads by 22 deg."""

from common import *

# Print fit results
t_peak_phi = (-psi_phi / omega) % (2 * np.pi / omega)

print(f"Geometry: span = {span:.0f} mm, r = 2/3 * span = {r:.1f} mm")
print(f"\nForewing fit (reference):")
print(f"  phi  = {np.degrees(C_phi):.1f} + {np.degrees(A_phi):.1f} * cos(omega*t + {np.degrees(psi_phi):+.1f} deg)  [deg]")
print(f"  s    = r * cos(phi)  =>  s in [{r*np.cos(C_phi+A_phi):.1f}, {r*np.cos(C_phi-A_phi):.1f}] mm")
print(f"  beta = {C_beta:.1f} + {A_beta:.1f} * cos(omega*t + {np.degrees(psi_beta):+.1f} deg)  [deg]")
print(f"  First phi peak at t = {t_peak_phi:.4f}")
print(f"  Phase between phi and beta: {np.degrees(psi_beta - psi_phi):+.1f} deg")
print(f"\nHindwing: same model with phase lead delta = {np.degrees(delta):.0f} deg")

# Model evaluation
t_model = np.linspace(0, 5, 500)

_, s_fore_model, beta_fore_model = model_kinematics(t_model, phase=0)
_, s_hind_model, beta_hind_model = model_kinematics(t_model, phase=delta)

# Model angle of attack: alpha = -beta (upstroke) or 180-beta (downstroke)
ds_fore = np.gradient(s_fore_model, t_model)
ds_hind = np.gradient(s_hind_model, t_model)
alpha_fore_model = np.where(ds_fore > 0, -beta_fore_model, 180 - beta_fore_model)
alpha_hind_model = np.where(ds_hind > 0, -beta_hind_model, 180 - beta_hind_model)

# Load hindwing experimental data
t_sh, sh = sorted_xy("s_hind")
t_bh, bh = sorted_xy("beta_hind")
t_af, af = sorted_xy("alpha_fore")
t_ah, ah = sorted_xy("alpha_hind")

t_af_b, af_b = break_discontinuities(t_af, af)
t_ah_b, ah_b = break_discontinuities(t_ah, ah)

# Plot comparison
fig, axes = plt.subplots(3, 1, figsize=(6.5, 4.5), sharex=True)

ax = axes[0]
ax.plot(t_sf, sf, "b.", markersize=2, alpha=0.4, label="fore (exp)")
ax.plot(t_sh, sh, "r.", markersize=2, alpha=0.4, label="hind (exp)")
ax.plot(t_model, s_fore_model, "-b", linewidth=1, label="fore (fit)")
ax.plot(t_model, s_hind_model, "--r", linewidth=1, label="hind (fit)")
ax.set_ylabel(r"$s$ (mm)")
ax.set_ylim(-30, 40)
ax.legend(ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=9)

ax = axes[1]
ax.plot(t_bf, bf, "b.", markersize=2, alpha=0.4)
ax.plot(t_bh, bh, "r.", markersize=2, alpha=0.4)
ax.plot(t_model, beta_fore_model, "-b", linewidth=1)
ax.plot(t_model, beta_hind_model, "--r", linewidth=1)
ax.set_ylabel(r"$\beta$ (deg)")
ax.set_ylim(0, 140)

ax = axes[2]
ax.plot(t_af_b, af_b, "b.", markersize=2, alpha=0.4)
ax.plot(t_ah_b, ah_b, "r.", markersize=2, alpha=0.4)
t_maf, amf = break_discontinuities(t_model, alpha_fore_model)
t_mah, amh = break_discontinuities(t_model, alpha_hind_model)
ax.plot(t_maf, amf, "-b", linewidth=1)
ax.plot(t_mah, amh, "--r", linewidth=1)
ax.set_ylabel(r"$\alpha$ (deg)")
ax.set_ylim(-50, 150)
ax.set_xlabel(r"$t/T_{wb}$")

ax.set_xlim(0, 5)
fig.tight_layout()
plt.savefig("fit_sinusoids.pdf", dpi=300)
