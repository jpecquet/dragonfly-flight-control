"""Blade element vertical force compared to CFD (Wang 2007 kinematics)."""

from common import *

t = np.linspace(0, 5, 500)

# --- Sinusoidal model force ---
phi_fore, _, beta_fore = model_kinematics(t, phase=0)
phi_hind, _, beta_hind = model_kinematics(t, phase=delta)

phi_dot_fore = A_phi * omega_phys * np.sin(omega * t + psi_phi)
phi_dot_hind = A_phi * omega_phys * np.sin(omega * t + psi_phi + delta)

Fz_model = (
    compute_Fz_pair(np.pi / 2 - phi_fore, phi_dot_fore,
                    np.pi / 2 - np.radians(beta_fore), gam_fore)
    + compute_Fz_pair(np.pi / 2 - phi_hind, phi_dot_hind,
                      np.pi / 2 - np.radians(beta_hind), gam_hind)
) / mg

# --- Experimental kinematics force (smoothed, 7 wingbeat harmonics) ---
n = 7
t_sf_s, sf_s = fourier_smooth(t_sf, sf, n)
t_sh_s, sh_s = fourier_smooth(*sorted_xy("s_hind"), n)
t_bf_s, bf_s = fourier_smooth(t_bf, bf, n)
t_bh_s, bh_s = fourier_smooth(*sorted_xy("beta_hind"), n)

sf_i = np.interp(t, t_sf_s, sf_s)
sh_i = np.interp(t, t_sh_s, sh_s)
bf_i = np.interp(t, t_bf_s, bf_s)
bh_i = np.interp(t, t_bh_s, bh_s)

phi_sim_fore = np.pi / 2 - np.arccos(np.clip(sf_i / r, -1, 1))
phi_sim_hind = np.pi / 2 - np.arccos(np.clip(sh_i / r, -1, 1))
phi_dot_fore_e = np.gradient(phi_sim_fore, t) * f_phys
phi_dot_hind_e = np.gradient(phi_sim_hind, t) * f_phys

Fz_exp = (
    compute_Fz_pair(phi_sim_fore, phi_dot_fore_e,
                    np.pi / 2 - np.radians(bf_i), gam_fore)
    + compute_Fz_pair(phi_sim_hind, phi_dot_hind_e,
                      np.pi / 2 - np.radians(bh_i), gam_hind)
) / mg

# --- CFD reference (normalized by mg, one wing per pair) ---
cfd = np.genfromtxt(os.path.join(data_dir, "cfd_data.csv"), delimiter=",")
t_cfd = cfd[:, 0]
Fz_cfd = 2 * cfd[:, 1]

print(f"Vertical force (blade element, 4 wings, f = {f_phys} Hz):")
print(f"  Sinusoidal model:  mean Fz/mg = {np.mean(Fz_model):.2f},  peak = {np.max(Fz_model):.2f}")
print(f"  Experimental kin.: mean Fz/mg = {np.mean(Fz_exp):.2f},  peak = {np.max(Fz_exp):.2f}")
print(f"  CFD:               mean Fz/mg = {np.mean(Fz_cfd):.2f}")

# Plot
fig, ax = plt.subplots(figsize=(6.5, 3.5))
ax.plot(t_cfd, Fz_cfd, "-", color="0.6", linewidth=1.5, label="CFD")
ax.plot(t, Fz_exp, "-b", linewidth=1, label="BE (exp. kin.)")
ax.plot(t, Fz_model, "k-", linewidth=1, label="BE (sinusoidal)")
ax.set_ylabel(r"$F_z\,/\,mg$")
ax.set_xlabel(r"$t/T_{wb}$")
ax.set_xlim(0, 5)
ax.legend(ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0), fontsize=9)
fig.tight_layout()
plt.savefig("force.pdf", dpi=300)
