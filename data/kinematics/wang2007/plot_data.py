"""Plot experimental wing kinematics from Wang 2007."""

from common import fourier_smooth, plt, sorted_xy


def main():
    n = 7  # wingbeat harmonics to keep

    variables = [
        ("s",     r"$s$ (mm)",      [-30, 40]),
        ("d",     r"$d$ (mm)",      [-5, 5]),
        ("beta",  r"$\beta$ (deg)", [0, 140]),
    ]

    fig, axes = plt.subplots(3, 1, figsize=(6.5, 4), sharex=True)

    for ax, (var, ylabel, ylim) in zip(axes, variables):
        # Raw data
        xf, yf = sorted_xy(f"{var}_fore")
        xh, yh = sorted_xy(f"{var}_hind")
        ax.plot(xf, yf, "b.", markersize=2, alpha=0.3)
        ax.plot(xh, yh, "r.", markersize=2, alpha=0.3)

        # Fourier-smoothed
        xf_s, yf_s = fourier_smooth(*sorted_xy(f"{var}_fore"), n)
        xh_s, yh_s = fourier_smooth(*sorted_xy(f"{var}_hind"), n)
        ax.plot(xf_s, yf_s, "-b", linewidth=1, label="forewing")
        ax.plot(xh_s, yh_s, "--r", linewidth=1, label="hindwing")

        ax.set_xlim(0, 5)
        ax.set_ylim(ylim)
        ax.set_ylabel(ylabel)

    axes[0].legend(ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    axes[-1].set_xlabel(r"$t/T_{wb}$")
    fig.tight_layout()
    plt.savefig("kinematics.pdf", dpi=300)


if __name__ == "__main__":
    main()
