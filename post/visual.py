import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as ani
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

plt.rcParams["font.family"] = "serif"
plt.rcParams["font.serif"] = ["Times New Roman"]
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.size'] = 12


def plotDragonfly(states, wing_vectors, params, outfile, animate=False):
    """
    Plot or animate the dragonfly simulation.

    Args:
        states: list of state arrays [x, y, z, ux, uy, uz]
        wing_vectors: list of wing vector dicts
        params: parameter dict
        outfile: output filename (.png or .mp4)
        animate: if True, create animation; otherwise static image
    """
    xb = states[0][0:3]
    v = wing_vectors[0]
    p = params

    fig = plt.figure(figsize=(4, 4))
    ax = fig.add_subplot(111, projection='3d')

    # Body segment lengths
    Lh = 0.1
    Lt = 0.25
    La = 1 - Lh - Lt

    # Body segment radii
    Rh = 0.07
    Rt = 0.05
    Ra = 0.03

    # Wing attachment positions
    dw = 0.2 * (p['lb0_f'] + p['lb0_h']) / 2
    fw_x0 = dw / 2
    hw_x0 = -dw / 2

    # Body segment centers
    a_xc = hw_x0 - La / 2
    t_xc = a_xc + La / 2 + Lt / 2
    h_xc = t_xc + Lt / 2 + Lh / 2

    # Draw body
    drawEllipsoid(ax, xb + np.array([h_xc, 0, 0]), Lh / 2, Rh, Lh / 2)
    drawEllipsoid(ax, xb + np.array([t_xc, 0, 0]), Lt / 2, Rt, Rt * 1.5)
    drawEllipsoid(ax, xb + np.array([a_xc, 0, 0]), La / 2, Ra, Ra)
    drawCylinder(ax, xb, t_xc, a_xc, Ra)

    # Draw wings
    drawWing(ax, xb + np.array([fw_x0, 0, 0]), v['fr'], p['lb0_f'], label=True)
    drawWing(ax, xb + np.array([fw_x0, 0, 0]), v['fl'], p['lb0_f'])
    drawWing(ax, xb + np.array([hw_x0, 0, 0]), v['hr'], p['lb0_h'])
    drawWing(ax, xb + np.array([hw_x0, 0, 0]), v['hl'], p['lb0_h'])

    # Set axis limits
    box_width = 1.2
    ax.set_xlim([xb[0] + a_xc - La / 2 - (box_width - 1) / 2,
                 xb[0] + a_xc + La / 2 + Lt + Lh + (box_width - 1) / 2])
    ax.set_ylim([xb[1] - box_width / 2, xb[1] + box_width / 2])
    ax.set_zlim([xb[2] - box_width / 2, xb[2] + box_width / 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1, 1, 1])
    ax.set_title(r'$\overline{u}_x = %.2f$, $\overline{u}_z = %.2f$' % (states[0][3], states[0][5]))
    plt.legend(loc='upper center')

    if not animate:
        plt.savefig(outfile, dpi=300, bbox_inches='tight')
        plt.close()
        return

    def update(i):
        xb = states[i][0:3]
        v = wing_vectors[i]

        for artist in ax.collections:
            artist.remove()
        for line in ax.lines:
            line.remove()

        drawEllipsoid(ax, xb + np.array([h_xc, 0, 0]), Lh / 2, Rh, Lh / 2)
        drawEllipsoid(ax, xb + np.array([t_xc, 0, 0]), Lt / 2, Rt, Rt * 1.5)
        drawEllipsoid(ax, xb + np.array([a_xc, 0, 0]), La / 2, Ra, Ra)
        drawCylinder(ax, xb, t_xc, a_xc, Ra)

        drawWing(ax, xb + np.array([fw_x0, 0, 0]), v['fr'], p['lb0_f'], label=True)
        drawWing(ax, xb + np.array([fw_x0, 0, 0]), v['fl'], p['lb0_f'])
        drawWing(ax, xb + np.array([hw_x0, 0, 0]), v['hr'], p['lb0_h'])
        drawWing(ax, xb + np.array([hw_x0, 0, 0]), v['hl'], p['lb0_h'])

        ax.set_xlim([xb[0] + a_xc - La / 2 - (box_width - 1) / 2,
                     xb[0] + a_xc + La / 2 + Lt + Lh + (box_width - 1) / 2])
        ax.set_ylim([xb[1] - box_width / 2, xb[1] + box_width / 2])
        ax.set_zlim([xb[2] - box_width / 2, xb[2] + box_width / 2])
        ax.set_title(r'$\overline{u}_x = %.2f$, $\overline{u}_z = %.2f$' % (states[i][3], states[i][5]))
        ax.set_box_aspect([1, 1, 1])

    anim = ani.FuncAnimation(fig=fig, func=update, frames=np.arange(len(states)), interval=30)
    anim.save(filename=outfile, writer="ffmpeg")
    plt.close()


def drawWing(ax, origin, vecs, r, label=False):
    """Draw a single wing as a 3D polygon with force vectors."""
    e_c = vecs['e_c']
    e_r = vecs['e_r']

    c = 0.2 * r
    rtip = 0.07 * r
    angle = (c - 2 * rtip) / (r / 2 - rtip)
    vertices = [origin]

    # Leading edge curve
    theta = np.linspace(0, np.pi - angle, 40)
    for tht in theta:
        vertices.append(origin + (r - rtip + rtip * np.sin(tht)) * e_r + (rtip * np.cos(tht) - rtip) * e_c)

    # Trailing edge
    theta = np.linspace(angle, 0, 10)
    for tht in theta:
        vertices.append(origin + (r / 2 + rtip * np.sin(tht)) * e_r + (-c + rtip * (1 - np.cos(tht))) * e_c)

    # Root curve
    theta = np.linspace(0, np.pi / 2, 20)
    for tht in theta:
        vertices.append(origin + (rtip - rtip * np.sin(tht)) * e_r + (-rtip * np.cos(tht) - c + rtip) * e_c)

    poly = Poly3DCollection([vertices], alpha=0.3, facecolor='lightgray', edgecolor='k', linewidth=1)
    ax.add_collection3d(poly)

    # Force vectors at 2/3 span
    x0 = origin + 2 / 3 * r * e_r - c / 2 * e_c
    coeff = 0.05
    xl = x0 + coeff * vecs['lift']
    xd = x0 + coeff * vecs['drag']

    if label:
        ax.plot([x0[0], xl[0]], [x0[1], xl[1]], [x0[2], xl[2]], 'b', label=r'$\vec{F}_L$')
        ax.plot([x0[0], xd[0]], [x0[1], xd[1]], [x0[2], xd[2]], 'r', label=r'$\vec{F}_D$')
    else:
        ax.plot([x0[0], xl[0]], [x0[1], xl[1]], [x0[2], xl[2]], 'b')
        ax.plot([x0[0], xd[0]], [x0[1], xd[1]], [x0[2], xd[2]], 'r')


def drawEllipsoid(ax, origin, Rx, Ry, Rz):
    """Draw an ellipsoid surface."""
    u = np.linspace(0, 2 * np.pi, 20)
    v = np.linspace(0, np.pi, 10)
    x = Rx * np.outer(np.cos(u), np.sin(v)) + origin[0]
    y = Ry * np.outer(np.sin(u), np.sin(v)) + origin[1]
    z = Rz * np.outer(np.ones_like(u), np.cos(v)) + origin[2]
    ax.plot_surface(x, y, z, color='lightgray', shade=True)


def drawCylinder(ax, origin, x1, x2, R):
    """Draw a cylinder along the x-axis."""
    theta = np.linspace(0, 2 * np.pi, 20)
    x = np.array([x1, x2])
    X, Theta = np.meshgrid(x, theta)
    X = X + np.ones_like(X) * origin[0]
    Y = R * np.cos(Theta) + np.ones_like(X) * origin[1]
    Z = R * np.sin(Theta) + np.ones_like(X) * origin[2]
    ax.plot_surface(X, Y, Z, color='lightgray', alpha=1, shade=True)
