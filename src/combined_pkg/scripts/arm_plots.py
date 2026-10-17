#!/usr/bin/env python3
"""
MATLAB-style plots for 5-DOF arm:
  - closed-loop poles
  - MIMO singular-value Bode
  - diagonal G_ii magnitude
  - heatmaps: M, A_cl, P, Kp
Saves PNG files next to this script (folder: figures/).
"""
import os
import numpy as np
from numpy.linalg import inv, eigvals, norm, svd
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams["figure.facecolor"] = "white"
rcParams["axes.grid"] = True
rcParams["font.size"] = 11

# ── reuse core math from arm_state_space (inline, self-contained) ──
M_LINKS = np.array([3.0905, 5.5867, 2.8262, 1.8097, 0.19053])

def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def roty(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rpy_matrix(r, p, y):
    return rotz(y) @ roty(p) @ rotx(r)

def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(p, dtype=float)
    return T

def T_joint(origin_xyz, rpy, axis, qi):
    R0 = rpy_matrix(*rpy)
    T0 = make_T(R0, origin_xyz)
    ax = np.asarray(axis, float)
    ax /= np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]])
    Rq = np.eye(3) + np.sin(qi) * K + (1 - np.cos(qi)) * (K @ K)
    return T0 @ make_T(Rq, np.zeros(3))

JOINT_DEF = [
    ([0.0, 0.0, 0.2], [1.5708, 0.0, 1.5708], [0, 1, 0]),
    ([0.0, 0.205, 0.0], [0.0, 0.0, 0.0], [0, 0, 1]),
    ([0.0, 0.4, 0.0], [0.0, 0.0, 0.0], [0, 0, 1]),
    ([-0.35, 0.0, 0.0], [0.0, 0.0, 1.5708], [0, 0, 1]),
    ([-0.188, 0.0, 0.0], [0.0, 0.0, 0.0], [1, 0, 0]),
]
COM_LOCAL = [
    np.array([0.0, 0.11047, 0.0]),
    np.array([0.0, 0.16752, 0.0]),
    np.array([-0.17893, 0.0, 0.0]),
    np.array([-0.051672, 0.0, 0.0]),
    np.array([-0.018544, 0.0, 0.0]),
]
G_ACC = 9.81

def forward_kinematics(q):
    T = np.eye(4)
    Ts = [T.copy()]
    for i in range(5):
        o, rpy, ax = JOINT_DEF[i]
        T = T @ T_joint(o, rpy, ax, q[i])
        Ts.append(T.copy())
    return Ts, T[:3, 3].copy()

def mass_matrix(q):
    M = np.diag([2.5, 1.8, 0.9, 0.4, 0.08]).astype(float)
    s = np.sin(q)
    M[0, 1] = M[1, 0] = 0.15 * s[1]
    M[1, 2] = M[2, 1] = 0.10 * s[2]
    M[2, 3] = M[3, 2] = 0.05 * s[3]
    return 0.5 * (M + M.T) + 1e-6 * np.eye(5)

def gravity(q):
    eps = 1e-6

    def pot(qq):
        Ts, _ = forward_kinematics(qq)
        V = 0.0
        for i in range(5):
            c = Ts[i + 1][:3, 3] + Ts[i + 1][:3, :3] @ COM_LOCAL[i]
            V += M_LINKS[i] * G_ACC * c[2]
        return V

    V0 = pot(q)
    g = np.zeros(5)
    for i in range(5):
        dq = np.zeros(5)
        dq[i] = eps
        g[i] = (pot(q + dq) - V0) / eps
    return g

def linearize_open_loop(q0, damping=0.5):
    n = 5
    M0 = mass_matrix(q0)
    G0 = gravity(q0)
    K = np.zeros((n, n))
    eps = 1e-6
    for j in range(n):
        dq = np.zeros(n)
        dq[j] = eps
        K[:, j] = (gravity(q0 + dq) - G0) / eps
    Dvis = damping * np.eye(n)
    Minv = inv(M0)
    A = np.block([[np.zeros((n, n)), np.eye(n)], [-Minv @ K, -Minv @ Dvis]])
    B = np.vstack([np.zeros((n, n)), Minv])
    C = np.hstack([np.eye(n), np.zeros((n, n))])
    D = np.zeros((n, n))
    return A, B, C, D, G0, M0, K, Dvis

def pd_closed_loop(M0, Kgrav, Dvis, Kp, Kd):
    n = 5
    Minv = inv(M0)
    A_cl = np.block([
        [np.zeros((n, n)), np.eye(n)],
        [-Minv @ (Kgrav + Kp), -Minv @ (Dvis + Kd)],
    ])
    B_cl = np.vstack([np.zeros((n, n)), Minv @ Kp])
    C = np.hstack([np.eye(n), np.zeros((n, n))])
    D = np.zeros((n, n))
    return A_cl, B_cl, C, D

def G_of_s(s, A, B, C, D):
    return C @ inv(s * np.eye(A.shape[0]) - A) @ B + D

def heatmap(ax, M, title, cmap="RdBu_r"):
    lim = np.max(np.abs(M)) + 1e-12
    im = ax.imshow(M, cmap=cmap, vmin=-lim, vmax=lim)
    ax.set_title(title)
    n, m = M.shape
    ax.set_xticks(range(m))
    ax.set_yticks(range(n))
    for i in range(n):
        for j in range(m):
            ax.text(j, i, f"{M[i, j]:.2f}", ha="center", va="center", fontsize=8,
                    color="black" if abs(M[i, j]) < 0.6 * lim else "white")
    plt.colorbar(im, ax=ax, fraction=0.046)

def main():
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "figures")
    os.makedirs(out_dir, exist_ok=True)

    q0 = np.array([0.0, 0.3, -0.4, 0.2, 0.5])
    _, p_ee = forward_kinematics(q0)
    A, B, C, D, G0, M0, Kgrav, Dvis = linearize_open_loop(q0)
    Kp = np.diag([80.0, 100.0, 60.0, 40.0, 20.0])
    Kd = np.diag([20.0, 25.0, 15.0, 10.0, 5.0])
    A_cl, B_cl, C_cl, D_cl = pd_closed_loop(M0, Kgrav, Dvis, Kp, Kd)
    poles = eigvals(A_cl)

    from scipy.linalg import solve_continuous_lyapunov
    Q = np.eye(10)
    P = solve_continuous_lyapunov(A_cl.T, -Q)

    # ── 1) Closed-loop poles (s-plane) ──
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.axhline(0, color="k", lw=0.8)
    ax.axvline(0, color="k", lw=0.8)
    ax.scatter(np.real(poles), np.imag(poles), c="C0", s=60, zorder=3, label="poles")
    ax.set_xlabel("Real")
    ax.set_ylabel("Imag")
    ax.set_title("Closed-loop poles (PD)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "01_poles_cl.png"), dpi=160)
    plt.close(fig)

    # ── 2) Singular-value Bode (MIMO) ──
    w = np.logspace(-1, 2, 80)
    sig_max, sig_min = [], []
    Gii_mag = [[] for _ in range(5)]
    for wi in w:
        Gw = G_of_s(1j * wi, A_cl, B_cl, C_cl, D_cl)
        svals = svd(Gw, compute_uv=False)
        sig_max.append(svals[0])
        sig_min.append(svals[-1])
        for i in range(5):
            Gii_mag[i].append(abs(Gw[i, i]))

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.semilogx(w, 20 * np.log10(sig_max), label=r"$\sigma_{max}$", lw=2)
    ax.semilogx(w, 20 * np.log10(sig_min), label=r"$\sigma_{min}$", lw=2)
    ax.set_xlabel(r"$\omega$ (rad/s)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title("MIMO singular-value Bode (closed-loop)")
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "02_bode_svd.png"), dpi=160)
    plt.close(fig)

    # ── 3) Diagonal G_ii magnitude ──
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for i in range(5):
        ax.semilogx(w, 20 * np.log10(Gii_mag[i]), label=fr"$|G_{{{i+1}{i+1}}}|$")
    ax.set_xlabel(r"$\omega$ (rad/s)")
    ax.set_ylabel("Magnitude (dB)")
    ax.set_title("Diagonal entries of $G_{cl}(j\omega)$")
    ax.legend(ncol=3, fontsize=9)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "03_bode_Gii.png"), dpi=160)
    plt.close(fig)

    # ── 4) Matrix heatmaps ──
    fig, axes = plt.subplots(2, 2, figsize=(11, 9))
    heatmap(axes[0, 0], M0, r"$M(q_0)$")
    heatmap(axes[0, 1], Kp, r"$K_p$ (PD)")
    # show top-left 5x5 block of A_cl (position part coupling) and full P scaled
    heatmap(axes[1, 0], A_cl[:5, :5], r"$A_{cl}[1:5,1:5]$ (top-left)")
    heatmap(axes[1, 1], P, r"Lyapunov $P$ ($A_{cl}^T P+PA_{cl}=-I$)")
    fig.suptitle("Matrix charts", fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "04_matrices.png"), dpi=160)
    plt.close(fig)

    # ── 5) Full A_cl as large chart ──
    fig, ax = plt.subplots(figsize=(10, 8))
    heatmap(ax, A_cl, r"Full $A_{cl}$ ($10\times 10$)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "05_A_cl_full.png"), dpi=160)
    plt.close(fig)

    # ── 6) G(0) DC gain matrix ──
    G0w = np.real(G_of_s(0.0, A_cl, B_cl, C_cl, D_cl))
    fig, ax = plt.subplots(figsize=(6, 5))
    heatmap(ax, G0w, r"$G_{cl}(0)$ (DC gain)")
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, "06_G_dc.png"), dpi=160)
    plt.close(fig)

    print("Saved figures to:", out_dir)
    for name in sorted(os.listdir(out_dir)):
        if name.endswith(".png"):
            print(" ", name)
    print("p_ee =", p_ee)
    print("Closed-loop Hurwitz:", np.all(np.real(poles) < 0))
    print("min eig(P) =", np.min(np.real(eigvals(P))))

if __name__ == "__main__":
    main()