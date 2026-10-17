#!/usr/bin/env python3
"""
FK + numerical linearization + MIMO G(s) + PD closed-loop Lyapunov
for combined_pkg 5-DOF arm (URDF joint origins).
"""
import numpy as np
from numpy.linalg import inv, eigvals, norm

# ───────── masses (kg) from URDF ─────────
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
    ax = np.asarray(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)
    K = np.array([[0, -ax[2], ax[1]],
                  [ax[2], 0, -ax[0]],
                  [-ax[1], ax[0], 0]])
    Rq = np.eye(3) + np.sin(qi) * K + (1 - np.cos(qi)) * (K @ K)
    return T0 @ make_T(Rq, np.zeros(3))

JOINT_DEF = [
    ([0.0, 0.0, 0.2],   [1.5708, 0.0, 1.5708], [0, 1, 0]),
    ([0.0, 0.205, 0.0], [0.0, 0.0, 0.0],       [0, 0, 1]),
    ([0.0, 0.4, 0.0],   [0.0, 0.0, 0.0],       [0, 0, 1]),
    ([-0.35, 0.0, 0.0], [0.0, 0.0, 1.5708],    [0, 0, 1]),
    ([-0.188, 0.0, 0.0],[0.0, 0.0, 0.0],       [1, 0, 0]),
]

def forward_kinematics(q):
    T = np.eye(4)
    Ts = [T.copy()]
    for i in range(5):
        o, rpy, ax = JOINT_DEF[i]
        T = T @ T_joint(o, rpy, ax, q[i])
        Ts.append(T.copy())
    return Ts, T[:3, 3].copy()

COM_LOCAL = [
    np.array([0.0, 0.11047, 0.0]),
    np.array([0.0, 0.16752, 0.0]),
    np.array([-0.17893, 0.0, 0.0]),
    np.array([-0.051672, 0.0, 0.0]),
    np.array([-0.018544, 0.0, 0.0]),
]
G_ACC = 9.81

def mass_matrix(q):
    M0 = np.diag([2.5, 1.8, 0.9, 0.4, 0.08])
    s = np.sin(q)
    M = M0.copy()
    M[0, 1] = M[1, 0] = 0.15 * s[1]
    M[1, 2] = M[2, 1] = 0.10 * s[2]
    M[2, 3] = M[3, 2] = 0.05 * s[3]
    return 0.5 * (M + M.T) + 1e-6 * np.eye(5)

def gravity(q):
    eps = 1e-6

    def potential(qq):
        Ts, _ = forward_kinematics(qq)
        V = 0.0
        for i in range(5):
            Rc = Ts[i + 1][:3, :3]
            pc = Ts[i + 1][:3, 3]
            c = pc + Rc @ COM_LOCAL[i]
            V += M_LINKS[i] * G_ACC * c[2]
        return V

    V0 = potential(q)
    g = np.zeros(5)
    for i in range(5):
        dq = np.zeros(5)
        dq[i] = eps
        g[i] = (potential(q + dq) - V0) / eps
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
    A = np.block([
        [np.zeros((n, n)), np.eye(n)],
        [-Minv @ K,        -Minv @ Dvis],
    ])
    B = np.vstack([np.zeros((n, n)), Minv])
    C = np.hstack([np.eye(n), np.zeros((n, n))])
    D = np.zeros((n, n))
    return A, B, C, D, G0, M0, K, Dvis

def pd_closed_loop(A, B, M0, Kgrav, Dvis, Kp, Kd):
    """
    τ = τ0 - Kp δq - Kd δdq
    → effective stiffness Kp+Kgrav, damping Kd+Dvis
    """
    n = 5
    Minv = inv(M0)
    A_cl = np.block([
        [np.zeros((n, n)), np.eye(n)],
        [-Minv @ (Kgrav + Kp), -Minv @ (Dvis + Kd)],
    ])
    # input is reference q_d (through Kp): u_ref = Kp q_d  (approx)
    B_cl = np.vstack([np.zeros((n, n)), Minv @ Kp])
    C = np.hstack([np.eye(n), np.zeros((n, n))])
    D = np.zeros((n, n))
    return A_cl, B_cl, C, D

def G_of_s(s, A, B, C, D):
    """MIMO G(s) = C(sI-A)^{-1}B + D  (complex matrix 5x5)"""
    n = A.shape[0]
    return C @ inv(s * np.eye(n) - A) @ B + D

def lyapunov_P(A, Q=None):
    from scipy.linalg import solve_continuous_lyapunov
    n = A.shape[0]
    if Q is None:
        Q = np.eye(n)
    # Only valid when A is Hurwitz
    if np.any(np.real(eigvals(A)) >= 1e-9):
        return None, False, eigvals(A)
    P = solve_continuous_lyapunov(A.T, -Q)
    ev = eigvals(P)
    return P, np.all(np.real(ev) > 0), ev

if __name__ == "__main__":
    q0 = np.array([0.0, 0.3, -0.4, 0.2, 0.5])
    _, p_ee = forward_kinematics(q0)
    print("=== Forward Kinematics ===")
    print("p_ee =", p_ee)

    A, B, C, D, G0, M0, Kgrav, Dvis = linearize_open_loop(q0, damping=0.5)
    poles_ol = eigvals(A)
    print("\n=== Open-loop ===")
    print("τ0 = G(q0) =\n", G0)
    print("M(q0) =\n", M0)
    print("poles (sorted by real part):\n", poles_ol[np.argsort(np.real(poles_ol))])
    print("Open-loop Hurwitz?", np.all(np.real(poles_ol) < 0))

    # PD gains (diagonal) — increase if still unstable
    Kp = np.diag([80.0, 100.0, 60.0, 40.0, 20.0])
    Kd = np.diag([20.0, 25.0, 15.0, 10.0, 5.0])
    A_cl, B_cl, C_cl, D_cl = pd_closed_loop(A, B, M0, Kgrav, Dvis, Kp, Kd)
    poles_cl = eigvals(A_cl)
    print("\n=== Closed-loop PD ===")
    print("poles:\n", poles_cl[np.argsort(np.real(poles_cl))])
    print("Closed-loop Hurwitz?", np.all(np.real(poles_cl) < 0))

    P, ok, evP = lyapunov_P(A_cl)
    print("\n=== Lyapunov on A_cl ===")
    if P is None:
        print("A_cl not Hurwitz → no PD Lyapunov solution. Increase Kp/Kd.")
    else:
        print("P > 0 ?", ok)
        print("min eig(P) =", np.min(np.real(evP)))

    print("\n=== ||G_cl(jw)||_F (resolvent, no ss2tf) ===")
    for w in [0.1, 1.0, 5.0, 20.0, 50.0]:
        Gw = G_of_s(1j * w, A_cl, B_cl, C_cl, D_cl)
        print(f"w={w:6.1f}  ||G||_F={norm(Gw, 'fro'):.4f}")

    # optional: print one entry G11 as rational-like numeric samples
    print("\n=== Sample G_cl(s)[0,0] at s=jω ===")
    for w in [0.5, 2.0, 10.0]:
        g00 = G_of_s(1j * w, A_cl, B_cl, C_cl, D_cl)[0, 0]
        print(f"G11(j{w}) = {g00.real:.4f}{g00.imag:+.4f}j")

    print("\nDone.")