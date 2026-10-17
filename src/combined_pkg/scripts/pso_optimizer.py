#!/usr/bin/env python3
"""
PSO around the user's MATLAB-scaled defaults.
Does NOT change controller formulas — only searches gains.
"""
import numpy as np

N = 5
DT = 0.02
N_STEPS = int(4.0 / DT)
Q_MIN = np.array([-1.47, -1.57, -1.571, -1.571, 0.05])
Q_MAX = np.array([3.142, 1.57, 3.142, 3.142, 3.142])
DQ_MAX, DDQ_MAX = 2.0, 20.0

# defaults = same as controllers
KP_DEF = np.array([150, 250, 200, 150, 100], float) * 0.02
KD_DEF = np.array([20, 30, 25, 18, 12], float) * 0.02
KI_DEF = np.array([5, 8, 6, 4, 3], float) * 0.02
LAM_DEF = 25.0 * 0.05
ETA_DEF = np.array([10, 15, 12, 10, 8], float) * 0.05
PHI_DEF = 0.1
QW_DEF, RW_DEF = 500.0, 0.001


def _step(q, dq, ddq):
    ddq = np.clip(ddq, -DDQ_MAX, DDQ_MAX)
    dq = np.clip(dq + ddq * DT, -DQ_MAX, DQ_MAX)
    q = np.clip(q + dq * DT, Q_MIN, Q_MAX)
    return q, dq


def _trial_targets(rng):
    q0 = rng.uniform(Q_MIN * 0.25, Q_MAX * 0.25)
    qd = rng.uniform(Q_MIN * 0.45, Q_MAX * 0.45)
    qd[4] = np.clip(qd[4], 0.05, 2.5)
    return q0, qd


def cost_pid(th, rng):
    kp, kd, ki = th[0:5], th[5:10], th[10:15]
    c = 0.0
    for _ in range(3):
        q, dq = _trial_targets(rng)[0], np.zeros(N)
        qd = _trial_targets(rng)[1]
        q, _ = _trial_targets(rng)
        q0, qd = _trial_targets(rng)
        q, dq = q0.copy(), np.zeros(N)
        ei = np.zeros(N)
        for _s in range(N_STEPS):
            e = q - qd
            ei = np.clip(ei + e * DT, -2, 2)
            ddq = -kp * e - kd * dq - ki * ei
            q, dq = _step(q, dq, ddq)
            c += float(e @ e) * DT
    return c / 3


def cost_smc(th, rng):
    lam, eta, phi = float(th[0]), th[1:6], float(max(th[6], 1e-4))
    c = 0.0
    for _ in range(3):
        q, qd = _trial_targets(rng)
        dq = np.zeros(N)
        for _s in range(N_STEPS):
            e = q - qd
            de = dq
            s = de + lam * e
            ddq = -lam * de - eta * np.tanh(s / phi)
            q, dq = _step(q, dq, ddq)
            c += float(e @ e) * DT
    return c / 3


def cost_mpc_proxy(th, rng):
    """Proxy cost: larger Q/R ratio → faster tracking on same plant (no full QP in PSO loop)."""
    qw, rw = float(max(th[0], 1.0)), float(max(th[1], 1e-6))
    gain = np.clip(0.02 * np.sqrt(qw / rw), 0.5, 25.0)
    c = 0.0
    for _ in range(3):
        q, qd = _trial_targets(rng)
        dq = np.zeros(N)
        for _s in range(N_STEPS):
            e = q - qd
            ddq = -gain * e - 0.3 * dq
            q, dq = _step(q, dq, ddq)
            c += float(e @ e) * DT
    return c / 3


def _pso(cost_fn, theta0, lo, hi, n_particles=16, n_iters=22, seed=0):
    rng = np.random.default_rng(seed)
    dim = len(theta0)
    X = np.clip(theta0 + rng.normal(0, 0.25, (n_particles, dim)) * np.maximum(np.abs(theta0), 1e-3), lo, hi)
    V = rng.normal(0, 0.05, (n_particles, dim))
    pb, pc = X.copy(), np.array([cost_fn(X[i], rng) for i in range(n_particles)])
    gi = int(np.argmin(pc))
    gb, gc = pb[gi].copy(), float(pc[gi])
    for _ in range(n_iters):
        for i in range(n_particles):
            r1, r2 = rng.random(dim), rng.random(dim)
            V[i] = np.clip(0.72 * V[i] + 1.4 * r1 * (pb[i] - X[i]) + 1.4 * r2 * (gb - X[i]), -2, 2)
            X[i] = np.clip(X[i] + V[i], lo, hi)
            c = cost_fn(X[i], rng)
            if c < pc[i]:
                pc[i], pb[i] = c, X[i].copy()
                if c < gc:
                    gc, gb = float(c), X[i].copy()
    return gb, gc


def pso_optimize_all(n_particles=16, n_iters=22, verbose=True):
    out = {}
    # PID 15
    th0 = np.hstack([KP_DEF, KD_DEF, KI_DEF])
    lo = th0 * 0.3
    hi = th0 * 3.0
    lo[10:15] = 0.0
    th, c = _pso(cost_pid, th0, lo, hi, n_particles, n_iters, 42)
    out["pid"] = (th, c)
    if verbose:
        print("[PSO] PID cost", c)
    # SMC 7: lam, eta(5), phi
    th0 = np.hstack([[LAM_DEF], ETA_DEF, [PHI_DEF]])
    lo = np.array([0.2, 0.1, 0.1, 0.1, 0.1, 0.1, 0.02])
    hi = np.array([5.0, 3.0, 3.0, 3.0, 3.0, 3.0, 0.5])
    th, c = _pso(cost_smc, th0, lo, hi, n_particles, n_iters, 43)
    out["smc"] = (th, c)
    if verbose:
        print("[PSO] SMC cost", c)
    # MPC 2: Q_W, R_W
    th0 = np.array([QW_DEF, RW_DEF])
    lo = np.array([50.0, 1e-5])
    hi = np.array([5000.0, 0.1])
    th, c = _pso(cost_mpc_proxy, th0, lo, hi, n_particles, n_iters, 44)
    out["mpc"] = (th, c)
    if verbose:
        print("[PSO] MPC cost", c, "Q_W,R_W", th)
    return out


if __name__ == "__main__":
    print(pso_optimize_all())