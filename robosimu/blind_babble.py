"""Prototype oracle-free reaching controller (Faz 1, 23.11.2026).

Input: ``TimestampedObservation`` only (pixels + timestamps).
No analytic Jacobian, no joint encoder, no metric gap, no ``set_delay``.

1. Motor babbling: a short +/- pulse on each of the 5 arm joints; pairs the
   efference copy (integral of issued commands) at each capture instant with
   the retinal error and fits the image Jacobian by least squares.
2. Broyden update of that Jacobian during the reach.
3. Gain k = c / (delta_obs + tau_servo), c = 1/e, delta_obs from timestamps.

Status: PROTOTYPE. See reports/faz1_oracle_denetimi.md for measured accuracy.
"""
from __future__ import annotations

import math

import numpy as np


class BlindBabbleGovernor:
    name = "bbg"
    zero_oracle = True
    outputs_joint_velocity = True

    def __init__(self, T=0.01, amp=0.15, bab_t=0.12, lam=2.0, eps_px=1.0,
                 lr=0.3, qmax=0.6, assumed_delay=None, servo_tau=0.1,
                 c=1.0 / math.e):
        self.T = T; self.amp = amp; self.bab_t = bab_t; self.lam = lam
        self.eps_px = eps_px; self.lr = lr; self.qmax = qmax; self.c = c
        self.assumed = assumed_delay; self.servo_tau = servo_tau
        self.q_hat = np.zeros(5); self.hist = []; self.J = None; self.t0 = None
        self.last_qd = np.zeros(5); self.last_dq_neck = np.zeros(2)
        self.samples = []; self.prev = None; self.info = {}

    def commit(self, t, qd, dq_neck, dt):
        self.q_hat = self.q_hat + np.asarray(qd, float)[:5] * dt
        self.hist.append((t, self.q_hat.copy()))

    def _qhat_at(self, t):
        if not self.hist:
            return np.zeros(5)
        ts = np.array([h[0] for h in self.hist])
        i = int(np.searchsorted(ts, t))
        if i <= 0:
            return self.hist[0][1]
        if i >= len(ts):
            return self.hist[-1][1]
        a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return (1 - a) * self.hist[i - 1][1] + a * self.hist[i][1]

    def step_obs(self, obs):
        e = obs.retinal_error
        dl = self.assumed if self.assumed else max(obs.delta_obs, 1e-3)
        if self.t0 is None:
            self.t0 = obs.t_now
        tt = obs.t_now - self.t0
        self.samples.append((self._qhat_at(obs.t_cap), e))
        per = 2 * self.bab_t + dl + self.servo_tau
        if self.J is None:
            j = int(tt // per)
            if j < 5:
                ph = tt - j * per
                qd = np.zeros(5)
                if ph < self.bab_t:
                    qd[j] = self.amp / self.bab_t
                elif ph < 2 * self.bab_t:
                    qd[j] = -self.amp / self.bab_t
                self.last_qd = qd
                return qd
            Q = np.array([x[0] for x in self.samples])
            E = np.array([x[1] for x in self.samples])
            self.J = np.linalg.lstsq(Q - Q[0], E - E[0], rcond=None)[0].T
        if self.prev is not None:
            dq = self.samples[-1][0] - self.prev[0]
            de = e - self.prev[1]
            if np.linalg.norm(dq) > 1e-4:
                self.J = self.J + self.lr * np.outer(de - self.J @ dq, dq) / (1e-3 + dq @ dq)
        self.prev = self.samples[-1]
        k = self.c / (dl + self.servo_tau)
        r = float(np.linalg.norm(e))
        A = self.J @ self.J.T + self.lam ** 2 * np.eye(3)
        qd = -k * (self.J.T @ np.linalg.solve(A, e)) if r > self.eps_px else np.zeros(5)
        n = float(np.max(np.abs(qd)))
        if n > self.qmax:
            qd *= self.qmax / n
        self.last_qd = qd
        self.info = {"k": k, "r_px": r}
        return qd
