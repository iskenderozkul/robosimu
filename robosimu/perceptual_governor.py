"""Perceptual 1/e Governor — oracle-free visual reaching.

The controller sees ONLY ``TimestampedObservation`` (pixel coordinates of the
hand and target in two cameras + capture/arrival/now timestamps).  It never
reads joint angles, the analytic Jacobian, metric gaps or the true delay.

Pipeline
--------
1. Body schema (once per arm, reusable): settled single-joint babbling.  The
   efference copy (integral of the issued commands) is paired with the
   retinal change at each capture instant; the image Jacobian J and the
   servo lag L are identified jointly by least squares over a lag grid.
2. Perceptual integration: moving average over n frames.
3. Gain from the loop delay:  k = c / delta_eff,
      delta_eff = delta_obs + L + (n-1) T / 2 + T / 2,
   delta_obs = max recent (t_now - t_cap) from the timestamps, c = 1/e.
4. Damped least-squares joint command, joint-rate saturation.
5. Optional Broyden refinement (default OFF: measured to raise overshoot).
6. Intermittent gaze: frames from a previous fixation are discarded, the
   longest observed blind interval enters delta_eff as a hold term, and
   between fixations the hand is steered by an efference-copy forward model
   (``step_blind``): e_hat = e_last + J (q_cmd(t-L) - q_cmd(t_last-L)).
"""
import math
import numpy as np


class PerceptualGovernor:
    """Pixels + timestamps in, 5 joint rates out.

    * body schema: image Jacobian learned by settled single-joint babbling
      (efference copy vs retinal change); reusable across reaches (J0)
    * servo lag L identified jointly with J from the babbling data; the
      internal (wn, zeta) servo model is only a fallback when L is not
      identified (ablation ``identify_lag=False``)
    * Broyden refinement during the reach: available, OFF by default
    * perceptual integration: moving average of n frames; its group delay
      (n-1)T/2 is added to delta_eff
    * gain k = c / delta_eff,
      delta_eff = delta_obs + L + (n-1) T / 2 + h / 2   (h = frame period, or the
      longest blind interval under intermittent gaze), delta_obs measured from
      timestamps (no set_delay).  ``robosimu.margin`` shows that this places k
      on the exact sampled no-overshoot boundary (k* delta_eff = 1/e +- 1.2 %).
    """
    name = "pg"
    zero_oracle = True
    outputs_joint_velocity = True

    def __init__(self, amp=0.06, ramp=0.1, settle=0.35, lam=0.3, eps_px=0.3,
                 lr=0.2, qmax=0.6, wn=10.0, zeta=1.0, c=1.0 / math.e,
                 assumed_delay=None, broyden=False, nwin=8, T=0.01, J0=None,
                 delay_margin=1.0, identify_lag=True, L0=None, hold_aware=True):
        self.amp = amp; self.ramp = ramp; self.settle = settle; self.lam = lam
        self.eps_px = eps_px; self.lr = lr; self.qmax = qmax; self.wn = wn
        self.zeta = zeta; self.c = c; self.assumed = assumed_delay
        self.broyden = broyden; self.nwin = int(nwin); self.T = T
        self.delay_margin = delay_margin
        self.servo_tau = 2.0 * zeta / wn
        self.q_cmd = np.zeros(5); self.q_m = np.zeros(5); self.qd_m = np.zeros(5)
        self.ht = [0.0]; self.hq = [np.zeros(5)]; self.hc = [np.zeros(5)]; self.cal = []
        self.identify_lag = identify_lag; self.L = None
        self.J = np.zeros((3, 5)); self.buf = []
        self.j = 0; self.stage = "ramp"; self.t_stage = None; self.e_before = None
        self.last_qd = np.zeros(5); self.last_dq_neck = np.zeros(2)
        self.prev = None; self.info = {}; self.learned = False; self.t_learned = None
        self.delay_hist = []
        if J0 is not None:
            self.J = np.array(J0, float).copy(); self.learned = True; self.t_learned = 0.0
        if L0 is not None: self.L = float(L0)
        self.hold_aware = hold_aware; self.t_last_arr = None; self.gaps = []

    def commit(self, t, qd, dq_neck, dt):
        qd = np.asarray(qd, float)[:5]
        self.q_cmd = self.q_cmd + qd * dt
        n = 10; h = dt / n
        for _ in range(n):
            a = self.wn ** 2 * (self.q_cmd - self.q_m) + 2 * self.zeta * self.wn * (qd - self.qd_m)
            self.qd_m = self.qd_m + h * a
            self.q_m = self.q_m + h * self.qd_m
        self.ht.append(t); self.hq.append(self.q_m.copy()); self.hc.append(self.q_cmd.copy())
        if len(self.ht) > 400:
            self.ht = self.ht[-300:]; self.hq = self.hq[-300:]; self.hc = self.hc[-300:]

    def step_blind(self, t):
        """No new frame: predict the retinal error from the efference copy (forward model)."""
        if not self.learned or getattr(self, "e_obs", None) is None or self.L is None:
            return self.last_qd
        dq = self._cmd_at(t - self.L) - self._cmd_at(self.t_obs - self.L)
        e = self.e_obs + self.J @ dq
        r = float(np.linalg.norm(e))
        A = self.J @ self.J.T + self.lam ** 2 * np.eye(3)
        qd = -self.k_last * (self.J.T @ np.linalg.solve(A, e)) if r > self.eps_px else np.zeros(5)
        n = float(np.max(np.abs(qd)))
        if n > self.qmax: qd *= self.qmax / n
        self.last_qd = qd
        return qd

    def _cmd_at(self, t):
        ts = self.ht; i = int(np.searchsorted(ts, t, side="right"))
        if i <= 0: return self.hc[0]
        return self.hc[min(i, len(ts)) - 1]

    def _identify(self):
        tc = np.array([c[0] for c in self.cal]); E = np.array([c[1] for c in self.cal])
        best = None
        for L in np.arange(0.0, 0.301, 0.005):
            Q = np.array([self._cmd_at(t - L) for t in tc])
            X = Q - Q[0]; Y = E - E[0]
            Jt = np.linalg.lstsq(X, Y, rcond=None)[0]
            rr = float(np.sum((X @ Jt - Y) ** 2))
            if best is None or rr < best[0]:
                best = (rr, L, Jt.T)
        self.L = float(best[1]); self.J = best[2]

    def _qm_at(self, t):
        ts = self.ht; i = int(np.searchsorted(ts, t))
        if i <= 0: return self.hq[0]
        if i >= len(ts): return self.hq[-1]
        a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
        return (1 - a) * self.hq[i - 1] + a * self.hq[i]

    def step_obs(self, o):
        if self.t_last_arr is not None:
            g = o.t_now - self.t_last_arr
            if g > 1.5 * self.T:          # a blind interval: drop frames from the previous fixation
                self.buf = []
            self.gaps.append(g); self.gaps = self.gaps[-200:]
        self.t_last_arr = o.t_now
        self.buf.append(o.retinal_error); self.buf = self.buf[-self.nwin:]
        e = np.mean(self.buf, axis=0)
        self.delay_hist.append(max(o.delta_obs, 1e-3)); self.delay_hist = self.delay_hist[-50:]
        dl = self.assumed if self.assumed else self.delay_margin * float(np.max(self.delay_hist))
        if not self.learned:
            self.cal.append((o.t_cap, o.retinal_error.copy()))
            if self.t_stage is None:
                self.t_stage = o.t_now; self.e_before = e.copy()
            if self.stage == "ramp":
                qd = np.zeros(5); qd[self.j] = self.amp / self.ramp
                if o.t_now - self.t_stage >= self.ramp - 1e-9:
                    self.stage = "hold"; self.t_stage = o.t_now; qd = np.zeros(5)
                self.last_qd = qd; return qd
            if o.t_cap - self.t_stage >= self.settle:
                self.J[:, self.j] = (e - self.e_before) / self.amp
                self.j += 1; self.stage = "ramp"; self.t_stage = o.t_now; self.e_before = e.copy()
                if self.j >= 5:
                    self.learned = True; self.t_learned = o.t_now
                    if self.identify_lag: self._identify()
            self.last_qd = np.zeros(5); return self.last_qd
        qm = self._cmd_at(o.t_cap - self.L) if self.L is not None else self._qm_at(o.t_cap)
        if self.broyden and self.prev is not None:
            dq = qm - self.prev[0]; de = e - self.prev[1]
            if np.linalg.norm(dq) > 2e-3:
                self.J = self.J + self.lr * np.outer(de - self.J @ dq, dq) / (1e-4 + dq @ dq)
        self.prev = (qm, e.copy())
        lag = self.L if self.L is not None else self.servo_tau
        hold = max(self.gaps) if (self.hold_aware and self.gaps) else self.T
        d_eff = dl + lag + (len(self.buf) - 1) * self.T / 2.0 + hold / 2.0
        k = self.c / d_eff
        r = float(np.linalg.norm(e))
        A = self.J @ self.J.T + self.lam ** 2 * np.eye(3)
        qd = -k * (self.J.T @ np.linalg.solve(A, e)) if r > self.eps_px else np.zeros(5)
        n = float(np.max(np.abs(qd)))
        if n > self.qmax: qd *= self.qmax / n
        self.last_qd = qd
        self.e_obs = e.copy(); self.t_obs = o.t_cap; self.k_last = k; self.dl_last = dl
        self.info = {"k": k, "r_px": r, "d_eff": d_eff, "L": self.L}
        return qd
