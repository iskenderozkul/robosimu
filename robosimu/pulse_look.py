"""Encoderless pulse-and-look controller (paper v6, Section 5.10).

Runs with ``SimConfig(encoderless=True)``: the plant has no joint servo and no
gravity compensation; the drive receives only a speed command and a holding
brake clamps every joint whose command is zero.
"""
import numpy as np

from .perceptual_governor import PerceptualGovernor


class PulseLook(PerceptualGovernor):
    """Encoderless 'look - pulse - brake - look' controller (the three refusals together).

    * No joint angle is read anywhere: the drive gets a speed command for a fixed pulse
      time tp and then the holding brake clamps the joint; gravity and friction decide
      how far it really moved.
    * The controller looks only while braked (n frames averaged), so delay does not
      enter the gain: the pulse is planned from a still image.
    * Each pulse corrects a fraction `frac` of the seen gap; frac plays the role of
      k*delta_eff and is kept below 1 so the sequence of gaps decays monotonically.
    * It learns the systematic drift per pulse (gravity sag) as an image-space bias b:
      b <- b + lb * (observed change - intended change - b), and pre-compensates it.
    * Optional ordinal mode: the controller is given only the *category* of the gap
      (far / near / pre-contact / contact) and its image direction, not its size.
    """
    ORD_EDGES_PX = (6.0, 20.0, 60.0)        # contact | pre-contact | near | far
    ORD_STEP_PX = (0.0, 4.0, 12.0, 36.0)    # assumed gap per category (lower edge midpoint)

    def __init__(s, *a, tp=0.12, look=0.10, frac=0.8, lb=0.5, ordinal=False, **kw):
        super().__init__(*a, **kw)
        s.tp = tp; s.look = look; s.frac = frac; s.lb = lb; s.ordinal = ordinal
        s.ph = "look"; s.t_ph = None; s.qpulse = np.zeros(5); s.b = np.zeros(3)
        s.e_before = None; s.de_int = None; s.pulses = 0

    def _percept(s, e):
        r = float(np.linalg.norm(e))
        if not s.ordinal:
            return e, r
        cat = int(np.searchsorted(s.ORD_EDGES_PX, r))           # 0..3
        u = e / max(r, 1e-9)
        return u * s.ORD_STEP_PX[cat], cat

    def step_obs(s, o):
        if not s.learned:
            return super().step_obs(o)
        if s.t_ph is None:
            s.t_ph = o.t_now; s.buf = []
        el = o.t_now - s.t_ph
        if s.ph == "pulse":
            if el >= s.tp:
                s.ph = "settle"; s.t_ph = o.t_now; s.last_qd = np.zeros(5); return s.last_qd
            s.last_qd = s.qpulse; return s.qpulse
        if s.ph == "settle":
            if o.t_cap - s.t_ph >= s.L + 0.02:
                s.ph = "look"; s.t_ph = o.t_now; s.buf = []
            s.last_qd = np.zeros(5); return s.last_qd
        s.buf.append(o.retinal_error)
        if el < s.look:
            s.last_qd = np.zeros(5); return s.last_qd
        e_raw = np.mean(s.buf, axis=0)
        e, r = s._percept(e_raw)
        if s.e_before is not None:                                  # learn the drift of the last pulse
            surprise = (e - s.e_before) - s.de_int
            s.b = s.b + s.lb * (surprise - s.b)
        stop = (r == 0) if s.ordinal else (r <= 3 * s.eps_px)
        if stop:
            s.e_before = None; s.t_ph = o.t_now; s.buf = []; s.last_qd = np.zeros(5); return s.last_qd
        A = s.J @ s.J.T + s.lam ** 2 * np.eye(3)
        dq = s.J.T @ np.linalg.solve(A, -s.frac * e - s.b)
        rate = dq / s.tp; n = float(np.max(np.abs(rate)))
        if n > s.qmax: rate *= s.qmax / n
        s.qpulse = rate; s.ph = "pulse"; s.t_ph = o.t_now; s.last_qd = rate; s.pulses += 1
        s.e_before = e; s.de_int = s.J @ (rate * s.tp); s.buf = []
        s.info = {"r": r, "pulses": s.pulses}
        return rate

