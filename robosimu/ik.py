"""Damped-least-squares inverse kinematics (position only)."""
from __future__ import annotations

import numpy as np

from .kinematics import end_effector_position, jacobian
from .model import ArmModel


def inverse_kinematics(model: ArmModel, target, q0, iters: int = 80,
                       damping: float = 0.02, tol: float = 1e-5,
                       max_step: float = 0.2) -> tuple[np.ndarray, float]:
    q = np.asarray(q0, float).copy()
    target = np.asarray(target, float)
    err = float(np.linalg.norm(target - end_effector_position(model, q)))
    for _ in range(iters):
        p = end_effector_position(model, q)
        e = target - p
        err = float(np.linalg.norm(e))
        if err < tol:
            break
        J = jacobian(model, q, model.n - 1)[:3]
        A = J @ J.T + (damping ** 2) * np.eye(3)
        dq = J.T @ np.linalg.solve(A, e)
        n = float(np.max(np.abs(dq)))
        if n > max_step:
            dq *= max_step / n
        q = q + dq
        for i, link in enumerate(model.links):
            q[i] = float(np.clip(q[i], link.limits[0], link.limits[1]))
    return q, err
