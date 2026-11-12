"""Forward kinematics, Jacobians and point-wise motion."""
from __future__ import annotations

import numpy as np

from .model import ArmModel


def dh_transform(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ])


def forward_kinematics(model: ArmModel, q) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Return per-link relative transforms and base-frame frames.

    ``Ts[i]`` maps frame i to frame i-1; ``frames[i]`` maps frame i to the base.
    ``frames`` has length n+1, with ``frames[0]`` the base identity.
    """
    q = np.asarray(q, float)
    T = np.eye(4)
    Ts: list[np.ndarray] = []
    frames = [T.copy()]
    for i, link in enumerate(model.links):
        Ti = dh_transform(link.a, link.alpha, link.d, q[i] + link.theta_offset)
        Ts.append(Ti)
        T = T @ Ti
        frames.append(T.copy())
    return Ts, frames


def joint_axes(frames: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Return joint origins and axes in base coordinates."""
    n = len(frames) - 1
    origins = np.zeros((n, 3))
    axes = np.zeros((n, 3))
    for i in range(n):
        origins[i] = frames[i][:3, 3]
        axes[i] = frames[i][:3, 2]
    return origins, axes


def point_position(model: ArmModel, q, link_index: int, point=None) -> np.ndarray:
    """Position in base coordinates of a point expressed in a link frame."""
    _, frames = forward_kinematics(model, q)
    p = np.zeros(3) if point is None else np.asarray(point, float)
    T = frames[link_index + 1]
    return T[:3, :3] @ p + T[:3, 3]


def end_effector_position(model: ArmModel, q) -> np.ndarray:
    _, frames = forward_kinematics(model, q)
    return frames[-1][:3, 3].copy()


def jacobian(model: ArmModel, q, link_index: int, point=None) -> np.ndarray:
    """Geometric Jacobian (6 x n) for a point on a link (0-based)."""
    q = np.asarray(q, float)
    _, frames = forward_kinematics(model, q)
    local = np.zeros(3) if point is None else np.asarray(point, float)
    T = frames[link_index + 1]
    p_point = T[:3, :3] @ local + T[:3, 3]
    origins = np.array([f[:3, 3] for f in frames])
    axes = np.array([f[:3, 2] for f in frames])
    z = axes[:link_index + 1]
    r = p_point - origins[:link_index + 1]
    J = np.zeros((6, model.n))
    J[:3, :link_index + 1] = np.cross(z, r).T
    J[3:, :link_index + 1] = z.T
    return J


def point_velocity(model: ArmModel, q, qd, link_index: int, point=None) -> tuple[np.ndarray, np.ndarray]:
    J = jacobian(model, q, link_index, point)
    qd = np.asarray(qd, float)
    return J[:3] @ qd, J[3:] @ qd


def point_acceleration(model: ArmModel, q, qd, qdd, link_index: int,
                       point=None, eps: float = 1e-6) -> tuple[np.ndarray, np.ndarray]:
    """Linear and angular acceleration; J-dot is evaluated by central differences."""
    q = np.asarray(q, float)
    qd = np.asarray(qd, float)
    qdd = np.asarray(qdd, float)
    J = jacobian(model, q, link_index, point)
    Jp = jacobian(model, q + eps * qd, link_index, point)
    Jm = jacobian(model, q - eps * qd, link_index, point)
    Jdot = (Jp - Jm) / (2.0 * eps)
    a_lin = J[:3] @ qdd + Jdot[:3] @ qd
    a_ang = J[3:] @ qdd + Jdot[3:] @ qd
    return a_lin, a_ang


def link_com_positions(model: ArmModel, q) -> list[np.ndarray]:
    _, frames = forward_kinematics(model, q)
    out = []
    for i, link in enumerate(model.links):
        T = frames[i + 1]
        out.append(T[:3, :3] @ link.com + T[:3, 3])
    return out


def link_rotations(model: ArmModel, q) -> list[np.ndarray]:
    _, frames = forward_kinematics(model, q)
    return [frames[i + 1][:3, :3].copy() for i in range(model.n)]
