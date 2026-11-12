"""Rigid-body dynamics via link Jacobians.

Inverse dynamics is assembled from the virtual work of each link:

    tau = sum_i ( Jv_i^T m_i (a_ci - g) + Jw_i^T (I_i alpha_i + w_i x I_i w_i) )

with all quantities expressed in the base frame.  This is equivalent to a
recursive Newton-Euler pass and avoids DH-convention ambiguity; the price is
one Jacobian per link, which is cheap for six axes.
"""
from __future__ import annotations

import numpy as np

from .kinematics import forward_kinematics, link_com_positions, point_velocity
from .model import ArmModel


def _link_jacobians(model: ArmModel, q, frames) -> list[np.ndarray]:
    """6 x n Jacobians of each link's centre of mass, in base coordinates."""
    n = model.n
    origins = np.array([f[:3, 3] for f in frames])
    axes = np.array([f[:3, 2] for f in frames])
    out = []
    for i, link in enumerate(model.links):
        T = frames[i + 1]
        p_point = T[:3, :3] @ link.com + T[:3, 3]
        z = axes[:i + 1]
        r = p_point - origins[:i + 1]
        J = np.zeros((6, n))
        J[:3, :i + 1] = np.cross(z, r).T
        J[3:, :i + 1] = z.T
        out.append(J)
    return out


def rnea(model: ArmModel, q, qd, qdd, eps: float = 1e-6) -> np.ndarray:
    """Inverse dynamics: joint torques for (q, qd, qdd), including gravity."""
    q = np.asarray(q, float)
    qd = np.asarray(qd, float)
    qdd = np.asarray(qdd, float)
    if q.shape != (model.n,) or qd.shape != (model.n,) or qdd.shape != (model.n,):
        raise ValueError("q, qd and qdd must have one entry per joint")
    _, frames = forward_kinematics(model, q)
    _, frames_p = forward_kinematics(model, q + eps * qd)
    _, frames_m = forward_kinematics(model, q - eps * qd)
    J0 = _link_jacobians(model, q, frames)
    Jp = _link_jacobians(model, q, frames_p)
    Jm = _link_jacobians(model, q, frames_m)

    tau = np.zeros(model.n)
    for i, link in enumerate(model.links):
        J = J0[i]
        Jdot = (Jp[i] - Jm[i]) / (2.0 * eps)
        a_lin = J[:3] @ qdd + Jdot[:3] @ qd
        a_ang = J[3:] @ qdd + Jdot[3:] @ qd
        w = J[3:] @ qd
        R = frames[i + 1][:3, :3]
        I = R @ link.inertia_matrix @ R.T
        F = link.mass * (a_lin - model.gravity)
        N = I @ a_ang + np.cross(w, I @ w)
        tau += J[:3].T @ F + J[3:].T @ N
    return tau


def gravity_torque(model: ArmModel, q) -> np.ndarray:
    return rnea(model, q, np.zeros(model.n), np.zeros(model.n))


def mass_matrix(model: ArmModel, q, eps: float = 1e-6) -> np.ndarray:
    """Mass matrix from the link Jacobians: M = sum m Jv' Jv + Jw' I Jw.

    Equivalent to the coefficient of ``qdd`` in :func:`rnea`, but a single
    pass instead of one inverse-dynamics call per joint.
    """
    _, frames = forward_kinematics(model, q)
    Js = _link_jacobians(model, q, frames)
    n = model.n
    M = np.zeros((n, n))
    for i, link in enumerate(model.links):
        J = Js[i]
        R = frames[i + 1][:3, :3]
        I = R @ link.inertia_matrix @ R.T
        M += link.mass * (J[:3].T @ J[:3]) + J[3:].T @ I @ J[3:]
    return 0.5 * (M + M.T)


def bias_torque(model: ArmModel, q, qd) -> np.ndarray:
    return rnea(model, q, qd, np.zeros(model.n))


def friction_torque(model: ArmModel, qd, eps: float = 0.01) -> np.ndarray:
    qd = np.asarray(qd, float)
    out = np.zeros(model.n)
    for i, link in enumerate(model.links):
        out[i] = link.viscous_friction * qd[i] + link.coulomb_friction * np.tanh(qd[i] / eps)
    return out


def forward_dynamics(model: ArmModel, q, qd, tau, with_friction: bool = True) -> np.ndarray:
    M = mass_matrix(model, q)
    rhs = np.asarray(tau, float) - bias_torque(model, q, qd)
    if with_friction:
        rhs -= friction_torque(model, qd)
    return np.linalg.solve(M, rhs)


def potential_energy(model: ArmModel, q) -> float:
    pts = link_com_positions(model, q)
    return float(-sum(link.mass * model.gravity @ p for link, p in zip(model.links, pts)))


def kinetic_energy(model: ArmModel, q, qd) -> float:
    qd = np.asarray(qd, float)
    _, frames = forward_kinematics(model, q)
    total = 0.0
    for i, link in enumerate(model.links):
        J = _link_jacobians(model, q, frames)[i]
        v = J[:3] @ qd
        w = J[3:] @ qd
        R = frames[i + 1][:3, :3]
        I = R @ link.inertia_matrix @ R.T
        total += 0.5 * link.mass * float(v @ v) + 0.5 * float(w @ I @ w)
    return total


def total_energy(model: ArmModel, q, qd) -> float:
    return potential_energy(model, q) + kinetic_energy(model, q, qd)


def numeric_gravity_torque(model: ArmModel, q, eps: float = 1e-6) -> np.ndarray:
    q = np.asarray(q, float)
    out = np.zeros(model.n)
    for j in range(model.n):
        dq = np.zeros(model.n)
        dq[j] = eps
        out[j] = (potential_energy(model, q + dq) - potential_energy(model, q - dq)) / (2 * eps)
    return out
