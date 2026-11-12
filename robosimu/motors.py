"""Motor and drive models: torque-speed envelope, current, voltage, energy."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .model import MotorSpec


@dataclass
class MotorState:
    tau_joint: float          # torque actually delivered at the joint (Nm)
    tau_motor: float          # motor-side torque (Nm)
    current: float            # motor current (A)
    voltage: float            # terminal voltage (V)
    speed_motor: float        # motor speed (rad/s)
    p_electrical: float       # terminal electrical power (W, >=0 unless regen)
    p_mechanical: float       # joint mechanical power (W)
    p_copper: float           # copper loss (W)
    tau_limit: float          # joint-side torque limit at this speed (Nm)
    saturated: bool
    over_continuous: bool


def torque_limit(motor: MotorSpec, qd: float) -> float:
    """Joint-side torque available at joint speed ``qd`` (Nm)."""
    wm = abs(float(qd)) * motor.gear_ratio
    if motor.type == "stepper":
        wc = max(motor.no_load_speed * 0.5, 1e-6)
        tau_m = motor.stall_torque / (1.0 + (wm / wc) ** 2)
    else:
        frac = max(0.0, 1.0 - wm / max(motor.no_load_speed, 1e-9))
        tau_m = motor.stall_torque * frac
    return tau_m * motor.gear_ratio * motor.efficiency


def drive(motor: MotorSpec, tau_joint: float, qd: float, dt: float,
          regen: bool = False, prev_tau: float | None = None) -> MotorState:
    """Apply the drive chain to a requested joint torque.

    Order of limits: torque-speed envelope, driver current limit, supply
    voltage, then the driver torque (current) ramp.  ``prev_tau`` is the joint
    torque commanded at the previous step, needed for the ramp limit.
    """
    limit = torque_limit(motor, qd)
    tau_cmd = float(np.clip(tau_joint, -limit, limit))
    saturated = abs(tau_cmd - tau_joint) > 1e-12

    # driver current limit
    if motor.current_limit_a > 0:
        tau_m = tau_cmd / (motor.gear_ratio * motor.efficiency)
        current = tau_m / motor.kt
        if abs(current) > motor.current_limit_a:
            tau_m = float(np.sign(current)) * motor.current_limit_a * motor.kt
            tau_cmd = tau_m * motor.gear_ratio * motor.efficiency
            saturated = True

    # supply voltage limit
    wm = float(qd) * motor.gear_ratio
    tau_m = tau_cmd / (motor.gear_ratio * motor.efficiency)
    current = tau_m / motor.kt
    emf = motor.kt * wm
    voltage = motor.resistance * current + emf
    if abs(voltage) > motor.voltage:
        sign = 1.0 if voltage >= 0 else -1.0
        current = (sign * motor.voltage - emf) / motor.resistance
        tau_m = motor.kt * current
        tau_cmd = tau_m * motor.gear_ratio * motor.efficiency
        voltage = sign * motor.voltage
        saturated = True

    # driver torque / current ramp
    if motor.torque_ramp_nm_s > 0 and prev_tau is not None:
        max_d = motor.torque_ramp_nm_s * dt
        ramped = float(np.clip(tau_cmd, prev_tau - max_d, prev_tau + max_d))
        if abs(ramped - tau_cmd) > 1e-12:
            saturated = True
        tau_cmd = ramped
        tau_m = tau_cmd / (motor.gear_ratio * motor.efficiency)
        current = tau_m / motor.kt
        voltage = motor.resistance * current + emf

    p_elec = voltage * current
    p_mech = tau_cmd * float(qd)
    p_copper = current * current * motor.resistance
    if p_elec < 0 and not regen:
        p_elec = 0.0
    over = abs(tau_m) > motor.continuous_torque
    return MotorState(
        tau_joint=tau_cmd,
        tau_motor=tau_m,
        current=current,
        voltage=voltage,
        speed_motor=wm,
        p_electrical=p_elec,
        p_mechanical=p_mech,
        p_copper=p_copper,
        tau_limit=limit,
        saturated=saturated,
        over_continuous=over,
    )


def drive_all(motor_specs: list[MotorSpec], tau, qd, dt: float) -> list[MotorState]:
    return [drive(m, float(t), float(v), dt) for m, t, v in zip(motor_specs, tau, qd)]
