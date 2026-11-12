"""Model definition and validation for the six-axis arm."""
from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

REQUIRED_LINK = ["a", "alpha_deg", "d", "mass", "com", "inertia", "limits_deg", "motor"]
REQUIRED_MOTOR = ["type", "gear_ratio", "efficiency", "kt", "resistance", "voltage",
                  "stall_torque", "no_load_speed", "continuous_torque"]


@dataclass
class MotorSpec:
    type: str
    gear_ratio: float
    efficiency: float
    kt: float
    resistance: float
    voltage: float
    stall_torque: float
    no_load_speed: float
    continuous_torque: float
    rotor_inertia: float = 0.0
    viscous_friction: float = 0.0
    coulomb_friction: float = 0.0
    current_limit_a: float = 0.0       # driver current limit (A); 0 = unlimited
    torque_ramp_nm_s: float = 0.0      # max joint torque rate (Nm/s); 0 = unlimited
    brake_torque_nm: float = 0.0       # holding brake capacity (Nm); 0 = none

    @property
    def torque_limit(self) -> float:
        """Joint-side stall torque (Nm)."""
        return self.gear_ratio * self.efficiency * self.stall_torque

    @property
    def speed_limit(self) -> float:
        """Joint-side no-load speed (rad/s)."""
        return self.no_load_speed / self.gear_ratio

    @property
    def continuous_torque_joint(self) -> float:
        return self.gear_ratio * self.efficiency * self.continuous_torque


@dataclass
class Link:
    name: str
    a: float
    alpha: float
    d: float
    theta_offset: float
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    limits: tuple[float, float]
    motor: MotorSpec
    viscous_friction: float = 0.0
    coulomb_friction: float = 0.0

    @property
    def inertia_matrix(self) -> np.ndarray:
        return np.diag(self.inertia)


@dataclass
class ArmModel:
    name: str
    links: list[Link]
    gravity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, -9.81]))
    payload_mass: float = 0.0
    payload_com: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def n(self) -> int:
        return len(self.links)

    def with_payload(self, mass: float, com: np.ndarray | None = None) -> "ArmModel":
        """Return a copy whose last link carries an extra point mass."""
        if mass <= 0:
            return self
        m = copy.deepcopy(self)
        point = m.payload_com if com is None else np.asarray(com, float)
        last = m.links[-1]
        total = last.mass + mass
        c = (last.mass * last.com + mass * point) / total
        I = last.inertia_matrix.copy()
        for mi, ci in ((last.mass, last.com), (mass, point)):
            r = ci - c
            I += mi * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
        last.mass = total
        last.com = c
        last.inertia = np.diag(I).copy()
        m.payload_mass = mass
        m.payload_com = point
        return m

    def summary(self) -> dict:
        return {
            "name": self.name,
            "n_joints": self.n,
            "reach_m": float(sum(abs(l.a) for l in self.links) + sum(abs(l.d) for l in self.links)),
            "total_mass_kg": float(sum(l.mass for l in self.links)),
            "gravity": self.gravity.tolist(),
        }


def _motor_from_dict(d: dict, where: str) -> MotorSpec:
    missing = [k for k in REQUIRED_MOTOR if k not in d]
    if missing:
        raise ValueError(f"{where}: missing motor fields {missing}")
    return MotorSpec(
        type=str(d["type"]).lower(),
        gear_ratio=float(d["gear_ratio"]),
        efficiency=float(d["efficiency"]),
        kt=float(d["kt"]),
        resistance=float(d["resistance"]),
        voltage=float(d["voltage"]),
        stall_torque=float(d["stall_torque"]),
        no_load_speed=float(d["no_load_speed"]),
        continuous_torque=float(d["continuous_torque"]),
        rotor_inertia=float(d.get("rotor_inertia", 0.0)),
        viscous_friction=float(d.get("viscous_friction", 0.0)),
        coulomb_friction=float(d.get("coulomb_friction", 0.0)),
        current_limit_a=float(d.get("current_limit_a", 0.0)),
        torque_ramp_nm_s=float(d.get("torque_ramp_nm_s", 0.0)),
        brake_torque_nm=float(d.get("brake_torque_nm", 0.0)),
    )


def validate_data(data: dict) -> list[str]:
    errors: list[str] = []
    if "links" not in data or not data["links"]:
        return ["model has no links"]
    for i, raw in enumerate(data["links"]):
        where = f"link[{i}]"
        missing = [k for k in REQUIRED_LINK if k not in raw]
        if missing:
            errors.append(f"{where}: missing {missing}")
            continue
        if len(raw["com"]) != 3 or len(raw["inertia"]) != 3:
            errors.append(f"{where}: com and inertia must have 3 entries")
        if raw["mass"] <= 0:
            errors.append(f"{where}: mass must be positive")
        if min(raw["inertia"]) <= 0:
            errors.append(f"{where}: inertia entries must be positive")
        if raw["limits_deg"][0] >= raw["limits_deg"][1]:
            errors.append(f"{where}: lower limit must be below upper limit")
        try:
            motor = _motor_from_dict(raw["motor"], where)
        except ValueError as exc:
            errors.append(str(exc))
            continue
        if motor.gear_ratio <= 0 or motor.efficiency <= 0 or motor.efficiency > 1:
            errors.append(f"{where}: gear_ratio>0 and 0<efficiency<=1 required")
        if motor.kt <= 0 or motor.resistance <= 0 or motor.voltage <= 0:
            errors.append(f"{where}: kt, resistance and voltage must be positive")
        if motor.type not in ("dc", "bldc", "stepper"):
            errors.append(f"{where}: motor type must be dc, bldc or stepper")
        if motor.current_limit_a < 0 or motor.torque_ramp_nm_s < 0 or motor.brake_torque_nm < 0:
            errors.append(f"{where}: current_limit_a, torque_ramp_nm_s and brake_torque_nm "
                         "must be non-negative")
    return errors


def model_from_dict(data: dict) -> ArmModel:
    errors = validate_data(data)
    if errors:
        raise ValueError("invalid model:\n  " + "\n  ".join(errors))
    links = []
    for i, raw in enumerate(data["links"]):
        motor = _motor_from_dict(raw["motor"], f"link[{i}]")
        links.append(Link(
            name=str(raw.get("name", f"link{i+1}")),
            a=float(raw["a"]),
            alpha=math.radians(float(raw["alpha_deg"])),
            d=float(raw["d"]),
            theta_offset=math.radians(float(raw.get("theta_offset_deg", 0.0))),
            mass=float(raw["mass"]),
            com=np.asarray(raw["com"], float),
            inertia=np.asarray(raw["inertia"], float),
            limits=(math.radians(raw["limits_deg"][0]), math.radians(raw["limits_deg"][1])),
            motor=motor,
            viscous_friction=float(raw.get("viscous_friction", 0.0)),
            coulomb_friction=float(raw.get("coulomb_friction", 0.0)),
        ))
    payload = data.get("payload", {})
    return ArmModel(
        name=str(data.get("name", "arm")),
        links=links,
        gravity=np.asarray(data.get("gravity", [0, 0, -9.81]), float),
        payload_mass=float(payload.get("mass", 0.0)),
        payload_com=np.asarray(payload.get("com", [0, 0, 0]), float),
    )


def load_model(path: str | Path, payload_mass: float | None = None) -> ArmModel:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    model = model_from_dict(data)
    if payload_mass is not None:
        model = model.with_payload(payload_mass)
    return model


def save_model(model: ArmModel, path: str | Path) -> None:
    data = {
        "name": model.name,
        "gravity": model.gravity.tolist(),
        "payload": {"mass": model.payload_mass, "com": model.payload_com.tolist()},
        "links": [
            {
                "name": l.name,
                "a": l.a,
                "alpha_deg": math.degrees(l.alpha),
                "d": l.d,
                "theta_offset_deg": math.degrees(l.theta_offset),
                "mass": l.mass,
                "com": l.com.tolist(),
                "inertia": l.inertia.tolist(),
                "limits_deg": [math.degrees(l.limits[0]), math.degrees(l.limits[1])],
                "viscous_friction": l.viscous_friction,
                "coulomb_friction": l.coulomb_friction,
                "motor": {
                    "type": l.motor.type,
                    "gear_ratio": l.motor.gear_ratio,
                    "efficiency": l.motor.efficiency,
                    "kt": l.motor.kt,
                    "resistance": l.motor.resistance,
                    "voltage": l.motor.voltage,
                    "stall_torque": l.motor.stall_torque,
                    "no_load_speed": l.motor.no_load_speed,
                    "continuous_torque": l.motor.continuous_torque,
                    "rotor_inertia": l.motor.rotor_inertia,
                    "viscous_friction": l.motor.viscous_friction,
                    "coulomb_friction": l.motor.coulomb_friction,
                    "current_limit_a": l.motor.current_limit_a,
                    "torque_ramp_nm_s": l.motor.torque_ramp_nm_s,
                    "brake_torque_nm": l.motor.brake_torque_nm,
                },
            }
            for l in model.links
        ],
    }
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")
