"""robosimu — dependency-light six-axis arm simulator."""
from .model import ArmModel, Link, MotorSpec, load_model, model_from_dict, validate_data
from .kinematics import (dh_transform, end_effector_position, forward_kinematics,
                         jacobian, point_acceleration, point_position, point_velocity)
from .dynamics import (bias_torque, forward_dynamics, gravity_torque, kinetic_energy,
                       mass_matrix, potential_energy, rnea, total_energy)
from .motors import MotorState, drive, torque_limit
from .ik import inverse_kinematics
from .controllers import (AdaptiveCartesian, BroydenCartesian, FixedCartesian,
                          GovernorCartesian, LqrCartesian, PidCartesian,
                          SmithCartesian, make_controller)
from .simulator import SimConfig, ReachResult, simulate_reach

__version__ = "0.1.0"

__all__ = [
    "ArmModel", "Link", "MotorSpec", "load_model", "model_from_dict", "validate_data",
    "dh_transform", "end_effector_position", "forward_kinematics", "jacobian",
    "point_acceleration", "point_position", "point_velocity",
    "bias_torque", "forward_dynamics", "gravity_torque", "kinetic_energy",
    "mass_matrix", "potential_energy", "rnea", "total_energy",
    "MotorState", "drive", "torque_limit",
    "inverse_kinematics", "FixedCartesian", "GovernorCartesian",
    "AdaptiveCartesian", "SmithCartesian", "BroydenCartesian", "PidCartesian",
    "LqrCartesian",
    "make_controller",
    "SimConfig", "ReachResult", "simulate_reach", "__version__",
]
