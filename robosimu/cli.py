"""Command line interface."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from . import __version__
from .analysis import (gravity_map, point_motion, run_motor_comparison,
                       run_stereo_singularity_study)
from .benchmark import run_academic_benchmark
from .controllers import FixedCartesian, GovernorCartesian, make_controller
from .experiments import REACH, Q0, run_suite
from .kinematics import end_effector_position
from .model import load_model, validate_data, load_postures
from .scenarios import SCENARIOS, run_all_scenarios, run_scenario
from .simulator import SimConfig, simulate_reach
from .stereo import StereoCameraConfig


def _model(path):
    return load_model(path)


def _initial_q0(args, model):
    posture = getattr(args, "posture", None)
    if posture:
        postures = load_postures(args.model)
        if posture not in postures:
            raise SystemExit(
                f"unknown posture {posture!r}; available: {sorted(postures)}")
        return np.radians(postures[posture])
    q0_raw = getattr(args, "q0", None)
    if q0_raw:
        vals = [float(v) for v in str(q0_raw).replace(",", " ").split()]
        if len(vals) != model.n:
            raise SystemExit(
                f"--q0 needs {model.n} joint angles in degrees, got {len(vals)}")
        return np.radians(vals)
    return Q0.copy()


def _target(args, model, q0):
    raw = getattr(args, "target", None)
    if raw:
        vals = [float(v) for v in str(raw).replace(",", " ").split()]
        if len(vals) != 3:
            raise SystemExit("--target needs 3 numbers: x y z (metres, absolute)")
        return np.array(vals, float)
    return end_effector_position(model, q0) + REACH


def cmd_validate(args):
    data = json.loads(Path(args.model).read_text(encoding="utf-8"))
    errors = validate_data(data)
    if errors:
        print("INVALID")
        for e in errors:
            print("  -", e)
        return 1
    m = load_model(args.model)
    print("OK")
    for k, v in m.summary().items():
        print(f"  {k}: {v}")
    for i, link in enumerate(m.links):
        print(f"  joint {i+1}: torque limit {link.motor.torque_limit:.1f} Nm, "
              f"speed limit {link.motor.speed_limit:.1f} rad/s")
    return 0


def cmd_simulate(args):
    m = load_model(args.model)
    q0 = _initial_q0(args, m)
    target = _target(args, m, q0)
    ctrl = make_controller(args.controller, delay=args.delay,
                           fps=1.0 / args.control_dt, control_dt=args.control_dt,
                           eps=args.eps, vmax=args.vmax, k=args.k)
    if getattr(args, "per_axis", False) and hasattr(ctrl, "per_axis"):
        ctrl.per_axis = True
    noise = tuple(args.noise_vec) if args.noise_vec else args.noise
    st_cfg = None
    if getattr(args, "stereo_baseline", 0.0) and args.stereo_baseline > 0:
        st_cfg = StereoCameraConfig(baseline_m=args.stereo_baseline,
                                    disparity_noise_px=getattr(args, "disparity_noise", 0.7))
    cfg = SimConfig(duration=args.duration, delay=args.delay, payload=args.payload,
                    seed=args.seed, eps=args.eps, noise=noise,
                    delay_jitter=args.delay_jitter, wn=12.0, zeta=1.0,
                    max_qd=1.5, max_qdd=5.0, stereo=st_cfg,
                    singularity_avoidance=getattr(args, "singularity_safe", False))
    r = simulate_reach(m, q0, target, ctrl, cfg)
    print(json.dumps({k: (round(v, 4) if isinstance(v, float) else v)
                      for k, v in r.metrics.items()}, indent=2))
    if args.gif:
        from .render import save_gif
        save_gif(m, r, args.gif)
        print("gif:", args.gif)
    if args.figure:
        from .render import save_figure
        save_figure(m, r, args.figure)
        print("figure:", args.figure)
    return 0


def cmd_experiment(args):
    summary = run_suite(args.model, out_dir=args.out,
                        seeds=tuple(range(args.seeds)),
                        make_figures=not args.no_figures)
    print(json.dumps(summary, indent=2))
    return 0


def cmd_motor_compare(args):
    summary = run_motor_comparison(args.model, out_dir=args.out,
                                   seeds=tuple(range(args.seeds)),
                                   make_figures=not args.no_figures)
    print(json.dumps(summary, indent=2))
    return 0


def cmd_gravity_map(args):
    rows = gravity_map(args.model, out_dir=args.out, n=args.grid,
                       payloads=tuple(args.payload),
                       make_figures=not args.no_figures)
    print(json.dumps(rows, indent=2))
    return 0


def cmd_point_motion(args):
    rows = point_motion(args.model, out_dir=args.out, seed=args.seed,
                        delay=args.delay, make_figures=not args.no_figures)
    print(json.dumps(rows, indent=2))
    return 0


def cmd_html(args):
    m = load_model(args.model)
    q0 = _initial_q0(args, m)
    target = _target(args, m, q0)
    ctrl = make_controller(args.controller, delay=args.delay,
                           fps=1.0 / args.control_dt, control_dt=args.control_dt,
                           eps=args.eps, vmax=args.vmax, k=args.k)
    if getattr(ctrl, "per_axis", None) is not None:
        ctrl.per_axis = True
    st_cfg = StereoCameraConfig(baseline_m=getattr(args, "baseline", 0.065),
                                focal_px=getattr(args, "focal", 520.0),
                                disparity_noise_px=getattr(args, "disparity_noise", 0.7))
    cfg = SimConfig(duration=args.duration, delay=args.delay, payload=args.payload,
                    seed=args.seed, eps=args.eps, noise=args.noise,
                    wn=12.0, zeta=1.0, max_qd=1.5, max_qdd=5.0,
                    stereo=st_cfg, singularity_avoidance=True)
    r = simulate_reach(m, q0, target, ctrl, cfg)
    from .render import save_html
    postures = load_postures(args.model) or None
    save_html(m, r, args.out, postures=postures)
    print("html:", args.out)
    return 0


def cmd_stereo_study(args):
    summary = run_stereo_singularity_study(args.model, out_dir=args.out,
                                           seeds=tuple(range(args.seeds)),
                                           make_figures=not args.no_figures)
    print(f"stereo-study -> {args.out}/stereo_singularity.md")
    return 0


def cmd_benchmark(args):
    summary = run_academic_benchmark(args.model, out_dir=args.out,
                                     seeds=tuple(range(args.seeds)))
    print(f"benchmark ({len(summary['rows'])} rows) -> {args.out}/academic_benchmark.md")
    return 0


def cmd_scenario(args):
    sc = run_scenario(args.name, args.model, out_dir=args.out,
                      seeds=tuple(range(args.seeds)), reaches=args.reaches,
                      jobs=args.jobs, make_figures=not args.no_figures)
    print(json.dumps(sc.summary, indent=2, default=float))
    return 0


def cmd_scenarios(args):
    names = args.names or None
    if names:
        bad = [n for n in names if n not in SCENARIOS]
        if bad:
            raise SystemExit(f"unknown scenario(s): {bad}; choose from {sorted(SCENARIOS)}")
    results = run_all_scenarios(args.model, out_dir=args.out,
                                seeds=tuple(range(args.seeds)), reaches=args.reaches,
                                jobs=args.jobs, scenarios=names,
                                make_figures=not args.no_figures)
    print(f"ran {len(results)} scenarios -> {args.out}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog="robosimu", description="Six-axis arm simulator")
    ap.add_argument("--version", action="version", version=__version__)
    sub = ap.add_subparsers(dest="command", required=True)

    p = sub.add_parser("validate", help="validate a model JSON")
    p.add_argument("model")
    p.set_defaults(func=cmd_validate)

    p = sub.add_parser("simulate", help="run one reaching simulation")
    p.add_argument("model")
    p.add_argument("--controller",
                   choices=["hlsl", "hdap", "fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr"],
                   default="hlsl")
    p.add_argument("--delay", type=float, default=0.06)
    p.add_argument("--payload", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=2.5)
    p.add_argument("--control-dt", type=float, default=0.01)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--noise", type=float, default=0.0005)
    p.add_argument("--noise-vec", type=float, nargs=3, default=None,
                   help="anisotropic noise (x, y, z) in metres")
    p.add_argument("--delay-jitter", type=float, default=0.0,
                   help="std-dev of the visual delay (s)")
    p.add_argument("--eps", type=float, default=0.004)
    p.add_argument("--k", type=float, default=None, help="override the controller gain")
    p.add_argument("--vmax", type=float, default=0.6)
    p.add_argument("--stereo-baseline", type=float, default=0.0,
                   help="two-eye stereo baseline in metres (0 = direct Cartesian)")
    p.add_argument("--disparity-noise", type=float, default=0.7,
                   help="stereo disparity noise std-dev in pixels")
    p.add_argument("--per-axis", action="store_true",
                   help="enable per-axis stereo Governor speed budget")
    p.add_argument("--singularity-safe", action="store_true",
                   help="enable singularity-aware weighted joint routing")
    p.add_argument("--posture", help="named preset from the model JSON, e.g. hanging_natural")
    p.add_argument("--q0", help="initial joint angles in degrees, space separated")
    p.add_argument("--target", help="target (x y z metres, absolute); default p0 + 0.15 m forward")
    p.add_argument("--gif")
    p.add_argument("--figure")
    p.set_defaults(func=cmd_simulate)

    p = sub.add_parser("experiment", help="run the delay and payload sweeps")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=2)
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_experiment)

    p = sub.add_parser("motor-compare", help="compare DC, BLDC and stepper drives")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=2)
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_motor_compare)

    p = sub.add_parser("gravity-map", help="joint gravity torques over a pose grid")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--grid", type=int, default=21)
    p.add_argument("--payload", type=float, nargs="*", default=[0.0, 4.0])
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_gravity_map)

    p = sub.add_parser("point-motion", help="point velocities and accelerations")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--delay", type=float, default=0.12)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_point_motion)

    p = sub.add_parser("html", aliases=["workbench"],
                       help="write an interactive Stereo Visual Servoing & Singularity Workbench HTML")
    p.add_argument("model")
    p.add_argument("--out", default="figures/arm_viewer.html")
    p.add_argument("--controller",
                   choices=["hlsl", "hdap", "fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr"],
                   default="hlsl")
    p.add_argument("--delay", type=float, default=0.06)
    p.add_argument("--payload", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=2.5)
    p.add_argument("--control-dt", type=float, default=0.01)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--noise", type=float, default=0.0005)
    p.add_argument("--baseline", type=float, default=0.065)
    p.add_argument("--focal", type=float, default=520.0)
    p.add_argument("--disparity-noise", type=float, default=0.7)
    p.add_argument("--eps", type=float, default=0.004)
    p.add_argument("--k", type=float, default=None, help="override the controller gain")
    p.add_argument("--vmax", type=float, default=0.6)
    p.add_argument("--posture", help="named preset from the model JSON, e.g. hanging_natural")
    p.add_argument("--q0", help="initial joint angles in degrees, space separated")
    p.add_argument("--target", help="target (x y z metres, absolute); default p0 + 0.15 m forward")
    p.set_defaults(func=cmd_html)

    p = sub.add_parser("stereo-study",
                       help="run two-eye stereo disparity and singularity-safe reaching study")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=3)
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_stereo_study)

    p = sub.add_parser("benchmark",
                       help="run standardized 5-scenario x 6-controller academic benchmark")
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=10)
    p.set_defaults(func=cmd_benchmark)

    p = sub.add_parser("scenario", help="run one pre-defined academic scenario")
    p.add_argument("name", choices=sorted(SCENARIOS))
    p.add_argument("model")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--reaches", type=int, default=6)
    p.add_argument("--jobs", type=int, default=0, help="parallel workers (0 = auto)")
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_scenario)

    p = sub.add_parser("scenarios", help="run all (or a subset of) scenarios")
    p.add_argument("model")
    p.add_argument("names", nargs="*")
    p.add_argument("--out", default="reports")
    p.add_argument("--seeds", type=int, default=5)
    p.add_argument("--reaches", type=int, default=6)
    p.add_argument("--jobs", type=int, default=0, help="parallel workers (0 = auto)")
    p.add_argument("--no-figures", action="store_true")
    p.set_defaults(func=cmd_scenarios)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
