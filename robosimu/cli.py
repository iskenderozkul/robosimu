"""Command line interface."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from . import __version__
from .analysis import gravity_map, point_motion, run_motor_comparison
from .controllers import FixedCartesian, GovernorCartesian, make_controller
from .experiments import REACH, Q0, run_suite
from .kinematics import end_effector_position
from .model import load_model, validate_data
from .scenarios import SCENARIOS, run_all_scenarios, run_scenario
from .simulator import SimConfig, simulate_reach


def _model(path):
    return load_model(path)


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
    q0 = Q0.copy()
    target = end_effector_position(m, q0) + REACH
    ctrl = make_controller(args.controller, delay=args.delay,
                           fps=1.0 / args.control_dt, control_dt=args.control_dt,
                           eps=args.eps, vmax=args.vmax, k=args.k)
    noise = tuple(args.noise_vec) if args.noise_vec else args.noise
    cfg = SimConfig(duration=args.duration, delay=args.delay, payload=args.payload,
                    seed=args.seed, eps=args.eps, noise=noise,
                    delay_jitter=args.delay_jitter)
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
    q0 = Q0.copy()
    target = end_effector_position(m, q0) + REACH
    ctrl = make_controller(args.controller, delay=args.delay,
                           fps=1.0 / args.control_dt, control_dt=args.control_dt,
                           eps=args.eps, vmax=args.vmax, k=args.k)
    cfg = SimConfig(duration=args.duration, delay=args.delay, payload=args.payload,
                    seed=args.seed, eps=args.eps, noise=args.noise)
    r = simulate_reach(m, q0, target, ctrl, cfg)
    from .render import save_html
    save_html(m, r, args.out)
    print("html:", args.out)
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
                   choices=["fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr"],
                   default="governor")
    p.add_argument("--delay", type=float, default=0.06)
    p.add_argument("--payload", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=2.5)
    p.add_argument("--control-dt", type=float, default=0.01)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--noise", type=float, default=0.0005)
    p.add_argument("--eps", type=float, default=0.004)
    p.add_argument("--k", type=float, default=None, help="override the controller gain")
    p.add_argument("--vmax", type=float, default=0.6)
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

    p = sub.add_parser("html", help="write an interactive HTML viewer")
    p.add_argument("model")
    p.add_argument("--out", default="figures/arm_viewer.html")
    p.add_argument("--controller",
                   choices=["fixed", "governor", "adaptive", "smith", "broyden", "pid", "lqr"],
                   default="governor")
    p.add_argument("--delay", type=float, default=0.06)
    p.add_argument("--payload", type=float, default=0.0)
    p.add_argument("--duration", type=float, default=2.5)
    p.add_argument("--control-dt", type=float, default=0.01)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--noise", type=float, default=0.0005)
    p.add_argument("--eps", type=float, default=0.004)
    p.add_argument("--k", type=float, default=None, help="override the controller gain")
    p.add_argument("--vmax", type=float, default=0.6)
    p.set_defaults(func=cmd_html)

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
