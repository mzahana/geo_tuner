"""CLI: compute geometric controller gains from vehicle characteristics.

Usage:
    geo-tuner-design --mass 2.5 --hover-throttle 0.45 \
        [--attctrl-tau 0.3] [--zeta 0.95] [--latency 0.08] [--wn 1.6] \
        [--out-dir config_out]

Writes geometric_controller.yaml and geometric_mavros.yaml, prints the
design summary with every constraint that shaped the result.
"""

from __future__ import annotations

import argparse
import os
import sys

import yaml

from geo_tuner.core.gain_design import (
    LoopShape, VehicleParams, design_gains,
    geometric_controller_yaml, geometric_mavros_yaml)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Design geometric controller gains from vehicle data")
    ap.add_argument("--mass", type=float, required=True, help="kg, with battery")
    ap.add_argument("--hover-throttle", type=float, required=True,
                    help="normalized hover throttle from geo-tuner-hover / PX4 log")
    ap.add_argument("--attctrl-tau", type=float, default=0.3)
    ap.add_argument("--zeta", type=float, default=0.95)
    ap.add_argument("--latency", type=float, default=0.08,
                    help="estimated EKF+mavros+offboard latency [s]")
    ap.add_argument("--separation", type=float, default=4.0,
                    help="attitude-BW / position-BW timescale ratio")
    ap.add_argument("--wn", type=float, default=None,
                    help="requested position-loop wn [rad/s]; default = max allowed")
    ap.add_argument("--max-tilt", type=float, default=0.52, help="rad")
    ap.add_argument("--max-accel", type=float, default=5.0, help="m/s^2")
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args(argv)

    if not (0.0 < args.hover_throttle < 1.0):
        ap.error("--hover-throttle must be in (0, 1)")

    vehicle = VehicleParams(mass=args.mass, hover_throttle=args.hover_throttle,
                            max_tilt_angle=args.max_tilt)
    shape = LoopShape(attctrl_tau=args.attctrl_tau, zeta=args.zeta,
                      latency=args.latency,
                      timescale_separation=args.separation)
    g = design_gains(vehicle, shape, wn_request=args.wn)

    print("=== Gain design summary ===")
    print(f"mass            : {args.mass:.3f} kg")
    print(f"hover throttle  : {args.hover_throttle:.3f}")
    print(f"max_thrust      : {g.max_thrust:.2f} N")
    print(f"accel headroom  : +{vehicle.vertical_accel_headroom:.1f} m/s^2 vertical, "
          f"{vehicle.lateral_accel_max:.1f} m/s^2 lateral @ tilt limit")
    print(f"attitude BW     : {shape.attitude_bandwidth:.1f} rad/s (attctrl_tau={args.attctrl_tau})")
    print(f"position wn     : xy={g.wn_xy:.2f}  z={g.wn_z:.2f} rad/s   zeta={g.zeta}")
    print(f"kx              : {g.kx}")
    print(f"kv              : {g.kv}")
    for n in g.notes:
        print(f"NOTE: {n}")

    os.makedirs(args.out_dir, exist_ok=True)
    ctrl_path = os.path.join(args.out_dir, "geometric_controller.yaml")
    mav_path = os.path.join(args.out_dir, "geometric_mavros.yaml")
    with open(ctrl_path, "w") as f:
        yaml.safe_dump(geometric_controller_yaml(
            g, mass=args.mass, max_tilt_angle=args.max_tilt,
            max_accel=args.max_accel), f, sort_keys=False)
    with open(mav_path, "w") as f:
        yaml.safe_dump(geometric_mavros_yaml(g), f, sort_keys=False)
    print(f"\nwrote {ctrl_path}\nwrote {mav_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
