"""CLI: extract hover throttle / max_thrust from a PX4 ulog hover flight.

Usage:
    geo-tuner-hover flight.ulg --mass 2.5

Then feed the reported hover throttle to geo-tuner-design, or run both at
once with --design to emit the yaml files directly.
"""

from __future__ import annotations

import argparse
import sys

from geo_tuner.core.thrust_model import analyze_hover_ulog


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Extract hover thrust operating point from a PX4 ulog")
    ap.add_argument("ulog", help="path to .ulg file of a hover flight")
    ap.add_argument("--mass", type=float, required=True, help="kg, with battery")
    ap.add_argument("--hover-speed-max", type=float, default=0.4,
                    help="m/s speed threshold for hover detection")
    ap.add_argument("--design", action="store_true",
                    help="also run gain design with defaults and write yaml")
    ap.add_argument("--out-dir", default=".")
    args = ap.parse_args(argv)

    a = analyze_hover_ulog(args.ulog, mass=args.mass,
                           hover_speed_max=args.hover_speed_max)
    print("=== Hover analysis ===")
    print(a.summary())

    if a.hover_throttle_std > 0.05:
        print("WARNING: hover throttle spread is large; fly a calmer/longer "
              "hover (low wind, steady altitude) and re-run.")

    if args.design:
        from geo_tuner.cli.design_gains import main as design_main
        print()
        return design_main(["--mass", str(args.mass),
                            "--hover-throttle", f"{a.hover_throttle:.4f}",
                            "--out-dir", args.out_dir])
    return 0


if __name__ == "__main__":
    sys.exit(main())
