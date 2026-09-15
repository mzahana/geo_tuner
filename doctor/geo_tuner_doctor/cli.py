"""geo-tuner-doctor command line (phase 1: discovery + inventory).

    geo-tuner-doctor ~/src/ihunter_logs --latest
    geo-tuner-doctor ~/src/ihunter_logs --list
    geo-tuner-doctor ~/src/ihunter_logs --all
    geo-tuner-doctor path/to/geo_tuner_report_....yaml
    geo-tuner-doctor path/to/sim_session_dir/

Diagnostic only: reads fetched files, writes only under --out (default
<logs>/doctor/<session>/). It never needs the drone to be reachable.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from . import __version__, timeutil
from .checks import build_context, run_checks
from .discover import discover
from .findings import banner
from .inventory import render_inventory
from .recommend import collect
from .render import render_json, render_markdown
from .session import load_session


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="geo-tuner-doctor",
        description="Post-flight analysis of a geo_tuner field session (diagnostic only).",
    )
    ap.add_argument("target", type=Path,
                    help="logs tree (e.g. ~/src/ihunter_logs), a report yaml, "
                         "or a sim session directory")
    ap.add_argument("--latest", action="store_true",
                    help="analyse only the most recent session in the tree")
    ap.add_argument("--all", action="store_true",
                    help="analyse every session found in the tree")
    ap.add_argument("--list", action="store_true",
                    help="list the sessions found and exit")
    ap.add_argument("--out", type=Path, default=None,
                    help="output directory (default: <logs tree>/doctor/<session>)")
    ap.add_argument("--no-bag", action="store_true",
                    help="skip reading the bag (fast; bag checks are skipped)")
    ap.add_argument("--no-plots", action="store_true",
                    help="skip writing plots/*.png")
    ap.add_argument("--utc-offset", metavar="+HH:MM", default=None,
                    help="the VEHICLE's UTC offset (e.g. +03:00), when this "
                         "machine's timezone differs from it; local report "
                         "stamps are interpreted in that offset when "
                         "matching the UTC-named log and bag")
    ap.add_argument("--history", nargs="?", const="", default=None,
                    metavar="DIR", type=str,
                    help="append the cross-session trend table (legacy "
                         "sessions re-identified from their bags); DIR "
                         "defaults to the logs tree being analysed")
    ap.add_argument("--version", action="version", version=__version__)
    args = ap.parse_args(argv)

    if args.utc_offset:
        import re as _re
        import time as _time
        m = _re.fullmatch(r"([+-])(\d{2}):(\d{2})", args.utc_offset)
        if not m:
            print("error: --utc-offset must look like +03:00", file=sys.stderr)
            return 2
        # POSIX TZ inverts the sign: TZ=LT-3 means UTC+3. Every loader takes
        # its default tz from the process-local clock, so setting it here
        # covers them all.
        sign = "-" if m.group(1) == "+" else "+"
        import os as _os
        _os.environ["TZ"] = f"LT{sign}{int(m.group(2))}:{m.group(3)}"
        _time.tzset()

    try:
        sessions = discover(args.target, latest=args.latest)
    except FileNotFoundError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if not sessions:
        print(f"error: no sessions found under {args.target}", file=sys.stderr)
        return 2

    if args.list:
        for art in sessions:
            marks = " ".join(
                lbl for lbl, v in [("csv", art.csv), ("episodes", art.episodes_dir),
                                   ("log", art.log), ("bag", art.bag)] if v)
            print(f"{art.name:<28} start {timeutil.fmt_both_clocks(art.start):<32} "
                  f"[{marks or 'report only'}]")
        return 0

    tree_mode = args.target.expanduser().is_dir() and not (
        args.target.expanduser() / "report.yaml").is_file()
    if tree_mode and not (args.latest or args.all):
        print("error: a logs tree needs --latest, --all or --list "
              "(or point at one report)", file=sys.stderr)
        return 2

    history_rows = history_findings = history_md = None
    if args.history is not None:
        from .history import build_history, gains_timeline, render_history_md
        hist_root = Path(args.history).expanduser() if args.history else args.target.expanduser()
        if not (hist_root / "mav_controllers_config").is_dir():
            print(f"warning: --history {hist_root} is not a logs tree; skipped",
                  file=sys.stderr)
        else:
            cache = hist_root / "doctor" / "history_cache"
            history_rows, history_findings = build_history(hist_root, cache)
            history_md = render_history_md(history_rows, gains_timeline(hist_root))

    failures = 0
    for art in sessions:
        if args.out is not None:
            out_dir = args.out if len(sessions) == 1 else args.out / art.name
        elif art.layout == "logs_tree":
            out_dir = art.report.parent.parent / "doctor" / art.name
        else:
            out_dir = art.report.parent / "doctor"
        out_dir.mkdir(parents=True, exist_ok=True)

        s = load_session(art, cache_dir=out_dir / "cache",
                         read_bag=not args.no_bag)
        text = render_inventory(s)
        (out_dir / "inventory.txt").write_text(text + "\n")

        ctx = build_context(s)
        findings = run_checks(s, ctx)
        if history_findings:
            findings.extend(history_findings)
        recs = collect(findings)
        plot_names: list[str] = []
        if not args.no_plots:
            from .plots import render_plots
            try:
                plot_names = render_plots(s, ctx, out_dir / "plots")
            except Exception as e:  # plots are decoration, never a failure
                print(f"  (plots skipped: {type(e).__name__}: {e})")
        (out_dir / "report.md").write_text(
            render_markdown(s, findings, recs, plot_names, history_md))
        (out_dir / "findings.json").write_text(render_json(s, findings, recs))

        verdict, why = banner(findings)
        print(f"{s.name}: {verdict} — {why}")
        for f in findings:
            if f.severity in ("FAIL", "WARN", "INFO"):
                ax = f" [{f.axis}]" if f.axis else ""
                print(f"  {f.severity:<4} {f.id}{ax}: {f.title}")
        print(f"  report: {out_dir / 'report.md'}")
        print()
        if s.load_errors:
            failures += 1
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
