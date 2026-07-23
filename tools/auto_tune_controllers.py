"""Synthesize and report controller-profile candidates for registered vehicles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.controller_autotune import auto_tune_lqr_profiles

ROOT = Path(__file__).resolve().parents[1]
STANDARD_VEHICLES = ("b747", "skywalker_x8", "hummingbird", "x15")


def build_parser() -> argparse.ArgumentParser:
    """Build the command-line interface for the repeatable profile sweep."""

    parser = argparse.ArgumentParser(
        description=(
            "Synthesize gentle/standard/aggressive LQR profiles from the "
            "canonical vehicle registry. Default output is design evidence, "
            "not a validated maneuver claim."
        )
    )
    parser.add_argument(
        "--vehicle",
        choices=("all", *STANDARD_VEHICLES),
        default="all",
        help="vehicle family to process (default: all)",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "artifacts/controller_autotune",
        help="directory for JSON reports (default: artifacts/controller_autotune)",
    )
    return parser
    ####


def main(argv: list[str] | None = None) -> int:
    """Run the profile sweep and write one report per vehicle."""

    args = build_parser().parse_args(argv)
    vehicles = STANDARD_VEHICLES if args.vehicle == "all" else (args.vehicle,)
    args.output.mkdir(parents=True, exist_ok=True)
    reports: dict[str, dict[str, object]] = {}
    failed = False
    for vehicle_id in vehicles:
        report = auto_tune_lqr_profiles(vehicle_id)
        payload = report.as_dict()
        reports[vehicle_id] = payload
        target = args.output / f"{vehicle_id}_profile_synthesis.json"
        target.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        best = report.best
        if best is None:
            failed = True
            print(f"{vehicle_id}: no safe candidate")
        else:
            print(f"{vehicle_id}: {best.profile_id} ({report.design_source})")
    summary = {
        "schema_version": 1,
        "claim_boundary": "synthesis-only; source-trim and maneuver evidence remain required",
        "vehicles": reports,
    }
    (args.output / "summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 2 if failed else 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
