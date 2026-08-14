"""Bind pilot trim worklists to source evaluators and write solve evidence."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.plugins import discover_plugins  # noqa: E402
from taoryx.vehicle_trim_adapters import (  # noqa: E402
    solve_all_vehicle_trim_evidence,
    solve_vehicle_trim_evidence,
)


def main(argv: list[str] | None = None) -> int:
    """Run declared family adapters; fail when an adapter solve fails."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="family ID or 'all'")
    parser.add_argument(
        "--plugin",
        action="append",
        dest="plugin_ids",
        metavar="PLUGIN_ID",
        help="limit discovery to one plug-in ID; repeat for direct plug-in dependencies",
    )
    parser.add_argument("--json", type=Path, default=ROOT / "verification/vehicle_trim_solve_evidence.json", help="write solve evidence")
    args = parser.parse_args(argv)
    plugins = discover_plugins(selected=args.plugin_ids) if args.plugin_ids else None
    reports = (
        solve_all_vehicle_trim_evidence(plugins=plugins)
        if args.family == "all"
        else (solve_vehicle_trim_evidence(args.family, plugins=plugins),)
    )
    for report in reports:
        print(f"{report.family_id}: {report.status} ({len(report.points)} points)")
        for finding in report.findings:
            print(f"  {finding}")
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(
            {
                "schema_version": "taoryx.vehicle-trim-solve/v1",
                "reports": [report.as_dict() for report in reports],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    return 0 if reports and all(report.status == "verified" for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
