"""Run provider-neutral controller and mission preflight diagnostics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_controller_mission_preflight import (  # noqa: E402
    VehicleControllerMissionPreflightReport,
    validate_all_vehicle_controller_mission_preflight,
    validate_vehicle_controller_mission_preflight,
)


def _render(report: VehicleControllerMissionPreflightReport) -> str:
    lines = [f"{report.family_id}: {report.status}"]
    lines.append(f"  controller: {report.controller.status} ({len(report.controller.profiles_checked)} profiles)")
    lines.append(f"  mission: {report.mission.status} ({report.mission.mission_id or 'no binding'})")
    if report.mission.metrics.get("estimated_duration_s_min") is not None:
        lines.append(
            "  estimated mission duration: "
            f"{report.mission.metrics['estimated_duration_s_min']:.1f}–"
            f"{report.mission.metrics['estimated_duration_s_max']:.1f} s"
        )
    for finding in (*report.controller.findings, *report.mission.findings):
        lines.append(f"  {finding.severity.upper()} {finding.code}: {finding.message}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run preflight without solving gains or executing a vehicle."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="family ID or 'all'")
    parser.add_argument("--json", type=Path, help="write a machine-readable report")
    parser.add_argument("--allow-blocked", action="store_true", help="report blockers but return success")
    args = parser.parse_args(argv)
    reports = validate_all_vehicle_controller_mission_preflight() if args.family == "all" else (validate_vehicle_controller_mission_preflight(args.family),)
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        args.json.write_text(
            json.dumps(
                {
                    "schema_version": "taoryx.vehicle-controller-mission-preflight/v1",
                    "reports": [report.as_dict() for report in reports],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    return 0 if args.allow_blocked or all(report.status != "blocked" for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
