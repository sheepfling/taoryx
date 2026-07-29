"""Run the A320 and NESC automatic-integration pilot diagnostics."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_integration_pilot import (  # noqa: E402
    PILOT_FAMILIES,
    VehicleIntegrationPilotReport,
    validate_all_vehicle_integration_pilots,
    validate_vehicle_integration_pilot,
    write_vehicle_integration_packet,
)


def _render(report: VehicleIntegrationPilotReport) -> str:
    lines = [
        f"{report.family_id}: {report.status} ({report.current_tier})",
        f"  manifest: {report.manifest_kind}; qualification: {report.qualification_class}",
        f"  next_gate: {report.next_gate}",
    ]
    for stage in report.stages:
        lines.append(f"  {stage.stage_id}: {stage.status} — {stage.claim}")
        for finding in stage.findings:
            lines.append(f"    {finding.severity.upper()} {finding.code}: {finding.message}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run collection/reference pilot diagnostics without inventing physics."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", choices=("all", *PILOT_FAMILIES))
    parser.add_argument("--json", type=Path, help="write a machine-readable report")
    parser.add_argument("--packet-dir", type=Path, help="write collection-aware packet(s) rooted at this directory")
    parser.add_argument("--allow-blocked", action="store_true", help="report blockers but return success")
    args = parser.parse_args(argv)
    reports = validate_all_vehicle_integration_pilots() if args.family == "all" else (validate_vehicle_integration_pilot(args.family),)
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": "taoryx.vehicle-integration-pilot/v1",
            "family": args.family,
            "reports": [report.as_dict() for report in reports],
        }
        args.json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.packet_dir:
        for report in reports:
            packet = write_vehicle_integration_packet(report.family_id, args.packet_dir, report=report)
            print(f"  packet: {packet}")
    return 0 if args.allow_blocked or all(not report.blockers for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
