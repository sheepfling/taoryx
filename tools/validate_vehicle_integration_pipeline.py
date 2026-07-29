"""Run the staged provider-neutral integration report for reference families."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_integration_pipeline import (  # noqa: E402
    VehicleIntegrationPipelineReport,
    validate_all_vehicle_integration_pipelines,
    validate_vehicle_integration_pipeline,
    write_vehicle_integration_packet,
)


def _render(report: VehicleIntegrationPipelineReport) -> str:
    lines = [
        f"{report.family_id}: {report.status} ({report.current_tier})",
        f"  next_gate: {report.next_gate}",
    ]
    for stage in report.stages:
        lines.append(f"  {stage.stage_id}: {stage.status} — {stage.claim}")
        for finding in stage.findings:
            lines.append(f"    {finding.severity.upper()} {finding.code}: {finding.message}")
    if report.blockers:
        lines.append("  blockers:")
        for finding in report.blockers:
            lines.append(f"    - {finding.code}: {finding.hint}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run staged integration diagnostics without inventing missing physics."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="supported family ID or 'all'")
    parser.add_argument("--json", type=Path, help="write a machine-readable report")
    parser.add_argument("--packet-dir", type=Path, help="write a self-contained metadata/evidence packet")
    parser.add_argument("--exclude-source-package", action="store_true", help="do not copy the pinned source package into the packet")
    parser.add_argument("--allow-blocked", action="store_true", help="report blockers but return success")
    args = parser.parse_args(argv)
    reports = (
        validate_all_vehicle_integration_pipelines()
        if args.family == "all"
        else (validate_vehicle_integration_pipeline(args.family),)
    )
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": "taoryx.vehicle-integration-pipeline/v1",
            "family": args.family,
            "reports": [report.as_dict() for report in reports],
        }
        args.json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.packet_dir:
        if args.family == "all":
            for report in reports:
                write_vehicle_integration_packet(
                    report.family_id,
                    args.packet_dir / report.family_id,
                    include_source_package=not args.exclude_source_package,
                )
        else:
            write_vehicle_integration_packet(
                args.family,
                args.packet_dir,
                include_source_package=not args.exclude_source_package,
            )
    return 0 if args.allow_blocked or all(not report.blockers for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
