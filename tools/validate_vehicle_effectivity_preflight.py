"""Validate numeric effectivity and downstream allocation overlays."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_effectivity_preflight import (  # noqa: E402
    VehicleEffectivityPreflightReport,
    validate_all_vehicle_effectivity_preflight,
    validate_vehicle_effectivity_preflight,
)


def _render(report: VehicleEffectivityPreflightReport) -> str:
    lines = [f"{report.family_id}: {report.status}"]
    if report.metrics:
        lines.append(f"  metrics: {json.dumps(dict(report.metrics), sort_keys=True)}")
    for evidence in report.evidence:
        lines.append(f"  evidence: {evidence}")
    for finding in report.findings:
        lines.append(f"  {finding.severity.upper()} {finding.code}: {finding.message}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run effectivity preflight without claiming controller qualification."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="family ID or 'all'")
    parser.add_argument("--json", type=Path, default=ROOT / "verification/vehicle_effectivity_preflight.json")
    parser.add_argument("--allow-blocked", action="store_true", help="report blockers but return success")
    args = parser.parse_args(argv)
    reports = validate_all_vehicle_effectivity_preflight() if args.family == "all" else (validate_vehicle_effectivity_preflight(args.family),)
    for report in reports:
        print(_render(report))
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(
        json.dumps(
            {
                "schema_version": "taoryx.vehicle-effectivity-preflight/v1",
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
