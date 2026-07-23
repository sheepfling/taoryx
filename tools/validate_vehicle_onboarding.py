"""Validate the complete metadata path for one or all registered vehicles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.vehicle_onboarding import VehicleOnboardingReport, validate_all_vehicle_onboarding, validate_vehicle_onboarding


def _render(report: VehicleOnboardingReport) -> str:
    """Render one onboarding report as concise actionable text."""

    vehicle_id = report.vehicle_id
    status = report.status
    lines = [f"{vehicle_id}: {status}"]
    for finding in report.findings:
        lines.append(f"  {finding.severity.upper()} {finding.code} [{finding.path}]")
        lines.append(f"    {finding.message}")
        lines.append(f"    hint: {finding.hint}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run the onboarding validator and return a CI-friendly status."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--vehicle",
        default="all",
        help="registered vehicle ID to inspect, or 'all' (default: all)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="treat explicit warnings such as source-only trims as blocking",
    )
    parser.add_argument("--json", type=Path, help="write the complete report to a JSON file")
    args = parser.parse_args(argv)

    reports = validate_all_vehicle_onboarding() if args.vehicle == "all" else (validate_vehicle_onboarding(args.vehicle),)
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "strict": bool(args.strict),
            "status": "pass" if all(report.ready(strict=args.strict) for report in reports) else "fail",
            "vehicles": [report.as_dict() for report in reports],
        }
        args.json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if all(report.ready(strict=args.strict) for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
