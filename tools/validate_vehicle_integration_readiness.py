"""Report source-family intake and fidelity readiness before runtime probes."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.vehicle_integration_readiness import (  # noqa: E402
    VehicleIntegrationReadinessReport,
    validate_all_vehicle_integration_readiness,
    validate_vehicle_integration_readiness,
)


def _render(report: VehicleIntegrationReadinessReport) -> str:
    lines = [f"{report.family_id}: {report.status}"]
    for finding in report.findings:
        lines.append(f"  {finding.severity.upper()} {finding.code}: {finding.message}")
    for profile in report.profiles:
        lines.append(
            f"  {profile.profile_id} ({profile.runtime_fidelity}): {profile.declared_status}; "
            f"metadata={'ready' if profile.metadata_ready else 'blocked'}; "
            f"automatic_lowering={'eligible' if profile.automatically_lowerable else 'not eligible'}"
        )
        for finding in profile.findings:
            lines.append(f"    {finding.severity.upper()} {finding.code}: {finding.message}")
    return "\n".join(lines)
    ####


def main(argv: list[str] | None = None) -> int:
    """Run the provider-neutral source-family readiness check."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--family", default="all", help="supported family ID or 'all'")
    parser.add_argument("--json", type=Path, help="write a machine-readable report")
    args = parser.parse_args(argv)
    reports = (
        validate_all_vehicle_integration_readiness()
        if args.family == "all"
        else (validate_vehicle_integration_readiness(args.family),)
    )
    for report in reports:
        print(_render(report))
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": "taoryx.vehicle-integration-readiness/v1",
            "family": args.family,
            "reports": [report.as_dict() for report in reports],
        }
        args.json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if all(not report.errors for report in reports) else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
