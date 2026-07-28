"""Report declared data readiness for one vehicle across fidelity tiers."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from taoryx.fidelity_readiness import FidelityReadinessReport, validate_all_fidelity_readiness
from taoryx.vehicle_registry import load_vehicle_registry

TIERS = (
    "point_mass_3dof",
    "pseudo_6dof_kinematic_bridge",
    "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated",
)


def _render(report: FidelityReadinessReport) -> str:
    lines = [f"{report.vehicle_id} / {report.tier}: {report.status} (runtime proof: {report.runtime_proof_status})"]
    for finding in report.findings:
        if finding.status != "present":
            lines.append(f"  {finding.severity.upper()} {finding.status}: {finding.requirement_id} — {finding.message}")
    return "\n".join(lines)
    ####


def _ordered_blocker(reports: tuple[FidelityReadinessReport, ...]) -> FidelityReadinessReport | None:
    """Return the first blocked prerequisite in an ordered tier sequence."""

    return next((report for report in reports if report.errors), None)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--vehicle", default="all", help="registered vehicle ID or 'all' (default: all)")
    tier_group = parser.add_mutually_exclusive_group()
    tier_group.add_argument("--tier", choices=TIERS + ("all",), default="all")
    tier_group.add_argument(
        "--through-tier",
        choices=TIERS,
        help="evaluate the ordered prerequisites from point-mass through this tier",
    )
    parser.add_argument("--json", type=Path, help="write the machine-readable report")
    args = parser.parse_args(argv)
    if args.through_tier:
        tiers = TIERS[: TIERS.index(args.through_tier) + 1]
    else:
        tiers = TIERS if args.tier == "all" else (args.tier,)
    vehicle_ids = tuple(load_vehicle_registry()) if args.vehicle == "all" else (args.vehicle,)
    reports = tuple(report for vehicle_id in vehicle_ids for report in validate_all_fidelity_readiness(vehicle_id, tiers))
    for report in reports:
        print(_render(report))
    ordered_blockers: dict[str, dict[str, str]] = {}
    if args.through_tier:
        for vehicle_id in vehicle_ids:
            vehicle_reports = tuple(report for report in reports if report.vehicle_id == vehicle_id)
            blocker = _ordered_blocker(vehicle_reports)
            if blocker is not None:
                ordered_blockers[vehicle_id] = {
                    "tier": blocker.tier,
                    "message": "required intake data are missing at this tier",
                }
                print(f"{vehicle_id}: ordered promotion blocked at {blocker.tier}")
    if args.json:
        args.json.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "vehicle": args.vehicle,
            "requested_tier": args.through_tier or args.tier,
            "ordered": bool(args.through_tier),
            "ordered_blockers": ordered_blockers,
            "reports": [report.as_dict() for report in reports],
        }
        args.json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if all(not report.errors for report in reports) and not ordered_blockers else 1
    ####


if __name__ == "__main__":
    raise SystemExit(main())
