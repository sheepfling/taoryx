#!/usr/bin/env python3
"""Run reusable lower-tier tuning-campaign witnesses.

This is a design-screen regression, not a controller or showcase promotion.
It demonstrates that an existing multirotor family and an existing
powered-fixed-wing family enter the same trim/derivative/authority/candidate
workflow without a vehicle-specific manual gain loop.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.control_allocation import ControlPlantAdapter
from taoryx.family_adapter import StandardFamilyAdapter, descriptor_from_control_plant
from taoryx.trajectory.a320_adapter import (
    A320Pseudo6DOFControlPlant,
    build_a320_pseudo_tuning_campaign,
)
from taoryx.trajectory.a320_pseudo6dof import A320Pseudo6DOFModel
from taoryx.trajectory.hummingbird_adapter import (
    HummingbirdPseudo6DOFControlPlant,
    build_hummingbird_pseudo_tuning_campaign,
)
from taoryx.tuning_campaign import run_tuning_campaign

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/generated/reduced_tuning_campaigns.json"


def _adapter(
    plant: ControlPlantAdapter,
    *,
    family_id: str,
    adapter_id: str,
    physical_family: str,
) -> StandardFamilyAdapter:
    """Wrap an existing reduced plant through the common family façade."""

    descriptor = descriptor_from_control_plant(
        plant,
        family_id=family_id,
        adapter_id=adapter_id,
        physical_family=physical_family,
        tier="pseudo_6dof",
        evidence_status="development",
        omitted_physics=("physical effector allocation", "nonlinear mission validation"),
    )
    return StandardFamilyAdapter.from_control_plant(descriptor, plant)
    ####


def build_report() -> dict[str, object]:
    """Return the two cross-topology lower-tier campaign witnesses."""

    hummingbird = run_tuning_campaign(
        _adapter(
            HummingbirdPseudo6DOFControlPlant(),
            family_id="hummingbird",
            adapter_id="taoryx.multirotor.native_quad_x.v1",
            physical_family="multirotor",
        ),
        build_hummingbird_pseudo_tuning_campaign(),
    )
    a320 = run_tuning_campaign(
        _adapter(
            A320Pseudo6DOFControlPlant(A320Pseudo6DOFModel.from_repository(ROOT)),
            family_id="a320_openap_3dof",
            adapter_id="taoryx.fixed_wing.openap.v1",
            physical_family="powered_fixed_wing",
        ),
        build_a320_pseudo_tuning_campaign(),
    )
    reports = (hummingbird, a320)
    return {
        "schema": "taoryx.reduced-tuning-campaign-witnesses/v1alpha1",
        "status": "candidate_ready" if all(report.status == "candidate_ready" for report in reports) else "blocked",
        "campaign_count": len(reports),
        "campaigns": [report.as_dict() for report in reports],
        "claim_boundary": (
            "Reusable lower-tier candidate-design screens only; no physical "
            "surface/rotor allocation, nonlinear mission, or family-envelope qualification claim."
        ),
    }
    ####


def _write_json(path: Path, payload: dict[str, object]) -> None:
    """Write stable JSON for a checked-in regression artifact."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def main(argv: list[str] | None = None) -> int:
    """Build or check the reusable campaign evidence artifact."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true", help="fail when the checked-in artifact differs")
    args = parser.parse_args(argv)
    report = build_report()
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.check:
        if not args.output.is_file() or args.output.read_text(encoding="utf-8") != rendered:
            print(f"reduced tuning-campaign artifact is stale: {args.output}")
            return 1
    else:
        _write_json(args.output, report)
    print(json.dumps({"campaign_count": report["campaign_count"], "status": report["status"]}, sort_keys=True))
    return 0 if report["status"] == "candidate_ready" else 2
    ####


if __name__ == "__main__":
    raise SystemExit(main())
