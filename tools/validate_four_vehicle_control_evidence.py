#!/usr/bin/env python3
"""Build the explicit four-vehicle physical-control evidence ledger.

This is a catalog-level verifier, not another controller.  It consumes the
four vehicle-specific artifacts, checks their tier and control-path claims,
and produces one compact Alpha-2 handoff record.  A blocked X-15 prerequisite
is a valid ledger result; it must not be masked by an unrelated direct-wrench
screen.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "verification/generated/four_vehicle_control_evidence.json"
ARTIFACTS = {
    "skywalker_x8": ROOT / "verification/generated/x8_table_coordinate_physical_lqr.json",
    "hummingbird": ROOT / "verification/generated/hummingbird_individual_rotor_physical_lqr.json",
    "b747": ROOT / "verification/generated/b747_condition3_physical_surface_lqr.json",
    "x15": ROOT / "verification/generated/x15_physical_lqr_readiness.json",
}


def _sha256(path: Path) -> str:
    """Return an immutable artifact digest."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _load(path: Path) -> dict[str, Any]:
    """Load one prerequisite artifact with a clear regeneration failure."""

    if not path.is_file():
        raise FileNotFoundError(
            f"missing physical-control evidence artifact {path.relative_to(ROOT)}; regenerate its vehicle-specific validator first"
        )
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"physical-control artifact must be a JSON object: {path.relative_to(ROOT)}")
    return payload
    ####


def _claim(payload: Mapping[str, Any], vehicle_id: str) -> Mapping[str, Any]:
    """Return a typed claim mapping rather than silently accepting absence."""

    claim = payload.get("claim")
    if not isinstance(claim, Mapping):
        raise ValueError(f"{vehicle_id} physical-control artifact lacks a claim mapping")
    return claim
    ####


def _record(
    vehicle_id: str,
    payload: Mapping[str, Any],
    path: Path,
    *,
    expected_schema: str,
    expected_tier: str,
    expected_status: str,
) -> dict[str, Any]:
    """Validate one distinct evidence tier without collapsing their meaning."""

    claim = _claim(payload, vehicle_id)
    if payload.get("schema") != expected_schema:
        raise ValueError(f"{vehicle_id} has unexpected evidence schema {payload.get('schema')!r}")
    if claim.get("earned_controller_evidence_tier") != expected_tier:
        raise ValueError(f"{vehicle_id} does not have expected evidence tier {expected_tier!r}")
    observed_status = payload.get("status", claim.get("status"))
    if observed_status != expected_status:
        raise ValueError(f"{vehicle_id} has unexpected evidence status {observed_status!r}")
    direct_injection = claim.get("direct_body_moment_injection")
    if direct_injection is not False:
        raise ValueError(f"{vehicle_id} does not explicitly rule out direct body-moment injection")
    return {
        "artifact": str(path.relative_to(ROOT)),
        "artifact_sha256": _sha256(path),
        "schema": expected_schema,
        "evidence_tier": expected_tier,
        "status": observed_status,
        "direct_body_moment_injection": direct_injection,
        "physical_allocation_evidence": claim.get("physical_allocation_evidence"),
        "proves": claim.get("proves"),
        "nonclaims": claim.get("nonclaims", []),
        "reproduction": payload.get("reproduction"),
    }
    ####


def build() -> dict[str, Any]:
    """Validate the four current evidence artifacts and return their ledger."""

    x8 = _load(ARTIFACTS["skywalker_x8"])
    hummingbird = _load(ARTIFACTS["hummingbird"])
    b747 = _load(ARTIFACTS["b747"])
    x15 = _load(ARTIFACTS["x15"])
    records = {
        "skywalker_x8": _record(
            "skywalker_x8",
            x8,
            ARTIFACTS["skywalker_x8"],
            expected_schema="taoryx.x8-table-coordinate-physical-lqr/v1alpha1",
            expected_tier="T3_linearly_controlled",
            expected_status="local_nonlinear_table_coordinate_validation",
        ),
        "hummingbird": _record(
            "hummingbird",
            hummingbird,
            ARTIFACTS["hummingbird"],
            expected_schema="taoryx.hummingbird-individual-rotor-physical-lqr/v1alpha1",
            expected_tier="T5_nonlinearly_validated",
            expected_status="local_nonlinear_individual_rotor_validation",
        ),
        "b747": _record(
            "b747",
            b747,
            ARTIFACTS["b747"],
            expected_schema="taoryx.b747-condition3-physical-surface-lqr/v1alpha1",
            expected_tier="T5_nonlinearly_validated",
            expected_status="local_nonlinear_surface_validation",
        ),
        "x15": _record(
            "x15",
            x15,
            ARTIFACTS["x15"],
            expected_schema="taoryx.x15-physical-lqr-readiness/v1alpha1",
            expected_tier="T0_structural",
            expected_status="blocked_before_T1_trim",
        ),
    }
    if x15.get("evidence_tiers", {}).get("T1_trimmed") != "blocked":
        raise ValueError("X-15 readiness artifact must explicitly block T1 trim")
    return {
        "schema": "taoryx.four-vehicle-control-evidence/v1alpha1",
        "scope": "Alpha-2 local physical-controller evidence ledger",
        "status": "tiered_evidence_complete",
        "claim": {
            "proves": (
                "Each Alpha-2 reference vehicle has a reproducible controller-evidence record whose realized "
                "control path and strongest earned tier are explicit."
            ),
            "nonclaims": [
                "This ledger is not a four-vehicle mission or envelope qualification.",
                "T5 local results do not establish gain scheduling, full-flight, or showcase-terminal control.",
                "The X-15 blocked readiness record is not a physical-controller result.",
            ],
            "direct_wrench_results_are_screen_only": True,
        },
        "vehicles": records,
        "promotion_summary": {
            "T5_local_nonlinear": ["hummingbird", "b747"],
            "T3_source_table_coordinate_only": ["skywalker_x8"],
            "T0_blocked_before_trim": ["x15"],
        },
        "required_regeneration_order": [
            "PYTHONPATH=src python3 tools/validate_x8_physical_lqr.py",
            "PYTHONPATH=src python3 tools/validate_hummingbird_physical_lqr.py",
            "PYTHONPATH=src python3 tools/validate_b747_physical_lqr.py",
            "PYTHONPATH=src python3 tools/validate_x15_physical_lqr.py",
            "PYTHONPATH=src python3 tools/validate_four_vehicle_control_evidence.py",
        ],
    }
    ####


def main() -> int:
    """Write or check the generated four-vehicle evidence ledger."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--output", type=Path, default=OUTPUT)
    arguments = parser.parse_args()
    expected = json.dumps(build(), indent=2, sort_keys=True) + "\n"
    if arguments.check:
        if not arguments.output.is_file() or arguments.output.read_text(encoding="utf-8") != expected:
            raise SystemExit(f"stale four-vehicle control-evidence ledger: {arguments.output}")
    else:
        arguments.output.parent.mkdir(parents=True, exist_ok=True)
        arguments.output.write_text(expected, encoding="utf-8")
        print(arguments.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
