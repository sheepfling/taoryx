#!/usr/bin/env python3
"""Record the X-15 physical-LQR promotion gate without manufacturing authority.

The public X-15 package contains aerodynamic source-control coordinates, but
the two source-bounded operating-condition attempts currently fail their trim
contracts.  A controller whose linearization is built at either point would
therefore be a mathematical construct detached from an equilibrium of the
nonlinear plant.  This tool preserves that result as an evidence artifact:
T0 structural evidence is present; promotion to T1 and above is blocked.

It intentionally does *not* synthesize an LQR, allocate a desired wrench, or
inject a direct force or moment.  The existing direct-wrench controller stays
available as a separately labelled screen-only path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from collections.abc import Mapping
from pathlib import Path
from typing import Any

try:
    from tools import solve_x15_powered_trim, solve_x15_trim
except ModuleNotFoundError:  # Script execution places ``tools/`` on sys.path.
    import solve_x15_powered_trim  # type: ignore[no-redef]
    import solve_x15_trim  # type: ignore[no-redef]

ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = ROOT / "tests/fixtures/x15_coherent_6dof_public_research_v1"
CONTROL_MAPPING = SOURCE_ROOT / "controls/control_coordinate_mapping.csv"
ACTUATOR_LIMITS = SOURCE_ROOT / "controls/actuator_limits.csv"
UNAVAILABLE_ITEMS = SOURCE_ROOT / "controls/unavailable_items.csv"
XLR99_HISTORY = SOURCE_ROOT / "tables/x15_xlr99_thrust_mdot.tbl"
OUTPUT = ROOT / "verification/generated/x15_physical_lqr_readiness.json"


def _sha256(path: Path) -> str:
    """Return an immutable digest for one source or generated input."""

    return hashlib.sha256(path.read_bytes()).hexdigest()
    ####


def _csv_rows(path: Path) -> list[dict[str, str]]:
    """Load a small source contract table as auditable JSON-compatible rows."""

    with path.open(encoding="utf-8", newline="") as handle:
        return [dict(row) for row in csv.DictReader(handle)]
    ####


def _trim_summary(report: Mapping[str, Any]) -> dict[str, Any]:
    """Keep the promotion artifact concise while retaining the decisive data."""

    return {
        "status": report["status"],
        "residual_norm_l2": report["residual_norm_l2"],
        "state": report["state"],
        "controls": report["controls"],
        "residual_normalized": report["residual_normalized"],
        "solver": report["solver"],
        "provenance": report["provenance"],
    }
    ####


def build_artifact(
    release_glide_report: Mapping[str, Any],
    powered_full_thrust_report: Mapping[str, Any],
) -> dict[str, Any]:
    """Build a fail-closed X-15 physical-control readiness report.

    No later evidence tier may be earned before an actual source-bounded trim
    passes.  Keeping this condition in an artifact makes the absence of a
    physical LQR visible to catalogs, showcase rendering, and future tuning
    utilities instead of silently falling back to direct moment injection.
    """

    release_ok = release_glide_report["status"] == "pass"
    powered_ok = powered_full_thrust_report["status"] == "pass"
    t1_passed = release_ok or powered_ok
    blocker_ids = [
        "no_source_bounded_x15_trim_operating_point",
        "xlr99_deck_is_time_indexed_full_thrust_not_a_throttle_map",
        "stabilator_coordinate_to_left_right_gearing_unresolved",
        "rcs_geometry_and_impulse_unavailable",
        "hard_rate_limits_missing_for_differential_stabilator_and_rudder",
    ]
    return {
        "schema": "taoryx.x15-physical-lqr-readiness/v1alpha1",
        "vehicle": "x15",
        "status": "ready_for_T1" if t1_passed else "blocked_before_T1_trim",
        "claim": {
            "proves": (
                "The source X-15 aerodynamic control-coordinate tables, declared travel bounds, and actuator-model "
                "gaps have been inventoried; independent unpowered-release and frozen full-thrust trim attempts are "
                "retained as prerequisite evidence rather than being replaced by a direct-wrench controller."
            ),
            "nonclaims": [
                "No actual-effector X-15 LQR, allocator, or nonlinear physical-controller recovery result is claimed.",
                "No direct body force or moment is injected by either trim probe or this readiness evaluation.",
                "The XLR99 history is not treated as a throttle command, engine map, or reusable powered trim model.",
                "No low-dynamic-pressure RCS blend is represented because the source package lacks RCS geometry and impulse data.",
                "Source stabilator coordinates are not claimed as reconciled left/right mechanical surface commands.",
            ],
            "earned_controller_evidence_tier": "T0_structural",
            "direct_body_moment_injection": False,
            "physical_lqr_synthesis_attempted": False,
            "constrained_allocator_attempted": False,
            "nonlinear_controller_validation_attempted": False,
        },
        "evidence_tiers": {
            "T0_structural": "passed",
            "T1_trimmed": "passed" if t1_passed else "blocked",
            "T2_linearized": "not_attempted_until_T1",
            "T3_linearly_controlled": "not_attempted_until_T2",
            "T4_physically_allocated": "not_attempted_until_T3",
            "T5_nonlinearly_validated": "not_attempted_until_T4",
            "T6_envelope_validated": "not_attempted_until_T5",
        },
        "promotion_gate": {
            "required_for_T1": "At least one actual-effectors source-bounded trim must satisfy its declared residual contract.",
            "release_glide_trim_passed": release_ok,
            "frozen_full_thrust_trim_passed": powered_ok,
            "passed": t1_passed,
            "blockers": blocker_ids if not t1_passed else [],
        },
        "trim_attempts": {
            "unpowered_release_glide": _trim_summary(release_glide_report),
            "frozen_full_thrust_t0": _trim_summary(powered_full_thrust_report),
        },
        "source_control_contract": {
            "control_coordinate_mapping": _csv_rows(CONTROL_MAPPING),
            "actuator_limits": _csv_rows(ACTUATOR_LIMITS),
            "unavailable_items": _csv_rows(UNAVAILABLE_ITEMS),
            "propulsion": {
                "source_asset": str(XLR99_HISTORY.relative_to(ROOT)),
                "classification": "time_indexed_full_thrust_and_mass_flow_history",
                "not_a_throttle_map": True,
            },
        },
        "required_before_physical_lqr": [
            "A source-bounded or explicitly labelled engineering-specified trim/reference operating point that passes T1.",
            "A propulsion command contract if powered trim or throttle allocation is to be claimed.",
            "Resolved physical left/right stabilator sign and gearing if hardware-surface allocation is to be claimed.",
            "RCS locations, thrust/impulse, tank/resource, and allocator data before low-authority blending is claimed.",
            "Hard actuator rate and delay assumptions for differential stabilator and rudder before phase-margin claims.",
            "A scheduled set of validated operating points before any boost-to-glide or authority-transition claim.",
        ],
        "provenance": {
            "source_assets": {
                str(path.relative_to(ROOT)): _sha256(path)
                for path in (CONTROL_MAPPING, ACTUATOR_LIMITS, UNAVAILABLE_ITEMS, XLR99_HISTORY)
            },
            "generated_trim_reports": {
                str(solve_x15_trim.OUTPUT.relative_to(ROOT)): _sha256(solve_x15_trim.OUTPUT),
                str(solve_x15_powered_trim.OUTPUT.relative_to(ROOT)): _sha256(solve_x15_powered_trim.OUTPUT),
            },
        },
        "reproduction": "PYTHONPATH=src python3 tools/validate_x15_physical_lqr.py",
    }
    ####


def _write_json(path: Path, payload: Mapping[str, Any]) -> None:
    """Write a stable JSON artifact without a hidden external dependency."""

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    ####


def main() -> int:
    """Regenerate the two trim probes and write their common readiness gate."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    arguments = parser.parse_args()
    release_glide_report = solve_x15_trim.build_report()
    powered_full_thrust_report = solve_x15_powered_trim.build_report()
    _write_json(solve_x15_trim.OUTPUT, release_glide_report)
    _write_json(solve_x15_powered_trim.OUTPUT, powered_full_thrust_report)
    artifact = build_artifact(release_glide_report, powered_full_thrust_report)
    _write_json(arguments.output, artifact)
    print(arguments.output)
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
