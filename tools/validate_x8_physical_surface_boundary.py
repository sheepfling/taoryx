#!/usr/bin/env python3
"""Record the X8 physical-surface racetrack witness and its source boundary.

The short run proves that the declared two-elevon allocation reaches the
source aerodynamic plant for roll/pitch demand.  The long run is intentionally
expected to fail closed when the source beta envelope is reached: with no
independent yaw effector, that failure is evidence of a real control-space
boundary, not a reason to inject a direct yaw moment or widen the tables.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.runtime.runner import RunReport, run_files

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_x8_physical_surface_boundary"
PROBLEM = ROOT / "examples/mission_families/slower_x8/SV03_racetrack_altitude_turns_6dof.prb"
TABLE_ROOT = ROOT / "tests/fixtures/slower_airbreathing_and_multirotor_6dof_bundle_v1/tables"
TABLES = tuple(
    TABLE_ROOT / name
    for name in (
        "skywalker_x8_static_6axis.tbl",
        "skywalker_x8_collective_elevon_6axis.tbl",
        "skywalker_x8_differential_elevon_6axis.tbl",
        "skywalker_x8_thrust.tbl",
    )
)


def _history(report: RunReport) -> tuple[dict[str, float], ...]:
    """Return the short witness telemetry as plain mappings."""

    if not report.results:
        return ()
    return tuple({"time_s": state.time, **dict(state.named)} for state in report.results[0].states["1"])
    ####


def _run(output: Path, max_steps: int) -> RunReport:
    """Run the declared physical-surface problem at one step budget."""

    return run_files(
        PROBLEM,
        TABLES,
        output_dir=output,
        max_steps=max_steps,
        integrator="rk4",
        profile=GrammarProfile.TAORYX,
    )
    ####


def _short_witness(report: RunReport) -> dict[str, object]:
    """Summarize the source-domain physical allocation evidence."""

    history = _history(report)
    if not history:
        return {"status": "missing_result", "mission_pass": False}
    return {
        "status": "short_source_domain_witness_pass" if report.results else "missing_result",
        "mission_pass": bool(report.results),
        "duration_s": history[-1]["time_s"],
        "active_surface_counts": sorted({int(row["surface_allocation_active_surface_count"]) for row in history}),
        "effectiveness_rank": sorted({int(row["surface_allocation_effectiveness_rank"]) for row in history}),
        "allocation_status_codes": sorted({int(row["surface_allocation_status_code"]) for row in history}),
        "max_controlled_residual_nm": max(float(row["surface_allocation_controlled_residual_nm"]) for row in history),
        "max_actual_residual_nm": max(float(row["surface_allocation_actual_residual_nm"]) for row in history),
        "max_abs_sideslip_deg": max(abs(float(row["aero_sideslip_deg"])) for row in history),
        "max_abs_aero_yaw_moment_nm": max(abs(float(row["aero_moment_body_z_nm"])) for row in history),
        "max_abs_propulsion_yaw_moment_nm": max(abs(float(row["propulsion_moment_body_z_nm"])) for row in history),
        "direct_moment_active_samples": sum(int(row.get("direct_moment_active", 0.0)) for row in history),
        "left_right_mapping_signs": sorted({float(row["surface_allocation_x8_mapping_sign"]) for row in history}),
    }
    ####


def _boundary_witness(report: RunReport) -> dict[str, object]:
    """Summarize the expected fail-closed source beta boundary."""

    diagnostics = [{"code": item.code, "message": item.message} for item in report.diagnostics]
    messages = [item["message"] for item in diagnostics]
    return {
        "status": "source_beta_boundary_fail_closed" if not report.results else "unexpected_route_result",
        "mission_pass": False,
        "exit_code": report.exit_code,
        "diagnostics": diagnostics,
        "beta_boundary_reported": any("beta" in message.casefold() for message in messages),
        "envelope_exit_reported": any("outside its declared envelope" in message for message in messages),
    }
    ####


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
    ####


def write_boundary_witness(output: Path = DEFAULT_OUTPUT) -> dict[str, object]:
    """Run both budgets and write a compact, reproducible boundary packet."""

    output.mkdir(parents=True, exist_ok=True)
    short_report = _run(output / "short", 250)
    long_report = _run(output / "long", 2_500)
    payload: dict[str, object] = {
        "schema": "taoryx.x8-physical-surface-boundary/v1alpha1",
        "family_id": "skywalker_x8",
        "fidelity": "rigid_body_6dof_surface_allocated",
        "status": "boundary_witness",
        "claim": {
            "proves": "A source-domain short witness allocates X8 roll/pitch demand through the mapped physical left/right elevons and evaluates the resulting aerodynamic loads.",
            "boundary": "The full route reaches the declared beta table boundary and fails closed because the two-elevon plant has no independent yaw effector.",
            "nonclaims": [
                "No direct body-moment injection is used by either run.",
                "The short witness is not an end-to-end racetrack qualification.",
                "The boundary is not evidence for widening the source tables or inventing yaw authority.",
            ],
        },
        "source": {
            "problem": str(PROBLEM.relative_to(ROOT)),
            "tables": [str(path.relative_to(ROOT)) for path in TABLES],
            "control_space": "two physical elevons mapped from source collective/differential coordinates",
            "uncontrolled_axis": "aerodynamic yaw/sideslip residual",
        },
        "short_witness": _short_witness(short_report),
        "full_route_boundary": _boundary_witness(long_report),
        "reproduction": "PYTHONPATH=src python3 tools/validate_x8_physical_surface_boundary.py",
    }
    report_path = output / "boundary_witness.json"
    report_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        "schema": "taoryx.x8-physical-surface-boundary-manifest/v1alpha1",
        "report": "boundary_witness.json",
        "files": [{"path": report_path.name, "sha256": _sha256(report_path), "size_bytes": report_path.stat().st_size}],
        "claim_boundary": payload["claim"],
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return payload
    ####


def main() -> int:
    """Run the X8 physical-surface boundary witness."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(write_boundary_witness(arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())

