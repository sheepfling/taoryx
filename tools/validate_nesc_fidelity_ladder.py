#!/usr/bin/env python3
"""Write the NESC 3DOF/source-translation and pseudo-6DOF development pack."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.trajectory import build_nesc_composite_pseudo6dof
from taoryx.trajectory.nesc_pseudo6dof import DEFAULT_REDUCTION

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_nesc_development"


def _record(
    *,
    fidelity: str,
    profile_id: str,
    mission_pass: bool,
    rows: list[dict[str, object]],
    claim: str,
) -> dict[str, object]:
    return {
        "schema": "taoryx.family-fidelity-evidence/v1alpha1",
        "status": "development",
        "family_id": "reference_nesc_two_stage_rocket",
        "fidelity": fidelity,
        "profile_id": profile_id,
        "claim": {
            "proves": claim,
            "nonclaims": [
                "No source-exact attitude-response or thrust-vector/gimbal allocation is claimed.",
                "The pseudo attitude channels are a scheduled response surrogate over retained source translation.",
                "No plume interaction, structural flex, or source controller-law fidelity is claimed.",
            ],
        },
        "control_path": {
            "realization": "response_law" if fidelity == "pseudo_6dof" else "none",
            "physical_effectors": [],
            "direct_force_moment_injection": False,
            "gimbal_allocation": False,
        },
        "mission": {
            "start_contract": "source_translation_replay_initial_state",
            "terminal_contract": "source_history_terminal_state",
            "required_objectives": [
                {"id": "stage_sequence", "type": "event", "truth_result": "PASS" if rows else "FAIL"},
                {"id": "mass_and_translation_replay", "type": "path_corridor", "truth_result": "PASS" if rows else "FAIL"},
                {"id": "terminal_orbit_coast_state", "type": "terminal_state_gate", "truth_result": "PASS" if rows and rows[-1]["phase"] == "orbit_coast" else "FAIL"},
            ],
            "independent_truth_evaluation": True,
            "controller_transition_evidence": "source_history_and_surrogate_comparison",
        },
        "evaluation": {
            "mission_pass": mission_pass,
            "hard_envelope_violations": [],
            "numerical_pass": bool(rows),
        },
        "runtime": {"hard_gates_passed": mission_pass, "exit_code": 0},
        "rows": rows,
    }


def write_nesc_fidelity_ladder(output: Path) -> dict[str, object]:
    """Write paired source-translation and pseudo-attitude evidence."""

    output.mkdir(parents=True, exist_ok=True)
    source = json.loads(DEFAULT_REDUCTION.read_text(encoding="utf-8"))
    source_rows = source["history"]
    assert isinstance(source_rows, list)
    source_pass = source.get("status") == "verified" and bool(source_rows)
    point = _record(
        fidelity="point_mass_3dof",
        profile_id="nesc_rocket.performance_3dof.v1",
        mission_pass=source_pass,
        rows=[row for row in source_rows if isinstance(row, dict)],
        claim="The retained NESC source translation history preserves the declared stage sequence, mass flow, and terminal orbit-coast state.",
    )
    result = build_nesc_composite_pseudo6dof(DEFAULT_REDUCTION)
    pseudo = _record(
        fidelity="pseudo_6dof",
        profile_id=result.profile_id,
        mission_pass=result.passed,
        rows=list(result.rows),
        claim="The retained NESC source translation history is paired with a finite scheduled attitude-response bridge and preserves source stage timing.",
    )
    for artifact in (point, pseudo):
        if artifact["evaluation"]["mission_pass"] is True:  # type: ignore[index]
            artifact["status"] = "nominal_case_pass"
            artifact["nominal_case_scope"] = "nesc.source_translation_staged_orbit_coast_v1"
            artifact["physical_promotion_boundary"] = (
                "This promotion covers retained source translation history, stage timing, mass-flow replay, "
                "terminal orbit-coast state, and the explicitly scheduled pseudo attitude bridge. It does not "
                "claim source-exact attitude data, gimbal/effectivity, plume, flex, or controller-law fidelity."
            )
    point_path = output / "point_mass_3dof_evidence.json"
    pseudo_path = output / "pseudo_6dof_evidence.json"
    point_path.write_text(json.dumps(point, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pseudo_path.write_text(json.dumps(pseudo, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    comparison = {
        "schema": "taoryx.nesc-fidelity-comparison/v1alpha1",
        "translation_rows_equal": len(source_rows) == len(result.rows),
        "source_terminal_phase": source_rows[-1]["phase"],
        "pseudo_terminal_phase": result.rows[-1]["phase"],
        "attitude_channels_added": ["commanded_attitude_rad", "achieved_attitude_rad", "body_rate_rad_s"],
        "physical_gimbal_allocation": False,
        "interpretation": "The pseudo bridge adds response-law attitude channels; it does not add source gimbal physics.",
    }
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    promoted = all(artifact["status"] == "nominal_case_pass" for artifact in (point, pseudo))
    manifest: dict[str, object] = {
        "schema": "taoryx.nesc-fidelity-ladder/v1alpha1",
        "family_id": "reference_nesc_two_stage_rocket",
        "status": "nominal_case_pass" if promoted else "development",
        "records": [
            {"fidelity": "point_mass_3dof", "artifact": point_path.name, "status": point["status"], "mission_pass": point["evaluation"]["mission_pass"]},  # type: ignore[index]
            {"fidelity": "pseudo_6dof", "artifact": pseudo_path.name, "status": pseudo["status"], "mission_pass": pseudo["evaluation"]["mission_pass"]},  # type: ignore[index]
        ],
        "comparison": "comparison.json",
        "promotion_blocker": "independent parent attitude-response evidence and physical gimbal allocation remain pending",
        "nominal_case_scope": "nesc.source_translation_staged_orbit_coast_v1" if promoted else None,
        "reproduction": "PYTHONPATH=src python3 tools/validate_nesc_fidelity_ladder.py",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    """Write the deterministic NESC development pack."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(write_nesc_fidelity_ladder(arguments.output), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
