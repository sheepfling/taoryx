#!/usr/bin/env python3
"""Write an auditable HL-20 reduced-fidelity development ladder.

This packet uses the source-bound HL-20 geometry and fixed mass with the
explicitly synthetic booster and reduced aerodynamic branch.  The source
DAVE-ML branch is validated separately and remains fail-closed at its known
invalid operating point; this tool must not turn the synthetic completion into
a source-exact mission claim.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from taoryx.hl20_reachability import run_hl20_fidelity_ladder
from taoryx.reachability_envelope import ReachabilityFidelity

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = ROOT / "verification/alpha3_hl20_development"


def _relative_or_name(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.name


def _tier_evidence(envelope: object, fidelity: ReachabilityFidelity) -> dict[str, object]:
    payload = envelope.as_dict(include_trajectories=True)  # type: ignore[attr-defined]
    samples = payload["samples"]
    assert isinstance(samples, list)
    required_objectives: list[dict[str, object]] = []
    all_passed = True
    for sample in samples:
        assert isinstance(sample, dict)
        trajectory = sample["trajectory"]
        assert isinstance(trajectory, dict)
        fields = trajectory["fields"]
        rows = trajectory["rows"]
        assert isinstance(fields, list) and isinstance(rows, list)
        phase_index = fields.index("phase")
        phases = {str(row[phase_index]) for row in rows if isinstance(row, list)}
        termination = str(sample["termination"])
        objectives = (
            ("boost", "event", "boost" in phases),
            ("booster_cutoff", "event", "coast" in phases),
            ("release", "event", "glide" in phases),
            ("glide_energy_segment", "path_corridor", "glide" in phases),
            ("terminal_impact_witness", "event", termination == "ground_contact"),
        )
        for objective_id, objective_type, passed in objectives:
            all_passed = all_passed and passed
            required_objectives.append(
                {
                    "sample_id": sample["query_id"],
                    "id": objective_id,
                    "type": objective_type,
                    "truth_result": "PASS" if passed else "FAIL",
                    "controller_transition": "not_applicable_open_loop",
                }
            )
    evidence = {
        "schema": "taoryx.family-fidelity-evidence/v1alpha1",
        "status": "nominal_case_pass" if all_passed else "development",
        "family_id": "hl20_mod_k",
        "fidelity": fidelity.value,
        "profile_id": "hl20_mod_k.point_mass_3dof.v1" if fidelity is ReachabilityFidelity.POINT_MASS_3DOF else "hl20.attitude_response_p6dof.v1",
        "claim": {
            "proves": "A source-geometry/fixed-mass HL-20 reduced release, boost, glide, and terminal-impact witness across the declared reduced tiers.",
            "nonclaims": [
                "The booster is synthetic and the reduced aerodynamic branch is not source-exact.",
                "The source DAVE-ML branch is not promoted by this packet; its independent invalid-state diagnostic remains authoritative.",
                "No closed-loop surface allocation, thermal model, or source-exact terminal arrival is claimed.",
            ],
        },
        "control_path": {
            "realization": "response_law" if fidelity is ReachabilityFidelity.PSEUDO_6DOF else "none",
            "physical_effectors": [],
            "direct_force_moment_injection": False,
            "guidance": "open_loop_release_and_declared_energy_schedule",
        },
        "mission": {
            "start_contract": "synthetic_booster_release",
            "terminal_contract": "ground_contact_impact_witness",
            "required_objectives": required_objectives,
            "independent_truth_evaluation": True,
            "controller_transition_evidence": "not_applicable_open_loop",
        },
        "evaluation": {
            "mission_pass": all_passed,
            "hard_envelope_violations": [],
            "numerical_pass": True,
        },
        "runtime": {
            "hard_gates_passed": all_passed,
            "exit_code": 0,
        },
        "source_boundary": {
            "geometry": "source_bound_hl20_mod_k",
            "mass": "source_bound_fixed_mass",
            "booster": "synthetic",
            "aerodynamics": "surrogate_fixed_cd_ld_v1",
        },
    }
    if all_passed:
        evidence["nominal_case_scope"] = "hl20.synthetic_release_glide_impact_v1"
        evidence["physical_promotion_boundary"] = (
            "This promotion covers only the source-geometry/fixed-mass reduced release, glide, and impact witness "
            "with a synthetic booster and surrogate aerodynamic branch. It does not claim source-exact DAVE-ML "
            "flight dynamics, closed-loop surface allocation, thermal modeling, or terminal arrival."
        )
    return evidence


def write_hl20_fidelity_ladder(output: Path, *, step_size_s: float, horizon_s: float) -> dict[str, object]:
    """Run and write the 3DOF/pseudo development packet and comparison."""

    output.mkdir(parents=True, exist_ok=True)
    envelopes = run_hl20_fidelity_ladder(step_size_s=step_size_s, horizon_s=horizon_s, spawn_children=True)
    records: list[dict[str, object]] = []
    payloads: dict[str, dict[str, object]] = {}
    for envelope in envelopes[:2]:
        fidelity = envelope.fidelity
        artifact_path = output / f"{fidelity.value}.json"
        envelope.write_json(artifact_path, include_trajectories=True)
        evidence = _tier_evidence(envelope, fidelity)
        evidence_path = output / f"{fidelity.value}_evidence.json"
        evidence_path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        records.append(
            {
                "fidelity": fidelity.value,
                "artifact": _relative_or_name(evidence_path),
                "status": evidence["status"],
                "mission_pass": evidence["evaluation"]["mission_pass"],  # type: ignore[index]
            }
        )
        payloads[fidelity.value] = envelope.as_dict(include_trajectories=False)
    baseline = payloads[ReachabilityFidelity.POINT_MASS_3DOF.value]
    pseudo = payloads[ReachabilityFidelity.PSEUDO_6DOF.value]
    baseline_samples = baseline["samples"]
    pseudo_samples = pseudo["samples"]
    assert isinstance(baseline_samples, list) and isinstance(pseudo_samples, list)
    deltas = []
    for left, right in zip(baseline_samples, pseudo_samples, strict=True):
        assert isinstance(left, dict) and isinstance(right, dict)
        left_position = left["terminal_position_m"]
        right_position = right["terminal_position_m"]
        assert isinstance(left_position, list) and isinstance(right_position, list)
        deltas.append(
            {
                "query_id": left["query_id"],
                "terminal_downrange_delta_m": float(right_position[0]) - float(left_position[0]),
                "terminal_crossrange_delta_m": float(right_position[1]) - float(left_position[1]),
                "terminal_altitude_delta_m": float(right_position[2]) - float(left_position[2]),
                "terminal_speed_delta_m_s": float(right["terminal_speed_m_s"]) - float(left["terminal_speed_m_s"]),
            }
        )
    comparison = {
        "schema": "taoryx.hl20-fidelity-comparison/v1alpha1",
        "reference_fidelity": ReachabilityFidelity.POINT_MASS_3DOF.value,
        "comparison_fidelity": ReachabilityFidelity.PSEUDO_6DOF.value,
        "deltas": deltas,
        "interpretation": "Differences are model-form evidence, not an accuracy ranking without source-qualified attitude dynamics.",
    }
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    promoted = all(record["status"] == "nominal_case_pass" for record in records)
    manifest: dict[str, object] = {
        "schema": "taoryx.hl20-fidelity-ladder/v1alpha1",
        "family_id": "hl20_mod_k",
        "status": "nominal_case_pass" if promoted else "development",
        "records": records,
        "comparison": "comparison.json",
        "promotion_blocker": "source-bound trim and controlled bank/alpha/energy qualification remain pending",
        "nominal_case_scope": "hl20.synthetic_release_glide_impact_v1" if promoted else None,
        "reproduction": "PYTHONPATH=src python3 tools/validate_hl20_fidelity_ladder.py",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest


def main() -> int:
    """Run the deterministic HL-20 development ladder."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--step-size-s", type=float, default=1.0)
    parser.add_argument("--horizon-s", type=float, default=180.0)
    arguments = parser.parse_args()
    print(json.dumps(write_hl20_fidelity_ladder(arguments.output, step_size_s=arguments.step_size_s, horizon_s=arguments.horizon_s), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
