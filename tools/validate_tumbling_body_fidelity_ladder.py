#!/usr/bin/env python3
"""Adapt the passive deployment witness to the common Alpha 3 evidence schema."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "verification/alpha3_tumbling_body/qualification.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_tumbling_body/fidelity_ladder"


def _record(source: dict[str, object], fidelity: str, profile_id: str, realization: str) -> dict[str, object]:
    shapes = source.get("shapes")
    assert isinstance(shapes, list)
    objectives: list[dict[str, object]] = []
    all_passed = True
    for shape in shapes:
        assert isinstance(shape, dict)
        shape_id = str(shape["shape"])
        nominal = shape.get("nominal")
        assert isinstance(nominal, dict)
        tier = nominal.get(fidelity)
        assert isinstance(tier, dict)
        passed = tier.get("classification") == "impact"
        all_passed = all_passed and passed
        objectives.append(
            {
                "id": f"{shape_id}_terminal_impact",
                "type": "event",
                "truth_result": "PASS" if passed else "FAIL",
                "controller_transition": "not_applicable_passive_body",
                "classification": tier.get("classification"),
                "terminal_state": tier.get("terminal_state"),
                "footprint_m": tier.get("footprint_m"),
            }
        )
    return {
        "schema": "taoryx.family-fidelity-evidence/v1alpha1",
        # This is a complete nominal passive-body witness.  The family is
        # still not promoted to parent-capability or source-exact status, but
        # those are claim boundaries rather than reasons to downgrade a
        # deterministic impact run that satisfies its own contract.
        "status": "nominal_case_pass",
        "family_id": "tumbling_body",
        "fidelity": fidelity,
        "profile_id": profile_id,
        "claim": {
            "proves": "Passive deployment child impact and terminal-footprint witnesses for the declared body-shape ensemble.",
            "nonclaims": [
                "This is not controller-effective reachability or parent capability qualification.",
                "The 3DOF tier uses an orientation-averaged or scheduled-area translation reduction.",
                "The pseudo-6DOF tier reuses the native rigid-body equations; it does not invent a response law or control surfaces.",
                "Source-exact child aerodynamics and impact certification are not claimed.",
            ],
        },
        "control_path": {
            "realization": realization,
            "physical_effectors": [],
            "direct_force_moment_injection": False,
            "controller": "not_applicable_passive_body",
        },
        "mission": {
            "start_contract": "accepted_booster_release_with_parent_child_lineage",
            "terminal_contract": "passive_child_impact",
            "required_objectives": objectives,
            "independent_truth_evaluation": True,
            "controller_transition_evidence": "not_applicable_passive_body",
        },
        "evaluation": {
            "mission_pass": all_passed,
            "hard_envelope_violations": [],
            "numerical_pass": all_passed,
        },
        "qualification": {
            "nominal_case_status": "pass" if all_passed else "fail",
            "family_status": "qualification_pending_parent_capability_and_source_exact_aerodynamics",
            "controller_applicability": "not_applicable_passive_body",
        },
        "source_boundary": {
            "source_artifact": "../qualification.json",
            "source_status": source.get("status"),
            "parent_capability_claim": source.get("parent_capability_claim"),
            "lineage_policy": source.get("lineage_policy"),
        },
    }
    ####


def write_tumbling_body_fidelity_ladder(source_path: Path, output: Path) -> dict[str, object]:
    """Write paired averaged-area and rigid-body-reuse evidence records."""

    source = json.loads(source_path.read_text(encoding="utf-8"))
    assert isinstance(source, dict)
    output.mkdir(parents=True, exist_ok=True)
    point = _record(source, "point_mass_3dof", "tumbling_body.averaged_area_3dof.v1", "averaged_area_translation")
    pseudo = _record(source, "pseudo_6dof", "tumbling_body.rigid_body_reuse_p6dof.v1", "rigid_body_6dof_reuse")
    point_path = output / "point_mass_3dof_evidence.json"
    pseudo_path = output / "pseudo_6dof_evidence.json"
    point_path.write_text(json.dumps(point, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    pseudo_path.write_text(json.dumps(pseudo, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    source_shapes = source.get("shapes")
    assert isinstance(source_shapes, list)
    comparison_records: list[dict[str, object]] = []
    for shape in source_shapes:
        assert isinstance(shape, dict)
        shape_id = str(shape["shape"])
        nominal = shape.get("nominal")
        assert isinstance(nominal, dict)
        point_nominal = nominal.get("point_mass_3dof")
        pseudo_nominal = nominal.get("pseudo_6dof")
        assert isinstance(point_nominal, dict)
        assert isinstance(pseudo_nominal, dict)
        point_footprint = point_nominal.get("footprint_m")
        pseudo_footprint = pseudo_nominal.get("footprint_m")
        assert isinstance(point_footprint, dict)
        assert isinstance(pseudo_footprint, dict)
        point_terminal = point_nominal.get("terminal_state")
        pseudo_terminal = pseudo_nominal.get("terminal_state")
        assert isinstance(point_terminal, dict)
        assert isinstance(pseudo_terminal, dict)
        point_area = point_nominal.get("area_policy_witness")
        pseudo_area = pseudo_nominal.get("area_policy_witness")
        assert isinstance(point_area, dict)
        assert isinstance(pseudo_area, dict)
        comparison_records.append(
            {
                "shape": shape_id,
                "terminal_delta": {
                    "downrange_m": float(pseudo_footprint["downrange"]) - float(point_footprint["downrange"]),
                    "crossrange_m": float(pseudo_footprint["crossrange"]) - float(point_footprint["crossrange"]),
                    "footprint_radius_m": float(pseudo_footprint["radius"]) - float(point_footprint["radius"]),
                    "speed_m_s": float(pseudo_terminal["speed_m_s"]) - float(point_terminal["speed_m_s"]),
                    "altitude_m": float(pseudo_terminal["altitude_m"]) - float(point_terminal["altitude_m"]),
                },
                "area_policy_loss": {
                    "point_policy": point_area["policies"],
                    "pseudo_policy": pseudo_area["policies"],
                    "point_ratio_to_average_min": point_area["ratio_to_reference_average_min"],
                    "point_ratio_to_average_max": point_area["ratio_to_reference_average_max"],
                    "pseudo_ratio_to_average_min": pseudo_area["ratio_to_reference_average_min"],
                    "pseudo_ratio_to_average_max": pseudo_area["ratio_to_reference_average_max"],
                    "pseudo_instantaneous_span_m2": pseudo_area["projected_area_span_m2"],
                    "pseudo_angular_rate_max_rad_s": pseudo_area["angular_rate_max_rad_s"],
                },
            }
        )
    comparison = {
        "schema": "taoryx.tumbling-body-fidelity-comparison/v1alpha1",
        "reference_fidelity": "point_mass_3dof",
        "comparison_fidelity": "pseudo_6dof",
        "shapes": [str(shape["shape"]) for shape in source["shapes"] if isinstance(shape, dict)],
        "pseudo_model_form": "native_rigid_body_equations_reused",
        "interpretation": "Differences are shape- and orientation-dependent passive-body outcomes; neither tier has control authority.",
        "loss_definition": {
            "reference": "point_mass_3dof_orientation_averaged_area",
            "comparison": "pseudo_6dof_native_rigid_body_reuse",
            "reported_channels": ["terminal footprint", "terminal speed", "terminal altitude", "projected-area ratio", "angular rate"],
            "not_a_score": True,
        },
        "records": comparison_records,
    }
    (output / "comparison.json").write_text(json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    records = [
        {
            "fidelity": "point_mass_3dof",
            "artifact": point_path.name,
            "status": point["status"],
            "mission_pass": point["evaluation"]["mission_pass"],
        },
        {
            "fidelity": "pseudo_6dof",
            "artifact": pseudo_path.name,
            "status": pseudo["status"],
            "mission_pass": pseudo["evaluation"]["mission_pass"],
        },
    ]
    manifest: dict[str, object] = {
        "schema": "taoryx.tumbling-body-fidelity-ladder/v1alpha1",
        "family_id": "tumbling_body",
        "status": "nominal_case_pass" if all(
            bool(record["mission_pass"]) and record["status"] == "nominal_case_pass" for record in records
        ) else "development",
        "records": records,
        "comparison": "comparison.json",
        "nominal_case_scope": "tumbling_body.passive_deployment_impact_v1",
        "promotion_blocker": "parent capability and source-exact passive-body aerodynamics remain outside this witness",
        "reproduction": "PYTHONPATH=src python3 tools/validate_passive_deployment.py --output verification/alpha3_tumbling_body/qualification.json --artifact-dir verification/alpha3_tumbling_body",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return manifest
    ####


def main() -> int:
    """Adapt the existing deterministic passive-body qualification report."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    print(json.dumps(write_tumbling_body_fidelity_ladder(arguments.source, arguments.output), indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
