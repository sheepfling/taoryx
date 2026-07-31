#!/usr/bin/env python3
"""Build the Alpha 3 fidelity and robustness readiness matrix.

This report is deliberately not a robustness simulator.  It is the release
index that separates nominal paired-fidelity evidence from R1 perturbation
evidence and records the exact next gate for every family.  Missing R1
evidence is reported as ``not_run`` rather than inferred from a nominal pass.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, cast

ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "verification/alpha3_fidelity_ladder/manifest.json"
CROSS_FIDELITY = ROOT / "verification/alpha3_cross_fidelity/manifest.json"
DEFAULT_OUTPUT = ROOT / "verification/alpha3_robustness_matrix"

R1_CASES: dict[str, tuple[str, ...]] = {
    "skywalker_x8": ("initial_altitude_plus", "initial_speed_plus", "initial_cross_velocity_plus", "initial_mass_plus"),
    "b747": ("initial_altitude_plus", "initial_speed_plus", "initial_cross_velocity_plus", "initial_mass_plus"),
    "a320": ("initial_position_offset", "initial_speed_offset", "initial_altitude_offset", "throttle_scale"),
    "f16_s119": ("initial_position_offset", "initial_speed_offset", "initial_bank_offset", "surface_authority"),
    "x15": ("release_angle_low", "release_angle_high", "initial_speed_plus", "booster_thrust_minus_10pct", "wind_vector_plus"),
    "hummingbird": ("initial_position_offset", "initial_yaw_offset", "thrust_authority_minus_10pct", "battery_energy_minus_40pct"),
    "hl20_mod_k": ("release_state_offset", "mass_scale", "wind_vector", "alpha_boundary"),
    "reference_nesc_two_stage_rocket": ("response_lag_fast", "response_lag_slow", "initial_attitude_offset", "nominal_replay_integrity"),
    "tumbling_body": ("shape_geometry", "initial_attitude", "initial_rate", "area_policy"),
}

NEXT_GATES: dict[str, str] = {
    "skywalker_x8": "The mapped two-elevon short physical-surface witness passes roll/pitch allocation and the full route now records a reproducible beta-domain fail-closed boundary; end-to-end physical racetrack promotion still requires an independent yaw realization or a deliberately source-bounded route contract.",
    "b747": "Source-derived condition-node intake now closes FC3--FC7 source trim; FC4--FC7 remain physical controller-authority boundary witnesses and FC8--FC10 remain source-trim/table-domain blockers before schedule interpolation can be promoted.",
    "a320": "Use the completed calibrated reduced R1 matrix; next add throttle and physical-control evidence.",
    "f16_s119": "The conservative four-node alpha interior is qualified; next broaden the physical schedule into beta/high-rate and higher-dynamic-pressure witnesses.",
    "x15": "The powered/coast/glide response schedule now runs from phase state and is recorded in telemetry; next add a controlled terminal-handoff gate and source-backed physical-effector evidence.",
    "hummingbird": "The aggregate runtime pad-to-pad packet and the separate individual-rotor native horizontal and vertical witnesses now pass; remaining gates are electrical battery/SOC, rotor-resolved wind/full-envelope promotion, and replacing the explicit static-pad contact contract with landing-gear or ground-effect physics if those are claimed.",
    "hl20_mod_k": "The pinned DAVE-ML source-direction, bounded six-axis allocation, and native open-loop surface replay probes now record source effectivity and no-direct-moment realization; next connect that source path to source-bound trim and controlled bank/alpha/energy evidence.",
    "reference_nesc_two_stage_rocket": "The pinned source-data audit confirms no declared control input, gimbal channel, or attitude input; next obtain authoritative gimbal/effectivity and event-aligned attitude data before parent comparison or allocation.",
    "tumbling_body": "Area-policy, passive-rotation, and cross-fidelity loss witnesses now pass for the four declared shapes; remaining promotion requires source-exact passive aerodynamics or parent-capability evidence, not a controller.",
}

SUPPORTING_ARTIFACTS: dict[str, tuple[tuple[str, str], ...]] = {
    "a320": (
        ("paired_reduced_r1", "verification/alpha3_a320_r1/manifest.json"),
    ),
    "skywalker_x8": (
        ("paired_reduced_r1", "verification/alpha3_airbreathing_r1/manifest.json"),
        ("physical_surface_boundary", "verification/alpha3_x8_physical_surface_boundary/manifest.json"),
        ("direct_wrench_6dof", "verification/alpha3_x8_direct_wrench/manifest.json"),
    ),
    "b747": (
        ("paired_reduced_r1", "verification/alpha3_airbreathing_r1/manifest.json"),
    ),
    "hl20_mod_k": (
        ("paired_reduced_r1", "verification/alpha3_hl20_r1/manifest.json"),
        ("source_control_direction", "verification/alpha3_hl20_source_control/manifest.json"),
        ("source_allocation", "verification/alpha3_hl20_source_allocation/manifest.json"),
        ("source_surface_replay", "verification/alpha3_hl20_source_surface_replay/manifest.json"),
    ),
    "x15": (("paired_reduced_r1", "verification/alpha3_x15_r1/manifest.json"),),
    "reference_nesc_two_stage_rocket": (
        ("paired_reduced_r1", "verification/alpha3_nesc_r1/manifest.json"),
        ("attitude_data_contract", "verification/alpha3_nesc_attitude_data_contract/manifest.json"),
    ),
    "hummingbird": (
        ("paired_reduced_r1", "verification/alpha3_hummingbird_r1/manifest.json"),
        ("native_hover_parent_parity", "artifacts/verification/fidelity_parity_hummingbird_euler/report.json"),
        ("native_semantic_parent", "artifacts/showcases/alpha2/final-catalog-v1/packs/hummingbird-pad-to-pad-altitude-yaw-v2/summary.json"),
        ("native_pseudo_channel_comparison", "verification/alpha3_hummingbird_native_comparison/manifest.json"),
        ("translated_physical_witness", "verification/alpha3_hummingbird_translated_physical/manifest.json"),
        ("directional_translation_witness", "verification/alpha3_hummingbird_directional/manifest.json"),
        ("native_horizontal_translation_witness", "verification/alpha3_hummingbird_native_horizontal/manifest.json"),
        ("native_vertical_force_witness", "verification/alpha3_hummingbird_native_vertical/manifest.json"),
    ),
    "f16_s119": (("physical_surface_r1", "verification/f16_racetrack_robustness_evidence.json"),),
    "tumbling_body": (
        ("passive_r1_matrix", "verification/alpha3_tumbling_body/r1_matrix/manifest.json"),
        ("passive_shape_ensemble", "verification/alpha3_tumbling_body/qualification.json"),
    ),
}

DIRECT_WRENCH_ARTIFACTS: dict[str, str] = {
    "skywalker_x8": "verification/alpha3_x8_direct_wrench/manifest.json",
    "b747": "artifacts/showcases/airbreathing-racetrack-fidelity-ladder/b747-racetrack-altitude-turns-6dof-v1/summary.json",
    "a320": "verification/alpha3_a320_direct_wrench/manifest.json",
    "f16_s119": "verification/alpha3_f16_direct_wrench/manifest.json",
}

REGIME_DIRECT_WRENCH_READINESS = ROOT / "verification/alpha3_regime_direct_wrench_readiness/manifest.json"

PHYSICAL_EFFECTOR_ARTIFACTS: dict[str, str] = {
    "skywalker_x8": "verification/generated/x8_table_coordinate_physical_lqr.json",
    "b747": "verification/generated/b747_condition3_physical_surface_lqr.json",
    "hummingbird": "verification/generated/hummingbird_individual_rotor_physical_lqr.json",
    "x15": "verification/generated/x15_physical_lqr_readiness.json",
}

PHYSICAL_R1_ARTIFACTS: dict[str, str] = {
    "skywalker_x8": "verification/alpha3_x8_physical_r1/manifest.json",
    "b747": "verification/alpha3_b747_physical_r1/manifest.json",
    "f16_s119": "verification/alpha3_f16_physical_r1/manifest.json",
    "hummingbird": "verification/alpha3_hummingbird_physical_r1/manifest.json",
}

PHYSICAL_SCHEDULE_ARTIFACTS: dict[str, str] = {
    "b747": "verification/alpha3_b747_physical_schedule/manifest.json",
    "f16_s119": "verification/alpha3_f16_physical_schedule/manifest.json",
}

PHYSICAL_SCHEDULE_TRANSITION_ARTIFACTS: dict[str, str] = {
    "f16_s119": "verification/alpha3_f16_physical_schedule_transition/manifest.json",
}

PHYSICAL_SCHEDULE_ENVELOPE_ARTIFACTS: dict[str, str] = {
    "f16_s119": "verification/alpha3_f16_physical_schedule_envelope/manifest.json",
}
PHYSICAL_SCHEDULE_INTERIOR_ARTIFACTS: dict[str, str] = {
    "f16_s119": "verification/alpha3_f16_physical_schedule_envelope/interior_qualification.json",
}

DIRECTIONAL_TRANSLATION_ARTIFACTS: dict[str, str] = {
    "hummingbird": "verification/alpha3_hummingbird_directional/manifest.json",
}

NATIVE_HORIZONTAL_ARTIFACTS: dict[str, str] = {
    "hummingbird": "verification/alpha3_hummingbird_native_horizontal/manifest.json",
}

NATIVE_VERTICAL_ARTIFACTS: dict[str, str] = {
    "hummingbird": "verification/alpha3_hummingbird_native_vertical/manifest.json",
}

NATIVE_PAD_TO_PAD_ARTIFACTS: dict[str, str] = {
    "hummingbird": "verification/alpha3_hummingbird_native_pad_to_pad/hummingbird-pad-to-pad-altitude-yaw-individual-rotor-v1/summary.json",
}

PHYSICAL_MAPPING_ARTIFACTS: dict[str, str] = {
    "skywalker_x8": "verification/alpha3_x8_physical_mapping/manifest.json",
}

REDUCED_R1_ARTIFACTS: dict[str, str] = {
    "a320": "verification/alpha3_a320_r1/manifest.json",
    "skywalker_x8": "verification/alpha3_airbreathing_r1/manifest.json",
    "b747": "verification/alpha3_airbreathing_r1/manifest.json",
    "f16_s119": "verification/alpha3_f16_r1/manifest.json",
    "hl20_mod_k": "verification/alpha3_hl20_r1/manifest.json",
    "hummingbird": "verification/alpha3_hummingbird_r1/manifest.json",
    "x15": "verification/alpha3_x15_r1/manifest.json",
    "reference_nesc_two_stage_rocket": "verification/alpha3_nesc_r1/manifest.json",
    "tumbling_body": "verification/alpha3_tumbling_body/r1_matrix/manifest.json",
}


def _load(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(payload, dict)
    return payload
    ####


def _artifact_status(family_id: str) -> dict[str, object]:
    available: list[dict[str, object]] = []
    for artifact_id, relative_path in SUPPORTING_ARTIFACTS.get(family_id, ()):
        path = ROOT / relative_path
        if path.is_file():
            payload = _load(path)
            available.append(
                {
                    "id": artifact_id,
                    "artifact": relative_path,
                    "schema": payload.get("schema", payload.get("schema_version")),
                    "status": payload.get("status", payload.get("parity_gate", "available")),
                }
            )
    if family_id in {"a320", "skywalker_x8", "b747"} and available:
        r1_status = "paired_reduced_r1_complete"
        interpretation = "The paired 3DOF/pseudo fixed initial-state R1 matrix is complete; physical-effector claims remain separate."
    elif family_id == "hl20_mod_k" and available:
        r1_status = "paired_reduced_r1_complete"
        interpretation = "The paired passive/open-loop reduced R1 matrix is complete; source-exact controlled-flight claims remain separate."
    elif family_id == "hummingbird" and any(item["id"] == "paired_reduced_r1" for item in available):
        r1_status = "paired_reduced_r1_complete"
        interpretation = "The aggregate-thrust pseudo and independent translation-only force-model R1 matrix is complete; individual-rotor claims remain separate."
    elif family_id == "x15" and available:
        r1_status = "paired_reduced_r1_complete"
        interpretation = "The staged reduced-order event-chain R1 matrix is complete; controlled terminal-handoff and physical-effector claims remain separate."
    elif family_id == "reference_nesc_two_stage_rocket" and available:
        r1_status = "paired_reduced_r1_complete"
        interpretation = "The source-replay and bounded response-law R1 matrix is complete; source-exact gimbal effectiveness and physical thrust-vector allocation remain separate."
    elif family_id == "tumbling_body" and any(item["id"] == "passive_r1_matrix" for item in available):
        r1_status = "passive_reduced_r1_complete"
        interpretation = "The passive shape/uncertainty matrix is complete; this is not controller robustness or parent capability qualification."
    elif family_id == "f16_s119" and available:
        r1_status = "boundary_witness_available"
        interpretation = "Physical-surface R1 evidence exists, but it is not inherited by the reduced 3DOF/pseudo pair."
    elif family_id == "hummingbird" and available:
        r1_status = "parent_evidence_attached_not_r1"
        interpretation = "Native hover and semantic parent evidence exist; a fixed disturbance/authority matrix is still absent."
    elif family_id == "tumbling_body" and available:
        r1_status = "passive_ensemble_available_not_r1"
        interpretation = "Passive shape ensemble evidence exists; this is not a controller robustness result."
    else:
        r1_status = "not_run"
        interpretation = "No fixed perturbation matrix is attached to the paired fidelity records."
    return {"status": r1_status, "interpretation": interpretation, "artifacts": available}
    ####


def _direct_wrench_status(family_id: str) -> dict[str, object]:
    """Index direct-wrench evidence without confusing it with R1 robustness."""

    relative_path = DIRECT_WRENCH_ARTIFACTS.get(family_id)
    if relative_path is None:
        return {"status": "not_run", "artifact": None}
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    evaluation = payload.get("truth_evaluation", payload.get("evaluation", {}))
    claim = payload.get("claim", {})
    if not isinstance(evaluation, dict):
        evaluation = {}
    if not isinstance(claim, dict):
        claim = {}
    status = str(payload.get("status", claim.get("status", "available")))
    return {
        "status": status,
        "artifact": relative_path,
        "fidelity": payload.get("fidelity", payload.get("fidelity_tier")),
        "mission_pass": payload.get(
            "mission_pass",
            evaluation.get("mission_pass", status == "nominal_case_pass"),
        ),
        "evidence_tier": payload.get("evidence_level", claim.get("evidence_tier")),
        "direct_body_moment_injection": payload.get("direct_body_moment_injection", claim.get("direct_body_moment_injection", True)),
        "physical_effector_allocation": payload.get("physical_effector_allocation", claim.get("physical_effector_allocation", False)),
        "control_path": payload.get("control_path"),
        "claim_boundary": payload.get("nonclaims", payload.get("claim_boundary")),
    }
    ####


def _regime_direct_wrench_status(family_id: str) -> dict[str, object]:
    """Index regime-family readiness without treating it as a direct pass."""

    if not REGIME_DIRECT_WRENCH_READINESS.is_file():
        return {"status": "not_run", "artifact": str(REGIME_DIRECT_WRENCH_READINESS.relative_to(ROOT))}
    payload = _load(REGIME_DIRECT_WRENCH_READINESS)
    records = payload.get("records", [])
    if not isinstance(records, list):
        return {"status": "malformed", "artifact": str(REGIME_DIRECT_WRENCH_READINESS.relative_to(ROOT))}
    record = next((item for item in records if isinstance(item, dict) and item.get("family_id") == family_id), None)
    if record is None:
        return {"status": "not_applicable", "artifact": str(REGIME_DIRECT_WRENCH_READINESS.relative_to(ROOT))}
    return {
        "status": record.get("status"),
        "artifact": str(REGIME_DIRECT_WRENCH_READINESS.relative_to(ROOT)),
        "fidelity": record.get("fidelity"),
        "evidence_tier": record.get("evidence_tier"),
        "direct_wrench_mode": record.get("direct_wrench_mode"),
        "mission_pass": record.get("mission_pass"),
        "blocker": record.get("blocker"),
        "required_inputs_to_promote": record.get("required_inputs_to_promote"),
    }
    ####


def _physical_effector_status(family_id: str) -> dict[str, object]:
    """Index existing physical-control evidence without promoting it silently."""

    relative_path = PHYSICAL_EFFECTOR_ARTIFACTS.get(family_id)
    if relative_path is None:
        return {
            "status": "not_run",
            "evidence_tier": None,
            "artifact": None,
            "interpretation": "No physical-effector artifact is attached to this family yet.",
        }
    path = ROOT / relative_path
    if not path.is_file():
        return {
            "status": "not_run",
            "evidence_tier": None,
            "artifact": relative_path,
            "interpretation": "The declared physical-effector artifact is missing.",
        }
    payload = _load(path)
    claim = payload.get("claim")
    if not isinstance(claim, dict):
        if family_id == "x15" and payload.get("status") == "blocked_before_T1_trim":
            return {
                "status": "blocked_before_T1_trim",
                "evidence_tier": "T0_structural",
                "artifact": relative_path,
                "direct_body_moment_injection": False,
                "promotion_blocker": "source-backed nonlinear trim prerequisite is not satisfied",
                "physical_allocation_evidence": None,
                "interpretation": "A fail-closed structural readiness record exists; no physical-effector controller was attempted.",
            }
        return {
            "status": "malformed",
            "evidence_tier": None,
            "artifact": relative_path,
            "interpretation": "The artifact exists but has no structured claim mapping.",
        }
    observed_status = str(claim.get("status", payload.get("status", "available")))
    return {
        "status": observed_status,
        "evidence_tier": claim.get("earned_controller_evidence_tier"),
        "artifact": relative_path,
        "direct_body_moment_injection": claim.get("direct_body_moment_injection"),
        "promotion_blocker": claim.get("promotion_blocker"),
        "physical_allocation_evidence": claim.get("physical_allocation_evidence"),
        "interpretation": "Physical-effector evidence is local to the declared operating point and does not imply family or envelope qualification.",
    }
    ####


def _reduced_r1_status(family_id: str) -> dict[str, object] | None:
    """Return a reduced-tier R1 record when a family-specific matrix exists."""

    relative_path = REDUCED_R1_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    status = str(payload.get("status", "available"))
    return {
        "status": "complete" if status in {"R1_fixed_matrix_complete", "PASSIVE_R1_MATRIX_COMPLETE"} else "boundary_recorded",
        "artifact": relative_path,
        "case_count": payload.get("case_count"),
        "passed_case_count": payload.get("passed_case_count"),
        "failed_case_count": payload.get("failed_case_count"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_r1_status(family_id: str) -> dict[str, object]:
    """Index a family-specific physical-effector R1 matrix when available."""

    relative_path = PHYSICAL_R1_ARTIFACTS.get(family_id)
    if relative_path is None:
        return {"status": "not_run", "artifact": None}
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "case_count": payload.get("case_count"),
        "passed_case_count": payload.get("passed_case_count"),
        "failed_case_count": payload.get("failed_case_count"),
        "boundary_failure_count": payload.get("boundary_failure_count"),
        "direct_body_moment_injection": payload.get("direct_body_moment_injection"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_schedule_status(family_id: str) -> dict[str, object] | None:
    """Index schedule-node evidence without promoting it to runtime scheduling."""

    relative_path = PHYSICAL_SCHEDULE_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    summary = payload.get("summary", {})
    contract = payload.get("schedule_contract", {})
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "node_count": summary.get("node_count"),
        "passed_node_count": summary.get("passed_node_count"),
        "case_count": summary.get("case_count"),
        "failed_case_count": summary.get("failed_case_count"),
        "transition_probe_count": summary.get("transition_probe_count"),
        "transition_probe_passed_count": summary.get("transition_probe_passed_count"),
        "transition_probe_failed_count": summary.get("transition_probe_failed_count"),
        "blockers_by_code": summary.get("blockers_by_code", {}),
        "blocked_nodes": [
            {
                "point_id": node.get("point_id"),
                "code": node.get("blocker", {}).get("code"),
                "worst_residual_name": node.get("blocker", {}).get("details", {}).get("worst_residual_name"),
                "scaled_residual_norm": node.get("blocker", {}).get("details", {}).get("scaled_residual_norm"),
                "hint": node.get("blocker", {}).get("hint"),
            }
            for node in payload.get("nodes", [])
            if isinstance(node, dict) and isinstance(node.get("blocker"), dict)
        ],
        "boundary_nodes": [
            {
                "point_id": node.get("point_id"),
                "reason": node.get("reason"),
                "controller_trial_count": len(node.get("controller_trials", [])),
                "interior_passed": node.get("interior_passed", False),
            }
            for node in payload.get("nodes", [])
            if isinstance(node, dict) and node.get("status") == "physical_surface_node_boundary"
        ],
        "runtime_gain_interpolation": contract.get("runtime_gain_interpolation"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_schedule_transition_status(family_id: str) -> dict[str, object] | None:
    """Index time-marching schedule evidence without granting envelope status."""

    relative_path = PHYSICAL_SCHEDULE_TRANSITION_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    summary = payload.get("summary", {})
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "case_count": summary.get("case_count"),
        "passed_case_count": summary.get("passed_case_count"),
        "failed_case_count": summary.get("failed_case_count"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_schedule_envelope_status(family_id: str) -> dict[str, object] | None:
    """Index local schedule-envelope witnesses without calling them full envelope validation."""

    relative_path = PHYSICAL_SCHEDULE_ENVELOPE_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    summary = payload.get("summary", {})
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "node_count": summary.get("node_count"),
        "case_count": summary.get("case_count"),
        "passed_case_count": summary.get("passed_case_count"),
        "boundary_case_count": summary.get("boundary_case_count"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_schedule_interior_status(family_id: str) -> dict[str, object] | None:
    """Index conservative schedule-wide interior evidence separately from boundaries."""

    relative_path = PHYSICAL_SCHEDULE_INTERIOR_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    summary = payload.get("summary", {})
    interior = payload.get("validated_interior", {})
    boundaries = payload.get("boundary_witnesses", {})
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "node_count": summary.get("node_count"),
        "interior_case_ids": interior.get("case_ids", []),
        "interior_case_count": summary.get("interior_case_count"),
        "boundary_case_ids": boundaries.get("case_ids", []),
        "boundary_case_count": summary.get("schedule_wide_boundary_case_count"),
        "direct_body_moment_injection": payload.get("direct_body_moment_injection"),
        "claim_boundary": payload.get("claim_boundary"),
    }
    ####


def _physical_mapping_status(family_id: str) -> dict[str, object] | None:
    """Index explicit physical-surface mapping evidence without promotion."""

    relative_path = PHYSICAL_MAPPING_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    claim = payload.get("claim", {})
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "promotion_authorized": claim.get("promotion_authorized"),
        "promotion_blocker": claim.get("promotion_blocker"),
        "hypothesis_count": len(payload.get("hypotheses", [])),
        "claim_boundary": claim.get("nonclaims"),
    }
    ####


def _directional_translation_status(family_id: str) -> dict[str, object] | None:
    """Index a directional mission witness without promoting rotor physics."""

    relative_path = DIRECTIONAL_TRANSLATION_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    records = payload.get("fidelity_records", [])
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "mission_id": payload.get("mission_id"),
        "objective_count": payload.get("objective_count"),
        "passed_objective_count": payload.get("passed_objective_count"),
        "fidelity_records": records,
        "physical_motor_allocation": payload.get("physical_motor_allocation"),
        "claim_boundary": "directional point-mass and aggregate pseudo evidence only; native rotor translation and electrical battery remain separate",
    }
    ####


def _native_horizontal_status(family_id: str) -> dict[str, object] | None:
    """Index the native source-plant horizontal witness without hiding its boundary."""

    relative_path = NATIVE_HORIZONTAL_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    evidence_path = ROOT / str(relative_path).replace("manifest.json", "evidence.json")
    evidence = _load(evidence_path) if evidence_path.is_file() else {}
    evaluation = evidence.get("evaluation", {})
    allocation_summary = evaluation.get("allocation_summary", {}) if isinstance(evaluation, dict) else {}
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "mission_pass": payload.get("mission_pass"),
        "objective_count": payload.get("objective_count"),
        "passed_objective_count": payload.get("passed_objective_count"),
        "physical_motor_allocation": payload.get("physical_motor_allocation"),
        "direct_body_moment_injection": payload.get("direct_body_moment_injection"),
        "allocation_summary": allocation_summary,
        "claim_boundary": payload.get("promotion_boundary"),
    }
    ####


def _native_vertical_status(family_id: str) -> dict[str, object] | None:
    """Index the native source-plant vertical-force witness explicitly."""

    relative_path = NATIVE_VERTICAL_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    evidence_path = ROOT / str(relative_path).replace("manifest.json", "evidence.json")
    evidence = _load(evidence_path) if evidence_path.is_file() else {}
    evaluation = evidence.get("evaluation", {})
    allocation_summary = evaluation.get("allocation_summary", {}) if isinstance(evaluation, dict) else {}
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "mission_pass": payload.get("mission_pass"),
        "objective_count": payload.get("objective_count"),
        "passed_objective_count": payload.get("passed_objective_count"),
        "physical_motor_allocation": payload.get("physical_motor_allocation"),
        "direct_body_force_injection": payload.get("direct_body_force_injection"),
        "direct_body_moment_injection": payload.get("direct_body_moment_injection"),
        "allocation_summary": allocation_summary,
        "claim_boundary": payload.get("promotion_boundary"),
    }
    ####


def _native_pad_to_pad_status(family_id: str) -> dict[str, object] | None:
    """Index the rotor-resolved end-to-end Hummingbird packet."""

    relative_path = NATIVE_PAD_TO_PAD_ARTIFACTS.get(family_id)
    if relative_path is None:
        return None
    path = ROOT / relative_path
    if not path.is_file():
        return {"status": "not_run", "artifact": relative_path}
    payload = _load(path)
    evaluation = payload.get("truth_evaluation", {})
    results = evaluation.get("results", []) if isinstance(evaluation, dict) else []
    return {
        "status": payload.get("status", "available"),
        "artifact": relative_path,
        "mission_id": payload.get("mission_id"),
        "mission_pass": payload.get("mission_pass"),
        "objective_count": len(results),
        "passed_objective_count": sum(item.get("status") == "pass" for item in results if isinstance(item, dict)),
        "fidelity": payload.get("fidelity"),
        "control_path": payload.get("control_path"),
        "landing_evidence": payload.get("landing_evidence"),
        "claim_boundary": payload.get("nonclaims"),
    }
    ####


def build_report() -> dict[str, object]:
    """Build the deterministic readiness report from generated manifests."""

    index = _load(INDEX)
    cross = _load(CROSS_FIDELITY)
    records = index.get("records", [])
    assert isinstance(records, list)
    by_family: dict[str, list[dict[str, Any]]] = {}
    for record in records:
        assert isinstance(record, dict)
        by_family.setdefault(str(record["family_id"]), []).append(record)
    cross_by_family = {
        str(item["family_id"]): item
        for item in cross.get("reports", [])
        if isinstance(item, dict) and "family_id" in item
    }
    families: list[dict[str, object]] = []
    for family_id in sorted(R1_CASES):
        family_records = by_family.get(family_id, [])
        fidelities = {str(record["fidelity"]): record for record in family_records}
        nominal_pair = all(fidelities.get(fidelity, {}).get("status") == "nominal_case_pass" for fidelity in ("point_mass_3dof", "pseudo_6dof"))
        cross_report = cross_by_family.get(family_id, {})
        r1 = _artifact_status(family_id)
        direct_wrench = _direct_wrench_status(family_id)
        regime_direct_wrench = _regime_direct_wrench_status(family_id)
        physical_effector = _physical_effector_status(family_id)
        reduced_r1 = _reduced_r1_status(family_id)
        physical_r1 = _physical_r1_status(family_id)
        physical_schedule = _physical_schedule_status(family_id)
        physical_schedule_transition = _physical_schedule_transition_status(family_id)
        physical_schedule_envelope = _physical_schedule_envelope_status(family_id)
        physical_schedule_interior = _physical_schedule_interior_status(family_id)
        physical_mapping = _physical_mapping_status(family_id)
        directional_translation = _directional_translation_status(family_id)
        native_horizontal = _native_horizontal_status(family_id)
        native_vertical = _native_vertical_status(family_id)
        native_pad_to_pad = _native_pad_to_pad_status(family_id)
        passive_pair_ready = (
            family_id == "tumbling_body"
            and all(fidelities.get(fidelity, {}).get("mission_pass") is True for fidelity in ("point_mass_3dof", "pseudo_6dof"))
            and r1["status"] == "passive_reduced_r1_complete"
        )
        solid_pair_ready = (
            all(fidelity in fidelities for fidelity in ("point_mass_3dof", "pseudo_6dof"))
            and all(fidelities[fidelity].get("mission_pass") is True for fidelity in ("point_mass_3dof", "pseudo_6dof"))
            and cross_report.get("status") == "pass"
        )
        pair_outcome = (
            "nominal_pair_ready"
            if nominal_pair
            else "passive_pair_ready"
            if passive_pair_ready
            else "development_pair"
            if solid_pair_ready
            else "incomplete_pair"
        )
        families.append(
            {
                "family_id": family_id,
                "fidelity_records": family_records,
                "paired_fidelity": all(fidelity in fidelities for fidelity in ("point_mass_3dof", "pseudo_6dof")),
                "solid_pair_ready": solid_pair_ready,
                "pair_outcome": pair_outcome,
                "nominal_pair_ready": nominal_pair,
                "passive_pair_ready": passive_pair_ready,
                "cross_fidelity_status": cross_report.get("status", "missing"),
                "r1_fixed_matrix": r1,
                "direct_wrench": direct_wrench,
                "regime_direct_wrench": regime_direct_wrench,
                "physical_effector": physical_effector,
                "physical_r1": physical_r1,
                "physical_schedule": physical_schedule,
                "physical_schedule_transition": physical_schedule_transition,
                "physical_schedule_envelope": physical_schedule_envelope,
                "physical_schedule_interior": physical_schedule_interior,
                "physical_mapping": physical_mapping,
                "directional_translation": directional_translation,
                "native_horizontal": native_horizontal,
                "native_vertical": native_vertical,
                "native_pad_to_pad": native_pad_to_pad,
                "reduced_r1": reduced_r1,
                "required_r1_cases": list(R1_CASES[family_id]),
                "next_gate": NEXT_GATES[family_id],
                # Passive-body pseudo evidence is native rigid-body reuse, not
                # an attitude-response/controller fallback.  Keep the pair
                # nominal for its declared passive contract but never let the
                # controller lowering path select it automatically.
                "automatic_lowering_authorized": nominal_pair and not passive_pair_ready,
            }
        )
    nominal_count = sum(bool(family["nominal_pair_ready"]) for family in families)
    passive_pair_count = sum(bool(family["passive_pair_ready"]) for family in families)
    r1_statuses = [cast(dict[str, object], family["r1_fixed_matrix"])["status"] for family in families]
    physical_statuses = [cast(dict[str, object], family["physical_effector"]) for family in families]
    direct_wrench_statuses = [cast(dict[str, object], family["direct_wrench"]) for family in families]
    regime_direct_wrench_statuses = [cast(dict[str, object], family["regime_direct_wrench"]) for family in families]
    r1_count = sum(status == "boundary_witness_available" for status in r1_statuses)
    r1_matrix_count = sum(status in {"paired_reduced_r1_complete", "boundary_witness_available", "passive_reduced_r1_complete"} for status in r1_statuses)
    reduced_r1_complete_count = sum(
        isinstance(family.get("reduced_r1"), dict)
        and cast(dict[str, object], family["reduced_r1"])["status"] == "complete"
        for family in families
    )
    physical_count = sum(
        item["status"] not in {"not_run", "malformed"} for item in physical_statuses
    )
    physical_t5_count = sum(
        item["evidence_tier"] == "T5_nonlinearly_validated" for item in physical_statuses
    )
    physical_r1_count = sum(
        cast(dict[str, object], family["physical_r1"]).get("status")
        in {
            "R1_physical_surface_matrix_complete",
            "R1_physical_individual_rotor_matrix_complete",
            "R1_physical_effector_matrix_complete",
            "R1_physical_source_coordinate_matrix_complete",
        }
        for family in families
    )
    physical_schedule_envelope_count = sum(
        isinstance(family.get("physical_schedule_envelope"), dict)
        and cast(dict[str, object], family["physical_schedule_envelope"]).get("status") not in {"not_run", "malformed"}
        for family in families
    )
    return {
        "schema": "taoryx.alpha3-readiness-matrix/v1alpha1",
        "status": "development_readiness_report",
        "claim_boundary": "This matrix inventories evidence and missing gates. A nominal paired-fidelity pass is not R1 robustness or family qualification.",
        "source_manifests": {
            "fidelity_ladder": INDEX.relative_to(ROOT).as_posix(),
            "cross_fidelity": CROSS_FIDELITY.relative_to(ROOT).as_posix(),
        },
        "summary": {
            "family_count": len(families),
            "paired_fidelity_count": sum(bool(family["paired_fidelity"]) for family in families),
            "solid_pair_ready_count": sum(bool(family["solid_pair_ready"]) for family in families),
            "nominal_pair_ready_count": nominal_count,
            "passive_pair_ready_count": passive_pair_count,
            "r1_boundary_witness_count": r1_count,
            "r1_matrix_complete_count": r1_matrix_count,
            "r1_pending_count": len(families) - r1_matrix_count,
            "reduced_r1_complete_count": reduced_r1_complete_count,
            "physical_effector_witness_count": physical_count,
            "direct_wrench_witness_count": sum(item["status"] not in {"not_run", "malformed"} for item in direct_wrench_statuses),
            "direct_wrench_nominal_pass_count": sum(bool(item.get("mission_pass")) for item in direct_wrench_statuses),
            "regime_direct_wrench_record_count": sum(item.get("status") not in {"not_run", "malformed", "not_applicable"} for item in regime_direct_wrench_statuses),
            "regime_direct_wrench_promoted_count": sum(bool(item.get("mission_pass")) for item in regime_direct_wrench_statuses),
            "physical_effector_t5_count": physical_t5_count,
            "physical_r1_matrix_count": physical_r1_count,
            "physical_schedule_envelope_count": physical_schedule_envelope_count,
            "physical_schedule_interior_count": sum(
                isinstance(family.get("physical_schedule_interior"), dict)
                and cast(dict[str, object], family["physical_schedule_interior"]).get("status")
                == "F16_physical_schedule_interior_qualified"
                for family in families
            ),
            "directional_translation_witness_count": sum(
                isinstance(family.get("directional_translation"), dict)
                and cast(dict[str, object], family["directional_translation"]).get("status")
                == "directional_translation_witness_pass"
                for family in families
            ),
            "native_horizontal_witness_count": sum(
                isinstance(family.get("native_horizontal"), dict)
                and cast(dict[str, object], family["native_horizontal"]).get("mission_pass") is True
                for family in families
            ),
            "native_vertical_witness_count": sum(
                isinstance(family.get("native_vertical"), dict)
                and cast(dict[str, object], family["native_vertical"]).get("mission_pass") is True
                for family in families
            ),
            "native_pad_to_pad_witness_count": sum(
                isinstance(family.get("native_pad_to_pad"), dict)
                and cast(dict[str, object], family["native_pad_to_pad"]).get("mission_pass") is True
                for family in families
            ),
            "cross_fidelity_pass_count": sum(family["cross_fidelity_status"] == "pass" for family in families),
        },
        "families": families,
        "interpretation": "Automatic lowering may use only the evidence-keyed nominal pairs. R1 and higher claims require family-specific reruns and are never inferred from this index.",
    }
    ####


def main() -> int:
    """Write the Alpha 3 readiness matrix."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    arguments = parser.parse_args()
    arguments.output.mkdir(parents=True, exist_ok=True)
    report = build_report()
    path = arguments.output / "manifest.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0
    ####


if __name__ == "__main__":
    raise SystemExit(main())
####
