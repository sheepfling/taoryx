from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from taoryx.control_allocation import EffectorLimits
from taoryx.trajectory import (
    F16AttitudeResponsePseudo6DOFModel,
    F16PointMass3DOFModel,
    F16ReferencePhysicalPlant,
    F16ReferenceRigidBodyPlant,
    load_f16_reference_plant,
)
from taoryx.trim import TrimResult, TrimSpec

ROOT = Path(__file__).resolve().parents[2]


def test_f16_reference_plant_closes_corrected_trim_in_body_frame() -> None:
    """The runtime adapter reproduces the corrected source-backed trim."""

    plant = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    evidence = json.loads(
        (ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(encoding="utf-8")
    )
    state = {
        "u_m_s": evidence["resolved_state"]["body_velocity_m_s"]["u"],
        "v_m_s": evidence["resolved_state"]["body_velocity_m_s"]["v"],
        "w_m_s": evidence["resolved_state"]["body_velocity_m_s"]["w"],
        "p_rad_s": evidence["resolved_state"]["body_rates_rad_s"]["p"],
        "q_rad_s": evidence["resolved_state"]["body_rates_rad_s"]["q"],
        "r_rad_s": evidence["resolved_state"]["body_rates_rad_s"]["r"],
    }
    controls = {
        "elevator_deg": evidence["controls"]["elevator_deg"],
        "aileron_deg": evidence["controls"].get("aileron_deg", 0.0),
        "rudder_deg": evidence["controls"].get("rudder_deg", 0.0),
        "throttle_fraction": evidence["control_contract"]["throttle_fraction"],
    }
    loads = plant.evaluate_loads(state, controls)
    derivatives = plant.state_derivative(
        state,
        controls,
        pitch_rad=evidence["resolved_state"]["attitude"]["pitch_rad"],
    )
    assert loads["speed_m_s"] == pytest.approx(152.4, abs=1.0e-10)
    assert derivatives["u_m_s"] == pytest.approx(0.0, abs=3.0e-10)
    assert derivatives["w_m_s"] == pytest.approx(0.0, abs=3.0e-10)
    assert derivatives["q_rad_s"] == pytest.approx(0.0, abs=3.0e-10)
    assert derivatives["v_m_s"] == pytest.approx(0.0, abs=1.0e-10)
    assert derivatives["p_rad_s"] == pytest.approx(0.0, abs=1.0e-10)
    assert derivatives["r_rad_s"] == pytest.approx(0.0, abs=1.0e-10)
    ####


def test_f16_rigid_body_wrapper_closes_local_trim_with_full_inertia() -> None:
    """The shared rigid-body state reaches the same source-backed trim."""

    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    evidence = json.loads(
        (ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(encoding="utf-8")
    )
    controls = {
        "elevator_deg": evidence["controls"]["elevator_deg"],
        "aileron_deg": 0.0,
        "rudder_deg": 0.0,
        "throttle_fraction": evidence["control_contract"]["throttle_fraction"],
    }
    wrapper = F16ReferenceRigidBodyPlant(source, controls)
    state = wrapper.initial_state(
        true_airspeed_m_s=evidence["operating_point"]["true_airspeed_m_s"],
        alpha_rad=evidence["resolved_state"]["attitude"]["pitch_rad"],
    )
    model = wrapper.model()
    rates = model.derivative(state)
    observables = model.observables(state)

    assert wrapper.body_state(state)["u_m_s"] == pytest.approx(
        evidence["resolved_state"]["body_velocity_m_s"]["u"], abs=1.0e-10
    )
    assert wrapper.body_state(state)["w_m_s"] == pytest.approx(
        evidence["resolved_state"]["body_velocity_m_s"]["w"], abs=1.0e-10
    )
    assert rates[3:6] == pytest.approx((0.0, 0.0, 0.0), abs=3.0e-10)
    assert rates[10:13] == pytest.approx((0.0, 0.0, 0.0), abs=3.0e-10)
    assert observables["translation_equation_residual_normalized"] < 3.0e-12
    assert observables["rotation_equation_residual_normalized"] < 3.0e-12
    assert observables["inertia_xz_kg_m2"] == pytest.approx(source.inertia_matrix_kg_m2[0][2])
    assert abs(observables["inertia_xz_kg_m2"]) > 1.0
    ####


def test_f16_local_linearization_comes_from_runtime_derivatives() -> None:
    """The controller matrix is derived twice from the actual source plant."""

    plant = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    evidence = json.loads(
        (ROOT / "verification/daveml_f16_equilibrium_trim_evidence.json").read_text(encoding="utf-8")
    )
    state = evidence["resolved_state"]["body_velocity_m_s"] | evidence["resolved_state"]["body_rates_rad_s"]
    state = {"u_m_s": state["u"], "v_m_s": state["v"], "w_m_s": state["w"], "p_rad_s": state["p"], "q_rad_s": state["q"], "r_rad_s": state["r"]}
    controls = {
        "elevator_deg": evidence["controls"]["elevator_deg"],
        "aileron_deg": 0.0,
        "rudder_deg": 0.0,
        "throttle_fraction": evidence["control_contract"]["throttle_fraction"],
    }
    result = plant.linearize_local(
        state,
        controls,
        trim_pitch_rad=evidence["resolved_state"]["attitude"]["pitch_rad"],
        state_step=1.0e-5,
        control_step=1.0e-5,
    )

    assert result.primary.a_matrix.shape == (6, 6)
    assert result.primary.b_matrix.shape == (6, 4)
    assert result.provenance.nonlinear_plant_id == "reference-f16-s119-source-runtime-plant"
    assert result.provenance.derivative_consistent
    assert result.primary.metadata_dict["linearization_scope"] == "fixed_altitude_local_body_dynamics"
    assert abs(float(result.primary.b_matrix[4, 0])) > 1.0e-4
    assert abs(float(result.primary.b_matrix[1, 2])) > 1.0e-4
    ####


def test_f16_runtime_linearization_artifact_declares_its_lower_claim_boundary() -> None:
    """The generated matrix is not confused with actuator or mission proof."""

    artifact = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "verified"
    assert artifact["plant_id"] == "reference-f16-s119-source-runtime-plant"
    assert artifact["derivative_consistency"]["passed"] is True
    assert "not scheduled control" in artifact["claim_boundary"]
    assert "nonlinear mission qualification" in artifact["claim_boundary"]
    ####


def test_f16_lqr_trim_hold_artifact_is_explicitly_development_only() -> None:
    """The first closed-loop screen records bounded effectors without overclaiming."""

    artifact = json.loads(
        (ROOT / "verification/f16_lqr_trim_hold_evidence.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "development_screen_passed"
    assert artifact["closed_loop"]["hurwitz"] is True
    assert artifact["control_path"] == "lqr_to_bounded_direct_effectors_to_source_nonlinear_plant"
    assert artifact["metrics"]["final_error_norm_mixed_units"] < artifact["metrics"]["final_error_limit_mixed_units"]
    assert artifact["metrics"]["saturation_fraction"] == 0.0
    assert "not scheduled control" in artifact["claim_boundary"]
    assert "wrench allocation" in artifact["claim_boundary"]
    assert "flight qualification" in artifact["claim_boundary"]
    ####


def test_f16_physical_adapter_derives_full_rank_local_wrench_effectiveness() -> None:
    """The source load perturbations produce an auditable four-axis adapter."""

    linearization = json.loads(
        (ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8")
    )
    state = {name: float(value) for name, value in linearization["trim_state"].items()}
    controls = {name: float(value) for name, value in linearization["trim_controls"].items()}
    state_names = tuple(state)
    control_names = tuple(controls)
    trim_spec = TrimSpec(
        state_names=state_names,
        control_names=control_names,
        residual_names=state_names,
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": float(linearization["metadata"]["trim_pitch_rad"])},
    )
    trim = TrimResult(
        trim_spec,
        state,
        controls,
        {name: 0.0 for name in state_names},
        0.0,
        True,
        1,
        "source operating point",
        0,
        0.0,
    )
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    limits = {
        name: EffectorLimits(
            name,
            -24.0 if name != "throttle_fraction" else 0.0,
            24.0 if name != "throttle_fraction" else 1.0,
            "fraction" if name == "throttle_fraction" else "deg",
            1.0 if name == "throttle_fraction" else 60.0,
            0.20 if name == "throttle_fraction" else 0.05,
        )
        for name in control_names
    }
    adapter = F16ReferencePhysicalPlant(
        source,
        trim,
        float(linearization["metadata"]["trim_pitch_rad"]),
        0.0,
        limits,
    )
    effectiveness = adapter.effectiveness(state, controls)
    allocation = adapter.allocate(state, effectiveness.reference_wrench, controls, 0.01)

    assert effectiveness.array.shape == (4, 4)
    assert np.linalg.matrix_rank(effectiveness.array) == 4
    assert allocation.allocation.status == "feasible"
    assert allocation.achieved_controlled_residual_norm < 1.0e-6
    assert effectiveness.source == "f16-s119-source-load-centered-finite-difference-v1"
    ####


def test_f16_physical_wrench_lqr_artifact_closes_local_recovery() -> None:
    """The allocator-backed LQR witness remains narrow and auditable."""

    artifact = json.loads(
        (ROOT / "verification/f16_physical_wrench_lqr_evidence.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "development_screen_passed"
    assert artifact["control_path"] == "lqr_to_desired_wrench_to_bounded_effectors_to_source_nonlinear_plant"
    assert artifact["metrics"]["allocation_statuses"] == ["feasible"]
    assert artifact["metrics"]["saturation_fraction"] == pytest.approx(0.0)
    assert artifact["metrics"]["final_normalized_feedback_error_norm"] < 1.0e-3
    assert "not scheduled control" in artifact["claim_boundary"]
    assert "flight qualification" in artifact["claim_boundary"]
    ####


def test_f16_physical_wrench_perturbation_artifact_passes_declared_local_matrix() -> None:
    """The selected generic profile passes the declared local witness matrix."""

    artifact = json.loads(
        (ROOT / "verification/f16_physical_wrench_perturbation_evidence.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "development_screen_passed"
    assert len(artifact["cases"]) == 5
    assert sum(case["passed"] for case in artifact["cases"].values()) == 5
    assert sum(not case["passed"] for case in artifact["cases"].values()) == 0
    assert all(case["allocation_statuses"] == ["feasible"] for case in artifact["cases"].values())
    assert all(case["saturation_fraction"] == pytest.approx(0.0) for case in artifact["cases"].values())
    assert artifact["tuning_profile"] == "state_and_wrench_balanced_q10_r0p01"
    assert "broad envelope validation" in artifact["claim_boundary"]
    ####


def test_f16_local_maneuver_artifact_exercises_rate_reversals() -> None:
    """The local maneuver packet records both signs of bank and pitch response."""

    artifact = json.loads(
        (ROOT / "verification/f16_local_maneuver_evidence.json").read_text(encoding="utf-8")
    )
    assert artifact["status"] == "development_screen_passed"
    phases = artifact["phases"]
    assert phases["bank_left"]["response_metrics"]["maximum_p_rad_s"] > 0.0
    assert phases["bank_reversal"]["response_metrics"]["minimum_p_rad_s"] < 0.0
    assert phases["pitch_up"]["response_metrics"]["maximum_q_rad_s"] > 0.0
    assert phases["pitch_reversal"]["response_metrics"]["minimum_q_rad_s"] < 0.0
    assert all(value["passed"] for value in phases.values())
    assert "not a translation/attitude trajectory" in artifact["claim_boundary"]
    ####


def test_f16_reduction_artifact_keeps_lower_fidelity_claims_explicit() -> None:
    """The first reductions are screened locally but remain unpromoted."""

    artifact = json.loads((ROOT / "verification/f16_reduction_evidence.json").read_text(encoding="utf-8"))
    assert artifact["status"] == "development_screen_passed"
    assert artifact["reductions"]["point_mass_3dof"]["passed"] is True
    assert artifact["reductions"]["pseudo_6dof"]["passed"] is True
    assert artifact["reductions"]["pseudo_6dof"]["maximum_relative_rate_error"] < 0.20
    assert "equivalence_pending" in artifact["claim_boundary"]
    ####


def test_f16_reduction_models_match_source_trim_channels() -> None:
    """The reduction classes preserve the source operating-point contract."""

    evidence = json.loads((ROOT / "verification/f16_runtime_linearization_evidence.json").read_text(encoding="utf-8"))
    source = load_f16_reference_plant(
        ROOT / "families/reference_f16_s119/plant/daveml-import.json",
        ROOT / "resources/aerospace/daveml/official-conformance-v1/atmos_76.dml",
    )
    state = {name: float(value) for name, value in evidence["trim_state"].items()}
    controls = {name: float(value) for name, value in evidence["trim_controls"].items()}
    spec = TrimSpec(
        state_names=tuple(state),
        control_names=tuple(controls),
        residual_names=tuple(state),
        state_initial=state,
        control_initial=controls,
        operating_point={"trim_pitch_rad": float(evidence["metadata"]["trim_pitch_rad"])},
    )
    trim = TrimResult(spec, state, controls, {name: 0.0 for name in state}, 0.0, True, 1, "source trim", 0, 0.0)
    linearization = source.linearize_local(
        state,
        controls,
        trim_pitch_rad=float(evidence["metadata"]["trim_pitch_rad"]),
        altitude_m=0.0,
        state_step=1.0e-5,
        control_step=1.0e-5,
    )
    point = F16PointMass3DOFModel(source, trim, float(evidence["metadata"]["trim_pitch_rad"]))
    pseudo = F16AttitudeResponsePseudo6DOFModel(
        source,
        trim,
        linearization,
        float(evidence["metadata"]["trim_pitch_rad"]),
    )
    point_rates = point.state_derivative(state, controls)
    source_rates = source.state_derivative(state, controls, pitch_rad=float(evidence["metadata"]["trim_pitch_rad"]))
    assert point_rates == pytest.approx({name: source_rates[name] for name in point.state_names}, abs=1.0e-10)
    pseudo_state = {
        **state,
        "roll_rad": 0.0,
        "pitch_rad": float(evidence["metadata"]["trim_pitch_rad"]),
        "yaw_rad": 0.0,
    }
    pseudo_rates = pseudo.state_derivative(pseudo_state, controls)
    assert tuple(pseudo_rates[name] for name in ("p_rad_s", "q_rad_s", "r_rad_s")) == pytest.approx((0.0, 0.0, 0.0), abs=1.0e-10)
    ####
