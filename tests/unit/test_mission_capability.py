from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path

import pytest

import taoryx.mission_capability as mission_capability_module
from taoryx.language_backed_racetrack import materialize_powered_fixed_wing_composition
from taoryx.mission_capability import (
    declared_mission_capability_adapter,
    estimate_mission_capability,
    resolve_mission_capability_adapter,
)
from taoryx.vehicle_composition import CompositionValue, compile_vehicle_composition, load_vehicle_composition_request
from taoryx.vehicle_execution_preflight import preflight_vehicle_composition
from taoryx.vehicle_runtime_lowering import lower_vehicle_composition

ROOT = Path(__file__).resolve().parents[2]


def test_powered_fixed_wing_racetrack_uses_one_declared_capability_adapter() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )

    adapter = resolve_mission_capability_adapter(composition)
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert adapter is not None
    assert estimate is not None
    assert estimate.adapter_id == "taoryx.x8_racetrack.source_route.v1"
    assert estimate.feasibility == "feasible"
    resolved_route = estimate.manifest["resolved_route"]
    assert isinstance(resolved_route, Mapping)
    assert resolved_route["vehicle_id"] == "skywalker_x8"
    assert preflight.translator_id == estimate.adapter_id
    capability_evidence = preflight.capability_estimate
    assert capability_evidence is not None
    assert capability_evidence["schema"] == "taoryx.concrete-capability-preflight/v1alpha1"
    assert capability_evidence["semantic_translator_id"] == preflight.translator_id
    assert capability_evidence["adapter_id"] == estimate.adapter_id
    assert capability_evidence["composition_identity_sha256"] == composition.identity_sha256
    assert capability_evidence["derived_mission_sha256"]
    capability_advertisement = capability_evidence["capability_advertisement"]
    assert isinstance(capability_advertisement, Mapping)
    assert capability_advertisement["schema"] == "taoryx.vehicle-capability-advertisement/v1alpha1"
    assert len(capability_advertisement["fingerprint_sha256"]) == 64
    assert capability_advertisement["selection"]["composition_id"] == composition.id
    assert capability_advertisement["interface"]["fingerprint_sha256"]
    assert capability_advertisement["family_owned"] == estimate.manifest["capability"]
    assert preflight.as_dict()["capability_estimate"] == capability_evidence
    assert declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    ) == estimate.adapter_id
    ####


def test_capability_adapter_selection_fails_closed_when_registry_identity_disagrees(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml")
    )
    monkeypatch.setattr(
        mission_capability_module,
        "declared_mission_capability_adapter",
        lambda *_args: "taoryx.synthetic.mismatched_capability.v1",
    )

    with pytest.raises(ValueError, match="mission-template capability adapter does not match"):
        resolve_mission_capability_adapter(composition)
    ####


def test_fixed_wing_composition_turn_radius_changes_the_materialized_template(tmp_path: Path) -> None:
    """A selected semantic radius reaches native inputs through common preflight."""

    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"
    )
    selected_radius_m = 600.0
    diameter_m = 2.0 * selected_radius_m
    segments = tuple(
        segment.model_copy(
            update={
                "inputs": {
                    **segment.inputs,
                    "turn_radius_m": CompositionValue(value=selected_radius_m, unit="m"),
                    **(
                        {
                            "gate_center_ned_m": CompositionValue(
                                value=[diameter_m, 1000.0, -188.0],
                                unit="m",
                            )
                        }
                        if segment.instance_id == "left-turn"
                        else {}
                    ),
                }
            }
        )
        if segment.instance_id in {"left-turn", "right-turn"}
        else segment.model_copy(
            update={
                "inputs": {
                    **segment.inputs,
                    "gate_center_ned_m": CompositionValue(
                        value=[diameter_m, 0.0, -178.0],
                        unit="m",
                    ),
                }
            }
        )
        if segment.id == "descent_level_gate"
        else segment
        for segment in request.segments
    )
    composition = compile_vehicle_composition(request.model_copy(update={"segments": segments}))

    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)
    materialized = materialize_powered_fixed_wing_composition(composition, tmp_path)

    assert estimate is not None
    derived = estimate.manifest["derived"]
    assert isinstance(derived, Mapping)
    assert float(derived["selected_turn_radius_m"]) == selected_radius_m
    assert preflight.status == "translation_ready"
    assert materialized.proposal.route.turn_radius_m == selected_radius_m
    assert f"racetrack-turn-radius-m={selected_radius_m:g}" in materialized.problem.read_text(encoding="utf-8")
    ####


def test_fixed_wing_composition_rejects_asymmetric_racetrack_turn_requests() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/x8_racetrack_capability_3dof_compose.yaml"
    )
    segments = tuple(
        segment.model_copy(
            update={
                "inputs": {
                    **segment.inputs,
                    "turn_radius_m": CompositionValue(value=300.0, unit="m"),
                }
            }
        )
        if segment.instance_id == "right-turn"
        else segment
        for segment in request.segments
    )
    composition = compile_vehicle_composition(request.model_copy(update={"segments": segments}))

    with pytest.raises(ValueError, match="equal left/right requested turn radii"):
        estimate_mission_capability(composition)
    ####


def test_hummingbird_uses_aggregate_thrust_planning_not_fixed_wing_geometry() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    composition = compile_vehicle_composition(request)
    estimate = estimate_mission_capability(composition)

    assert resolve_mission_capability_adapter(composition) is not None
    assert estimate is not None
    assert estimate.adapter_id == "taoryx.multirotor_hover_translation.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["physical_motor_allocation"] is False
    assert float(capability["thrust_margin_n"]) > 0.0
    assert declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    ) == estimate.adapter_id
    ####


def test_hl20_source_release_replay_is_explicitly_source_scheduled() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(
            ROOT / "examples/vehicle_composition/hl20_source_booster_release_replay_pseudo6dof_compose.yaml"
        )
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.hl20_source_booster_release_replay.capability.v1"
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == "taoryx.hl20_source_booster_release_replay.v1"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["source_aerodynamic_graph"] is True
    assert capability["participating_guidance_controller"] is False
    assert capability["physical_effector_allocation"] is False
    ####


def test_hummingbird_overweight_grounded_reset_is_certainly_infeasible_before_runtime() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hummingbird_hover_yaw_episode_pseudo6dof_compose.yaml")
    grounded = request.initialization.model_copy(
        update={
            "id": "grounded_idle",
            "inputs": {
                "pad_north_m": CompositionValue(value=0.0, unit="m"),
                "pad_east_m": CompositionValue(value=0.0, unit="m"),
                "heading_deg": CompositionValue(value=0.0, unit="deg"),
                "mass_kg": CompositionValue(value=6.0, unit="kg"),
            },
        }
    )
    composition = compile_vehicle_composition(request.model_copy(update={"initialization": grounded}))
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.feasibility == "certainly_infeasible"
    assert preflight.status == "blocked"
    assert any(check.id == "hummingbird.aggregate_hover_thrust_margin" and not check.passed for check in preflight.checks)
    ####


def test_passive_tumbling_uses_a_release_capability_adapter_without_controls() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/tumbling_body_direct_release_pseudo6dof_compose.yaml")
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.passive_tumbling_release.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["control_realization"] == "uncontrolled_passive_body"
    assert capability["physical_effector_allocation"] is False
    assert capability["direct_wrench_injection"] is False
    assert capability["horizon_contains_vacuum_fall_lower_bound"] is True
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == estimate.adapter_id
    assert any(check.id == "tumbling_body.release_horizon" and check.passed for check in preflight.checks)
    assert declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    ) == estimate.adapter_id
    ####


def test_nesc_source_replay_owns_stage_event_planning_without_gimbal_claims() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/nesc_staged_source_replay_pseudo6dof_compose.yaml")
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.nesc_staged_source_replay.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["participating_nonlinear_plant"] is False
    assert capability["physical_gimbal_allocation"] is False
    assert capability["stage_event_order_valid"] is True
    assert int(capability["required_truth_event_count"]) == 6
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == estimate.adapter_id
    assert any(check.id == "nesc.source_stage_event_order" and check.passed for check in preflight.checks)
    ####


def test_x15_staged_witness_owns_chronology_without_promoting_direct_wrench() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_staged_booster_reachability_pseudo6dof_compose.yaml")
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.x15_staged_reachability.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["control_realization"] == "response_law"
    assert capability["direct_wrench_injection"] is False
    assert capability["physical_effector_allocation"] is False
    assert capability["staging_and_horizon_order_valid"] is True
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == estimate.adapter_id
    assert any(check.id == "x15.staging_and_horizon_order" and check.passed for check in preflight.checks)
    ####


def test_x15_local_direct_wrench_screen_declares_bounded_local_authority_only() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/x15_local_direct_wrench_screen_compose.yaml")
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.x15_local_direct_wrench_screen.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["control_realization"] == "direct_wrench"
    assert capability["participating_nonlinear_plant"] is True
    assert capability["physical_effector_allocation"] is False
    assert capability["source_physical_trim"] is False
    limits = capability["direct_wrench_limits"]
    assert isinstance(limits, Mapping)
    assert all(float(value) > 0.0 for value in limits["authority_span"].values())
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == estimate.adapter_id
    assert any(check.id == "x15.direct_wrench_authority_bounds" and check.passed for check in preflight.checks)
    assert declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    ) == estimate.adapter_id
    ####


def test_hl20_energy_bank_intent_translates_before_native_runtime_binding() -> None:
    composition = compile_vehicle_composition(
        load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml")
    )
    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.adapter_id == "taoryx.hl20_glide_energy.capability.v1"
    assert estimate.feasibility == "likely_feasible"
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    assert capability["control_realization"] == "unpowered_lifting_body_energy_bank_intent"
    assert capability["physical_effector_allocation"] is False
    assert capability["direct_wrench_injection"] is False
    assert capability["opposing_bank_intent"] is True
    assert float(capability["available_specific_energy_margin_j_kg"]) > 0.0
    source_runtime_admission = capability["source_runtime_admission"]
    assert isinstance(source_runtime_admission, Mapping)
    assert source_runtime_admission["status"] == "outside_source_domain"
    assert source_runtime_admission["source_domain_covered"] is False
    assert source_runtime_admission["runtime_factory_declared"] is False
    assert source_runtime_admission["release"] == {"mach": 5.0, "altitude_m": 80_000.0}
    assert source_runtime_admission["declared_source_envelope"] == {
        "mach": [0.3, 4.0],
        "altitude_m": [-1_000.0, 20_000.0],
    }
    assert any("outside the declared DAVE-ML source" in diagnostic for diagnostic in estimate.diagnostics)
    assert preflight.status == "translation_ready"
    assert preflight.translator_id == "taoryx.hl20_glide_energy_intent.v1"
    assert any(check.id == "hl20.glide_energy_semantic_plan" and check.passed for check in preflight.checks)
    assert preflight.derived_mission is not None
    assert preflight.derived_mission["schema"] == "taoryx.hl20-glide-energy-mission-plan/v1alpha1"
    assert preflight.capability_estimate is not None
    assert preflight.capability_estimate["adapter_id"] == estimate.adapter_id
    public_advertisement = preflight.capability_estimate["capability_advertisement"]
    assert isinstance(public_advertisement, Mapping)
    public_admission = public_advertisement["family_owned"]
    assert isinstance(public_admission, Mapping)
    assert public_admission["source_runtime_admission"] == source_runtime_admission
    assert "no source-owned HL-20 reduced-fidelity runtime factory is declared" in preflight.diagnostics[1]
    assert "exact lowering" in str(preflight.as_dict()["claim_boundary"])
    lowering = lower_vehicle_composition(composition, preflight_result=preflight)
    assert lowering.status == "blocked"
    assert lowering.translator_status == "pending"
    assert "no batch execution binding" in lowering.diagnostics[0]
    assert declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    ) == estimate.adapter_id
    ####


def test_hl20_energy_bank_intent_translator_supports_the_declared_pseudo_6dof_tier() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml"
    )
    composition = compile_vehicle_composition(request.model_copy(update={"fidelity": "pseudo_6dof"}))

    preflight = preflight_vehicle_composition(composition)

    assert preflight.status == "translation_ready"
    assert preflight.translator_id == "taoryx.hl20_glide_energy_intent.v1"
    assert preflight.derived_mission is not None
    assert preflight.derived_mission["fidelity"] == "pseudo_6dof"
    ####


def test_hl20_energy_bank_intent_advertises_an_in_domain_but_runtime_unbound_release() -> None:
    request = load_vehicle_composition_request(
        ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml"
    )
    initialization = request.initialization.model_copy(
        update={
            "inputs": {
                **request.initialization.inputs,
                "altitude_m": CompositionValue(value=20_000.0, unit="m"),
                "mach": CompositionValue(value=4.0, unit="dimensionless"),
            }
        }
    )
    segments = tuple(
        segment.model_copy(
            update={
                "inputs": {
                    **segment.inputs,
                    "target_energy": CompositionValue(value=1_000_000.0, unit="J/kg"),
                }
            }
        )
        if segment.id == "energy_handoff"
        else segment
        for segment in request.segments
    )
    composition = compile_vehicle_composition(request.model_copy(update={"initialization": initialization, "segments": segments}))

    estimate = estimate_mission_capability(composition)

    assert estimate is not None
    capability = estimate.manifest["capability"]
    assert isinstance(capability, Mapping)
    source_runtime_admission = capability["source_runtime_admission"]
    assert isinstance(source_runtime_admission, Mapping)
    assert source_runtime_admission["status"] == "source_domain_covered_runtime_unbound"
    assert source_runtime_admission["source_domain_covered"] is True
    assert source_runtime_admission["runtime_factory_declared"] is False
    assert "runtime_factory_binding" in source_runtime_admission["required_before_runtime_binding"]
    assert any("inside the declared DAVE-ML" in diagnostic for diagnostic in estimate.diagnostics)
    ####


def test_hl20_capability_rejects_terminal_energy_above_unpowered_release_energy() -> None:
    request = load_vehicle_composition_request(ROOT / "examples/vehicle_composition/hl20_glide_energy_capability_3dof_compose.yaml")
    segments = tuple(
        segment.model_copy(
            update={
                "inputs": {
                    **segment.inputs,
                    "target_energy": CompositionValue(value=3_000_000.0, unit="J/kg"),
                }
            }
        )
        if segment.id == "energy_handoff"
        else segment
        for segment in request.segments
    )
    composition = compile_vehicle_composition(request.model_copy(update={"segments": segments}))

    estimate = estimate_mission_capability(composition)
    preflight = preflight_vehicle_composition(composition)

    assert estimate is not None
    assert estimate.feasibility == "certainly_infeasible"
    assert any("exceeds the unpowered release" in diagnostic for diagnostic in estimate.diagnostics)
    assert preflight.status == "blocked"
    assert any(check.id == "hl20.unpowered_specific_energy_margin" and not check.passed for check in preflight.checks)
    ####
