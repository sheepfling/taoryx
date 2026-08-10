"""Semantic preflight handlers owned by the reachability plug-in."""

from __future__ import annotations

from taoryx.vehicle_composition import CompiledVehicleComposition
from taoryx.vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    SemanticPreflightHandler,
    VehicleExecutionPreflight,
    _capability_estimate_and_manifest,
    _capability_estimate_evidence,
    _capability_number,
)


def _preflight_hl20_source_booster_release_replay(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Preflight the exact source-aerodynamic HL-20 release replay."""

    from taoryx.hl20_source_release_mission_translation import compile_hl20_source_booster_release_mission

    plan = compile_hl20_source_booster_release_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    ordered = bool(capability["source_schedule_order_valid"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if ordered else "blocked",
        "taoryx.hl20_source_booster_release_replay.v1",
        (
            ExecutionPreflightCheck(
                "hl20.source_scheduled_release_plan",
                "pinned_booster_release_opposing_bank_ground_contact_replay",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("hl20.source_schedule_order", True, ordered, None, ordered),
        ),
        (
            "composition exactly matches the retained HL-20 source-aerodynamic/synthetic-booster scheduled witness",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_hl20_glide_energy_intent(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Translate the public HL-20 glide intent without claiming a native runner."""

    from taoryx.hl20_glide_energy_mission_translation import compile_hl20_glide_energy_mission

    plan = compile_hl20_glide_energy_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    energy_margin_j_kg = _capability_number(capability, "available_specific_energy_margin_j_kg")
    opposing_bank_intent = capability.get("opposing_bank_intent") is True
    finite_bank_geometry = capability.get("finite_bank_geometry") is True
    translatable = estimate.feasibility != "certainly_infeasible" and opposing_bank_intent and finite_bank_geometry
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if translatable else "blocked",
        "taoryx.hl20_glide_energy_intent.v1",
        (
            ExecutionPreflightCheck(
                "hl20.glide_energy_semantic_plan",
                "release_trim_opposing_bank_energy_handoff",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "hl20.unpowered_specific_energy_margin",
                0.0,
                energy_margin_j_kg,
                "J/kg",
                energy_margin_j_kg >= 0.0,
            ),
            ExecutionPreflightCheck(
                "hl20.opposing_finite_bank_intent",
                True,
                opposing_bank_intent and finite_bank_geometry,
                None,
                opposing_bank_intent and finite_bank_geometry,
            ),
        ),
        (
            "composition lowers exactly to the declared public HL-20 release/trim/opposing-bank/energy-handoff intent plan",
            "no source-owned HL-20 reduced-fidelity runtime factory is declared; runtime lowering remains blocked",
            *estimate.diagnostics,
        ),
        plan.manifest(),
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_x15_staged_reachability(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Preflight the retained X-15-scaled booster/release witness."""

    from taoryx.x15_staged_mission_translation import compile_x15_staged_reachability_mission

    plan = compile_x15_staged_reachability_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    chronology_ok = bool(capability["staging_and_horizon_order_valid"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if chronology_ok else "blocked",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                "x15.semantic_staged_reachability_plan",
                "source_pinned_booster_coast_release_open_loop_glide_impact",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("x15.staging_and_horizon_order", True, chronology_ok, None, chronology_ok),
        ),
        (
            "composition exactly matches the retained X-15-scaled local source-staging witness; "
            "it remains separate from the synthetic California-to-Hawaii showcase",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_passive_tumbling_release(
    composition: CompiledVehicleComposition,
) -> VehicleExecutionPreflight:
    """Preflight the explicitly uncontrolled direct passive-cylinder release."""

    from taoryx.passive_tumbling_mission_translation import compile_passive_tumbling_mission

    plan = compile_passive_tumbling_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    horizon_ok = bool(capability["horizon_contains_vacuum_fall_lower_bound"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if horizon_ok else "blocked",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                "tumbling_body.semantic_direct_release_plan",
                "canonical_passive_cylinder_release_area_policy_impact",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "tumbling_body.release_horizon",
                _capability_number(capability, "vacuum_fall_time_lower_bound_s"),
                _capability_number(capability, "simulation_horizon_s"),
                "s",
                horizon_ok,
            ),
        ),
        (
            "composition exactly matches the declared direct passive-cylinder release witness; it exposes no control, wrench, or allocation path",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def reachability_semantic_handlers() -> tuple[SemanticPreflightHandler, ...]:
    """Return all semantic translators layered onto reachability workflows."""

    return (
        SemanticPreflightHandler(
            "taoryx.hl20_source_booster_release_replay.v1",
            _preflight_hl20_source_booster_release_replay,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_glide_energy_intent.v1",
            _preflight_hl20_glide_energy_intent,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_staged_reachability.capability.v1",
            _preflight_x15_staged_reachability,
        ),
        SemanticPreflightHandler(
            "taoryx.passive_tumbling_release.capability.v1",
            _preflight_passive_tumbling_release,
        ),
    )
    ####


__all__ = ["reachability_semantic_handlers"]
####
