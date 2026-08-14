"""Semantic preflight handler owned by the passive-bodies plug-in."""

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


def _preflight_passive_tumbling_release(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the explicitly uncontrolled direct passive-body release."""

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
            "composition exactly matches the declared direct passive-body release witness; it exposes no control, wrench, or allocation path",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def passive_semantic_handlers() -> tuple[SemanticPreflightHandler, ...]:
    """Return semantic translators owned by the passive-bodies plug-in."""

    return (
        SemanticPreflightHandler(
            "taoryx.passive_tumbling_release.capability.v1",
            _preflight_passive_tumbling_release,
        ),
    )
    ####


__all__ = ["passive_semantic_handlers"]
