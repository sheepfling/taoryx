"""NESC-owned semantic preflight for the pinned source replay."""

from __future__ import annotations

from taoryx.mission_capability import estimate_mission_capability
from taoryx.vehicle_composition import CompiledVehicleComposition
from taoryx.vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    SemanticPreflightHandler,
    VehicleExecutionPreflight,
    build_concrete_capability_preflight_evidence,
)

_TRANSLATOR_ID = "taoryx.nesc_staged_source_replay.capability.v1"


def preflight_nesc_source_replay(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the source-pinned NESC replay chronology through its owner."""

    from taoryx.nesc_mission_translation import compile_nesc_source_replay_mission

    plan = compile_nesc_source_replay_mission(composition)
    estimate = estimate_mission_capability(composition)
    if estimate is None:
        raise ValueError("NESC source replay has no installed capability adapter")
    if (
        estimate.adapter_id != _TRANSLATOR_ID
        or estimate.family_id != composition.family_id
        or estimate.mission_id != composition.mission
        or estimate.fidelity != composition.fidelity
    ):
        raise ValueError("NESC source replay capability adapter does not match the selected composition")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, dict):
        raise ValueError("NESC source replay capability adapter returned no capability manifest")
    event_order_valid = bool(capability.get("stage_event_order_valid"))
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if event_order_valid else "blocked",
        _TRANSLATOR_ID,
        (
            ExecutionPreflightCheck(
                "nesc.semantic_source_replay_plan",
                "pinned_launch_staging_orbit_replay",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("nesc.source_stage_event_order", True, event_order_valid, None, event_order_valid),
        ),
        (
            "composition exactly matches the pinned NESC source-replay launch, staging, and terminal witness",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        build_concrete_capability_preflight_evidence(composition, estimate),
    )
    ####


def nesc_semantic_handlers() -> tuple[SemanticPreflightHandler, ...]:
    """Return the family-owned handler selected by the NESC mission template."""

    return (SemanticPreflightHandler(_TRANSLATOR_ID, preflight_nesc_source_replay),)
    ####


__all__ = ["nesc_semantic_handlers", "preflight_nesc_source_replay"]
