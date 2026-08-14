"""Hummingbird-owned semantic preflight handlers."""

from __future__ import annotations

from taoryx.hummingbird_mission_translation import compile_hummingbird_pseudo_mission

from taoryx.vehicle_composition import CompiledVehicleComposition
from taoryx.vehicle_execution_preflight import (
    ExecutionPreflightCheck,
    VehicleExecutionPreflight,
    _capability_estimate_and_manifest,
    _capability_estimate_evidence,
    _capability_number,
)


def preflight_hummingbird_hover_yaw(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the declared aggregate-thrust Hummingbird mission translator."""

    plan = compile_hummingbird_pseudo_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    thrust_margin_n = _capability_number(capability, "thrust_margin_n")
    capability_ok = estimate.feasibility != "certainly_infeasible"
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if capability_ok else "blocked",
        "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
        (
            ExecutionPreflightCheck(
                "hummingbird.semantic_plan",
                "declared_hover_yaw_translation_contact_segments",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "hummingbird.aggregate_hover_thrust_margin",
                0.0,
                thrust_margin_n,
                "N",
                capability_ok,
            ),
        ),
        (
            "composition lowers exactly through the declared Hummingbird pseudo-6DOF mission translator",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


__all__ = ["preflight_hummingbird_hover_yaw"]
