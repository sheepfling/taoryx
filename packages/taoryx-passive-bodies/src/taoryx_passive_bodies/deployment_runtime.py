"""Cross-plug-in runtime for synthetic passive released-body children.

The runtime owns only the child model and local atmospheric propagation.  A
parent plug-in must explicitly construct a :class:`DeploymentReleaseRequest`
and document any frame projection or separation-state assumption before this
runtime is invoked.
"""

from __future__ import annotations

import math
from typing import Literal

from taoryx.passive_body_dynamics import (
    EnvelopeTermination,
    ReachabilityFidelity,
    RocketGlideVehicle,
    simulate_passive_body_from_release_state,
)

from taoryx.contracts import Vector3
from taoryx.deployment import (
    DeploymentBinding,
    DeploymentChildExecution,
    DeploymentReleaseRequest,
)
from taoryx.vehicle import DetachedBodyDefinition, TumblingPolicy

PASSIVE_LOCAL_ATMOSPHERE_RELEASE_RUNTIME_ID = "taoryx.passive-bodies.local-atmosphere-release.v1"
_NESC_SYNTHETIC_CYLINDER_ID = "nesc-synthetic-cylinder"
_NESC_SYNTHETIC_CYLINDER_MASS_KG = 19_000.0
_NESC_SYNTHETIC_CYLINDER_REFERENCE_AREA_M2 = 7.0
_NESC_SYNTHETIC_CYLINDER_LENGTH_M = 10.0
_NESC_SYNTHETIC_CYLINDER_BODY_RATE_RAD_S = Vector3(0.012, 0.017, 0.023)


class PassiveLocalAtmosphereReleaseRuntime:
    """Propagate the declared synthetic NESC cylinder from a local state.

    The intentionally narrow first runtime demonstrates a package boundary:
    it has no import of the NESC parent or reference-model package.  More
    reusable passive child models can be added by widening ``supports`` only
    when their geometry, environmental assumptions, and evidence are also
    declared.
    """

    id = PASSIVE_LOCAL_ATMOSPHERE_RELEASE_RUNTIME_ID

    def supports(self, binding: DeploymentBinding) -> bool:
        """Return whether a binding selects this declared child fixture."""

        return (
            binding.child.plugin_id == "taoryx.passive-bodies"
            and binding.child.runtime_id == self.id
            and binding.child.model_id == _NESC_SYNTHETIC_CYLINDER_ID
            and binding.child.family_id == "tumbling_body"
            and binding.child.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def execute(self, request: DeploymentReleaseRequest) -> DeploymentChildExecution:
        """Propagate the selected child from its parent-declared local state."""

        if not self.supports(request.binding):
            raise ValueError(f"passive child runtime {self.id!r} does not support the selected binding")
        if request.release_state.frame != "local_tangent":
            raise ValueError(f"passive child runtime {self.id!r} accepts only an explicit local_tangent release state, not {request.release_state.frame!r}")
        body = _nesc_synthetic_cylinder(request.binding.child_object_id)
        fidelity = _passive_fidelity(request.binding.child.fidelity)
        vehicle = RocketGlideVehicle(
            vehicle_id="taoryx-passive-nesc-synthetic-cylinder-v1",
            dry_mass_kg=body.mass_kg,
            reference_area_m2=body.reference_area_m2,
            drag_coefficient=0.25,
            initial_speed_m_s=_norm(request.release_state.velocity_m_s),
            initial_altitude_m=max(0.0, request.release_state.position_m[2]),
            aerodynamic_model_id="synthetic_passive_cylinder_drag_surrogate_v1",
            actuator_profile_id="none",
            mission_profile_id="independent_parent_release_v1",
            configuration_variant_id="nesc-synthetic-cylinder-v1",
            mass_property_profile_id="analytic-cylinder-inertia-v1",
        )
        trajectory = simulate_passive_body_from_release_state(
            vehicle,
            body,
            position_m=request.release_state.position_m,
            velocity_m_s=request.release_state.velocity_m_s,
            release_time_s=request.release_state.time_s,
            fidelity=fidelity,
            step_size_s=0.5,
            horizon_s=200.0,
            parent_event_id=request.release_event_id,
        )
        status: Literal["completed", "invalid"] = "invalid" if trajectory.termination is EnvelopeTermination.INVALID else "completed"
        return DeploymentChildExecution(
            status=status,
            binding_id=request.binding.id,
            deployment_id=request.binding.deployment_id,
            relationship_kind="separation",
            state_transfer=request.binding.state_transfer,
            parent_plugin_id=request.parent_plugin_id,
            parent_model_id=request.parent_model_id,
            parent_family_id=request.parent_family_id,
            parent_composition_id=request.parent_composition_id,
            parent_object_id=request.parent_object_id,
            child_object_id=request.binding.child_object_id,
            parent_event_id=request.release_event_id,
            runtime_plugin_id="taoryx.passive-bodies",
            runtime_id=self.id,
            child_model_id=request.binding.child.model_id,
            child_family_id=request.binding.child.family_id,
            requested_fidelity=request.binding.child.fidelity,
            realized_fidelity=trajectory.fidelity.value,
            release_state=request.release_state,
            termination=trajectory.termination.value,
            telemetry=trajectory.telemetry,
            provenance={
                "child_fixture": "synthetic-passive-cylinder-v1",
                "mass_kg": body.mass_kg,
                "reference_area_m2": body.reference_area_m2,
                "step_size_s": 0.5,
                "horizon_s": 200.0,
                "area_policy": (
                    "orientation_averaged_projected_area"
                    if trajectory.fidelity is ReachabilityFidelity.POINT_MASS_3DOF
                    else "native_rigid_body_reuse_instantaneous_projected_area"
                ),
            },
            claim_boundary=(
                "This independently propagates only the declared synthetic cylinder in a local exponential-atmosphere "
                "surrogate. It does not establish source-exact NESC spent-stage aerodynamics, a separation impulse, "
                "a rotating-Earth ECI propagation, active guidance, or a change to the parent source replay."
            ),
        )
        ####

    ####


def _nesc_synthetic_cylinder(child_object_id: str) -> DetachedBodyDefinition:
    """Return the fixture whose mass and frontal area are declared by NESC data."""

    radius_m = math.sqrt(_NESC_SYNTHETIC_CYLINDER_REFERENCE_AREA_M2 / math.pi)
    inertia = Vector3(
        0.5 * _NESC_SYNTHETIC_CYLINDER_MASS_KG * radius_m**2,
        _NESC_SYNTHETIC_CYLINDER_MASS_KG * (3.0 * radius_m**2 + _NESC_SYNTHETIC_CYLINDER_LENGTH_M**2) / 12.0,
        _NESC_SYNTHETIC_CYLINDER_MASS_KG * (3.0 * radius_m**2 + _NESC_SYNTHETIC_CYLINDER_LENGTH_M**2) / 12.0,
    )
    return DetachedBodyDefinition.cylinder(
        child_object_id,
        mass_kg=_NESC_SYNTHETIC_CYLINDER_MASS_KG,
        radius_m=radius_m,
        length_m=_NESC_SYNTHETIC_CYLINDER_LENGTH_M,
        tumbling_policy=TumblingPolicy.PASSIVE_TUMBLE,
        inertia_kg_m2=inertia,
        initial_angular_rate_body_rad_s=_NESC_SYNTHETIC_CYLINDER_BODY_RATE_RAD_S,
    )
    ####


def _passive_fidelity(value: str) -> ReachabilityFidelity:
    """Map the composition's two supported child tiers to the local kernel."""

    if value == "point_mass_3dof":
        return ReachabilityFidelity.POINT_MASS_3DOF
    if value == "pseudo_6dof":
        return ReachabilityFidelity.PSEUDO_6DOF
    raise ValueError(f"passive child runtime has no implementation for fidelity {value!r}")
    ####


def _norm(value: tuple[float, float, float]) -> float:
    """Return the Euclidean magnitude of a finite local velocity."""

    return math.sqrt(sum(component * component for component in value))
    ####


__all__ = ["PASSIVE_LOCAL_ATMOSPHERE_RELEASE_RUNTIME_ID", "PassiveLocalAtmosphereReleaseRuntime"]
