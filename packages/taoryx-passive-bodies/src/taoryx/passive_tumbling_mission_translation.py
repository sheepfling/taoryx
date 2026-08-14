"""Semantic lowering for the canonical passive tumbling-body release witnesses.

The passive family intentionally has no controller, wrench, or allocation
bridge.  This translator maps one declared atmospheric release into the
existing detached-body propagation kernel and one of four named engineering
geometry fixtures.  They are reproducible composition witnesses, not claims
that a fixture represents a particular spent stage or that averaged 3DOF drag
proves physical tumble.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

from .contracts import Vector3
from .passive_body_dynamics import LaunchCommand, ReachabilityFidelity, RocketGlideVehicle
from .vehicle import DetachedBodyDefinition, TumblingPolicy
from .vehicle_composition import CompiledVehicleComposition

_CANONICAL_ALTITUDE_M = 1_000.0
_CANONICAL_SPEED_M_S = 250.0
_CANONICAL_HORIZON_S = 120.0
_CANONICAL_BODY_RATE_RAD_S = (0.25, 0.4, 0.6)
_CANONICAL_IMPACT_ALTITUDE_M = 0.0
_CANONICAL_BODY_SHAPES = ("cylinder", "sphere", "cone", "triaxial_ellipsoid")


@dataclass(frozen=True, slots=True)
class PassiveTumblingMissionSegment:
    """One semantic passive-body segment lowered to a truth event contract."""

    instance_id: str
    segment_id: str
    required_truth_events: tuple[str, ...]

    def manifest(self) -> dict[str, object]:
        """Return the traceable lowering for one segment."""

        return {
            "instance_id": self.instance_id,
            "segment_id": self.segment_id,
            "required_truth_events": list(self.required_truth_events),
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class PassiveTumblingMissionPlan:
    """Exact release-to-impact plan for one named passive-body witness."""

    fidelity: ReachabilityFidelity
    vehicle: RocketGlideVehicle
    body: DetachedBodyDefinition
    command: LaunchCommand
    horizon_s: float
    impact_plane_altitude_m: float
    area_policy: str
    segments: tuple[PassiveTumblingMissionSegment, ...]

    def manifest(self) -> dict[str, object]:
        """Return the complete source/assumption-bounded lowering record."""

        return {
            "translator_id": "taoryx.passive_tumbling.direct_release.v1",
            "fidelity": self.fidelity.value,
            "release": {
                "altitude_m": self.vehicle.initial_altitude_m,
                "speed_m_s": self.vehicle.initial_speed_m_s,
                "azimuth_rad": self.command.azimuth_rad,
                "elevation_rad": self.command.elevation_rad,
                "bank_rad": self.command.bank_rad,
                "body_rate_rad_s": list(_CANONICAL_BODY_RATE_RAD_S),
            },
            "body": {
                "id": self.body.body_id,
                "shape": self.body.shape.value,
                "mass_kg": self.body.mass_kg,
                "dimensions_m": list(self.body.dimensions_m),
                "reference_area_m2": self.body.reference_area_m2,
                "inertia_kg_m2": None
                if self.body.inertia_kg_m2 is None
                else [self.body.inertia_kg_m2.x, self.body.inertia_kg_m2.y, self.body.inertia_kg_m2.z],
                "tumbling_policy": self.body.tumbling_policy.value,
            },
            "horizon_s": self.horizon_s,
            "impact_plane_altitude_m": self.impact_plane_altitude_m,
            "area_policy": self.area_policy,
            "segments": [segment.manifest() for segment in self.segments],
            "claim_boundary": (
                f"This is a direct atmospheric release of the named engineering {self.body.shape.value} fixture. "
                "At point-mass 3DOF it uses orientation-averaged projected area and cannot prove tumble. "
                "At pseudo-6DOF it reuses the native passive rigid-body equations, without a controller, "
                "commanded wrench, or physical-effector allocation."
            ),
        }
        ####
    ####


def canonical_passive_tumbling_body() -> DetachedBodyDefinition:
    """Return the legacy canonical cylinder fixture."""

    return passive_tumbling_body("cylinder")
    ####


def passive_tumbling_body(shape: str) -> DetachedBodyDefinition:
    """Build one named passive-body geometry with explicit mass properties."""

    mass_kg = 12.0
    angular_rate = Vector3(*_CANONICAL_BODY_RATE_RAD_S)
    if shape == "cylinder":
        radius_m = 0.35
        length_m = 2.0
        inertia = Vector3(
            0.5 * mass_kg * radius_m**2,
            mass_kg * (3.0 * radius_m**2 + length_m**2) / 12.0,
            mass_kg * (3.0 * radius_m**2 + length_m**2) / 12.0,
        )
        return DetachedBodyDefinition.cylinder(
            "passive-cylinder-v1",
            mass_kg=mass_kg,
            radius_m=radius_m,
            length_m=length_m,
            tumbling_policy=TumblingPolicy.PASSIVE_TUMBLE,
            inertia_kg_m2=inertia,
            initial_angular_rate_body_rad_s=angular_rate,
        )
    if shape == "sphere":
        radius_m = 0.5
        inertia_value = 0.4 * mass_kg * radius_m**2
        return DetachedBodyDefinition.sphere(
            "passive-sphere-v1",
            mass_kg=mass_kg,
            radius_m=radius_m,
            tumbling_policy=TumblingPolicy.PASSIVE_TUMBLE,
            inertia_kg_m2=Vector3(inertia_value, inertia_value, inertia_value),
            initial_angular_rate_body_rad_s=angular_rate,
        )
    if shape == "cone":
        radius_m = 0.45
        height_m = 1.5
        return DetachedBodyDefinition.cone(
            "passive-cone-v1",
            mass_kg=mass_kg,
            base_radius_m=radius_m,
            height_m=height_m,
            tumbling_policy=TumblingPolicy.PASSIVE_TUMBLE,
            inertia_kg_m2=Vector3(
                0.3 * mass_kg * radius_m**2,
                0.15 * mass_kg * (radius_m**2 + 4.0 * height_m**2),
                0.15 * mass_kg * (radius_m**2 + 4.0 * height_m**2),
            ),
            initial_angular_rate_body_rad_s=angular_rate,
        )
    if shape == "triaxial_ellipsoid":
        semi_axis_x_m = 0.8
        semi_axis_y_m = 0.45
        semi_axis_z_m = 0.3
        return DetachedBodyDefinition.triaxial_ellipsoid(
            "passive-triaxial-ellipsoid-v1",
            mass_kg=mass_kg,
            semi_axis_x_m=semi_axis_x_m,
            semi_axis_y_m=semi_axis_y_m,
            semi_axis_z_m=semi_axis_z_m,
            tumbling_policy=TumblingPolicy.PASSIVE_TUMBLE,
            inertia_kg_m2=Vector3(
                mass_kg * (semi_axis_y_m**2 + semi_axis_z_m**2) / 5.0,
                mass_kg * (semi_axis_x_m**2 + semi_axis_z_m**2) / 5.0,
                mass_kg * (semi_axis_x_m**2 + semi_axis_y_m**2) / 5.0,
            ),
            initial_angular_rate_body_rad_s=angular_rate,
        )
    raise ValueError(f"unknown passive tumbling body shape {shape!r}; expected one of {_CANONICAL_BODY_SHAPES!r}")
    ####


def compile_passive_tumbling_mission(composition: CompiledVehicleComposition) -> PassiveTumblingMissionPlan:
    """Fail closed on any request outside the retained passive witness contract."""

    if composition.family_id != "tumbling_body":
        raise ValueError(f"passive tumbling translator cannot lower family {composition.family_id!r}")
    if composition.mission != "tumbling_body_release_damping_impact_v1":
        raise ValueError(f"passive tumbling translator cannot lower mission {composition.mission!r}")
    fidelity = _reachability_fidelity(composition.fidelity)
    if composition.initialization.id != "atmospheric_release":
        raise ValueError("passive tumbling witness requires atmospheric_release initialization")
    if tuple(segment.id for segment in composition.segments) != ("passive_coast", "atmospheric_descent"):
        raise ValueError("passive tumbling witness requires passive_coast then atmospheric_descent")

    initialization = composition.initialization.inputs
    _require_number(initialization, "altitude_m", _CANONICAL_ALTITUDE_M, "m")
    _require_number(initialization, "speed_m_s", _CANONICAL_SPEED_M_S, "m/s")
    _require_vector(initialization, "body_rates_rad_s", _CANONICAL_BODY_RATE_RAD_S, "rad/s")
    attitude = initialization.get("quaternion_wxyz")
    if attitude is not None:
        _require_vector(initialization, "quaternion_wxyz", (1.0, 0.0, 0.0, 0.0), "dimensionless")

    expected_area_policy = (
        "orientation_averaged_projected_area"
        if fidelity is ReachabilityFidelity.POINT_MASS_3DOF
        else "native_rigid_body_reuse_instantaneous_projected_area"
    )
    _require_text(initialization, "area_policy", expected_area_policy)
    coast, descent = composition.segments
    _require_number(coast.inputs, "duration_s", _CANONICAL_HORIZON_S, "s")
    _require_number(descent.inputs, "impact_plane_altitude_m", _CANONICAL_IMPACT_ALTITUDE_M, "m")

    body_shape = _optional_text(initialization, "body_shape", default="cylinder")
    if body_shape not in _CANONICAL_BODY_SHAPES:
        raise ValueError(f"passive tumbling witness has no named geometry realization {body_shape!r}")
    body = passive_tumbling_body(body_shape)
    profile_shape = body_shape.replace("_", "-")
    vehicle = RocketGlideVehicle(
        vehicle_id=f"taoryx-passive-{profile_shape}-direct-release-v1",
        dry_mass_kg=body.mass_kg,
        propellant_mass_kg=0.0,
        thrust_n=0.0,
        burn_time_s=0.0,
        reference_area_m2=body.reference_area_m2,
        drag_coefficient=0.25,
        lift_to_drag=1.0,
        initial_speed_m_s=_CANONICAL_SPEED_M_S,
        initial_altitude_m=_CANONICAL_ALTITUDE_M,
        aerodynamic_model_id=f"passive_{body_shape}_drag_surrogate_v1",
        actuator_profile_id="none",
        mission_profile_id="passive_tumbling_direct_release_v1",
        configuration_variant_id=f"canonical-{profile_shape}-v1",
        mass_property_profile_id=f"{profile_shape}-inertia-analytic-v1",
    )
    return PassiveTumblingMissionPlan(
        fidelity=fidelity,
        vehicle=vehicle,
        body=body,
        command=LaunchCommand(0.0, 0.0, 0.0),
        horizon_s=_CANONICAL_HORIZON_S,
        impact_plane_altitude_m=_CANONICAL_IMPACT_ALTITUDE_M,
        area_policy=expected_area_policy,
        segments=(
            PassiveTumblingMissionSegment(coast.instance_id, coast.id, ("release_state", "passive_descent")),
            PassiveTumblingMissionSegment(descent.instance_id, descent.id, ("terminal_impact",)),
        ),
    )
    ####


def _reachability_fidelity(value: str) -> ReachabilityFidelity:
    """Map the passive family's two composition tiers to implemented dynamics."""

    if value == "point_mass_3dof":
        return ReachabilityFidelity.POINT_MASS_3DOF
    if value == "pseudo_6dof":
        return ReachabilityFidelity.PSEUDO_6DOF
    raise ValueError(f"passive tumbling witness has no runtime for tier {value!r}")
    ####


def _require_number(inputs: Mapping[str, object], name: str, expected: float, unit: str) -> None:
    """Require one exact finite canonical witness scalar."""

    item = inputs.get(name)
    value = getattr(item, "value", None)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"passive tumbling composition input {name!r} must be finite numeric")
    if not math.isclose(float(value), expected, abs_tol=1.0e-9):
        raise ValueError(f"passive tumbling witness requires {name}={expected:.12g} {unit}")
    ####


def _require_vector(inputs: Mapping[str, object], name: str, expected: tuple[float, ...], unit: str) -> None:
    """Require one exact finite canonical vector without hidden normalization."""

    item = inputs.get(name)
    value = getattr(item, "value", None)
    if not isinstance(value, list | tuple) or len(value) != len(expected):
        raise ValueError(f"passive tumbling composition input {name!r} must contain {len(expected)} values")
    actual = tuple(float(component) for component in value)
    if not all(math.isfinite(component) for component in actual) or any(
        not math.isclose(component, target, abs_tol=1.0e-9) for component, target in zip(actual, expected, strict=True)
    ):
        rendered = ", ".join(f"{component:.12g}" for component in expected)
        raise ValueError(f"passive tumbling witness requires {name}=[{rendered}] {unit}")
    ####


def _require_text(inputs: Mapping[str, object], name: str, expected: str) -> None:
    """Require a declared area policy rather than choosing one silently."""

    item = inputs.get(name)
    value = getattr(item, "value", None)
    if value != expected:
        raise ValueError(f"passive tumbling witness requires {name}={expected!r}")
    ####


def _optional_text(inputs: Mapping[str, object], name: str, *, default: str) -> str:
    """Resolve one optional categorical input while preserving legacy requests."""

    item = inputs.get(name)
    if item is None:
        return default
    value = getattr(item, "value", None)
    if not isinstance(value, str) or not value:
        raise ValueError(f"passive tumbling composition input {name!r} must be non-empty text")
    return value
    ####


__all__ = [
    "PassiveTumblingMissionPlan",
    "PassiveTumblingMissionSegment",
    "canonical_passive_tumbling_body",
    "compile_passive_tumbling_mission",
    "passive_tumbling_body",
]
