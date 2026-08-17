"""Shared reduced-order lateral-control authority for interceptor tiers."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Final, Literal

ControlConfiguration = Literal["aerodynamic", "thrust_assisted", "mixed"]
ControlAllocationPolicy = Literal["aerodynamic_first", "thrust_vector_first", "proportional"]
CONTROL_ALLOCATION_POLICIES: Final[tuple[ControlAllocationPolicy, ...]] = (
    "aerodynamic_first",
    "thrust_vector_first",
    "proportional",
)


@dataclass(frozen=True, slots=True)
class ControlAuthorityEvaluation:
    """Instantaneous authority components and structural clipping result."""

    configuration: ControlConfiguration
    aerodynamic_mps2: float
    thrust_vector_mps2: float
    combined_unclipped_mps2: float
    available_mps2: float
    structural_limit_mps2: float
    structural_limit_active: bool

    def __post_init__(self) -> None:
        values = (
            self.aerodynamic_mps2,
            self.thrust_vector_mps2,
            self.combined_unclipped_mps2,
            self.available_mps2,
            self.structural_limit_mps2,
        )
        if any(not math.isfinite(item) or item < 0.0 for item in values):
            raise ValueError("control-authority values must be finite and nonnegative")
        if self.available_mps2 > self.structural_limit_mps2 + 1.0e-12:
            raise ValueError("available control authority cannot exceed its structural limit")
        if self.structural_limit_active != (self.combined_unclipped_mps2 > self.structural_limit_mps2 + 1.0e-12):
            raise ValueError("structural-limit status must match the unclipped authority")
        ####

    @property
    def saturation_reason(self) -> str:
        """Return the stable dynamic-authority reason for this configuration."""

        return {
            "aerodynamic": "aerodynamic_authority_saturation",
            "thrust_assisted": "thrust_vector_authority_saturation",
            "mixed": "combined_authority_saturation",
        }[self.configuration]
        ####

    def commanded_support_fraction(self, requested_mps2: float) -> float:
        """Return the fraction of a nonnegative command currently supportable.

        Zero demand returns one by convention because no authority is required;
        callers that need hardware/force availability must inspect
        ``available_mps2`` separately.
        """

        if not math.isfinite(requested_mps2) or requested_mps2 < 0.0:
            raise ValueError("requested lateral acceleration must be finite and nonnegative")
        if requested_mps2 <= 1.0e-12:
            return 1.0
        return min(self.available_mps2 / requested_mps2, 1.0)
        ####

    ####


@dataclass(frozen=True, slots=True)
class ControlAuthorityAllocation:
    """Achieved lateral-force allocation and remaining axial thrust."""

    policy: ControlAllocationPolicy
    requested_mps2: float
    achieved_mps2: float
    aerodynamic_achieved_mps2: float
    thrust_vector_achieved_mps2: float
    thrust_vector_angle_rad: float
    axial_thrust_n: float

    def __post_init__(self) -> None:
        values = (
            self.requested_mps2,
            self.achieved_mps2,
            self.aerodynamic_achieved_mps2,
            self.thrust_vector_achieved_mps2,
            self.thrust_vector_angle_rad,
            self.axial_thrust_n,
        )
        if any(not math.isfinite(item) or item < 0.0 for item in values):
            raise ValueError("control-authority allocation values must be finite and nonnegative")
        if not math.isclose(
            self.achieved_mps2,
            self.aerodynamic_achieved_mps2 + self.thrust_vector_achieved_mps2,
            rel_tol=1.0e-12,
            abs_tol=1.0e-12,
        ):
            raise ValueError("achieved authority must equal its allocated components")
        if self.achieved_mps2 > self.requested_mps2 + 1.0e-12:
            raise ValueError("achieved authority cannot exceed its request")
        if self.thrust_vector_angle_rad >= math.pi / 2.0:
            raise ValueError("achieved thrust-vector angle must remain below pi/2")
        ####

    ####


def evaluate_control_authority(
    *,
    configuration: ControlConfiguration,
    dynamic_pressure_pa: float,
    reference_area_m2: float,
    normal_force_coefficient_limit: float,
    thrust_n: float,
    mass_kg: float,
    max_thrust_vector_angle_rad: float,
    structural_limit_mps2: float,
) -> ControlAuthorityEvaluation:
    """Evaluate a transparent force-authority surrogate.

    Aerodynamic authority is ``q S Cn / m``. Thrust-assisted authority is the
    lateral component of current thrust at the resolved vector-angle bound.
    The two add only for ``mixed`` and are finally clipped by the resolved
    structural/maneuverability limit. This is not an actuator, autopilot, or
    aerodynamic coefficient deck.
    """

    values = (
        dynamic_pressure_pa,
        reference_area_m2,
        normal_force_coefficient_limit,
        thrust_n,
        mass_kg,
        max_thrust_vector_angle_rad,
        structural_limit_mps2,
    )
    if any(not math.isfinite(item) for item in values):
        raise ValueError("control-authority inputs must be finite")
    if dynamic_pressure_pa < 0.0 or thrust_n < 0.0:
        raise ValueError("dynamic pressure and thrust must be nonnegative")
    if reference_area_m2 <= 0.0 or mass_kg <= 0.0 or structural_limit_mps2 <= 0.0:
        raise ValueError("control-authority geometry, mass, and structural limit must be positive")
    if normal_force_coefficient_limit < 0.0:
        raise ValueError("normal-force coefficient limit must be nonnegative")
    if not 0.0 <= max_thrust_vector_angle_rad < math.pi / 2.0:
        raise ValueError("maximum thrust-vector angle must lie in [0, pi/2)")
    if configuration in {"aerodynamic", "mixed"} and normal_force_coefficient_limit <= 0.0:
        raise ValueError("aerodynamic and mixed configurations require positive normal-force authority")
    if configuration in {"thrust_assisted", "mixed"} and max_thrust_vector_angle_rad <= 0.0:
        raise ValueError("thrust-assisted and mixed configurations require a positive thrust-vector angle")

    aerodynamic = dynamic_pressure_pa * reference_area_m2 * normal_force_coefficient_limit / mass_kg if configuration in {"aerodynamic", "mixed"} else 0.0
    thrust_vector = thrust_n * math.sin(max_thrust_vector_angle_rad) / mass_kg if configuration in {"thrust_assisted", "mixed"} else 0.0
    combined = aerodynamic + thrust_vector
    return ControlAuthorityEvaluation(
        configuration=configuration,
        aerodynamic_mps2=aerodynamic,
        thrust_vector_mps2=thrust_vector,
        combined_unclipped_mps2=combined,
        available_mps2=min(combined, structural_limit_mps2),
        structural_limit_mps2=structural_limit_mps2,
        structural_limit_active=combined > structural_limit_mps2 + 1.0e-12,
    )
    ####


def allocate_control_authority(
    authority: ControlAuthorityEvaluation,
    *,
    requested_mps2: float,
    policy: ControlAllocationPolicy,
    thrust_n: float,
    mass_kg: float,
) -> ControlAuthorityAllocation:
    """Allocate achieved lateral demand without double-counting thrust.

    ``aerodynamic_first`` spends aerodynamic authority before vectoring thrust;
    ``thrust_vector_first`` reverses that priority; and ``proportional`` divides
    achieved demand by the instantaneous unclipped component-authority ratio.
    The axial component is the corresponding force-vector projection. These
    are reduced-order bookkeeping rules, not actuators or physical control
    allocators.
    """

    if not math.isfinite(requested_mps2) or requested_mps2 < 0.0:
        raise ValueError("requested lateral authority must be finite and nonnegative")
    if not math.isfinite(thrust_n) or thrust_n < 0.0:
        raise ValueError("allocation thrust must be finite and nonnegative")
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("allocation mass must be finite and positive")
    if policy not in CONTROL_ALLOCATION_POLICIES:
        raise ValueError(f"unsupported control-allocation policy {policy!r}")

    achieved = min(requested_mps2, authority.available_mps2)
    if policy == "aerodynamic_first":
        aerodynamic = min(achieved, authority.aerodynamic_mps2)
        thrust_vector = min(achieved - aerodynamic, authority.thrust_vector_mps2)
    elif policy == "thrust_vector_first":
        thrust_vector = min(achieved, authority.thrust_vector_mps2)
        aerodynamic = min(achieved - thrust_vector, authority.aerodynamic_mps2)
    elif authority.combined_unclipped_mps2 <= 1.0e-15:
        aerodynamic = 0.0
        thrust_vector = 0.0
    else:
        aerodynamic = achieved * authority.aerodynamic_mps2 / authority.combined_unclipped_mps2
        thrust_vector = achieved - aerodynamic
    lateral_thrust_n = thrust_vector * mass_kg
    if thrust_n <= 1.0e-15 or lateral_thrust_n <= 1.0e-15:
        vector_angle = 0.0
        axial_thrust = thrust_n
    else:
        ratio = min(max(lateral_thrust_n / thrust_n, 0.0), 1.0)
        vector_angle = math.asin(ratio)
        axial_thrust = math.sqrt(max(thrust_n**2 - lateral_thrust_n**2, 0.0))
    return ControlAuthorityAllocation(
        policy=policy,
        requested_mps2=requested_mps2,
        achieved_mps2=aerodynamic + thrust_vector,
        aerodynamic_achieved_mps2=aerodynamic,
        thrust_vector_achieved_mps2=thrust_vector,
        thrust_vector_angle_rad=vector_angle,
        axial_thrust_n=axial_thrust,
    )
    ####


__all__ = [
    "CONTROL_ALLOCATION_POLICIES",
    "ControlAllocationPolicy",
    "ControlAuthorityAllocation",
    "ControlAuthorityEvaluation",
    "ControlConfiguration",
    "allocate_control_authority",
    "evaluate_control_authority",
]
####
