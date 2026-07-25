"""Reusable specialized segment contracts for SimpleAero-like trajectories.

The SimpleAero fixtures are synthetic translations, not a recovered SimpleAero
runtime.  This module captures the reusable phase intent separately from the
fixture syntax so the same segment contract can be applied to a rocket,
air-breathing vehicle, glider, or pseudo-6-DOF bridge.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

SpecializedSegmentType = Literal[
    "powered_ascent",
    "ballistic_coast",
    "bank_maneuver",
    "alpha_profile",
    "skip_maneuver",
    "terminal_pronav",
    "moving_target_intercept",
]


@dataclass(frozen=True, slots=True)
class SpecializedSegmentContract:
    """Vehicle-neutral acceptance contract for one reusable phase type."""

    name: SpecializedSegmentType
    composition_template: str
    description: str
    applicable_modes: tuple[str, ...]
    required_channels: tuple[str, ...]
    quality_gates: tuple[str, ...]
    claim_boundary: str
    ####


SIMPLE_AERO_SEGMENT_CONTRACTS: tuple[SpecializedSegmentContract, ...] = (
    SpecializedSegmentContract(
        name="powered_ascent",
        composition_template="powered_ascent",
        description="Thrust-driven boost or powered ascent before the passive trajectory phase.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=("altitude_m", "speed_m_s", "mass.total", "propellant_mass_kg"),
        quality_gates=("mass-flow sign", "thrust/propulsion closure", "bounded acceleration", "time-step convergence"),
        claim_boundary="bounded powered phase; not a propulsion certification",
    ),
    SpecializedSegmentContract(
        name="ballistic_coast",
        composition_template="ballistic_coast",
        description="Passive or low-control coast through the ballistic or apogee corridor.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=("altitude_m", "speed_m_s", "flight_path_angle_deg", "mass.total"),
        quality_gates=("force closure", "gravity/frame convention", "apogee/ground termination", "time-step convergence"),
        claim_boundary="bounded coast corridor; not a full reentry or impact claim",
    ),
    SpecializedSegmentContract(
        name="bank_maneuver",
        composition_template="bank_maneuver",
        description="Bounded bank, crossrange, CBCR, slalom, or weave steering phase.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=("bank_deg", "altitude_m", "speed_m_s", "crossrange_m"),
        quality_gates=("signed bank response", "crossrange direction", "alpha/beta envelope", "actuator/slew limits"),
        claim_boundary="bounded steering maneuver; not an optimized range solution",
    ),
    SpecializedSegmentContract(
        name="alpha_profile",
        composition_template="alpha_profile",
        description="Angle-of-attack, phugoid, or range-extension profile with explicit table bounds.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=("aero_alpha_deg", "altitude_m", "speed_m_s", "flight_path_angle_deg"),
        quality_gates=("alpha profile interpolation", "aero table bounds", "energy closure", "time-step convergence"),
        claim_boundary="bounded alpha/energy maneuver; not a vehicle-wide aero validation",
    ),
    SpecializedSegmentContract(
        name="skip_maneuver",
        composition_template="skip_maneuver",
        description="Skip-flight entry/exit phase with a declared reentry or lift modulation profile.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=("altitude_m", "speed_m_s", "flight_path_angle_deg", "aero_alpha_deg"),
        quality_gates=("entry/exit event", "heat or dynamic-pressure bound", "alpha/beta envelope", "convergence"),
        claim_boundary="bounded skip-like phase; not a thermal-protection certification",
    ),
    SpecializedSegmentContract(
        name="terminal_pronav",
        composition_template="terminal_pronav",
        description="Terminal proportional-navigation handoff with explicit range and closure evidence.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=(
            "pro_nav_active",
            "pro_nav_los_range_m",
            "pro_nav_closing_velocity_m_s",
            "pro_nav_acceleration_response_residual_m_s2",
        ),
        quality_gates=("handoff event", "LOS/range closure", "achieved-versus-commanded response", "terminal miss-distance"),
        claim_boundary="bounded terminal guidance segment; not an intercept or landing certification",
    ),
    SpecializedSegmentContract(
        name="moving_target_intercept",
        composition_template="moving_target_intercept",
        description="Moving-target intercept geometry with an explicit target track reference.",
        applicable_modes=("point_mass_3dof", "kinematic_3_plus_3", "rigid_body_6dof"),
        required_channels=(
            "pro_nav_active",
            "pro_nav_los_range_m",
            "pro_nav_closing_velocity_m_s",
            "pro_nav_acceleration_response_residual_m_s2",
        ),
        quality_gates=("target position/velocity provenance", "LOS/range closure", "response residual", "terminal miss-distance"),
        claim_boundary="moving-target guidance evidence only; not a complete vehicle mission claim",
    ),
)


SIMPLE_AERO_FAMILY_SEGMENTS: dict[str, tuple[SpecializedSegmentType, ...]] = {
    "ballistic": ("powered_ascent", "ballistic_coast"),
    "cbcr": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
    "crossrange": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
    "marv": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
    "phugoid": ("powered_ascent", "ballistic_coast", "alpha_profile", "terminal_pronav"),
    "range_extension": ("powered_ascent", "ballistic_coast", "alpha_profile", "terminal_pronav"),
    "skip": ("powered_ascent", "ballistic_coast", "skip_maneuver", "terminal_pronav"),
    "slalom": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
    "weave": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
    "propnav": ("powered_ascent", "ballistic_coast", "bank_maneuver", "terminal_pronav"),
}


def specialized_segment_contract(name: SpecializedSegmentType) -> SpecializedSegmentContract:
    """Return one reusable contract by stable segment name."""

    for contract in SIMPLE_AERO_SEGMENT_CONTRACTS:
        if contract.name == name:
            return contract
    raise KeyError(f"unknown specialized segment contract {name!r}")
    ####


def simple_aero_family_segments(family: str) -> tuple[SpecializedSegmentContract, ...]:
    """Return the reusable contract sequence for one SimpleAero family."""

    try:
        names = SIMPLE_AERO_FAMILY_SEGMENTS[family.casefold()]
    except KeyError as error:
        raise KeyError(f"unknown SimpleAero family {family!r}") from error
    return tuple(specialized_segment_contract(name) for name in names)
    ####


__all__ = [
    "SIMPLE_AERO_FAMILY_SEGMENTS",
    "SIMPLE_AERO_SEGMENT_CONTRACTS",
    "SpecializedSegmentContract",
    "SpecializedSegmentType",
    "specialized_segment_contract",
    "simple_aero_family_segments",
]
