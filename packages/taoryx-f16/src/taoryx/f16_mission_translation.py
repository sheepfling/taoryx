"""F-16-owned lowering for the common powered-fixed-wing racetrack."""

from __future__ import annotations

import math
from typing import Any

from taoryx_f16.resources import model_resource_root

from taoryx.powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from taoryx.racetrack_template import RacetrackFidelity
from taoryx.vehicle_composition import CompiledSegment, CompiledVehicleComposition

_MISSION_PROFILES = model_resource_root() / "verification/powered_fixed_wing_mission_profiles.yaml"
_RACETRACK_FIDELITY_BY_TIER: dict[str, RacetrackFidelity] = {
    "point_mass_3dof": "point_mass_3dof",
    "pseudo_6dof": "pseudo_6dof_kinematic_bridge",
    "rigid_body_6dof_direct_wrench": "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated": "rigid_body_6dof_surface_allocated",
}


def compile_f16_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compile only the F-16's declared semantic racetrack route.

    This package owns the source-subsonic profile and validates every semantic
    segment before calling the shared, vehicle-neutral route compiler.  It
    deliberately does not turn this planning result into an actuator or
    flight-qualification claim.
    """

    if composition.family_id != "f16_s119" or composition.mission != "powered_fixed_wing_racetrack_v1":
        raise ValueError("F-16 racetrack lowering requires the declared F-16 powered-fixed-wing mission")
    try:
        fidelity = _RACETRACK_FIDELITY_BY_TIER[composition.fidelity]
    except KeyError as error:
        raise ValueError(f"F-16 racetrack lowering has no realization for tier {composition.fidelity!r}") from error
    capability, profile_intent = resolve_powered_fixed_wing_mission_profile(_MISSION_PROFILES, "f16-subsonic", fidelity)
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments.get("left-turn")
    right_turn = segments.get("right-turn")
    if left_turn is None or right_turn is None:
        raise ValueError("F-16 powered-fixed-wing racetrack requires left-turn and right-turn segment instances")
    if _choice(left_turn, "turn_direction") != "left" or _choice(right_turn, "turn_direction") != "right":
        raise ValueError("F-16 powered-fixed-wing racetrack requires left-turn then right-turn directions")
    left_radius_m = _number(left_turn, "turn_radius_m")
    right_radius_m = _number(right_turn, "turn_radius_m")
    if not math.isclose(left_radius_m, right_radius_m, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("F-16 powered-fixed-wing racetrack requires equal left/right requested turn radii")
    left_bank_deg = _number(left_turn, "bank_limit_deg")
    right_bank_deg = _number(right_turn, "bank_limit_deg")
    if not math.isclose(left_bank_deg, right_bank_deg, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("F-16 powered-fixed-wing racetrack requires equal left/right bank limits")
    return compile_powered_fixed_wing_racetrack(
        capability,
        PoweredFixedWingRacetrackIntent(
            id=profile_intent.id,
            low_altitude_m=_number(descent, "target_altitude_m"),
            high_altitude_m=_number(climb, "target_altitude_m"),
            requested_speed_m_s=_number(initialization, "speed_m_s"),
            requested_bank_deg=left_bank_deg,
            requested_turn_radius_m=left_radius_m,
            minimum_straight_length_m=abs(_ned(climb, "gate_center_ned_m")[1]),
            level_dwell_s=profile_intent.level_dwell_s,
            turn_radius_margin=profile_intent.turn_radius_margin,
            simulation_margin_s=profile_intent.simulation_margin_s,
            gate_corridor_m=_number(climb, "corridor_m"),
            gate_altitude_tolerance_m=profile_intent.gate_altitude_tolerance_m,
            gate_speed_tolerance_mps=profile_intent.gate_speed_tolerance_mps,
        ),
        binding_id=f"{composition.id}-preflight",
        fidelity=fidelity,
        source_realization="f16_composed_semantic_preflight",
    )
    ####


def _segment(composition: CompiledVehicleComposition, segment_id: str) -> CompiledSegment:
    matches = tuple(segment for segment in composition.segments if segment.id == segment_id)
    if len(matches) != 1:
        raise ValueError(f"composition requires exactly one {segment_id!r} segment")
    return matches[0]
    ####


def _number(inputs: Any, field: str) -> float:
    values = inputs if isinstance(inputs, dict) else inputs.inputs
    value = values[field].value
    if isinstance(value, bool):
        raise ValueError(f"{field} must be numeric")
    return float(value)
    ####


def _choice(inputs: Any, field: str) -> str:
    values = inputs if isinstance(inputs, dict) else inputs.inputs
    value = values[field].value
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{field} must be a non-empty string")
    return value.strip().lower()
    ####


def _ned(segment: CompiledSegment, field: str) -> tuple[float, float, float]:
    value = segment.inputs[field].value
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{field} must be a three-component NED vector")
    return (float(value[0]), float(value[1]), float(value[2]))
    ####


__all__ = ["compile_f16_powered_fixed_wing_racetrack_from_composition"]
