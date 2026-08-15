"""Family-owned, fail-closed first-pass mission-capability planning.

Composition needs a planning seam before a native runtime is selected: callers
should be able to see which family owns a route/energy/hover/release estimate,
what it derived, and whether the estimate is applicable.  This module defines
that seam without inventing a cross-family performance model.  Each adapter is
responsible only for the semantic mission and physical family it explicitly
declares.

The first implementation wraps the existing source-provenanced powered
fixed-wing racetrack compiler.  Other families intentionally resolve to no
adapter until their own capability model exists; a missing adapter is a
preflight gap, never permission to reuse fixed-wing geometry.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

import yaml
from taoryx_source_table_fixed_wing.resources import model_resource_root

from .mission_capability import MissionCapabilityAdapter, MissionCapabilityEstimate, MissionFeasibility
from .powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from .racetrack_template import RacetrackBinding, RacetrackFidelity, resolve_racetrack_binding
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition

_ROOT = model_resource_root()
_MISSION_PROFILES = _ROOT / "verification" / "powered_fixed_wing_mission_profiles.yaml"
_RACETRACK_FIDELITY_BY_TIER: dict[str, RacetrackFidelity] = {
    "point_mass_3dof": "point_mass_3dof",
    "pseudo_6dof": "pseudo_6dof_kinematic_bridge",
    "rigid_body_6dof_direct_wrench": "rigid_body_6dof_direct_wrench",
    "rigid_body_6dof_surface_allocated": "rigid_body_6dof_surface_allocated",
}
_POWERED_FIXED_WING_PROFILE_BY_FAMILY = {
    "skywalker_x8": "x8-cruise",
    "b747": "b747-cruise",
}


class B747SourceRacetrackCapabilityAdapter:
    """Own B747 route lowering, including its distinct direct-wrench packet."""

    id = "taoryx.b747_racetrack.source_route.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "b747" and composition.mission == "powered_fixed_wing_racetrack_v1" and composition.fidelity in _RACETRACK_FIDELITY_BY_TIER
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Select generic planning or the exact source-owned direct route."""

        if composition.fidelity == "rigid_body_6dof_direct_wrench":
            proposal = compile_b747_source_direct_wrench_racetrack_from_composition(composition)
            feasibility: MissionFeasibility = "likely_feasible"
            diagnostics = proposal.diagnostics
        else:
            proposal = compile_powered_fixed_wing_racetrack_from_composition(composition)
            feasibility = "feasible" if proposal.status == "capability_feasible" else "likely_feasible"
            diagnostics = proposal.diagnostics
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=diagnostics,
            manifest=proposal.manifest(),
            plan=proposal,
        )
        ####

    ####


class X8SourceRacetrackCapabilityAdapter:
    """Own X8 route lowering, including its source direct-wrench packet."""

    id = "taoryx.x8_racetrack.source_route.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "skywalker_x8"
            and composition.mission == "powered_fixed_wing_racetrack_v1"
            and composition.fidelity in _RACETRACK_FIDELITY_BY_TIER
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Select generic planning or the exact source-owned direct route."""

        if composition.fidelity == "rigid_body_6dof_direct_wrench":
            proposal = compile_x8_source_direct_wrench_racetrack_from_composition(composition)
            feasibility: MissionFeasibility = "likely_feasible"
            diagnostics = proposal.diagnostics
        else:
            proposal = compile_powered_fixed_wing_racetrack_from_composition(composition)
            feasibility = "feasible" if proposal.status == "capability_feasible" else "likely_feasible"
            diagnostics = proposal.diagnostics
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=diagnostics,
            manifest=proposal.manifest(),
            plan=proposal,
        )
        ####

    ####


def reference_mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return the declared family-owned planners in deterministic order."""

    return (
        B747SourceRacetrackCapabilityAdapter(),
        X8SourceRacetrackCapabilityAdapter(),
    )
    ####


def compile_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compile the fixed-wing plan from semantic values and profile evidence."""

    profile_id = _POWERED_FIXED_WING_PROFILE_BY_FAMILY.get(composition.family_id)
    if profile_id is None or composition.mission != "powered_fixed_wing_racetrack_v1":
        raise ValueError("powered-fixed-wing racetrack translation requires a registered family and the powered_fixed_wing_racetrack_v1 mission")
    if composition.fidelity not in _RACETRACK_FIDELITY_BY_TIER:
        raise ValueError(f"no powered-fixed-wing racetrack realization is registered for tier {composition.fidelity!r}")
    capability, profile_intent = resolve_powered_fixed_wing_mission_profile(
        _MISSION_PROFILES,
        profile_id,
        _RACETRACK_FIDELITY_BY_TIER[composition.fidelity],
    )
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments.get("left-turn")
    right_turn = segments.get("right-turn")
    if left_turn is None or right_turn is None:
        raise ValueError("powered-fixed-wing racetrack requires segment instances 'left-turn' and 'right-turn'")
    if _choice(left_turn, "turn_direction") != "left" or _choice(right_turn, "turn_direction") != "right":
        raise ValueError("powered-fixed-wing racetrack requires left-turn then right-turn segment directions")
    left_radius_m = _number(left_turn, "turn_radius_m")
    right_radius_m = _number(right_turn, "turn_radius_m")
    if not math.isclose(left_radius_m, right_radius_m, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("powered-fixed-wing racetrack requires equal left/right requested turn radii")
    left_bank_deg = _number(left_turn, "bank_limit_deg")
    right_bank_deg = _number(right_turn, "bank_limit_deg")
    if not math.isclose(left_bank_deg, right_bank_deg, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("powered-fixed-wing racetrack requires equal left/right bank limits")

    intent = PoweredFixedWingRacetrackIntent(
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
    )
    return compile_powered_fixed_wing_racetrack(
        capability,
        intent,
        binding_id=f"{composition.id}-preflight",
        fidelity=_RACETRACK_FIDELITY_BY_TIER[composition.fidelity],
        source_realization="composed_semantic_preflight",
    )
    ####


def compile_b747_source_direct_wrench_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Lower B747 semantic route inputs through the pinned direct packet.

    The transport direct-wrench packet is intentionally not a coordinated-turn
    planner. Its autonomous source controller owns its capture gains and can
    execute the declared 8.3 km/14 degree route, so applying the generic
    bank-radius safety clip would change the source mission before runtime.
    """

    if composition.family_id != "b747" or composition.mission != "powered_fixed_wing_racetrack_v1" or composition.fidelity != "rigid_body_6dof_direct_wrench":
        raise ValueError("B747 source direct-wrench lowering requires its declared racetrack realization")
    capability, profile_intent = resolve_powered_fixed_wing_mission_profile(
        _MISSION_PROFILES,
        "b747-cruise",
        "rigid_body_6dof_direct_wrench",
    )
    source_route = _b747_source_direct_wrench_route_settings()
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments.get("left-turn")
    right_turn = segments.get("right-turn")
    if left_turn is None or right_turn is None:
        raise ValueError("B747 source direct-wrench racetrack requires segment instances 'left-turn' and 'right-turn'")
    if _choice(left_turn, "turn_direction") != "left" or _choice(right_turn, "turn_direction") != "right":
        raise ValueError("B747 source direct-wrench racetrack requires left-turn then right-turn segment directions")
    left_radius_m = _number(left_turn, "turn_radius_m")
    right_radius_m = _number(right_turn, "turn_radius_m")
    if not math.isclose(left_radius_m, right_radius_m, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("B747 source direct-wrench racetrack requires equal left/right requested turn radii")
    left_bank_deg = _number(left_turn, "bank_limit_deg")
    right_bank_deg = _number(right_turn, "bank_limit_deg")
    if not math.isclose(left_bank_deg, right_bank_deg, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("B747 source direct-wrench racetrack requires equal left/right bank limits")
    straight_length_m = abs(_ned(climb, "gate_center_ned_m")[1])
    if straight_length_m <= 0.0:
        raise ValueError("B747 source direct-wrench racetrack requires a positive high-level gate distance")
    intent = PoweredFixedWingRacetrackIntent(
        id=profile_intent.id,
        low_altitude_m=_number(descent, "target_altitude_m"),
        high_altitude_m=_number(climb, "target_altitude_m"),
        requested_speed_m_s=_number(initialization, "speed_m_s"),
        requested_bank_deg=left_bank_deg,
        requested_turn_radius_m=left_radius_m,
        minimum_straight_length_m=straight_length_m,
        level_dwell_s=profile_intent.level_dwell_s,
        turn_radius_margin=profile_intent.turn_radius_margin,
        simulation_margin_s=_setting_number(source_route, "simulation_margin_s"),
        gate_corridor_m=_number(climb, "corridor_m"),
        gate_altitude_tolerance_m=profile_intent.gate_altitude_tolerance_m,
        gate_speed_tolerance_mps=profile_intent.gate_speed_tolerance_mps,
    )
    route = resolve_racetrack_binding(
        "powered_fixed_wing_racetrack_v1",
        f"{composition.id}-source-direct-wrench",
        RacetrackBinding.model_validate(
            {
                "vehicle_id": "b747",
                "fidelity": "rigid_body_6dof_direct_wrench",
                "straight_length_m": straight_length_m,
                "turn_radius_m": left_radius_m,
                "speed_m_s": _number(initialization, "speed_m_s"),
                "low_altitude_m": _number(descent, "target_altitude_m"),
                "high_altitude_m": _number(climb, "target_altitude_m"),
                "climb_rate_m_s": _number(climb, "climb_rate_m_s"),
                "descent_rate_m_s": _number(descent, "descent_rate_m_s"),
                "left_turn_bank_deg": -left_bank_deg,
                "right_turn_bank_deg": right_bank_deg,
                "simulation_margin_s": _setting_number(source_route, "simulation_margin_s"),
                "altitude_capture_gain_per_s": _setting_number(source_route, "altitude_capture_gain_per_s"),
                "altitude_capture_max_mps": _setting_number(source_route, "altitude_capture_max_mps"),
                "position_capture_gain": _setting_number(source_route, "position_capture_gain_per_m"),
                "position_capture_max_correction_mps": _setting_number(source_route, "position_capture_max_correction_mps"),
                "gate_corridor_m": _number(climb, "corridor_m"),
                "gate_altitude_tolerance_m": profile_intent.gate_altitude_tolerance_m,
                "gate_speed_tolerance_mps": profile_intent.gate_speed_tolerance_mps,
                "source_realization": str(source_route["control_owner"]),
                "status": "source_nominal_baseline",
            }
        ),
    )
    return CapabilityScaledRacetrack(
        capability=capability,
        intent=intent,
        route=route,
        status="source_nominal_baseline",
        diagnostics=(
            "B747 direct-wrench route preserves source-owned autonomous capture gains and settling margin; "
            "its direct wrench is not an externally callable action.",
        ),
        minimum_turn_radius_m=left_radius_m,
        minimum_straight_length_m=straight_length_m,
    )
    ####


def _b747_source_direct_wrench_route_settings() -> dict[str, object]:
    """Load the plug-in-owned controller settings for the B747 source packet."""

    payload = yaml.safe_load(_MISSION_PROFILES.read_text(encoding="utf-8"))
    try:
        settings = payload["profiles"]["b747-cruise"]["source_direct_wrench_route"]
    except (KeyError, TypeError) as error:
        raise ValueError("B747 mission profile lacks source_direct_wrench_route settings") from error
    if not isinstance(settings, dict):
        raise ValueError("B747 source_direct_wrench_route settings must be a mapping")
    required = {
        "source_problem",
        "control_owner",
        "simulation_margin_s",
        "altitude_capture_gain_per_s",
        "altitude_capture_max_mps",
        "position_capture_gain_per_m",
        "position_capture_max_correction_mps",
        "claim_boundary",
    }
    missing = sorted(required.difference(settings))
    if missing:
        raise ValueError("B747 source_direct_wrench_route settings omit: " + ", ".join(missing))
    return settings
    ####


def compile_x8_source_direct_wrench_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Lower X8 semantic route inputs through the pinned direct packet.

    The source direct-moment packet uses a deliberately non-coordinated turn
    and asymmetric signed bank convention. The generic planner would replace
    that proven source route with a different conservative geometry, so this
    translator keeps the source controller while mapping the public segments.
    """

    if (
        composition.family_id != "skywalker_x8"
        or composition.mission != "powered_fixed_wing_racetrack_v1"
        or composition.fidelity != "rigid_body_6dof_direct_wrench"
    ):
        raise ValueError("X8 source direct-wrench lowering requires its declared racetrack realization")
    capability, profile_intent = resolve_powered_fixed_wing_mission_profile(
        _MISSION_PROFILES,
        "x8-cruise",
        "rigid_body_6dof_direct_wrench",
    )
    source_route = _x8_source_direct_wrench_route_settings()
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments.get("left-turn")
    right_turn = segments.get("right-turn")
    if left_turn is None or right_turn is None:
        raise ValueError("X8 source direct-wrench racetrack requires segment instances 'left-turn' and 'right-turn'")
    if _choice(left_turn, "turn_direction") != "left" or _choice(right_turn, "turn_direction") != "right":
        raise ValueError("X8 source direct-wrench racetrack requires left-turn then right-turn segment directions")
    left_radius_m = _number(left_turn, "turn_radius_m")
    right_radius_m = _number(right_turn, "turn_radius_m")
    if not math.isclose(left_radius_m, right_radius_m, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("X8 source direct-wrench racetrack requires equal left/right requested turn radii")
    left_bank_deg = _number(left_turn, "bank_limit_deg")
    right_bank_deg = _number(right_turn, "bank_limit_deg")
    if not math.isclose(left_bank_deg, right_bank_deg, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("X8 source direct-wrench racetrack requires equal left/right bank limits")
    straight_length_m = abs(_ned(climb, "gate_center_ned_m")[1])
    if straight_length_m <= 0.0:
        raise ValueError("X8 source direct-wrench racetrack requires a positive high-level gate distance")
    intent = PoweredFixedWingRacetrackIntent(
        id=profile_intent.id,
        low_altitude_m=_number(descent, "target_altitude_m"),
        high_altitude_m=_number(climb, "target_altitude_m"),
        requested_speed_m_s=_number(initialization, "speed_m_s"),
        requested_bank_deg=left_bank_deg,
        requested_turn_radius_m=left_radius_m,
        minimum_straight_length_m=straight_length_m,
        level_dwell_s=profile_intent.level_dwell_s,
        turn_radius_margin=profile_intent.turn_radius_margin,
        simulation_margin_s=_setting_number(source_route, "simulation_margin_s"),
        gate_corridor_m=_number(climb, "corridor_m"),
        gate_altitude_tolerance_m=profile_intent.gate_altitude_tolerance_m,
        gate_speed_tolerance_mps=profile_intent.gate_speed_tolerance_mps,
    )
    route = resolve_racetrack_binding(
        "powered_fixed_wing_racetrack_v1",
        f"{composition.id}-source-direct-wrench",
        RacetrackBinding.model_validate(
            {
                "vehicle_id": "skywalker_x8",
                "fidelity": "rigid_body_6dof_direct_wrench",
                "straight_length_m": straight_length_m,
                "turn_radius_m": left_radius_m,
                "speed_m_s": _number(initialization, "speed_m_s"),
                "low_altitude_m": _number(descent, "target_altitude_m"),
                "high_altitude_m": _number(climb, "target_altitude_m"),
                "climb_rate_m_s": _number(climb, "climb_rate_m_s"),
                "descent_rate_m_s": _number(descent, "descent_rate_m_s"),
                "left_turn_bank_deg": -left_bank_deg,
                "right_turn_bank_deg": _setting_number(source_route, "right_turn_bank_sign") * right_bank_deg,
                "simulation_margin_s": _setting_number(source_route, "simulation_margin_s"),
                "altitude_capture_gain_per_s": _setting_number(source_route, "altitude_capture_gain_per_s"),
                "altitude_capture_max_mps": _setting_number(source_route, "altitude_capture_max_mps"),
                "position_capture_gain": _setting_number(source_route, "position_capture_gain_per_m"),
                "position_capture_max_correction_mps": _setting_number(source_route, "position_capture_max_correction_mps"),
                "gate_corridor_m": _number(climb, "corridor_m"),
                "gate_altitude_tolerance_m": profile_intent.gate_altitude_tolerance_m,
                "gate_speed_tolerance_mps": profile_intent.gate_speed_tolerance_mps,
                "source_realization": str(source_route["control_owner"]),
                "status": "source_nominal_baseline",
            }
        ),
    )
    return CapabilityScaledRacetrack(
        capability=capability,
        intent=intent,
        route=route,
        status="source_nominal_baseline",
        diagnostics=(
            "X8 direct-wrench route preserves source-owned autonomous capture gains, settling margin, and signed "
            "non-coordinated turn convention; its direct wrench is not an externally callable action.",
        ),
        minimum_turn_radius_m=left_radius_m,
        minimum_straight_length_m=straight_length_m,
    )
    ####


def _x8_source_direct_wrench_route_settings() -> dict[str, object]:
    """Load the plug-in-owned controller settings for the X8 source packet."""

    payload = yaml.safe_load(_MISSION_PROFILES.read_text(encoding="utf-8"))
    try:
        settings = payload["profiles"]["x8-cruise"]["source_direct_wrench_route"]
    except (KeyError, TypeError) as error:
        raise ValueError("X8 mission profile lacks source_direct_wrench_route settings") from error
    if not isinstance(settings, dict):
        raise ValueError("X8 source_direct_wrench_route settings must be a mapping")
    required = {
        "source_problem",
        "control_owner",
        "simulation_margin_s",
        "altitude_capture_gain_per_s",
        "altitude_capture_max_mps",
        "position_capture_gain_per_m",
        "position_capture_max_correction_mps",
        "right_turn_bank_sign",
        "claim_boundary",
    }
    missing = sorted(required.difference(settings))
    if missing:
        raise ValueError("X8 source_direct_wrench_route settings omit: " + ", ".join(missing))
    return settings
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


def _setting_number(settings: Mapping[str, object], field: str) -> float:
    """Read one finite numeric source-route setting with a stable error."""

    value = settings.get(field)
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"source route setting {field!r} must be finite numeric")
    return float(value)
    ####


def _choice(inputs: Any, field: str) -> str:
    """Return one required discrete segment value without coercing numerics."""

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


__all__ = [
    "B747SourceRacetrackCapabilityAdapter",
    "X8SourceRacetrackCapabilityAdapter",
    "compile_b747_source_direct_wrench_racetrack_from_composition",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "compile_x8_source_direct_wrench_racetrack_from_composition",
    "reference_mission_capability_adapters",
]
