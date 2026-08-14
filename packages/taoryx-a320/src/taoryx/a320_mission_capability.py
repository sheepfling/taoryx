"""Family-owned capability planning for the A320 OpenAP products."""

from __future__ import annotations

import math
from typing import Any

from taoryx_a320.resources import model_resource_root

from taoryx.local_native_coordinate_lqi_mission_translation import compile_local_native_coordinate_lqi_screen_mission
from taoryx.local_native_coordinate_lqi_screen_registry import resolve_local_native_coordinate_lqi_screen_definition
from taoryx.mission_capability import MissionCapabilityEstimate, MissionFeasibility
from taoryx.powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from taoryx.racetrack_template import RacetrackFidelity
from taoryx.vehicle_composition import CompiledSegment, CompiledVehicleComposition

A320_RACETRACK_CAPABILITY_ADAPTER_ID = "taoryx.a320_openap_racetrack.capability_scaled.v1"
_LOCAL_NATIVE_COORDINATE_LQI_ADAPTER_ID = "taoryx.a320.local_native_coordinate_lqi_screen.capability.v1"
_RACETRACK_FIDELITY_BY_TIER: dict[str, RacetrackFidelity] = {
    "point_mass_3dof": "point_mass_3dof",
    "pseudo_6dof": "pseudo_6dof_kinematic_bridge",
}


class A320OpenAPRacetrackCapabilityAdapter:
    """Lower the A320 semantic racetrack with the package-owned profile."""

    id = A320_RACETRACK_CAPABILITY_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "a320_openap_3dof"
            and composition.mission == "powered_fixed_wing_racetrack_v1"
            and composition.fidelity in _RACETRACK_FIDELITY_BY_TIER
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        proposal = compile_a320_powered_fixed_wing_racetrack_from_composition(composition)
        feasibility: MissionFeasibility = "feasible" if proposal.status == "capability_feasible" else "likely_feasible"
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=proposal.diagnostics,
            manifest=proposal.manifest(),
            plan=proposal,
        )
        ####

    ####


class A320LocalNativeCoordinateLqiCapabilityAdapter:
    """Advertise the pinned A320 pseudo-6DOF named-coordinate recovery."""

    id = _LOCAL_NATIVE_COORDINATE_LQI_ADAPTER_ID

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        definition = resolve_local_native_coordinate_lqi_screen_definition(composition)
        return definition is not None and definition.capability_adapter_id == self.id
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        definition = resolve_local_native_coordinate_lqi_screen_definition(composition)
        if definition is None or definition.capability_adapter_id != self.id:
            raise ValueError("no A320 local native-coordinate LQI screen is registered for this composition")
        config = definition.config_factory()
        plan = compile_local_native_coordinate_lqi_screen_mission(
            composition,
            family_id=definition.family_id,
            mission_id=definition.mission_id,
            fidelity=definition.fidelity,
            initialization_id=definition.initialization_id,
            segment_id=definition.segment_id,
            screen_config_id=config.id,
        )
        candidate = config.candidate
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "native_named_coordinates",
            "participating_nonlinear_plant": True,
            "physical_effector_allocation": False,
            "source_physical_trim": False,
            "navigation_guidance": False,
            "screen_config_id": config.id,
            "plant_id": config.plant_id,
            "state_names": list(config.plant.state_names),
            "assessment_state_names": list(config.assessment_state_names),
            "native_control_names": list(candidate.control_names),
            "native_control_limits": {
                "lower": dict(config.control_lower),
                "upper": dict(config.control_upper),
            },
            "integration_dt_s": config.dt_s,
            "screen_duration_s": config.duration_s,
            "controller_method": "lqi",
            "controller_tuning_campaign_id": config.campaign_id,
            "integral_output_names": list(candidate.lqi.output_names if candidate.lqi is not None else ()),
            "controller_screen_execution": {
                "status": "executed_by_this_lqi_screen",
                "mission_id": definition.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": "native_named_coordinates",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
            "claim_boundary": definition.claim_boundary,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                "pinned A320 pseudo-6DOF local LQI screen retains finite named control bounds and a common-host "
                "candidate; execution remains required to establish its local recovery result",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####

    ####


def compile_a320_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compile the A320 route from its package-owned profile and semantic values."""

    if composition.family_id != "a320_openap_3dof" or composition.mission != "powered_fixed_wing_racetrack_v1":
        raise ValueError("A320 powered-fixed-wing racetrack requires the declared A320 family and mission")
    fidelity = _RACETRACK_FIDELITY_BY_TIER.get(composition.fidelity)
    if fidelity is None:
        raise ValueError(f"A320 has no powered-fixed-wing racetrack realization for tier {composition.fidelity!r}")
    capability, profile_intent = resolve_powered_fixed_wing_mission_profile(
        model_resource_root() / "verification/powered_fixed_wing_mission_profiles.yaml",
        "a320-cruise",
        fidelity,
    )
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments.get("left-turn")
    right_turn = segments.get("right-turn")
    if left_turn is None or right_turn is None:
        raise ValueError("A320 racetrack requires segment instances 'left-turn' and 'right-turn'")
    if _choice(left_turn, "turn_direction") != "left" or _choice(right_turn, "turn_direction") != "right":
        raise ValueError("A320 racetrack requires left-turn then right-turn segment directions")
    left_radius_m = _number(left_turn, "turn_radius_m")
    right_radius_m = _number(right_turn, "turn_radius_m")
    if not math.isclose(left_radius_m, right_radius_m, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("A320 racetrack requires equal left/right requested turn radii")
    left_bank_deg = _number(left_turn, "bank_limit_deg")
    right_bank_deg = _number(right_turn, "bank_limit_deg")
    if not math.isclose(left_bank_deg, right_bank_deg, rel_tol=1.0e-9, abs_tol=1.0e-6):
        raise ValueError("A320 racetrack requires equal left/right bank limits")
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
        fidelity=fidelity,
        source_realization="a320_openap_composed_semantic_preflight",
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
    if not isinstance(value, str):
        raise ValueError(f"{field} must be text")
    return value
    ####


def _ned(segment: CompiledSegment, field: str) -> tuple[float, float, float]:
    value = segment.inputs[field].value
    if not isinstance(value, list | tuple) or len(value) != 3:
        raise ValueError(f"{field} must be a three-component NED vector")
    return (float(value[0]), float(value[1]), float(value[2]))
    ####


__all__ = [
    "A320LocalNativeCoordinateLqiCapabilityAdapter",
    "A320OpenAPRacetrackCapabilityAdapter",
    "A320_RACETRACK_CAPABILITY_ADAPTER_ID",
    "compile_a320_powered_fixed_wing_racetrack_from_composition",
]
