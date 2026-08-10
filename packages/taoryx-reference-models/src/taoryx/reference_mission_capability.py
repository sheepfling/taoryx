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
from typing import Any

import yaml
from taoryx_reference_models.resources import model_resource_root

from .local_direct_wrench_screen_registry import (
    LocalDirectWrenchScreenDefinition,
    local_direct_wrench_screen_definitions,
)
from .local_native_coordinate_lqi_mission_translation import compile_local_native_coordinate_lqi_screen_mission
from .local_native_coordinate_lqi_screen_registry import (
    LocalNativeCoordinateLqiScreenDefinition,
    local_native_coordinate_lqi_screen_definitions,
)
from .mission_capability import MissionCapabilityAdapter, MissionCapabilityEstimate, MissionFeasibility
from .powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from .racetrack_template import RacetrackFidelity, resolve_racetrack_binding
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
    "a320_openap_3dof": "a320-cruise",
    "f16_s119": "f16-subsonic",
}


class PoweredFixedWingRacetrackCapabilityAdapter:
    """Capability-scaled planning for the shared airbreather racetrack."""

    id = "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id in _POWERED_FIXED_WING_PROFILE_BY_FAMILY
            and composition.family_id not in {"b747", "skywalker_x8"}
            and composition.mission == "powered_fixed_wing_racetrack_v1"
            and composition.fidelity in _RACETRACK_FIDELITY_BY_TIER
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        proposal = compile_powered_fixed_wing_racetrack_from_composition(composition)
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


class B747SourceRacetrackCapabilityAdapter:
    """Own B747 route lowering, including its distinct direct-wrench packet."""

    id = "taoryx.b747_racetrack.source_route.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "b747"
            and composition.mission == "powered_fixed_wing_racetrack_v1"
            and composition.fidelity in _RACETRACK_FIDELITY_BY_TIER
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


class HummingbirdHoverTranslationCapabilityAdapter:
    """Aggregate-thrust feasibility estimate for the Hummingbird pseudo tier."""

    id = "taoryx.multirotor_hover_translation.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hummingbird"
            and composition.mission == "multirotor_pad_box_yaw_recovery_land_v1"
            and composition.fidelity == "pseudo_6dof"
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Derive aggregate thrust reserve without claiming rotor allocation."""

        from .hummingbird_mission_translation import compile_hummingbird_pseudo_mission
        from .trajectory.hummingbird_pseudo6dof import HummingbirdPseudo6DOFModel

        plan = compile_hummingbird_pseudo_mission(composition)
        model = HummingbirdPseudo6DOFModel(mass_kg=plan.initial_mass_kg)
        hover_thrust_n = model.mass_kg * model.gravity_m_s2
        thrust_margin_n = model.maximum_thrust_n - hover_thrust_n
        maximum_vertical_acceleration_m_s2 = thrust_margin_n / model.mass_kg
        maximum_level_lateral_acceleration_m_s2 = math.sqrt(
            max(0.0, model.maximum_thrust_n**2 - hover_thrust_n**2)
        ) / model.mass_kg
        infeasible = thrust_margin_n <= 0.0
        feasibility: MissionFeasibility = "certainly_infeasible" if infeasible else "likely_feasible"
        diagnostics = (
            "aggregate-thrust reserve is insufficient to maintain a stationary hover"
            if infeasible
            else "aggregate-thrust reserve supports hover; translation/yaw tracking remains a response-law and mission-execution check"
        )
        manifest = plan.manifest()
        manifest["capability"] = {
            "model_id": model.profile_id,
            "control_realization": "aggregate_thrust_vector_surrogate",
            "mass_kg": model.mass_kg,
            "maximum_thrust_n": model.maximum_thrust_n,
            "hover_thrust_n": hover_thrust_n,
            "thrust_margin_n": thrust_margin_n,
            "maximum_vertical_acceleration_m_s2": maximum_vertical_acceleration_m_s2,
            "maximum_level_lateral_acceleration_m_s2": maximum_level_lateral_acceleration_m_s2,
            "physical_motor_allocation": False,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=(diagnostics,),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


class NescSourceReplayCapabilityAdapter:
    """Stage/event continuity report for the immutable NESC replay witness."""

    id = "taoryx.nesc_staged_source_replay.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "reference_nesc_two_stage_rocket"
            and composition.mission == "staged_rocket_launch_target_state_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose pinned stage chronology without calling it a rocket controller."""

        from .nesc_mission_translation import compile_nesc_source_replay_mission

        plan = compile_nesc_source_replay_mission(composition)
        segments = tuple(plan.segments)
        event_order_valid = all(
            segment.end_time_s >= segment.start_time_s for segment in segments
        ) and all(
            current.start_time_s >= previous.start_time_s
            for previous, current in zip(segments, segments[1:], strict=False)
        )
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": (
                "uncontrolled_source_replay"
                if composition.fidelity == "point_mass_3dof"
                else "named_attitude_response_source_replay"
            ),
            "participating_nonlinear_plant": False,
            "physical_gimbal_allocation": False,
            "source_initial_mass_kg": plan.source_initial_mass_kg,
            "source_initial_heading_deg": plan.source_initial_heading_deg,
            "source_replay_duration_s": segments[-1].end_time_s,
            "stage_event_order_valid": event_order_valid,
            "required_truth_event_count": sum(len(segment.required_truth_events) for segment in segments),
        }
        feasibility: MissionFeasibility = "likely_feasible" if event_order_valid else "unknown"
        diagnostic = (
            "pinned source history provides ordered ignition, separation, cutoff, and terminal events; "
            "the replay has no participating propulsion, gimbal, or guidance plant"
            if event_order_valid
            else "pinned source history has non-monotonic stage intervals; replay feasibility is unknown"
        )
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=(diagnostic,),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


class LocalDirectWrenchScreenCapabilityAdapter:
    """Expose one bounded authority contract for a pinned local source screen.

    This is intentionally a capability record for one local controller
    witness, not an aerodynamic mission planner.  It makes discovery and
    preflight agree with an installed runnable screen while retaining its
    strict no-navigation/no-effector-allocation boundary.
    """

    def __init__(self, definition: LocalDirectWrenchScreenDefinition) -> None:
        self.definition = definition
        self.id = definition.capability_adapter_id
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return self.definition.supports(composition)
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Return the source-pinned local state, cadence, and wrench contract."""

        from .local_direct_wrench_mission_translation import compile_local_direct_wrench_screen_mission

        config = self.definition.config_factory()
        plan = compile_local_direct_wrench_screen_mission(
            composition,
            family_id=self.definition.family_id,
            mission_id=self.definition.mission_id,
            initialization_id=self.definition.initialization_id,
            segment_id=self.definition.segment_id,
            screen_config_id=config.id,
        )
        authority_span = {
            axis: float(config.limits.upper[axis]) - float(config.limits.lower[axis])
            for axis in config.limits.axes
        }
        rate_limited_axes = tuple(
            axis for axis in config.limits.axes if config.limits.rate_limit_per_s[axis] is not None
        )
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "direct_wrench",
            "participating_nonlinear_plant": True,
            "physical_effector_allocation": False,
            "source_physical_trim": False,
            "navigation_guidance": False,
            "screen_config_id": config.id,
            "plant_id": config.plant_id,
            "state_names": list(config.state_names),
            "assessment_state_names": list(config.assessment_state_names or config.state_names),
            "integration_dt_s": config.dt_s,
            "screen_duration_s": config.duration_s,
            "controller_method": config.controller_method,
            "integral_output_names": list(config.lqi_result.output_names) if config.lqi_result is not None else [],
            "controller_tuning_campaign_id": config.lqi_campaign_id,
            "controller_screen_execution": {
                "status": f"executed_by_this_{config.controller_method}_screen",
                "mission_id": self.definition.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": "direct_wrench",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
            "claim_boundary": self.definition.claim_boundary,
            "direct_wrench_limits": {
                "lower": dict(config.limits.lower),
                "upper": dict(config.limits.upper),
                "rate_limit_per_s": dict(config.limits.rate_limit_per_s),
                "authority_span": authority_span,
                "rate_limited_axes": list(rate_limited_axes),
            },
        }
        if self.definition.paired_lqi_mission_id is not None:
            if (
                self.definition.paired_lqi_capability_adapter_id is None
                or self.definition.paired_lqi_campaign_id is None
            ):
                raise ValueError("paired local direct-wrench LQI advertisement is incomplete")
            manifest["capability"]["offset_free_tuning_candidate"] = {
                "campaign_id": self.definition.paired_lqi_campaign_id,
                "method": "lqi",
                "availability": "executed_by_paired_composition_screen",
                "controller_screen_execution": {
                    "status": "executed_by_paired_lqi_screen",
                    "mission_id": self.definition.paired_lqi_mission_id,
                    "capability_adapter_id": self.definition.paired_lqi_capability_adapter_id,
                    "operations": ["validate", "batch"],
                    "control_realization": "direct_wrench",
                },
                "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
                "claim_boundary": (
                    "The current public batch screen executes its declared LQR controller. The paired LQI mission "
                    "executes one integrated body-speed recovery through the same bounded generalized direct-wrench "
                    "bridge; neither screen establishes physical effectors, a wind or mass-variation environment, "
                    "navigation, or a vehicle qualification result."
                ),
            }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                f"pinned {self.definition.family_id} local source-load screen declares finite six-axis direct-wrench "
                "bounds and cadence; executing the recovery screen remains required to establish its local result",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


class LocalNativeCoordinateLqiScreenCapabilityAdapter:
    """Advertise one pinned native-control LQI recovery without effector claims."""

    def __init__(self, definition: LocalNativeCoordinateLqiScreenDefinition) -> None:
        self.definition = definition
        self.id = definition.capability_adapter_id
        ####

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return self.definition.supports(composition)
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose the exact candidate, native controls, and local recovery scope."""

        config = self.definition.config_factory()
        plan = compile_local_native_coordinate_lqi_screen_mission(
            composition,
            family_id=self.definition.family_id,
            mission_id=self.definition.mission_id,
            fidelity=self.definition.fidelity,
            initialization_id=self.definition.initialization_id,
            segment_id=self.definition.segment_id,
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
                "mission_id": self.definition.mission_id,
                "capability_adapter_id": self.id,
                "operations": ["validate", "batch"],
                "control_realization": "native_named_coordinates",
            },
            "persistent_disturbance_status": "not_executable_without_a_declared_source_derivative_environment",
            "claim_boundary": self.definition.claim_boundary,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible",
            diagnostics=(
                f"pinned {self.definition.family_id} local screen retains a safe common-host LQI candidate, finite "
                "named-control bounds, and a fixed nonlinear cadence; execution remains required for its local result",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


def reference_mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return the declared family-owned planners in deterministic order."""

    return (
        PoweredFixedWingRacetrackCapabilityAdapter(),
        B747SourceRacetrackCapabilityAdapter(),
        X8SourceRacetrackCapabilityAdapter(),
        HummingbirdHoverTranslationCapabilityAdapter(),
        NescSourceReplayCapabilityAdapter(),
        *(LocalDirectWrenchScreenCapabilityAdapter(definition) for definition in local_direct_wrench_screen_definitions()),
        *(LocalNativeCoordinateLqiScreenCapabilityAdapter(definition) for definition in local_native_coordinate_lqi_screen_definitions()),
    )
    ####


def compile_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compile the fixed-wing plan from semantic values and profile evidence."""

    profile_id = _POWERED_FIXED_WING_PROFILE_BY_FAMILY.get(composition.family_id)
    if profile_id is None or composition.mission != "powered_fixed_wing_racetrack_v1":
        raise ValueError(
            "powered-fixed-wing racetrack translation requires a registered family and the "
            "powered_fixed_wing_racetrack_v1 mission"
        )
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

    if (
        composition.family_id != "b747"
        or composition.mission != "powered_fixed_wing_racetrack_v1"
        or composition.fidelity != "rigid_body_6dof_direct_wrench"
    ):
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
        simulation_margin_s=float(source_route["simulation_margin_s"]),
        gate_corridor_m=_number(climb, "corridor_m"),
        gate_altitude_tolerance_m=profile_intent.gate_altitude_tolerance_m,
        gate_speed_tolerance_mps=profile_intent.gate_speed_tolerance_mps,
    )
    route = resolve_racetrack_binding(
        "powered_fixed_wing_racetrack_v1",
        f"{composition.id}-source-direct-wrench",
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
            "simulation_margin_s": float(source_route["simulation_margin_s"]),
            "altitude_capture_gain_per_s": float(source_route["altitude_capture_gain_per_s"]),
            "altitude_capture_max_mps": float(source_route["altitude_capture_max_mps"]),
            "position_capture_gain": float(source_route["position_capture_gain_per_m"]),
            "position_capture_max_correction_mps": float(source_route["position_capture_max_correction_mps"]),
            "gate_corridor_m": _number(climb, "corridor_m"),
            "gate_altitude_tolerance_m": profile_intent.gate_altitude_tolerance_m,
            "gate_speed_tolerance_mps": profile_intent.gate_speed_tolerance_mps,
            "source_realization": str(source_route["control_owner"]),
            "status": "source_nominal_baseline",
        },
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
        simulation_margin_s=float(source_route["simulation_margin_s"]),
        gate_corridor_m=_number(climb, "corridor_m"),
        gate_altitude_tolerance_m=profile_intent.gate_altitude_tolerance_m,
        gate_speed_tolerance_mps=profile_intent.gate_speed_tolerance_mps,
    )
    route = resolve_racetrack_binding(
        "powered_fixed_wing_racetrack_v1",
        f"{composition.id}-source-direct-wrench",
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
            "right_turn_bank_deg": float(source_route["right_turn_bank_sign"]) * right_bank_deg,
            "simulation_margin_s": float(source_route["simulation_margin_s"]),
            "altitude_capture_gain_per_s": float(source_route["altitude_capture_gain_per_s"]),
            "altitude_capture_max_mps": float(source_route["altitude_capture_max_mps"]),
            "position_capture_gain": float(source_route["position_capture_gain_per_m"]),
            "position_capture_max_correction_mps": float(source_route["position_capture_max_correction_mps"]),
            "gate_corridor_m": _number(climb, "corridor_m"),
            "gate_altitude_tolerance_m": profile_intent.gate_altitude_tolerance_m,
            "gate_speed_tolerance_mps": profile_intent.gate_speed_tolerance_mps,
            "source_realization": str(source_route["control_owner"]),
            "status": "source_nominal_baseline",
        },
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
    "HummingbirdHoverTranslationCapabilityAdapter",
    "NescSourceReplayCapabilityAdapter",
    "LocalDirectWrenchScreenCapabilityAdapter",
    "LocalNativeCoordinateLqiScreenCapabilityAdapter",
    "PoweredFixedWingRacetrackCapabilityAdapter",
    "X8SourceRacetrackCapabilityAdapter",
    "compile_b747_source_direct_wrench_racetrack_from_composition",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "compile_x8_source_direct_wrench_racetrack_from_composition",
    "reference_mission_capability_adapters",
]
