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
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, Protocol

from .local_direct_wrench_screen_registry import (
    LocalDirectWrenchScreenDefinition,
    local_direct_wrench_screen_definitions,
)
from .powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from .racetrack_template import RacetrackFidelity
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition

MissionFeasibility = Literal[
    "feasible",
    "likely_feasible",
    "unknown",
    "likely_infeasible",
    "certainly_infeasible",
]

_ROOT = Path(__file__).resolve().parents[2]
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


@dataclass(frozen=True, slots=True)
class MissionCapabilityEstimate:
    """One transparent first-pass feasibility estimate for a composition."""

    adapter_id: str
    family_id: str
    mission_id: str
    fidelity: str
    feasibility: MissionFeasibility
    diagnostics: tuple[str, ...]
    manifest: dict[str, object]
    plan: object

    def as_dict(self) -> dict[str, object]:
        """Return the discovery/preflight-safe part of the estimate."""

        return {
            "schema": "taoryx.mission-capability-estimate/v1alpha1",
            "adapter_id": self.adapter_id,
            "family_id": self.family_id,
            "mission_id": self.mission_id,
            "fidelity": self.fidelity,
            "feasibility": self.feasibility,
            "diagnostics": list(self.diagnostics),
            "derived_mission": self.manifest,
            "claim_boundary": (
                "This is a family-owned first-pass capability estimate. It does not replace native adapter "
                "binding, trim, control, integration, truth-objective evaluation, or qualification."
            ),
        }
        ####
    ####


class MissionCapabilityAdapter(Protocol):
    """Family-specific semantic mission estimator contract."""

    id: str

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        """Return whether this exact family/mission/fidelity is owned here."""
        ...

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Derive a source-owned first-pass estimate or raise for invalid intent."""
        ...


class PoweredFixedWingRacetrackCapabilityAdapter:
    """Capability-scaled planning for the shared airbreather racetrack."""

    id = "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id in _POWERED_FIXED_WING_PROFILE_BY_FAMILY
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


class PassiveTumblingReleaseCapabilityAdapter:
    """Release/impact plausibility for the explicitly uncontrolled body witness."""

    id = "taoryx.passive_tumbling_release.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "tumbling_body"
            and composition.mission == "tumbling_body_release_damping_impact_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Report the canonical body's release-energy and area-policy contract.

        The estimate intentionally stops short of predicting a detailed impact
        state. The native passive propagation remains the authority for that
        result; this seam only proves that the declared release is finite,
        passive, and has enough horizon to contain even the vacuum fall time.
        """

        from .passive_tumbling_mission_translation import compile_passive_tumbling_mission

        plan = compile_passive_tumbling_mission(composition)
        release_altitude_m = plan.vehicle.initial_altitude_m
        release_speed_m_s = plan.vehicle.initial_speed_m_s
        gravity_m_s2 = 9.80665
        vacuum_fall_time_s = math.sqrt(2.0 * release_altitude_m / gravity_m_s2)
        ballistic_coefficient_kg_m2 = plan.body.mass_kg / (
            plan.vehicle.drag_coefficient * plan.body.reference_area_m2
        )
        within_horizon = vacuum_fall_time_s <= plan.horizon_s
        feasibility: MissionFeasibility = "likely_feasible" if within_horizon else "unknown"
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "uncontrolled_passive_body",
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "release_altitude_m": release_altitude_m,
            "release_speed_m_s": release_speed_m_s,
            "release_specific_energy_j_kg": 0.5 * release_speed_m_s**2 + gravity_m_s2 * release_altitude_m,
            "ballistic_coefficient_kg_m2": ballistic_coefficient_kg_m2,
            "vacuum_fall_time_lower_bound_s": vacuum_fall_time_s,
            "simulation_horizon_s": plan.horizon_s,
            "horizon_contains_vacuum_fall_lower_bound": within_horizon,
            "area_policy": plan.area_policy,
        }
        diagnostic = (
            "declared release horizon contains the vacuum-fall lower bound; native passive propagation still "
            "determines atmospheric impact and rotational behavior"
            if within_horizon
            else "declared horizon is shorter than the vacuum-fall lower bound; impact cannot be preflight-assured"
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


class X15StagedReachabilityCapabilityAdapter:
    """Source-pinned staging and horizon estimate for the reduced X-15 witness."""

    id = "taoryx.x15_staged_reachability.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "x15"
            and composition.mission == "x15_staged_booster_reachability_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose source-pinned boost/release chronology and reduced-model limits."""

        from .x15_staged_mission_translation import compile_x15_staged_reachability_mission

        plan = compile_x15_staged_reachability_mission(composition)
        staging = dict(plan.source_staging)
        powered_duration_s = float(staging["powered_duration_s"])
        release_time_s = float(staging["release_time_s"])
        ordered = 0.0 <= powered_duration_s <= release_time_s <= plan.horizon_s
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": "open_loop" if composition.fidelity == "point_mass_3dof" else "response_law",
            "participating_nonlinear_plant": False,
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "source_launch_mass_kg": float(staging["launch_mass_kg"]),
            "source_initial_speed_m_s": float(staging["initial_speed_m_s"]),
            "powered_duration_s": powered_duration_s,
            "release_time_s": release_time_s,
            "simulation_horizon_s": plan.horizon_s,
            "staging_and_horizon_order_valid": ordered,
        }
        feasibility: MissionFeasibility = "likely_feasible" if ordered else "unknown"
        diagnostic = (
            "source-pinned boost, release, and reduced glide horizon are ordered; the retained witness still "
            "does not provide guidance, allocation, or a physical X-15 plant"
            if ordered
            else "source staging/release chronology is inconsistent with the declared horizon"
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
            "integration_dt_s": config.dt_s,
            "screen_duration_s": config.duration_s,
            "direct_wrench_limits": {
                "lower": dict(config.limits.lower),
                "upper": dict(config.limits.upper),
                "rate_limit_per_s": dict(config.limits.rate_limit_per_s),
                "authority_span": authority_span,
                "rate_limited_axes": list(rate_limited_axes),
            },
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


class HL20GlideEnergyCapabilityAdapter:
    """Source-bound first-pass energy and bank-intent check for the HL-20.

    This owns only the declared unpowered lifting-body mission at the two
    reduced tiers.  It deliberately does not lower a composition into the
    local source-surface model or infer physical elevon allocation.  That
    requires a separate native adapter, trim, and control-validation path.
    """

    id = "taoryx.hl20_glide_energy.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hl20_mod_k"
            and composition.mission == "lifting_body_glide_energy_management_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Check energy direction and required opposing bank intent.

        Specific mechanical energy is used only as a necessary-condition
        screen for an unpowered glide: a requested terminal energy greater
        than the declared release energy cannot be achieved without an
        omitted energy source.  A positive margin is deliberately only
        ``likely_feasible`` because aerodynamic, crossrange, trim, and
        control authority remain native-runtime questions.
        """

        from .hl20_reachability import (
            HL20_FIXED_MASS_KG,
            HL20_PACKAGE_SHA256,
            HL20_SOURCE_SPEED_OF_SOUND_M_S,
        )
        from .reachability_aerodynamics import HL20_SOURCE_MODEL_ID

        release = composition.initialization.inputs
        trim_capture = _segment(composition, "trim_capture")
        bank_reversals = tuple(segment for segment in composition.segments if segment.id == "glide_bank_reversal")
        handoff = _segment(composition, "energy_handoff")
        if len(bank_reversals) != 2:
            raise ValueError("HL-20 glide-energy mission requires exactly two glide_bank_reversal segments")

        release_altitude_m = _number(release, "altitude_m")
        release_mach = _number(release, "mach")
        release_heading_deg = _number(release, "heading_deg")
        release_mass_kg = _optional_number(release, "mass_kg", HL20_FIXED_MASS_KG)
        release_speed_m_s = release_mach * HL20_SOURCE_SPEED_OF_SOUND_M_S
        gravity_m_s2 = 9.80665
        release_specific_energy_j_kg = 0.5 * release_speed_m_s**2 + gravity_m_s2 * release_altitude_m

        bank_commands_deg = tuple(_number(segment, "bank_command_deg") for segment in bank_reversals)
        crossrange_targets_m = tuple(_number(segment, "target_crossrange_m") for segment in bank_reversals)
        target_altitude_m = _number(handoff, "target_altitude_m")
        target_energy_j_kg = _number(handoff, "target_energy")
        target_heading_deg = _number(handoff, "target_heading_deg")
        trim_duration_s = _number(trim_capture, "duration_s")
        energy_margin_j_kg = release_specific_energy_j_kg - target_energy_j_kg
        opposing_bank_intent = bank_commands_deg[0] * bank_commands_deg[1] < 0.0
        finite_bank_geometry = all(abs(command_deg) < 90.0 for command_deg in bank_commands_deg)
        valid_release = release_altitude_m >= 0.0 and release_mach >= 0.0 and release_mass_kg > 0.0
        valid_trim_duration = trim_duration_s > 0.0

        diagnostics: list[str] = [
            "HL-20 estimate is an unpowered specific-energy necessary-condition screen; it does not prove "
            "aerodynamic trim, crossrange closure, physical effector allocation, or nonlinear execution",
        ]
        if not valid_release:
            feasibility: MissionFeasibility = "certainly_infeasible"
            diagnostics.append("release altitude, Mach, and mass must define a nonnegative, positive-mass release state")
        elif energy_margin_j_kg < 0.0:
            feasibility = "certainly_infeasible"
            diagnostics.append("terminal specific-energy target exceeds the unpowered release specific energy")
        elif not finite_bank_geometry:
            feasibility = "certainly_infeasible"
            diagnostics.append("bank-intent geometry reaches or exceeds 90 degrees and is not representable as a finite glide turn")
        elif not valid_trim_duration or not opposing_bank_intent:
            feasibility = "likely_infeasible"
            diagnostics.append(
                "mission lacks a positive trim-capture duration or opposing signed bank commands, so it does not yet "
                "express the declared bank-reversal objective"
            )
        else:
            feasibility = "likely_feasible"
            diagnostics.append(
                "release energy exceeds the requested handoff energy and two finite, opposing bank commands are declared; "
                "native trajectory execution must still establish crossrange and terminal-corridor success"
            )

        manifest: dict[str, object] = {
            "schema": "taoryx.hl20-glide-energy-capability/v1alpha1",
            "source_model_id": HL20_SOURCE_MODEL_ID,
            "source_package_sha256": HL20_PACKAGE_SHA256,
            "release": {
                "altitude_m": release_altitude_m,
                "mach": release_mach,
                "speed_of_sound_m_s": HL20_SOURCE_SPEED_OF_SOUND_M_S,
                "speed_m_s": release_speed_m_s,
                "heading_deg": release_heading_deg,
                "mass_kg": release_mass_kg,
                "specific_energy_j_kg": release_specific_energy_j_kg,
            },
            "mission_intent": {
                "trim_capture_duration_s": trim_duration_s,
                "bank_commands_deg": list(bank_commands_deg),
                "crossrange_targets_m": list(crossrange_targets_m),
                "opposing_bank_intent": opposing_bank_intent,
                "target_altitude_m": target_altitude_m,
                "target_specific_energy_j_kg": target_energy_j_kg,
                "target_heading_deg": target_heading_deg,
            },
            "capability": {
                "control_realization": "unpowered_lifting_body_energy_bank_intent",
                "participating_nonlinear_plant": False,
                "physical_effector_allocation": False,
                "direct_wrench_injection": False,
                "release_specific_energy_j_kg": release_specific_energy_j_kg,
                "target_specific_energy_j_kg": target_energy_j_kg,
                "available_specific_energy_margin_j_kg": energy_margin_j_kg,
                "opposing_bank_intent": opposing_bank_intent,
                "finite_bank_geometry": finite_bank_geometry,
                "valid_trim_duration": valid_trim_duration,
            },
            "claim_boundary": (
                "This capability estimate is not a trim result, a native flight-plan lowering, a surface-allocation "
                "claim, or a qualified HL-20 trajectory."
            ),
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility=feasibility,
            diagnostics=tuple(diagnostics),
            manifest=manifest,
            plan=manifest,
        )
        ####
    ####


class HL20SourceBoosterReleaseReplayCapabilityAdapter:
    """Chronology check for the exact source-aerodynamic HL-20 release witness."""

    id = "taoryx.hl20_source_booster_release_replay.capability.v1"

    def supports(self, composition: CompiledVehicleComposition) -> bool:
        return (
            composition.family_id == "hl20_mod_k"
            and composition.mission == "hl20_source_booster_release_replay_v1"
            and composition.fidelity in {"point_mass_3dof", "pseudo_6dof"}
        )
        ####

    def estimate(self, composition: CompiledVehicleComposition) -> MissionCapabilityEstimate:
        """Expose the source schedule without promoting it to guidance control."""

        from .hl20_source_release_mission_translation import compile_hl20_source_booster_release_mission

        plan = compile_hl20_source_booster_release_mission(composition)
        ordered = all(
            current.end_time_s >= current.start_time_s
            and current.start_time_s >= previous.start_time_s
            for previous, current in zip(plan.segments, plan.segments[1:], strict=False)
        )
        manifest = plan.manifest()
        manifest["capability"] = {
            "control_realization": (
                "source_scheduled_force_model"
                if composition.fidelity == "point_mass_3dof"
                else "source_scheduled_named_attitude_response"
            ),
            "source_aerodynamic_graph": True,
            "participating_guidance_controller": False,
            "physical_effector_allocation": False,
            "direct_wrench_injection": False,
            "source_schedule_order_valid": ordered,
        }
        return MissionCapabilityEstimate(
            adapter_id=self.id,
            family_id=composition.family_id,
            mission_id=composition.mission,
            fidelity=composition.fidelity,
            feasibility="likely_feasible" if ordered else "unknown",
            diagnostics=(
                "the fixed source booster/release and bank schedule is chronologically ordered; it is a source-scheduled "
                "aerodynamic witness, not a user-commanded guidance or effector-allocation mission",
            ),
            manifest=manifest,
            plan=plan,
        )
        ####
    ####


_MISSION_CAPABILITY_ADAPTERS: tuple[MissionCapabilityAdapter, ...] = (
    PoweredFixedWingRacetrackCapabilityAdapter(),
    HummingbirdHoverTranslationCapabilityAdapter(),
    PassiveTumblingReleaseCapabilityAdapter(),
    NescSourceReplayCapabilityAdapter(),
    X15StagedReachabilityCapabilityAdapter(),
    *(LocalDirectWrenchScreenCapabilityAdapter(definition) for definition in local_direct_wrench_screen_definitions()),
    HL20GlideEnergyCapabilityAdapter(),
    HL20SourceBoosterReleaseReplayCapabilityAdapter(),
)


def mission_capability_adapters() -> tuple[MissionCapabilityAdapter, ...]:
    """Return the declared family-owned planners in deterministic order."""

    return _MISSION_CAPABILITY_ADAPTERS
    ####


def resolve_mission_capability_adapter(composition: CompiledVehicleComposition) -> MissionCapabilityAdapter | None:
    """Resolve one exact planner, without a nearest-family fallback.

    The implementation registry remains the executable authority, while the
    mission template is the discoverable declaration.  Both must agree before
    an adapter can be selected; this prevents a new adapter class from being
    silently available to a composition client before its family records the
    intended semantic ownership.
    """

    matches = tuple(adapter for adapter in mission_capability_adapters() if adapter.supports(composition))
    if len(matches) > 1:
        raise ValueError(
            "multiple mission capability adapters claim "
            f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
        )
    declared = declared_mission_capability_adapter(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    if not matches:
        if declared is not None:
            raise ValueError(
                "mission template declares capability adapter "
                f"{declared!r}, but no installed adapter supports "
                f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
            )
        return None
    adapter = matches[0]
    if declared is None:
        raise ValueError(
            "installed capability adapter "
            f"{adapter.id!r} has no mission-template declaration for "
            f"{composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}"
        )
    if adapter.id != declared:
        raise ValueError(
            "mission-template capability adapter does not match installed adapter: "
            f"declared {declared!r}, observed {adapter.id!r}"
        )
    return adapter
    ####


def declared_mission_capability_adapter(
    family_id: str,
    mission_id: str,
    fidelity: str,
) -> str | None:
    """Return a discoverable adapter ID without constructing a composition."""

    from .vehicle_composition_registry import load_vehicle_composition_registry

    registry = load_vehicle_composition_registry()
    family = next((item for item in registry.vehicles if item.family_id == family_id), None)
    if family is None:
        return None
    mission = next((item for item in family.mission_templates if item.id == mission_id), None)
    if mission is None or fidelity not in mission.mission_capability_fidelities:
        return None
    return mission.mission_capability_adapter_id
    ####


def estimate_mission_capability(composition: CompiledVehicleComposition) -> MissionCapabilityEstimate | None:
    """Return the exact declared estimate or ``None`` when no family owns it."""

    adapter = resolve_mission_capability_adapter(composition)
    return None if adapter is None else adapter.estimate(composition)
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


def _optional_number(inputs: Any, field: str, default: float) -> float:
    """Read one optional semantic number without manufacturing an input."""

    values = inputs if isinstance(inputs, dict) else inputs.inputs
    return default if field not in values else _number(values, field)
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
    "MissionCapabilityAdapter",
    "MissionCapabilityEstimate",
    "MissionFeasibility",
    "HummingbirdHoverTranslationCapabilityAdapter",
    "HL20GlideEnergyCapabilityAdapter",
    "NescSourceReplayCapabilityAdapter",
    "PassiveTumblingReleaseCapabilityAdapter",
    "LocalDirectWrenchScreenCapabilityAdapter",
    "X15StagedReachabilityCapabilityAdapter",
    "PoweredFixedWingRacetrackCapabilityAdapter",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "declared_mission_capability_adapter",
    "estimate_mission_capability",
    "mission_capability_adapters",
    "resolve_mission_capability_adapter",
]
