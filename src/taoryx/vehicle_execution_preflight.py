"""Fail-closed semantic preflight for composed vehicle missions.

The composition compiler verifies that a caller selected a declared vehicle,
fidelity, initialization contract, and ordered segment sequence.  It cannot
by itself prove that the selected segment values map to an executable native
mission.  This module owns the deliberately narrow next check: compare a
composed semantic mission with the capability-derived geometry accepted by a
shared mission compiler.

It is intentionally *not* a dynamics preflight.  A ``translation_ready``
result says only that a composition has an exact route representation for the
declared translator.  Adapter binding, trim, control, integration, and
independent qualification remain separate gates.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

from .hummingbird_mission_translation import compile_hummingbird_pseudo_mission
from .local_direct_wrench_mission_translation import compile_local_direct_wrench_screen_mission
from .nesc_mission_translation import compile_nesc_source_replay_mission
from .passive_tumbling_mission_translation import compile_passive_tumbling_mission
from .powered_fixed_wing_mission_compiler import (
    CapabilityScaledRacetrack,
    PoweredFixedWingRacetrackIntent,
    compile_powered_fixed_wing_racetrack,
    resolve_powered_fixed_wing_mission_profile,
)
from .racetrack_template import RacetrackFidelity
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition
from .x15_staged_mission_translation import compile_x15_staged_reachability_mission

ExecutionPreflightStatus = Literal["translation_ready", "blocked", "not_applicable"]

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
class ExecutionPreflightCheck:
    """One comparison of a composed semantic value against derived geometry."""

    id: str
    expected: Any
    actual: Any
    unit: str | None
    passed: bool

    def as_dict(self) -> dict[str, Any]:
        """Return stable machine-readable proof for one preflight predicate."""

        return {
            "id": self.id,
            "expected": self.expected,
            "actual": self.actual,
            "unit": self.unit,
            "passed": self.passed,
        }
        ####
    ####


@dataclass(frozen=True, slots=True)
class VehicleExecutionPreflight:
    """Fail-closed result for one semantic-to-native mission translation."""

    composition_id: str
    composition_identity_sha256: str
    vehicle_id: str
    family_id: str
    fidelity: str
    status: ExecutionPreflightStatus
    translator_id: str | None
    checks: tuple[ExecutionPreflightCheck, ...]
    diagnostics: tuple[str, ...]
    derived_mission: dict[str, Any] | None

    def as_dict(self) -> dict[str, Any]:
        """Return the self-contained preflight artifact payload."""

        return {
            "schema": "taoryx.vehicle-execution-preflight/v1alpha1",
            "composition_id": self.composition_id,
            "composition_identity_sha256": self.composition_identity_sha256,
            "vehicle_id": self.vehicle_id,
            "family_id": self.family_id,
            "fidelity": self.fidelity,
            "status": self.status,
            "translator_id": self.translator_id,
            "checks": [check.as_dict() for check in self.checks],
            "diagnostics": list(self.diagnostics),
            "derived_mission": self.derived_mission,
            "claim_boundary": (
                "translation_ready proves only that this semantic composition has an exact lowering through "
                "its declared family translator. It does not by itself bind an adapter, trim a plant, run a "
                "controller, integrate dynamics, or qualify the vehicle."
            ),
        }
        ####
    ####


def preflight_vehicle_composition(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Validate that a composed mission has an exact declared route lowering.

    The first supported vertical slices are the powered-fixed-wing racetrack
    and the Hummingbird pseudo-6DOF hover/yaw/contact translator. Other
    compositions remain explicitly ``not_applicable`` until their family
    adapter supplies a semantic translator. This is more honest than accepting
    a nearby hand-authored route and letting a tool silently substitute its own
    mission.
    """

    if composition.family_id == "hummingbird" and composition.mission == "multirotor_pad_box_yaw_recovery_land_v1":
        try:
            hummingbird_plan = compile_hummingbird_pseudo_mission(composition)
        except (KeyError, TypeError, ValueError) as error:
            return _blocked(composition, (), f"semantic Hummingbird translation is invalid: {error}", None)
        return VehicleExecutionPreflight(
            composition_id=composition.id,
            composition_identity_sha256=composition.identity_sha256,
            vehicle_id=composition.vehicle_id,
            family_id=composition.family_id,
            fidelity=composition.fidelity,
            status="translation_ready",
            translator_id="taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
            checks=(
                ExecutionPreflightCheck(
                    "hummingbird.semantic_plan",
                    "declared_hover_yaw_translation_contact_segments",
                    [segment.instance_id for segment in hummingbird_plan.segments],
                    None,
                    True,
                ),
            ),
            diagnostics=("composition lowers exactly through the declared Hummingbird pseudo-6DOF mission translator",),
            derived_mission=hummingbird_plan.manifest(),
        )
    if composition.family_id == "reference_nesc_two_stage_rocket" and composition.mission == "staged_rocket_launch_target_state_v1":
        try:
            nesc_plan = compile_nesc_source_replay_mission(composition)
        except (KeyError, TypeError, ValueError) as error:
            return _blocked(composition, (), f"semantic NESC source-replay translation is invalid: {error}", None)
        return VehicleExecutionPreflight(
            composition_id=composition.id,
            composition_identity_sha256=composition.identity_sha256,
            vehicle_id=composition.vehicle_id,
            family_id=composition.family_id,
            fidelity=composition.fidelity,
            status="translation_ready",
            translator_id="taoryx.nesc.staged_source_replay.v1",
            checks=(
                ExecutionPreflightCheck(
                    "nesc.semantic_source_replay_plan",
                    "pinned_launch_staging_orbit_replay",
                    [segment.instance_id for segment in nesc_plan.segments],
                    None,
                    True,
                ),
            ),
            diagnostics=(
                "composition exactly matches the pinned NESC source-replay launch, staging, and terminal witness",
            ),
            derived_mission=nesc_plan.manifest(),
        )
    if composition.family_id == "x15" and composition.mission == "x15_staged_booster_reachability_v1":
        try:
            x15_plan = compile_x15_staged_reachability_mission(composition)
        except (KeyError, TypeError, ValueError) as error:
            return _blocked(composition, (), f"semantic X-15 staged translation is invalid: {error}", None)
        return VehicleExecutionPreflight(
            composition_id=composition.id,
            composition_identity_sha256=composition.identity_sha256,
            vehicle_id=composition.vehicle_id,
            family_id=composition.family_id,
            fidelity=composition.fidelity,
            status="translation_ready",
            translator_id="taoryx.x15.staged_reachability.v1",
            checks=(
                ExecutionPreflightCheck(
                    "x15.semantic_staged_reachability_plan",
                    "source_pinned_booster_coast_release_open_loop_glide_impact",
                    [segment.instance_id for segment in x15_plan.segments],
                    None,
                    True,
                ),
            ),
            diagnostics=(
                "composition exactly matches the retained X-15-scaled local source-staging witness; "
                "it remains separate from the synthetic California-to-Hawaii showcase",
            ),
            derived_mission=x15_plan.manifest(),
        )
    if composition.family_id == "x15" and composition.mission == "x15_local_direct_wrench_screen_v1":
        try:
            local_plan = compile_local_direct_wrench_screen_mission(
                composition,
                family_id="x15",
                mission_id="x15_local_direct_wrench_screen_v1",
                initialization_id="source_release_glide_local_point",
                screen_config_id="x15-source-release-glide-local-direct-wrench-v1",
            )
        except (KeyError, TypeError, ValueError) as error:
            return _blocked(composition, (), f"semantic X-15 local direct-wrench translation is invalid: {error}", None)
        return VehicleExecutionPreflight(
            composition_id=composition.id,
            composition_identity_sha256=composition.identity_sha256,
            vehicle_id=composition.vehicle_id,
            family_id=composition.family_id,
            fidelity=composition.fidelity,
            status="translation_ready",
            translator_id="taoryx.local_direct_wrench_screen.v1",
            checks=(
                ExecutionPreflightCheck(
                    "x15.semantic_local_direct_wrench_screen",
                    "pinned_source_release_glide_local_recovery_screen",
                    [segment.instance_id for segment in composition.segments],
                    None,
                    True,
                ),
            ),
            diagnostics=(
                "composition lowers exactly to the pinned source-local X-15 direct-wrench recovery screen; "
                "it is not a release-to-handoff mission translator",
            ),
            derived_mission=local_plan.manifest(),
        )
    if composition.family_id == "tumbling_body" and composition.mission == "tumbling_body_release_damping_impact_v1":
        try:
            tumbling_plan = compile_passive_tumbling_mission(composition)
        except (KeyError, TypeError, ValueError) as error:
            return _blocked(composition, (), f"semantic passive tumbling translation is invalid: {error}", None)
        return VehicleExecutionPreflight(
            composition_id=composition.id,
            composition_identity_sha256=composition.identity_sha256,
            vehicle_id=composition.vehicle_id,
            family_id=composition.family_id,
            fidelity=composition.fidelity,
            status="translation_ready",
            translator_id="taoryx.passive_tumbling.direct_release.v1",
            checks=(
                ExecutionPreflightCheck(
                    "tumbling_body.semantic_direct_release_plan",
                    "canonical_passive_cylinder_release_area_policy_impact",
                    [segment.instance_id for segment in tumbling_plan.segments],
                    None,
                    True,
                ),
            ),
            diagnostics=(
                "composition exactly matches the declared direct passive-cylinder release witness; "
                "it exposes no control, wrench, or allocation path",
            ),
            derived_mission=tumbling_plan.manifest(),
        )
    if composition.family_id not in _POWERED_FIXED_WING_PROFILE_BY_FAMILY or composition.mission != "powered_fixed_wing_racetrack_v1":
        return _not_applicable(
            composition,
            "no semantic execution preflight is registered for this family and mission yet",
        )
    if composition.fidelity not in _RACETRACK_FIDELITY_BY_TIER:
        return _not_applicable(composition, f"no racetrack realization is registered for tier {composition.fidelity!r}")

    try:
        return _preflight_powered_fixed_wing_racetrack(composition)
    except (KeyError, TypeError, ValueError) as error:
        return _blocked(composition, (), f"semantic racetrack translation is invalid: {error}", None)
    ####


def _preflight_powered_fixed_wing_racetrack(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Compare a composed fixed-wing racetrack to its selected capability profile."""

    compiled = compile_powered_fixed_wing_racetrack_from_composition(composition)
    route = compiled.route
    initialization = composition.initialization.inputs
    segments = {segment.instance_id: segment for segment in composition.segments}
    climb = _segment(composition, "climb_level_gate")
    descent = _segment(composition, "descent_level_gate")
    left_turn = segments["left-turn"]
    right_turn = segments["right-turn"]
    terminal = _segment(composition, "terminal_state_gate")
    gates = {gate.id: gate for gate in route.gates}
    expected_initial_heading_deg = 90.0
    expected_left_bank_deg = abs(route.left_turn_bank_deg)
    expected_right_bank_deg = abs(route.right_turn_bank_deg)

    checks = (
        _scalar_check("initialization.altitude", route.low_altitude_m, _number(initialization, "altitude_m"), "m"),
        _scalar_check("initialization.speed", route.speed_m_s, _number(initialization, "speed_m_s"), "m/s"),
        _scalar_check("initialization.heading", expected_initial_heading_deg, _number(initialization, "heading_deg"), "deg"),
        _scalar_check("climb.target_altitude", route.high_altitude_m, _number(climb, "target_altitude_m"), "m"),
        _scalar_check("climb.rate", route.climb_rate_m_s, _number(climb, "climb_rate_m_s"), "m/s"),
        _vector_check(
            "climb.high_level_gate",
            _gate_ned(gates["high-altitude-level-gate"]),
            _ned(climb, "gate_center_ned_m"),
            "m",
        ),
        _scalar_check("left_turn.radius", route.turn_radius_m, _number(left_turn, "turn_radius_m"), "m"),
        _scalar_check("left_turn.bank_limit", expected_left_bank_deg, _number(left_turn, "bank_limit_deg"), "deg"),
        _categorical_check("left_turn.direction", "left", _text(left_turn, "turn_direction")),
        _vector_check(
            "left_turn.exit_gate",
            _gate_ned(gates["left-turn-exit-gate"]),
            _ned(left_turn, "gate_center_ned_m"),
            "m",
        ),
        _scalar_check("descent.target_altitude", route.low_altitude_m, _number(descent, "target_altitude_m"), "m"),
        _scalar_check("descent.rate", route.descent_rate_m_s, _number(descent, "descent_rate_m_s"), "m/s"),
        _vector_check(
            "descent.low_level_gate",
            _gate_ned(gates["low-altitude-level-gate"]),
            _ned(descent, "gate_center_ned_m"),
            "m",
        ),
        _scalar_check("right_turn.radius", route.turn_radius_m, _number(right_turn, "turn_radius_m"), "m"),
        _scalar_check("right_turn.bank_limit", expected_right_bank_deg, _number(right_turn, "bank_limit_deg"), "deg"),
        _categorical_check("right_turn.direction", "right", _text(right_turn, "turn_direction")),
        _vector_check(
            "right_turn.exit_gate",
            _gate_ned(gates["terminal-start-finish-gate"]),
            _ned(right_turn, "gate_center_ned_m"),
            "m",
        ),
        _vector_check(
            "terminal.position",
            _gate_ned(gates["terminal-start-finish-gate"]),
            _ned(terminal, "target_ned_m"),
            "m",
        ),
        _scalar_check("terminal.altitude", route.low_altitude_m, _number(terminal, "target_altitude_m"), "m"),
        _scalar_check("terminal.speed", route.speed_m_s, _number(terminal, "target_speed_m_s"), "m/s"),
        _scalar_check("terminal.heading", expected_initial_heading_deg, _number(terminal, "target_heading_deg"), "deg"),
    )
    failed = tuple(check.id for check in checks if not check.passed)
    diagnostics = list(compiled.diagnostics)
    if failed:
        diagnostics.append("semantic values do not match the derived racetrack translator geometry: " + ", ".join(failed))
    else:
        diagnostics.append(
            f"composition exactly matches the capability-derived {composition.family_id} racetrack geometry"
        )
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="translation_ready" if not failed else "blocked",
        translator_id="taoryx.powered_fixed_wing_racetrack.capability_scaled.v1",
        checks=checks,
        diagnostics=tuple(diagnostics),
        derived_mission=compiled.manifest(),
    )
    ####


def compile_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compile one registered fixed-wing semantic-to-racetrack translation.

    This function is shared by preflight and materialization so a later
    runtime cannot use a different geometry than the one independently
    accepted by the preflight report.
    """

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

    intent = PoweredFixedWingRacetrackIntent(
        id=profile_intent.id,
        low_altitude_m=_number(descent, "target_altitude_m"),
        high_altitude_m=_number(climb, "target_altitude_m"),
        requested_speed_m_s=_number(initialization, "speed_m_s"),
        requested_bank_deg=_number(left_turn, "bank_limit_deg"),
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


def compile_x8_racetrack_from_composition(composition: CompiledVehicleComposition) -> CapabilityScaledRacetrack:
    """Compatibility alias for the original X8-only public helper.

    New callers must use :func:`compile_powered_fixed_wing_racetrack_from_composition`
    so the shared fixed-wing translation is explicit.
    """

    if composition.family_id != "skywalker_x8":
        raise ValueError("X8 compatibility helper requires the Skywalker X8 family")
    return compile_powered_fixed_wing_racetrack_from_composition(composition)
    ####


def _not_applicable(composition: CompiledVehicleComposition, diagnostic: str) -> VehicleExecutionPreflight:
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="not_applicable",
        translator_id=None,
        checks=(),
        diagnostics=(diagnostic,),
        derived_mission=None,
    )
    ####


def _blocked(
    composition: CompiledVehicleComposition,
    checks: tuple[ExecutionPreflightCheck, ...],
    diagnostic: str,
    derived_mission: dict[str, Any] | None,
) -> VehicleExecutionPreflight:
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="blocked",
        translator_id="taoryx.powered_fixed_wing_racetrack.capability_scaled.v1",
        checks=checks,
        diagnostics=(diagnostic,),
        derived_mission=derived_mission,
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


def _text(segment: CompiledSegment, field: str) -> str:
    value = segment.inputs[field].value
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


def _gate_ned(gate: Any) -> tuple[float, float, float]:
    return (float(gate.north_m), float(gate.east_m), -float(gate.altitude_m))
    ####


def _scalar_check(identifier: str, expected: float, actual: float, unit: str) -> ExecutionPreflightCheck:
    return ExecutionPreflightCheck(identifier, expected, actual, unit, math.isclose(expected, actual, abs_tol=1e-6))
    ####


def _vector_check(
    identifier: str,
    expected: tuple[float, float, float],
    actual: tuple[float, float, float],
    unit: str,
) -> ExecutionPreflightCheck:
    return ExecutionPreflightCheck(
        identifier,
        list(expected),
        list(actual),
        unit,
        all(math.isclose(left, right, abs_tol=1e-6) for left, right in zip(expected, actual, strict=True)),
    )
    ####


def _categorical_check(identifier: str, expected: str, actual: str) -> ExecutionPreflightCheck:
    return ExecutionPreflightCheck(identifier, expected, actual, None, expected == actual)
    ####


__all__ = [
    "ExecutionPreflightCheck",
    "ExecutionPreflightStatus",
    "VehicleExecutionPreflight",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "compile_x8_racetrack_from_composition",
    "preflight_vehicle_composition",
]
