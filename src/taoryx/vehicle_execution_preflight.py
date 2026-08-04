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

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Any, Callable, Literal

from .hl20_glide_energy_mission_translation import compile_hl20_glide_energy_mission
from .hl20_source_release_mission_translation import compile_hl20_source_booster_release_mission
from .hummingbird_mission_translation import compile_hummingbird_pseudo_mission
from .local_direct_wrench_mission_translation import compile_local_direct_wrench_screen_mission
from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_definition
from .mission_capability import (
    MissionCapabilityEstimate,
    estimate_mission_capability,
    mission_capability_adapters,
    resolve_mission_capability_adapter,
)
from .mission_capability import (
    compile_powered_fixed_wing_racetrack_from_composition as _compile_powered_fixed_wing_racetrack_from_composition,
)
from .nesc_mission_translation import compile_nesc_source_replay_mission
from .passive_tumbling_mission_translation import compile_passive_tumbling_mission
from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition
from .vehicle_composition_registry import (
    ResolvedVehicleCompositionCatalog,
    load_resolved_vehicle_composition_catalog,
    mission_graph_execution_contract,
    mission_semantic_translator_id,
)
from .x15_staged_mission_translation import compile_x15_staged_reachability_mission

ExecutionPreflightStatus = Literal["translation_ready", "blocked", "not_applicable"]


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
    capability_estimate: dict[str, object] | None = None

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
            "capability_estimate": self.capability_estimate,
            "claim_boundary": _preflight_claim_boundary(self.status, self.capability_estimate),
        }
        ####

    ####


TranslationPreflightHandler = Callable[[CompiledVehicleComposition], VehicleExecutionPreflight]


def _preflight_claim_boundary(
    status: ExecutionPreflightStatus,
    capability_estimate: dict[str, object] | None,
) -> str:
    """State exactly what a preflight disposition proves.

    A capability-only blocked result is useful planning evidence, but its
    output must not borrow the ``translation_ready`` claim language.
    """

    if status == "translation_ready":
        return (
            "translation_ready proves only that this semantic composition has an exact lowering through its "
            "declared family translator. It does not by itself bind an adapter, trim a plant, run a controller, "
            "integrate dynamics, or qualify the vehicle."
        )
    if status == "blocked" and capability_estimate is not None:
        return (
            "blocked capability preflight exposes a family-owned planning estimate only. No native lowering, "
            "adapter binding, trim, controller, integration, truth-objective result, or qualification is available."
        )
    if status == "blocked":
        return (
            "blocked preflight establishes that the selected semantic composition cannot yet be lowered through a "
            "declared native execution path. It establishes no dynamics or qualification result."
        )
    return (
        "not_applicable preflight has no declared family-owned semantic execution path for this selection and "
        "establishes no dynamics or qualification result."
    )
    ####


@dataclass(frozen=True, slots=True)
class SemanticPreflightHandler:
    """One family-owned semantic translator preflight implementation."""

    translator_id: str
    handler: TranslationPreflightHandler

    def __post_init__(self) -> None:
        if not self.translator_id.strip():
            raise ValueError("semantic preflight handler requires a nonempty translator_id")
        ####

    ####


@dataclass(frozen=True, slots=True)
class SemanticPreflightHandlerRegistry:
    """Validated extension surface for family-owned semantic preflight handlers."""

    handlers: tuple[SemanticPreflightHandler, ...]

    def __post_init__(self) -> None:
        identifiers = tuple(item.translator_id for item in self.handlers)
        if len(identifiers) != len(set(identifiers)):
            raise ValueError("semantic preflight handler registry contains duplicate translator IDs")
        ####

    def resolve(self, translator_id: str) -> TranslationPreflightHandler | None:
        """Resolve one exact handler without a family or topology fallback."""

        match = next((item for item in self.handlers if item.translator_id == translator_id), None)
        return None if match is None else match.handler
        ####

    ####


def _preflight_vehicle_composition_unchecked(
    composition: CompiledVehicleComposition,
    *,
    handler_registry: SemanticPreflightHandlerRegistry | None = None,
) -> VehicleExecutionPreflight:
    """Dispatch only to the mission template's explicitly declared translator.

    Source-specific semantic checks remain in their family handlers.  This
    composition-level selector deliberately knows no family name, segment
    topology, or plant implementation beyond the registry declaration and the
    common graph rules.
    """

    graph_contract = mission_graph_execution_contract(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    supports_family_graph_extension = graph_contract["status"] == "family_extension_declared"
    if (
        composition.mission_graph is not None
        and composition.mission_graph.status not in {"linear_sequence_only", "authored_linear_sequence_lowered"}
        and not supports_family_graph_extension
    ):
        return _blocked(
            composition,
            (),
            "caller-authored mission graph is semantically valid but no selected native translator declares graph execution support",
            None,
        )

    declared_translator_id = mission_semantic_translator_id(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    capability_adapter = resolve_mission_capability_adapter(composition)
    if capability_adapter is None:
        diagnostic = (
            "mission template declares semantic translation but no tier-compatible capability adapter is installed"
            if declared_translator_id is not None
            else "no semantic execution preflight is registered for this family, mission, and fidelity yet"
        )
        return _not_applicable(composition, diagnostic)
    if declared_translator_id is None:
        estimate = estimate_mission_capability(composition)
        return _blocked(
            composition,
            (),
            "mission template has a family-owned capability estimate but no tier-compatible semantic_translator_id; "
            "native lowering and execution remain blocked",
            None if estimate is None else estimate.manifest,
            None if estimate is None else _capability_estimate_evidence(composition, estimate),
        )
    registry = handler_registry or semantic_preflight_handler_registry()
    handler = registry.resolve(declared_translator_id)
    if handler is None:
        return _blocked(
            composition,
            (),
            f"no installed semantic preflight handler is registered for {declared_translator_id!r}",
            None,
        )
    try:
        return handler(composition)
    except (KeyError, TypeError, ValueError) as error:
        return _blocked(
            composition,
            (),
            f"semantic translation through {declared_translator_id!r} is invalid: {error}",
            None,
        )
    ####


def preflight_vehicle_composition(
    composition: CompiledVehicleComposition,
    *,
    handler_registry: SemanticPreflightHandlerRegistry | None = None,
) -> VehicleExecutionPreflight:
    """Run semantic preflight and enforce the registry-declared translator.

    Individual family translators still own their source-specific plan checks,
    but readiness is not permitted to depend on an undisclosed code path.  A
    ``translation_ready`` result must name the exact translator declared by the
    selected mission template; a mismatch is a fail-closed integration error.
    """

    result = _preflight_vehicle_composition_unchecked(composition, handler_registry=handler_registry)
    if result.status != "translation_ready":
        return result
    declared_translator_id = mission_semantic_translator_id(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    if declared_translator_id is None:
        return _blocked(
            composition,
            result.checks,
            "semantic preflight reached translation_ready without a registry-declared semantic_translator_id",
            result.derived_mission,
        )
    if result.translator_id != declared_translator_id:
        return _blocked(
            composition,
            result.checks,
            "semantic preflight translator does not match the selected mission template: "
            f"declared {declared_translator_id!r}, observed {result.translator_id!r}",
            result.derived_mission,
        )
    return result
    ####


def _capability_estimate_and_manifest(
    composition: CompiledVehicleComposition,
) -> tuple[MissionCapabilityEstimate, dict[str, object]]:
    """Return the exact declared estimate and its family-owned capability map."""

    estimate = estimate_mission_capability(composition)
    if estimate is None:
        raise ValueError("declared semantic translator has no tier-compatible capability adapter")
    if (
        estimate.family_id != composition.family_id
        or estimate.mission_id != composition.mission
        or estimate.fidelity != composition.fidelity
    ):
        raise ValueError("capability adapter returned an estimate for a different composition selection")
    capability = estimate.manifest.get("capability")
    if not isinstance(capability, dict):
        raise ValueError("capability adapter returned no capability manifest")
    return estimate, capability
    ####


def _capability_estimate_evidence(
    composition: CompiledVehicleComposition,
    estimate: MissionCapabilityEstimate,
) -> dict[str, object]:
    """Return a fingerprinted capability projection for a concrete preflight.

    The full derived mission remains a sibling preflight field because native
    lowerers already consume that representation.  This compact projection
    lets discovery and endpoint witnesses prove which family-owned estimate
    produced it, its feasibility disposition, and that it belongs to this
    exact immutable composition.
    """

    try:
        encoded_manifest = json.dumps(
            estimate.manifest,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ValueError(f"capability adapter manifest is not fingerprintable: {error}") from error
    return {
        "schema": "taoryx.concrete-capability-preflight/v1alpha1",
        "composition_id": composition.id,
        "composition_identity_sha256": composition.identity_sha256,
        "semantic_translator_id": mission_semantic_translator_id(
            composition.family_id,
            composition.mission,
            composition.fidelity,
        ),
        "adapter_id": estimate.adapter_id,
        "family_id": estimate.family_id,
        "mission_id": estimate.mission_id,
        "fidelity": estimate.fidelity,
        "feasibility": estimate.feasibility,
        "diagnostics": list(estimate.diagnostics),
        "derived_mission_sha256": hashlib.sha256(encoded_manifest).hexdigest(),
        "claim_boundary": (
            "This fingerprinted family-owned capability estimate establishes only semantic mission feasibility "
            "for the selected composition. It does not establish native execution, control realization, "
            "integration, truth-objective success, or qualification."
        ),
    }
    ####


def _capability_number(capability: dict[str, object], key: str) -> float:
    """Read one finite numeric capability field without coercing booleans."""

    value = capability.get(key)
    if not isinstance(value, int | float) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError(f"capability field {key!r} must be a finite numeric value")
    return float(value)
    ####


def _preflight_hummingbird_hover_yaw(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the declared aggregate-thrust Hummingbird mission translator."""

    plan = compile_hummingbird_pseudo_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    thrust_margin_n = _capability_number(capability, "thrust_margin_n")
    capability_ok = estimate.feasibility != "certainly_infeasible"
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if capability_ok else "blocked",
        "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
        (
            ExecutionPreflightCheck(
                "hummingbird.semantic_plan",
                "declared_hover_yaw_translation_contact_segments",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "hummingbird.aggregate_hover_thrust_margin",
                0.0,
                thrust_margin_n,
                "N",
                capability_ok,
            ),
        ),
        (
            "composition lowers exactly through the declared Hummingbird pseudo-6DOF mission translator",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_nesc_source_replay(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the source-pinned NESC replay chronology."""

    plan = compile_nesc_source_replay_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    event_order_valid = bool(capability["stage_event_order_valid"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if event_order_valid else "blocked",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                "nesc.semantic_source_replay_plan",
                "pinned_launch_staging_orbit_replay",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("nesc.source_stage_event_order", True, event_order_valid, None, event_order_valid),
        ),
        (
            "composition exactly matches the pinned NESC source-replay launch, staging, and terminal witness",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_hl20_source_booster_release_replay(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the exact source-aerodynamic HL-20 release replay."""

    plan = compile_hl20_source_booster_release_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    ordered = bool(capability["source_schedule_order_valid"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if ordered else "blocked",
        "taoryx.hl20_source_booster_release_replay.v1",
        (
            ExecutionPreflightCheck(
                "hl20.source_scheduled_release_plan",
                "pinned_booster_release_opposing_bank_ground_contact_replay",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("hl20.source_schedule_order", True, ordered, None, ordered),
        ),
        (
            "composition exactly matches the retained HL-20 source-aerodynamic/synthetic-booster scheduled witness",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_hl20_glide_energy_intent(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Translate the public HL-20 glide intent without claiming a native runner."""

    plan = compile_hl20_glide_energy_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    energy_margin_j_kg = _capability_number(capability, "available_specific_energy_margin_j_kg")
    opposing_bank_intent = capability.get("opposing_bank_intent") is True
    finite_bank_geometry = capability.get("finite_bank_geometry") is True
    translatable = estimate.feasibility != "certainly_infeasible" and opposing_bank_intent and finite_bank_geometry
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if translatable else "blocked",
        "taoryx.hl20_glide_energy_intent.v1",
        (
            ExecutionPreflightCheck(
                "hl20.glide_energy_semantic_plan",
                "release_trim_opposing_bank_energy_handoff",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "hl20.unpowered_specific_energy_margin",
                0.0,
                energy_margin_j_kg,
                "J/kg",
                energy_margin_j_kg >= 0.0,
            ),
            ExecutionPreflightCheck(
                "hl20.opposing_finite_bank_intent",
                True,
                opposing_bank_intent and finite_bank_geometry,
                None,
                opposing_bank_intent and finite_bank_geometry,
            ),
        ),
        (
            "composition lowers exactly to the declared public HL-20 release/trim/opposing-bank/energy-handoff intent plan",
            "no source-owned HL-20 reduced-fidelity runtime factory is declared; runtime lowering remains blocked",
            *estimate.diagnostics,
        ),
        plan.manifest(),
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_x15_staged_reachability(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the retained X-15-scaled booster/release witness."""

    plan = compile_x15_staged_reachability_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    chronology_ok = bool(capability["staging_and_horizon_order_valid"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if chronology_ok else "blocked",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                "x15.semantic_staged_reachability_plan",
                "source_pinned_booster_coast_release_open_loop_glide_impact",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck("x15.staging_and_horizon_order", True, chronology_ok, None, chronology_ok),
        ),
        (
            "composition exactly matches the retained X-15-scaled local source-staging witness; "
            "it remains separate from the synthetic California-to-Hawaii showcase",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_local_direct_wrench(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight one source-local direct-wrench screen without effector claims."""

    definition = resolve_local_direct_wrench_screen_definition(composition)
    if definition is None:
        raise ValueError("no installed local direct-wrench screen matches this composition")
    config = definition.config_factory()
    compile_local_direct_wrench_screen_mission(
        composition,
        family_id=definition.family_id,
        mission_id=definition.mission_id,
        initialization_id=definition.initialization_id,
        screen_config_id=config.id,
    )
    estimate, capability = _capability_estimate_and_manifest(composition)
    limits = capability.get("direct_wrench_limits")
    if not isinstance(limits, dict):
        raise ValueError(f"{definition.family_id} local direct-wrench capability adapter returned no authority limits")
    authority_span = limits.get("authority_span")
    if not isinstance(authority_span, dict) or any(float(value) <= 0.0 for value in authority_span.values()):
        raise ValueError(f"{definition.family_id} local direct-wrench authority bounds are invalid")
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                f"{definition.family_id}.semantic_local_direct_wrench_screen",
                f"pinned_{definition.initialization_id}_local_recovery_screen",
                [segment.instance_id for segment in composition.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                f"{definition.family_id}.direct_wrench_authority_bounds",
                "positive finite span on every canonical wrench axis",
                authority_span,
                "N or N m by axis",
                True,
            ),
        ),
        (
            f"composition lowers exactly to the pinned source-local {definition.family_id} direct-wrench recovery screen; "
            "it is not a route or physical-effector mission translator",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_passive_tumbling_release(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight the explicitly uncontrolled direct passive-cylinder release."""

    plan = compile_passive_tumbling_mission(composition)
    estimate, capability = _capability_estimate_and_manifest(composition)
    horizon_ok = bool(capability["horizon_contains_vacuum_fall_lower_bound"])
    return VehicleExecutionPreflight(
        composition.id,
        composition.identity_sha256,
        composition.vehicle_id,
        composition.family_id,
        composition.fidelity,
        "translation_ready" if horizon_ok else "blocked",
        estimate.adapter_id,
        (
            ExecutionPreflightCheck(
                "tumbling_body.semantic_direct_release_plan",
                "canonical_passive_cylinder_release_area_policy_impact",
                [segment.instance_id for segment in plan.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                "tumbling_body.release_horizon",
                _capability_number(capability, "vacuum_fall_time_lower_bound_s"),
                _capability_number(capability, "simulation_horizon_s"),
                "s",
                horizon_ok,
            ),
        ),
        (
            "composition exactly matches the declared direct passive-cylinder release witness; it exposes no control, wrench, or allocation path",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def _preflight_powered_fixed_wing_racetrack(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Compare a composed fixed-wing racetrack to its selected capability profile."""

    compiled = compile_powered_fixed_wing_racetrack_from_composition(composition)
    estimate, _capability = _capability_estimate_and_manifest(composition)
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
        diagnostics.append(f"composition exactly matches the capability-derived {composition.family_id} racetrack geometry")
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
        derived_mission=estimate.manifest,
        capability_estimate=_capability_estimate_evidence(composition, estimate),
    )
    ####


_DEFAULT_SEMANTIC_PREFLIGHT_HANDLER_REGISTRY = SemanticPreflightHandlerRegistry(
    (
        SemanticPreflightHandler(
            "taoryx.powered_fixed_wing_racetrack.capability_scaled.v1",
            _preflight_powered_fixed_wing_racetrack,
        ),
        SemanticPreflightHandler(
            "taoryx.hummingbird.hover_yaw_contact.pseudo6dof.v1",
            _preflight_hummingbird_hover_yaw,
        ),
        SemanticPreflightHandler(
            "taoryx.nesc_staged_source_replay.capability.v1",
            _preflight_nesc_source_replay,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_source_booster_release_replay.v1",
            _preflight_hl20_source_booster_release_replay,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_glide_energy_intent.v1",
            _preflight_hl20_glide_energy_intent,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_staged_reachability.capability.v1",
            _preflight_x15_staged_reachability,
        ),
        SemanticPreflightHandler(
            "taoryx.x15_local_direct_wrench_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.hl20_local_direct_wrench_screen.capability.v1",
            _preflight_local_direct_wrench,
        ),
        SemanticPreflightHandler(
            "taoryx.passive_tumbling_release.capability.v1",
            _preflight_passive_tumbling_release,
        ),
    )
)


def semantic_preflight_handler_registry() -> SemanticPreflightHandlerRegistry:
    """Return the immutable registry of installed source-owned translators."""

    return _DEFAULT_SEMANTIC_PREFLIGHT_HANDLER_REGISTRY
    ####


def build_semantic_preflight_handler_report(
    catalog: ResolvedVehicleCompositionCatalog | None = None,
    *,
    handler_registry: SemanticPreflightHandlerRegistry | None = None,
    capability_adapter_ids: tuple[str, ...] | None = None,
) -> dict[str, object]:
    """Audit declared capability/translator identities against installed code.

    This is intentionally a structural catalog audit. It proves that a tier
    which declares a capability adapter or executable semantic translator
    names installed code. It does not compile a composition, prove an adapter
    supports its declared family, resolve a capability estimate, or establish
    that a particular mission is executable.
    """

    selected_catalog = catalog or load_resolved_vehicle_composition_catalog()
    selected_handlers = handler_registry or semantic_preflight_handler_registry()
    installed_capability_adapter_ids = (
        capability_adapter_ids if capability_adapter_ids is not None else tuple(adapter.id for adapter in mission_capability_adapters())
    )
    if len(installed_capability_adapter_ids) != len(set(installed_capability_adapter_ids)):
        raise ValueError("mission capability adapter registry contains duplicate adapter IDs")
    installed_capability_adapter_id_set = set(installed_capability_adapter_ids)
    records: list[dict[str, object]] = []
    errors: list[str] = []
    status_counts: dict[str, int] = {
        "not_declared": 0,
        "translator_pending": 0,
        "missing_capability_adapter": 0,
        "registered": 0,
        "invalid_declaration": 0,
        "missing_handler": 0,
    }
    for vehicle in selected_catalog.vehicles:
        for mission in vehicle.declaration.mission_templates:
            for fidelity in mission.compatible_fidelities:
                capability_adapter_id = mission.mission_capability_adapter_id if fidelity in mission.mission_capability_fidelities else None
                translator_id = mission.semantic_translator_id if fidelity in mission.semantic_translator_fidelities else None
                capability_adapter_registered = capability_adapter_id in installed_capability_adapter_id_set
                handler_registered = selected_handlers.resolve(translator_id) is not None if isinstance(translator_id, str) else False
                if capability_adapter_id is not None and not capability_adapter_registered:
                    status = "missing_capability_adapter"
                    next_step = "Install the declared mission capability adapter before selecting this tier."
                    errors.append(
                        f"{vehicle.family.family_id}/{mission.id}/{fidelity} declares capability adapter {capability_adapter_id!r} without an installed adapter"
                    )
                elif translator_id is None:
                    status = "translator_pending" if capability_adapter_id is not None else "not_declared"
                    next_step = (
                        "Declare a tier-scoped semantic translator and install its handler before claiming translation readiness."
                        if capability_adapter_id is not None
                        else "No native semantic translation is declared for this tier."
                    )
                elif capability_adapter_id is None:
                    status = "invalid_declaration"
                    next_step = "Declare the matching tier-scoped capability adapter before installing a translator."
                    errors.append(
                        f"{vehicle.family.family_id}/{mission.id}/{fidelity} declares semantic translator {translator_id!r} without a capability adapter"
                    )
                elif not handler_registered:
                    status = "missing_handler"
                    next_step = "Install and register the declared semantic preflight handler."
                    errors.append(
                        f"{vehicle.family.family_id}/{mission.id}/{fidelity} declares semantic translator {translator_id!r} without an installed handler"
                    )
                else:
                    status = "registered"
                    next_step = "Compile a concrete composition to exercise this installed translator."
                status_counts[status] += 1
                records.append(
                    {
                        "family_id": vehicle.family.family_id,
                        "vehicle_id": vehicle.family.family.vehicle_registry_id or vehicle.family.family_id,
                        "mission_id": mission.id,
                        "fidelity": fidelity,
                        "mission_capability_adapter_id": capability_adapter_id,
                        "capability_adapter_registered": capability_adapter_registered,
                        "semantic_translator_id": translator_id,
                        "handler_registered": handler_registered,
                        "status": status,
                        "next_step": next_step,
                    }
                )
    return {
        "schema": "taoryx.semantic-preflight-handler-report/v1alpha1",
        "status": "pass" if not errors else "fail",
        "installed_handler_count": len(selected_handlers.handlers),
        "installed_capability_adapter_count": len(installed_capability_adapter_ids),
        "declared_capability_adapter_count": sum(1 for record in records if record["mission_capability_adapter_id"] is not None),
        "declared_translator_count": sum(1 for record in records if record["semantic_translator_id"] is not None),
        "status_counts": dict(sorted(status_counts.items())),
        "error_count": len(errors),
        "errors": errors,
        "records": records,
        "claim_boundary": (
            "This report proves only that catalog-declared capability adapters and semantic translators have "
            "an installed identity. It does not prove an adapter supports the selected composition, capability "
            "feasibility, native compilation, adapter binding, control, integration, or vehicle qualification."
        ),
    }
    ####


def compile_powered_fixed_wing_racetrack_from_composition(
    composition: CompiledVehicleComposition,
) -> CapabilityScaledRacetrack:
    """Compatibility projection of the registered fixed-wing capability planner."""

    return _compile_powered_fixed_wing_racetrack_from_composition(composition)
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
    capability_estimate: dict[str, object] | None = None,
) -> VehicleExecutionPreflight:
    declared_translator_id = mission_semantic_translator_id(
        composition.family_id,
        composition.mission,
        composition.fidelity,
    )
    return VehicleExecutionPreflight(
        composition_id=composition.id,
        composition_identity_sha256=composition.identity_sha256,
        vehicle_id=composition.vehicle_id,
        family_id=composition.family_id,
        fidelity=composition.fidelity,
        status="blocked",
        translator_id=declared_translator_id,
        checks=checks,
        diagnostics=(diagnostic,),
        derived_mission=derived_mission,
        capability_estimate=capability_estimate,
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
    "SemanticPreflightHandler",
    "SemanticPreflightHandlerRegistry",
    "VehicleExecutionPreflight",
    "build_semantic_preflight_handler_report",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "compile_x8_racetrack_from_composition",
    "preflight_vehicle_composition",
    "semantic_preflight_handler_registry",
]
