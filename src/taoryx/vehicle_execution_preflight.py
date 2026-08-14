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
from collections.abc import Iterator, Mapping
from contextlib import nullcontext
from dataclasses import dataclass
from typing import Any, Callable, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .local_direct_wrench_mission_translation import compile_local_direct_wrench_screen_mission
from .local_direct_wrench_screen_registry import resolve_local_direct_wrench_screen_definition
from .local_native_coordinate_lqi_mission_translation import compile_local_native_coordinate_lqi_screen_mission
from .local_native_coordinate_lqi_screen_registry import resolve_local_native_coordinate_lqi_screen_definition
from .mission_capability import (
    MissionCapabilityEstimate,
    estimate_mission_capability,
    mission_capability_adapters,
    resolve_mission_capability_adapter,
)
from .mission_capability import (
    compile_powered_fixed_wing_racetrack_from_composition as _compile_powered_fixed_wing_racetrack_from_composition,
)
from .plugins import DeferredSemanticPreflightHandler, PluginCatalog, current_plugin_catalog, discover_plugins, plugin_catalog_scope
from .powered_fixed_wing_mission_compiler import CapabilityScaledRacetrack
from .vehicle_composition import CompiledSegment, CompiledVehicleComposition
from .vehicle_composition_registry import (
    ResolvedVehicleCompositionCatalog,
    load_resolved_vehicle_composition_catalog,
    mission_graph_execution_contract,
    mission_semantic_translator_id,
)

ExecutionPreflightStatus = Literal["translation_ready", "blocked", "not_applicable"]

_CAPABILITY_ADVERTISEMENT_SCHEMA = "taoryx.vehicle-capability-advertisement/v1alpha1"
_CONCRETE_CAPABILITY_PREFLIGHT_SCHEMA = "taoryx.concrete-capability-preflight/v1alpha1"


class _PortableMappingModel(BaseModel, Mapping[str, object]):
    """Pydantic contract that remains readable through legacy mapping callers."""

    def as_dict(self) -> dict[str, object]:
        """Return the portable JSON-facing view of the validated contract."""

        return self.model_dump(mode="json", by_alias=True)
        ####

    def __getitem__(self, key: str) -> object:
        return self.as_dict()[key]
        ####

    def __iter__(self) -> Iterator[str]:  # type: ignore[override]
        return iter(self.as_dict())
        ####

    def __len__(self) -> int:
        return len(self.as_dict())
        ####

    ####


class CapabilitySelection(_PortableMappingModel):
    """Exact immutable composition selection advertised by a capability adapter."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    composition_id: str = Field(min_length=1)
    composition_identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    vehicle_id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    mission_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    ####


class CapabilityInterfaceAdvertisement(_PortableMappingModel):
    """Typed identity fields for the rich generic interface projection."""

    model_config = ConfigDict(frozen=True, extra="allow")

    interface_id: str = Field(min_length=1)
    fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    ####

    @model_validator(mode="after")
    def verify_interface_fingerprint(self) -> CapabilityInterfaceAdvertisement:
        payload = self.as_dict()
        observed = payload.pop("fingerprint_sha256")
        payload.pop("interface_id", None)
        if observed != _mapping_fingerprint(payload):
            raise ValueError("interface fingerprint is invalid")
        return self
        ####

    ####


class CapabilityAdvertisement(_PortableMappingModel):
    """Common generic/family-owned capability advertisement boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.vehicle-capability-advertisement/v1alpha1"] = Field(alias="schema")
    selection: CapabilitySelection
    interface: CapabilityInterfaceAdvertisement
    family_owned: Mapping[str, Any]
    claim_boundary: str = Field(min_length=1)
    fingerprint_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    ####

    @model_validator(mode="after")
    def verify_advertisement_fingerprint(self) -> CapabilityAdvertisement:
        payload = self.as_dict()
        observed = payload.pop("fingerprint_sha256")
        if observed != _mapping_fingerprint(payload):
            raise ValueError("capability advertisement fingerprint is invalid")
        return self
        ####

    ####


class ConcreteCapabilityPreflightEvidence(_PortableMappingModel):
    """Validated capability evidence retained beside a semantic preflight."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.concrete-capability-preflight/v1alpha1"] = Field(alias="schema")
    composition_id: str = Field(min_length=1)
    composition_identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    semantic_translator_id: str | None = Field(default=None, min_length=1)
    adapter_id: str = Field(min_length=1)
    family_id: str = Field(min_length=1)
    mission_id: str = Field(min_length=1)
    fidelity: str = Field(min_length=1)
    feasibility: Literal["feasible", "likely_feasible", "unknown", "likely_infeasible", "certainly_infeasible"]
    diagnostics: tuple[str, ...] = ()
    capability_advertisement: CapabilityAdvertisement
    derived_mission_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")
    claim_boundary: str = Field(min_length=1)
    ####


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
    capability_estimate: ConcreteCapabilityPreflightEvidence | None = None

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
            "capability_estimate": None if self.capability_estimate is None else self.capability_estimate.as_dict(),
            "claim_boundary": _preflight_claim_boundary(self.status, self.capability_estimate),
        }
        ####

    ####


TranslationPreflightHandler = Callable[[CompiledVehicleComposition], VehicleExecutionPreflight]


def _preflight_claim_boundary(
    status: ExecutionPreflightStatus,
    capability_estimate: ConcreteCapabilityPreflightEvidence | None,
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
    plugins: PluginCatalog | None = None,
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
    capability_adapter = resolve_mission_capability_adapter(composition, plugins=plugins)
    if capability_adapter is None:
        diagnostic = (
            "mission template declares semantic translation but no tier-compatible capability adapter is installed"
            if declared_translator_id is not None
            else "no semantic execution preflight is registered for this family, mission, and fidelity yet"
        )
        return _not_applicable(composition, diagnostic)
    if declared_translator_id is None:
        estimate = estimate_mission_capability(composition, plugins=plugins)
        return _blocked(
            composition,
            (),
            "mission template has a family-owned capability estimate but no tier-compatible semantic_translator_id; "
            "native lowering and execution remain blocked",
            None if estimate is None else estimate.manifest,
            None if estimate is None else _capability_estimate_evidence(composition, estimate),
        )
    registry = handler_registry or semantic_preflight_handler_registry(plugins=plugins)
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
    plugins: PluginCatalog | None = None,
) -> VehicleExecutionPreflight:
    """Run semantic preflight and enforce the registry-declared translator.

    Individual family translators still own their source-specific plan checks,
    but readiness is not permitted to depend on an undisclosed code path.  A
    ``translation_ready`` result must name the exact translator declared by the
    selected mission template; a mismatch is a fail-closed integration error.
    """

    scope = plugin_catalog_scope(plugins) if plugins is not None else nullcontext()
    with scope:
        result = _preflight_vehicle_composition_unchecked(
            composition,
            handler_registry=handler_registry,
            plugins=plugins,
        )
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
    *,
    plugins: PluginCatalog | None = None,
) -> tuple[MissionCapabilityEstimate, dict[str, object]]:
    """Return the exact declared estimate and its family-owned capability map."""

    estimate = estimate_mission_capability(composition, plugins=plugins)
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


def build_concrete_capability_preflight_evidence(
    composition: CompiledVehicleComposition,
    estimate: MissionCapabilityEstimate,
) -> ConcreteCapabilityPreflightEvidence:
    """Return a fingerprinted capability projection for a concrete preflight.

    The full derived mission remains a sibling preflight field because native
    lowerers already consume that representation.  This common projection
    lets discovery and endpoint witnesses prove which family-owned estimate
    produced it, its feasibility disposition, and that it belongs to this
    exact immutable composition. Its generic capability advertisement carries
    family-owned data, control, resource, and runtime-admission metadata
    without forcing every vehicle into one physical model.
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
    family_owned_advertisement = estimate.manifest.get("capability")
    if not isinstance(family_owned_advertisement, dict):
        raise ValueError("capability adapter manifest has no generic capability advertisement")
    advertisement = _public_capability_advertisement(
        composition,
        family_owned_advertisement,
    )
    return ConcreteCapabilityPreflightEvidence.model_validate({
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
        "capability_advertisement": advertisement,
        "derived_mission_sha256": hashlib.sha256(encoded_manifest).hexdigest(),
        "claim_boundary": (
            "This fingerprinted family-owned capability estimate and its generic advertisement establish only "
            "semantic mission feasibility for the selected composition. They do not establish native execution, "
            "control realization, integration, truth-objective success, or qualification."
        ),
    })
    ####


def _public_capability_advertisement(
    composition: CompiledVehicleComposition,
    family_owned_advertisement: dict[str, object],
) -> CapabilityAdvertisement:
    """Join generic interface metadata to one family-owned capability payload.

    The resolved interface already owns availability, value-space, frame, and
    execution-binding metadata. Keeping it in a stable common envelope lets
    agents compare unlike vehicle plug-ins without flattening their distinct
    physical data into a fictional shared plant schema.
    """

    from .vehicle_composition import resolve_vehicle_composition_interface_contract

    interface = resolve_vehicle_composition_interface_contract(composition)
    payload: dict[str, object] = {
        "schema": "taoryx.vehicle-capability-advertisement/v1alpha1",
        "selection": {
            "composition_id": composition.id,
            "composition_identity_sha256": composition.identity_sha256,
            "vehicle_id": composition.vehicle_id,
            "family_id": composition.family_id,
            "mission_id": composition.mission,
            "fidelity": composition.fidelity,
        },
        "interface": interface.as_dict(),
        "family_owned": dict(family_owned_advertisement),
        "claim_boundary": (
            "This joins the exact generic interface contract to family-owned capability metadata. It does not "
            "bind a runtime, create controls, establish trim, integrate a trajectory, or qualify a vehicle."
        ),
    }
    payload["fingerprint_sha256"] = _mapping_fingerprint(payload)
    return CapabilityAdvertisement.model_validate(payload)
    ####


def validate_public_capability_advertisement(
    advertisement: Mapping[str, object] | CapabilityAdvertisement,
    *,
    expected_selection: Mapping[str, object],
) -> tuple[str, ...]:
    """Validate the common capability advertisement without recreating a plan.

    Witnesses and result catalogs use this to bind the host-owned interface
    projection and plug-in-owned capability facts to one exact composed
    selection. It is intentionally an integrity check, not a new feasibility
    calculation or a runtime admission decision.
    """

    try:
        parsed = (
            advertisement
            if isinstance(advertisement, CapabilityAdvertisement)
            else CapabilityAdvertisement.model_validate(advertisement)
        )
    except ValueError as error:
        return (f"capability advertisement violates the common contract: {error}",)
    errors: list[str] = []
    selection = parsed.selection
    for field in (
        "composition_id",
        "composition_identity_sha256",
        "vehicle_id",
        "family_id",
        "mission_id",
        "fidelity",
    ):
        observed = getattr(selection, field)
        if observed != expected_selection.get(field):
            errors.append(
                f"capability advertisement selection {field!r} is {observed!r}, "
                f"expected {expected_selection.get(field)!r}"
            )
    expected_interface_id = f"{expected_selection.get('family_id')}/{expected_selection.get('fidelity')}"
    if parsed.interface.interface_id != expected_interface_id:
        errors.append(
            f"capability advertisement interface ID is {parsed.interface.interface_id!r}, "
            f"expected {expected_interface_id!r}"
        )
    return tuple(errors)
    ####


def _mapping_fingerprint(payload: Mapping[str, object]) -> str:
    """Return the canonical SHA-256 identity for one JSON-safe mapping."""

    try:
        encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise ValueError(f"capability advertisement is not fingerprintable: {error}") from error
    return hashlib.sha256(encoded).hexdigest()
    ####


def _capability_estimate_evidence(
    composition: CompiledVehicleComposition,
    estimate: MissionCapabilityEstimate,
) -> ConcreteCapabilityPreflightEvidence:
    """Retain the internal name while custom translators use the public builder."""

    return build_concrete_capability_preflight_evidence(composition, estimate)
    ####


def _capability_number(capability: dict[str, object], key: str) -> float:
    """Read one finite numeric capability field without coercing booleans."""

    value = capability.get(key)
    if not isinstance(value, int | float) or isinstance(value, bool) or not math.isfinite(float(value)):
        raise ValueError(f"capability field {key!r} must be a finite numeric value")
    return float(value)
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
        segment_id=definition.segment_id,
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


def _preflight_local_native_coordinate_lqi(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight one exact named-control LQI screen without effector claims."""

    definition = resolve_local_native_coordinate_lqi_screen_definition(composition)
    if definition is None:
        raise ValueError("no installed local native-coordinate LQI screen matches this composition")
    config = definition.config_factory()
    compile_local_native_coordinate_lqi_screen_mission(
        composition,
        family_id=definition.family_id,
        mission_id=definition.mission_id,
        fidelity=definition.fidelity,
        initialization_id=definition.initialization_id,
        segment_id=definition.segment_id,
        screen_config_id=config.id,
    )
    estimate, capability = _capability_estimate_and_manifest(composition)
    native_controls = capability.get("native_control_names")
    native_limits = capability.get("native_control_limits")
    if not isinstance(native_controls, list) or set(native_controls) != set(config.candidate.control_names):
        raise ValueError(f"{definition.family_id} native-coordinate LQI capability control names are invalid")
    if not isinstance(native_limits, dict) or not isinstance(native_limits.get("lower"), dict) or not isinstance(native_limits.get("upper"), dict):
        raise ValueError(f"{definition.family_id} native-coordinate LQI capability limits are invalid")
    lower = native_limits["lower"]
    upper = native_limits["upper"]
    if any(float(lower[name]) >= float(upper[name]) for name in config.candidate.control_names):
        raise ValueError(f"{definition.family_id} native-coordinate LQI control bounds have no positive span")
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
                f"{definition.family_id}.semantic_local_native_coordinate_lqi_screen",
                f"pinned_{definition.initialization_id}_local_lqi_screen",
                [segment.instance_id for segment in composition.segments],
                None,
                True,
            ),
            ExecutionPreflightCheck(
                f"{definition.family_id}.native_control_authority_bounds",
                "positive finite span on every declared native LQI coordinate",
                {
                    name: float(upper[name]) - float(lower[name])
                    for name in config.candidate.control_names
                },
                "native coordinate by axis",
                True,
            ),
            ExecutionPreflightCheck(
                f"{definition.family_id}.local_lqi_candidate",
                "safe retained LQI candidate with integral outputs",
                {
                    "candidate_status": config.candidate.status,
                    "integral_output_names": list(config.candidate.lqi.output_names if config.candidate.lqi else ()),
                },
                None,
                config.candidate.safe and config.candidate.lqi is not None,
            ),
        ),
        (
            f"composition lowers exactly to the pinned {definition.family_id} local native-coordinate LQI recovery screen; "
            "it is not a route, physical-effector, or navigation translator",
            *estimate.diagnostics,
        ),
        estimate.manifest,
        _capability_estimate_evidence(composition, estimate),
    )
    ####


def preflight_local_native_coordinate_lqi(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight a plug-in-owned named-coordinate local LQI screen.

    The common host owns the semantic and status checks; the selected plug-in
    owns the exact screen definition, control plant, and runtime evidence.
    """

    return _preflight_local_native_coordinate_lqi(composition)
    ####


def preflight_powered_fixed_wing_racetrack(composition: CompiledVehicleComposition) -> VehicleExecutionPreflight:
    """Preflight a family-owned powered fixed-wing racetrack translator.

    This is the stable geometric preflight seam for an external vehicle
    plug-in that owns its mission translator while sharing the standard
    capability-scaled racetrack representation.  It establishes route
    translation only; dynamics and control qualification stay package-owned
    follow-on gates.
    """

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
        translator_id=estimate.adapter_id,
        checks=checks,
        diagnostics=tuple(diagnostics),
        derived_mission=estimate.manifest,
        capability_estimate=_capability_estimate_evidence(composition, estimate),
    )
    ####


def semantic_preflight_handler_registry(
    *,
    plugins: PluginCatalog | None = None,
) -> SemanticPreflightHandlerRegistry:
    """Return the immutable registry of installed source-owned translators."""

    catalog = plugins if plugins is not None else current_plugin_catalog()
    if catalog is None:
        catalog = discover_plugins()
    handlers: list[SemanticPreflightHandler] = []
    for contribution in catalog.records("semantic_preflight_handler"):
        value = contribution.value
        if isinstance(value, SemanticPreflightHandler):
            handlers.append(value)
            continue
        if isinstance(value, DeferredSemanticPreflightHandler):
            handlers.append(
                SemanticPreflightHandler(
                    value.translator_id,
                    cast(TranslationPreflightHandler, value.handler),
                )
            )
            continue
        else:
            raise TypeError(
                f"plug-in {contribution.plugin.id!r} supplied an invalid semantic preflight handler "
                f"for {contribution.id!r}"
            )
    return SemanticPreflightHandlerRegistry(tuple(handlers))
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
    *,
    plugins: PluginCatalog | None = None,
) -> CapabilityScaledRacetrack:
    """Compatibility projection of the registered fixed-wing capability planner."""

    return _compile_powered_fixed_wing_racetrack_from_composition(composition, plugins=plugins)
    ####


def compile_x8_racetrack_from_composition(
    composition: CompiledVehicleComposition,
    *,
    plugins: PluginCatalog | None = None,
) -> CapabilityScaledRacetrack:
    """Compatibility alias for the original X8-only public helper.

    New callers must use :func:`compile_powered_fixed_wing_racetrack_from_composition`
    so the shared fixed-wing translation is explicit.
    """

    if composition.family_id != "skywalker_x8":
        raise ValueError("X8 compatibility helper requires the Skywalker X8 family")
    return compile_powered_fixed_wing_racetrack_from_composition(composition, plugins=plugins)
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
    capability_estimate: ConcreteCapabilityPreflightEvidence | None = None,
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
    "CapabilityAdvertisement",
    "CapabilityInterfaceAdvertisement",
    "CapabilitySelection",
    "ConcreteCapabilityPreflightEvidence",
    "ExecutionPreflightCheck",
    "ExecutionPreflightStatus",
    "SemanticPreflightHandler",
    "SemanticPreflightHandlerRegistry",
    "VehicleExecutionPreflight",
    "build_concrete_capability_preflight_evidence",
    "build_semantic_preflight_handler_report",
    "compile_powered_fixed_wing_racetrack_from_composition",
    "_preflight_local_native_coordinate_lqi",
    "compile_x8_racetrack_from_composition",
    "preflight_local_native_coordinate_lqi",
    "preflight_powered_fixed_wing_racetrack",
    "preflight_vehicle_composition",
    "semantic_preflight_handler_registry",
    "validate_public_capability_advertisement",
]
