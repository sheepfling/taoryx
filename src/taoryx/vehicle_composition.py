"""Compilation of user-selected vehicle composition into a semantic scenario.

The vehicle-composition registry is a discovery contract.  This module turns
one selection from that registry into an immutable semantic scenario with an
explicit initial-state contract, ordered segment instances, and a native
adapter handoff.  It deliberately does *not* synthesize forces, moments, or
actuator commands: only the selected family adapter may do that at runtime.
"""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import FidelityTier, runtime_fidelity_for
from .vehicle_composition_registry import (
    CompositionParameter,
    InitializationContract,
    MissionTemplateContract,
    ResolvedVehicleComposition,
    ResolvedVehicleCompositionCatalog,
    SegmentContract,
    load_resolved_vehicle_composition_catalog,
    resolved_control_realization_for,
)

if TYPE_CHECKING:
    from .vehicle_interface import AuthorityProfile, InterfaceAvailability, InterfaceChannel, ObservationProfile, VehicleInterfaceContract


class VehicleCompositionError(ValueError):
    """Fail-closed diagnostic for an invalid user composition request."""

    def __init__(self, code: str, message: str, *, field: str | None = None) -> None:
        self.code = code
        self.field = field
        prefix = f"{code}: "
        if field is not None:
            prefix = f"{prefix}{field}: "
        super().__init__(prefix + message)
        ####
    ####


class CompositionValue(BaseModel):
    """One explicit semantic input value and its supplied unit, if any."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: Any
    unit: str | None = None
####


class InitializationSelection(BaseModel):
    """Selected initialization contract and values for its public inputs."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    inputs: dict[str, CompositionValue] = Field(default_factory=dict)
####


class SegmentSelection(BaseModel):
    """One occurrence of a segment in an ordered semantic mission."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    instance_id: str | None = None
    inputs: dict[str, CompositionValue] = Field(default_factory=dict)
####


class MissionSelection(BaseModel):
    """Selected reusable mission template."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
####


class DeclaredSensorChannelError(BaseModel):
    """A portable scalar measurement transform, declared rather than inferred."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    bias: float = 0.0
    gaussian_stddev: float = Field(default=0.0, ge=0.0)
    quantization_step: float | None = Field(default=None, gt=0.0)

    @model_validator(mode="after")
    def validate_finite_values(self) -> DeclaredSensorChannelError:
        values = (self.bias, self.gaussian_stddev, self.quantization_step)
        if any(value is not None and not math.isfinite(value) for value in values):
            raise ValueError("declared sensor measurement values must be finite")
        return self
        ####
####


class DeclaredSensorSelection(BaseModel):
    """One composition-owned observation sensor, sampled at truth boundaries."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str = Field(default="declared_sensor", min_length=1)
    channel_ids: tuple[str, ...] = Field(min_length=1)
    cadence_s: float = Field(gt=0.0)
    latency_s: float = Field(ge=0.0)
    channel_errors: dict[str, DeclaredSensorChannelError] = Field(default_factory=dict)
####


class ObservationSelection(BaseModel):
    """Policy observation selection resolved with the semantic composition."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str = Field(default="truth_debug", min_length=1)
    declared_sensor: DeclaredSensorSelection | None = None
####


class VehicleCompositionRequest(BaseModel):
    """Human-authored request to compose one registered vehicle mission."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(default="taoryx.vehicle-compose-request/v1alpha1", alias="schema")
    id: str = Field(min_length=1)
    vehicle: str = Field(min_length=1)
    fidelity: FidelityTier
    initialization: InitializationSelection
    mission: MissionSelection
    segments: tuple[SegmentSelection, ...] = Field(min_length=1)
    observation: ObservationSelection = Field(default_factory=ObservationSelection)
####


class ResolvedCompositionValue(BaseModel):
    """A value accepted in exactly the canonical unit for this first compiler."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: Any
    canonical_unit: str | None = None
    input_unit: str | None = None
####


class CompiledInitialization(BaseModel):
    """Validated initialization contract ready for adapter-specific lowering."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str
    status: str
    description: str
    inputs: dict[str, ResolvedCompositionValue]
####


class CompiledSegment(BaseModel):
    """Validated ordered semantic segment and its truth-state handoff rule."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str
    instance_id: str
    status: str
    description: str
    inputs: dict[str, ResolvedCompositionValue]
    required_control_intents: tuple[str, ...]
    permitted_transition_events: tuple[str, ...]
    state_transfer: str
####


class CompiledDeclaredSensor(BaseModel):
    """Immutable declared-sensor configuration for one compiled episode."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str
    channel_ids: tuple[str, ...]
    cadence_s: float
    latency_s: float
    channel_errors: dict[str, DeclaredSensorChannelError] = Field(default_factory=dict)
####


class CompiledObservation(BaseModel):
    """Resolved profile selection, separate from plant truth and diagnostics."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    profile_id: str
    declared_sensor: CompiledDeclaredSensor | None = None
####


class CompiledVehicleComposition(BaseModel):
    """Immutable semantic scenario ready for a declared native adapter handoff."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(default="taoryx.vehicle-composition/v1alpha1", alias="schema", serialization_alias="schema")
    id: str
    vehicle_id: str
    family_id: str
    physical_family: str
    fidelity: FidelityTier
    runtime_fidelity: str
    control_realization: str
    composition_status: str
    initialization: CompiledInitialization
    mission: str
    backing_templates: tuple[str, ...]
    qualification_missions: tuple[str, ...]
    segments: tuple[CompiledSegment, ...]
    observation: CompiledObservation = Field(default_factory=lambda: CompiledObservation(profile_id="truth_debug"))
    native_adapter_handoff: dict[str, Any]
    identity_sha256: str = Field(pattern=r"^[0-9a-f]{64}$")

    def canonical_payload(self) -> dict[str, Any]:
        """Return a reproducible payload excluding its self-referential digest."""

        payload = self.model_dump(mode="json", by_alias=True)
        payload.pop("identity_sha256", None)
        return payload
        ####

    def write_json(self, path: str | Path) -> None:
        """Write the composition in deterministic, human-readable JSON."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(self.model_dump(mode="json", by_alias=True), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        ####
####


def load_vehicle_composition_request(path: str | Path) -> VehicleCompositionRequest:
    """Load one user-authored composition request from YAML or JSON."""

    source = Path(path)
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise VehicleCompositionError("invalid-request", f"{source} must contain a mapping")
    return VehicleCompositionRequest.model_validate(payload)
    ####


def load_compiled_vehicle_composition(path: str | Path) -> CompiledVehicleComposition:
    """Load a previously compiled semantic composition artifact."""

    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise VehicleCompositionError("invalid-composition", f"{source} must contain a mapping")
    return CompiledVehicleComposition.model_validate(payload)
    ####


def compile_vehicle_composition(
    request: VehicleCompositionRequest,
    *,
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> CompiledVehicleComposition:
    """Resolve a selected initialization and exact mission segment sequence.

    Units are intentionally fail-closed in this v1 alpha compiler.  A supplied
    unit must equal the registry's canonical unit; omitted units mean the
    canonical unit.  This keeps the generated handoff unambiguous until the
    general unit-conversion service is shared by both trajectory contracts.
    """

    resolved_catalog = catalog or load_resolved_vehicle_composition_catalog()
    vehicle = _vehicle(resolved_catalog, request.vehicle)
    tier_binding = vehicle.family.family.tiers[request.fidelity]
    if tier_binding.profile_id is None:
        raise VehicleCompositionError(
            "fidelity-not-applicable",
            f"{request.fidelity!r} is not applicable to {vehicle.family.family_id!r}",
            field="fidelity",
        )

    mission = _mission(vehicle, request.mission.id)
    if request.fidelity not in mission.compatible_fidelities:
        raise VehicleCompositionError(
            "mission-fidelity-incompatible",
            f"mission {mission.id!r} is not declared for {request.fidelity!r}",
            field="fidelity",
        )
    initialization = _initialization(vehicle, request.initialization.id)
    if initialization.id not in mission.initialization_contracts:
        raise VehicleCompositionError(
            "initialization-not-allowed",
            f"mission {mission.id!r} does not allow initialization {initialization.id!r}",
            field="initialization.id",
        )
    if request.fidelity not in initialization.compatible_fidelities:
        raise VehicleCompositionError(
            "initialization-fidelity-incompatible",
            f"initialization {initialization.id!r} is not declared for {request.fidelity!r}",
            field="fidelity",
        )

    expected_ids = mission.segment_sequence
    actual_ids = tuple(item.id for item in request.segments)
    if actual_ids != expected_ids:
        raise VehicleCompositionError(
            "segment-sequence-mismatch",
            f"mission {mission.id!r} requires {list(expected_ids)!r}, received {list(actual_ids)!r}",
            field="segments",
        )

    compiled_initialization = CompiledInitialization(
        id=initialization.id,
        status=initialization.status,
        description=initialization.description,
        inputs=_resolve_inputs(initialization.parameters, request.initialization.inputs, f"initialization:{initialization.id}"),
    )
    compiled_segments: list[CompiledSegment] = []
    seen_instances: set[str] = set()
    for index, selected in enumerate(request.segments, start=1):
        segment = _segment(vehicle, selected.id)
        if request.fidelity not in segment.compatible_fidelities:
            raise VehicleCompositionError(
                "segment-fidelity-incompatible",
                f"segment {segment.id!r} is not declared for {request.fidelity!r}",
                field=f"segments[{index - 1}].id",
            )
        instance_id = selected.instance_id or f"{index:02d}-{segment.id}"
        if instance_id in seen_instances:
            raise VehicleCompositionError("duplicate-segment-instance", f"duplicate instance {instance_id!r}", field="segments")
        seen_instances.add(instance_id)
        compiled_segments.append(
            CompiledSegment(
                id=segment.id,
                instance_id=instance_id,
                status=segment.status,
                description=segment.description,
                inputs=_resolve_inputs(segment.parameters, selected.inputs, f"segment:{instance_id}"),
                required_control_intents=segment.required_control_intents,
                permitted_transition_events=segment.permitted_transition_events,
                state_transfer="previous_terminal_truth_state" if segment.preserves_state else "declared_physical_transition",
            )
        )

    statuses = [mission.status, initialization.status, *(segment.status for segment in compiled_segments)]
    status = _aggregate_status(statuses)
    compiled_observation = _resolve_observation(request.observation, vehicle.family.family_id, request.fidelity)
    adapter_handoff = {
        "adapter_id": vehicle.family.family.adapter_id,
        "profile_id": tier_binding.profile_id,
        "promotion_status": tier_binding.promotion_status,
        "required_operations": list(tier_binding.required_operations),
        "blockers": list(tier_binding.blockers),
        "handoff_status": "semantic_compiled_runtime_lowering_pending",
        "claim_boundary": (
            "This artifact validates user-facing mission composition only. "
            "It does not execute a plant, inject forces or moments, or promote the selected fidelity."
        ),
    }
    draft = CompiledVehicleComposition(
        id=request.id,
        vehicle_id=vehicle.family.family.vehicle_registry_id or vehicle.family.family_id,
        family_id=vehicle.family.family_id,
        physical_family=vehicle.family.family.physical_family,
        fidelity=request.fidelity,
        runtime_fidelity=runtime_fidelity_for(request.fidelity),
        control_realization=resolved_control_realization_for(vehicle.family.family_id, request.fidelity),
        composition_status=status,
        initialization=compiled_initialization,
        mission=mission.id,
        backing_templates=mission.backing_templates,
        qualification_missions=mission.qualification_missions,
        segments=tuple(compiled_segments),
        observation=compiled_observation,
        native_adapter_handoff=adapter_handoff,
        identity_sha256="0" * 64,
    )
    digest = hashlib.sha256(
        json.dumps(draft.canonical_payload(), sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return draft.model_copy(update={"identity_sha256": digest})
    ####


def resolve_vehicle_composition_interface_contract(
    composition: CompiledVehicleComposition,
    *,
    catalog: ResolvedVehicleCompositionCatalog | None = None,
) -> VehicleInterfaceContract:
    """Resolve the exact caller-facing interface selected by one composition.

    The family/fidelity contract alone is insufficient when a composition
    declares a sampled observation profile. This late resolver keeps the
    composition compiler free of an import cycle while making the exact
    profile fingerprint discoverable before an episode or batch run opens.
    """

    from .vehicle_execution_bindings import (
        VehicleExecutionBindingError,
        bindings_for_family,
        resolve_vehicle_execution_binding,
    )
    from .vehicle_interface import bind_declared_sensor_profile, resolve_vehicle_interface_contract

    contract = resolve_vehicle_interface_contract(composition.family_id, composition.fidelity, catalog=catalog)
    try:
        resolve_vehicle_execution_binding(composition, "episode")
        episode_runnable = True
    except VehicleExecutionBindingError:
        episode_runnable = False
    try:
        resolve_vehicle_execution_binding(composition, "batch")
        batch_runnable = True
    except VehicleExecutionBindingError:
        batch_runnable = False
    exact_records = tuple(
        item.model_dump(mode="json")
        for item in bindings_for_family(composition.family_id)
        if item.mission == composition.mission and item.fidelity == composition.fidelity
    )
    contract = _gate_interface_to_composition_execution(
        contract,
        episode_runnable=episode_runnable,
        batch_runnable=batch_runnable,
        execution_records=exact_records,
    )
    declared = composition.observation.declared_sensor
    if declared is None:
        contract.observation_profile(composition.observation.profile_id)
        return contract
    return bind_declared_sensor_profile(
        contract,
        channel_ids=declared.channel_ids,
        cadence_s=declared.cadence_s,
        latency_s=declared.latency_s,
        channel_errors={identifier: error.model_dump(mode="json") for identifier, error in declared.channel_errors.items()},
        profile_id=declared.profile_id,
    )
    ####


def _gate_interface_to_composition_execution(
    contract: VehicleInterfaceContract,
    *,
    episode_runnable: bool,
    batch_runnable: bool,
    execution_records: tuple[dict[str, object], ...],
) -> VehicleInterfaceContract:
    """Scope family/fidelity channels to one exact mission execution binding.

    The catalog-level contract remains useful for discovery, but it may have a
    runnable episode for a different mission at the same family/fidelity. A
    compiled composition must not inherit that neighboring endpoint. This is
    especially important for direct-wrench bridge tiers, where a local screen
    and an end-to-end mission have intentionally different authority claims.
    """

    def runtime_channel(channel: InterfaceChannel) -> InterfaceChannel:
        if channel.availability not in {"available", "available_in_batch"}:
            return channel
        availability: InterfaceAvailability = (
            "available" if episode_runnable else "available_in_batch" if batch_runnable else "unavailable_at_runtime"
        )
        return replace(channel, availability=availability)
        ####

    def action_channel(channel: InterfaceChannel) -> InterfaceChannel:
        if channel.availability != "available":
            return channel
        return replace(channel, availability="available" if episode_runnable else "unavailable_at_runtime")
        ####

    def authority_profile(profile: AuthorityProfile) -> AuthorityProfile:
        if profile.availability != "available":
            return profile
        return replace(profile, availability="available" if episode_runnable else "unavailable_at_runtime")
        ####

    def observation_profile(profile: ObservationProfile) -> ObservationProfile:
        if profile.availability != "available":
            return profile
        return replace(profile, availability="available" if episode_runnable else "unavailable_at_runtime")
        ####

    return type(contract)(
        vehicle_id=contract.vehicle_id,
        family_id=contract.family_id,
        physical_family=contract.physical_family,
        fidelity=contract.fidelity,
        control_realization=contract.control_realization,
        evidence_status=contract.evidence_status,
        parameter_channels=contract.parameter_channels,
        action_channels=tuple(action_channel(item) for item in contract.action_channels),
        effector_channels=contract.effector_channels,
        status_channels=tuple(runtime_channel(item) for item in contract.status_channels),
        resource_channels=tuple(runtime_channel(item) for item in contract.resource_channels),
        diagnostic_channels=tuple(runtime_channel(item) for item in contract.diagnostic_channels),
        authority_profiles=tuple(authority_profile(item) for item in contract.authority_profiles),
        observation_profiles=tuple(observation_profile(item) for item in contract.observation_profiles),
        execution_records=execution_records,
        claim_boundary=(
            f"{contract.claim_boundary} This composition-scoped projection exposes only the exact "
            f"episode={episode_runnable} and batch={batch_runnable} execution bindings for mission {composition_mission_label(execution_records)!r}."
        ),
        schema=contract.schema,
    )
    ####


def composition_mission_label(execution_records: tuple[dict[str, object], ...]) -> str:
    """Return the one mission label retained by the exact execution projection."""

    missions = {str(item.get("mission")) for item in execution_records if isinstance(item.get("mission"), str)}
    return next(iter(missions)) if len(missions) == 1 else "unbound_mission"
    ####


def _vehicle(catalog: ResolvedVehicleCompositionCatalog, identifier: str) -> ResolvedVehicleComposition:
    try:
        return catalog.vehicle(identifier)
    except (KeyError, ValueError) as error:
        raise VehicleCompositionError("unknown-vehicle", str(error), field="vehicle") from error
    ####


def _initialization(vehicle: ResolvedVehicleComposition, identifier: str) -> InitializationContract:
    matches = tuple(item for item in vehicle.declaration.initialization_contracts if item.id == identifier)
    if len(matches) != 1:
        raise VehicleCompositionError("unknown-initialization", f"unknown initialization {identifier!r}", field="initialization.id")
    return matches[0]
    ####


def _segment(vehicle: ResolvedVehicleComposition, identifier: str) -> SegmentContract:
    matches = tuple(item for item in vehicle.declaration.segment_contracts if item.id == identifier)
    if len(matches) != 1:
        raise VehicleCompositionError("unknown-segment", f"unknown segment {identifier!r}", field="segments")
    return matches[0]
    ####


def _mission(vehicle: ResolvedVehicleComposition, identifier: str) -> MissionTemplateContract:
    matches = tuple(item for item in vehicle.declaration.mission_templates if item.id == identifier)
    if len(matches) != 1:
        raise VehicleCompositionError("unknown-mission", f"unknown mission template {identifier!r}", field="mission.id")
    return matches[0]
    ####


def _resolve_inputs(
    parameters: tuple[CompositionParameter, ...],
    values: Mapping[str, CompositionValue],
    scope: str,
) -> dict[str, ResolvedCompositionValue]:
    """Require all mandatory inputs and preserve canonical-unit provenance."""

    schemas = {item.id: item for item in parameters}
    unknown = sorted(set(values) - set(schemas))
    if unknown:
        raise VehicleCompositionError("unknown-input", f"undeclared input(s): {', '.join(unknown)}", field=scope)
    missing = sorted(item.id for item in parameters if item.required and item.id not in values)
    if missing:
        raise VehicleCompositionError("missing-input", f"required input(s): {', '.join(missing)}", field=scope)
    resolved: dict[str, ResolvedCompositionValue] = {}
    for parameter in parameters:
        supplied = values.get(parameter.id)
        if supplied is None:
            continue
        if parameter.canonical_unit is None and supplied.unit is not None:
            raise VehicleCompositionError(
                "unexpected-unit",
                "this parameter is unitless or categorical",
                field=f"{scope}.{parameter.id}",
            )
        if parameter.canonical_unit is not None and supplied.unit not in {None, parameter.canonical_unit}:
            raise VehicleCompositionError(
                "unit-mismatch",
                f"expected canonical unit {parameter.canonical_unit!r}, received {supplied.unit!r}",
                field=f"{scope}.{parameter.id}",
            )
        resolved[parameter.id] = ResolvedCompositionValue(
            value=supplied.value,
            canonical_unit=parameter.canonical_unit,
            input_unit=supplied.unit,
        )
    return resolved
    ####


def _resolve_observation(
    selection: ObservationSelection,
    family_id: str,
    fidelity: FidelityTier,
) -> CompiledObservation:
    """Bind a declared sensor selection to portable available channels only."""

    from .vehicle_interface import bind_declared_sensor_profile, resolve_vehicle_interface_contract

    contract = resolve_vehicle_interface_contract(family_id, fidelity)
    sensor = selection.declared_sensor
    if sensor is None:
        contract.observation_profile(selection.profile_id)
        return CompiledObservation(profile_id=selection.profile_id)
    if sensor.profile_id != selection.profile_id:
        raise VehicleCompositionError(
            "sensor-profile-mismatch",
            "observation.profile_id must equal observation.declared_sensor.profile_id",
            field="observation",
        )
    try:
        bind_declared_sensor_profile(
            contract,
            channel_ids=sensor.channel_ids,
            cadence_s=sensor.cadence_s,
            latency_s=sensor.latency_s,
            channel_errors={identifier: error.model_dump(mode="json") for identifier, error in sensor.channel_errors.items()},
            profile_id=sensor.profile_id,
        )
    except ValueError as error:
        raise VehicleCompositionError("invalid-declared-sensor", str(error), field="observation.declared_sensor") from error
    return CompiledObservation(
        profile_id=sensor.profile_id,
        declared_sensor=CompiledDeclaredSensor(
            profile_id=sensor.profile_id,
            channel_ids=sensor.channel_ids,
            cadence_s=sensor.cadence_s,
            latency_s=sensor.latency_s,
            channel_errors=dict(sensor.channel_errors),
        ),
    )
    ####


def _aggregate_status(statuses: list[str]) -> str:
    """Return the least mature declaration without conflating it with evidence."""

    rank = {"runnable": 0, "development": 1, "planned": 2}
    return max(statuses, key=lambda item: rank[item])
    ####


__all__ = [
    "CompiledVehicleComposition",
    "CompiledDeclaredSensor",
    "CompiledObservation",
    "CompositionValue",
    "DeclaredSensorChannelError",
    "DeclaredSensorSelection",
    "InitializationSelection",
    "MissionSelection",
    "ObservationSelection",
    "ResolvedCompositionValue",
    "SegmentSelection",
    "VehicleCompositionError",
    "VehicleCompositionRequest",
    "compile_vehicle_composition",
    "load_compiled_vehicle_composition",
    "load_vehicle_composition_request",
    "resolve_vehicle_composition_interface_contract",
]
