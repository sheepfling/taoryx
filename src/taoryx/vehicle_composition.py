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
from typing import TYPE_CHECKING, Any, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import FidelityTier, runtime_fidelity_for
from .value_space import validate_value_space_value
from .vehicle_composition_registry import (
    CompositionParameter,
    InitializationContract,
    MissionTemplateContract,
    ResolvedVehicleComposition,
    ResolvedVehicleCompositionCatalog,
    SegmentContract,
    VariantParameterBinding,
    load_resolved_vehicle_composition_catalog,
    resolved_control_realization_for,
)

if TYPE_CHECKING:
    from .vehicle_execution_bindings import VehicleExecutionBinding
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


MissionTransitionKind = Literal["success", "abort", "resource_limit", "envelope_limit", "timeout"]
StateTransferKind = Literal["previous_terminal_truth_state", "declared_physical_transition"]


class MissionTransitionSelection(BaseModel):
    """One explicit semantic graph edge or named terminal outcome.

    ``target_instance_id: null`` is a declared terminal outcome for this
    trigger. It is never an implicit timeout-as-success shortcut.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    target_instance_id: str | None = None
    state_transfer: StateTransferKind = "previous_terminal_truth_state"
####


class MissionGraphNodeSelection(BaseModel):
    """Transitions owned by one selected segment instance."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    instance_id: str = Field(min_length=1)
    success_transition: MissionTransitionSelection | None = None
    abort_transition: MissionTransitionSelection | None = None
    resource_limit_transition: MissionTransitionSelection | None = None
    envelope_limit_transition: MissionTransitionSelection | None = None
    timeout_transition: MissionTransitionSelection | None = None
####


class MissionGraphSelection(BaseModel):
    """Caller-authored graph over the request's named segment instances.

    The first graph slice supports acyclic typed transitions and validates
    them before execution. Native translators still advertise whether they
    can realize anything beyond the template's legacy success sequence.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    entry_instance_id: str = Field(min_length=1)
    nodes: tuple[MissionGraphNodeSelection, ...] = Field(min_length=1)
####


class VariantSelection(BaseModel):
    """Optional bounded configuration values applied before initialization."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    inputs: dict[str, CompositionValue] = Field(default_factory=dict)
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
    mission_graph: MissionGraphSelection | None = None
    variant: VariantSelection = Field(default_factory=VariantSelection)
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


class CompiledVariantRuntimeBinding(BaseModel):
    """One immutable runtime modifier trace and declared coupling boundary.

    The binding retains the public scalar topology used during resolution.
    This prevents a downstream runtime, UI, or optimization client from
    retaining only a native input path while losing the bounds and transform
    that make the supplied value meaningful.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    runtime_adapter_id: str
    runtime_input_path: str
    target_initialization_id: str
    target_parameter_id: str
    compatible_fidelities: tuple[FidelityTier, ...]
    value_space: dict[str, object]
    transform: Literal["identity", "log", "logit"]
    hard_lower: float | None
    hard_upper: float | None
    coupling_group: str
    coupling_policy: Literal["exclusive", "composable"]
    derived_status_channels: tuple[str, ...]
    status_derivation_relation: Literal["equal_to_target", "not_asserted"]
    resource_derivation: Literal["runtime_adapter_owned", "not_represented", "planned"]
    derivation_claim_boundary: str
####


class CompiledVariantProjection(BaseModel):
    """One explicit optimizer/input repair performed by a declared policy."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    requested_value: float
    resolved_value: float
    normalized_distance: float
    constraints: tuple[str, ...]
####


class CompiledVariantResolution(BaseModel):
    """Immutable record of runtime-bound modifier application."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    inputs: dict[str, ResolvedCompositionValue] = Field(default_factory=dict)
    runtime_bindings: dict[str, CompiledVariantRuntimeBinding] = Field(default_factory=dict)
    projections: dict[str, CompiledVariantProjection] = Field(default_factory=dict)
    invalidations: tuple[str, ...] = ()
    resolution_policy: Literal["reject_invalid", "project_to_valid", "mixed"] = "reject_invalid"
    qualification: Literal["baseline", "qualified", "extended"] = "baseline"
    qualification_findings: tuple[str, ...] = ()
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


class CompiledMissionTransition(BaseModel):
    """Resolved graph edge with a named trigger and state-transfer rule."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    target_instance_id: str | None = None
    state_transfer: StateTransferKind = "previous_terminal_truth_state"
####


class CompiledMissionGraphNode(BaseModel):
    """One resolved segment node and all named outcome transitions."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    instance_id: str
    segment_id: str
    success_transition: CompiledMissionTransition | None = None
    abort_transition: CompiledMissionTransition | None = None
    resource_limit_transition: CompiledMissionTransition | None = None
    envelope_limit_transition: CompiledMissionTransition | None = None
    timeout_transition: CompiledMissionTransition | None = None
####


class CompiledMissionGraph(BaseModel):
    """Immutable graph carried with the resolved composition artifact."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(default="taoryx.mission-graph/v1alpha1", alias="schema", serialization_alias="schema")
    status: Literal[
        "linear_sequence_only",
        "authored_linear_sequence_lowered",
        "authored_graph_not_lowered",
    ]
    entry_instance_id: str
    nodes: tuple[CompiledMissionGraphNode, ...]
    claim_boundary: str
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
    variant: CompiledVariantResolution = Field(default_factory=CompiledVariantResolution)
    initialization: CompiledInitialization
    mission: str
    backing_templates: tuple[str, ...]
    qualification_missions: tuple[str, ...]
    segments: tuple[CompiledSegment, ...]
    mission_graph: CompiledMissionGraph | None = None
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

    resolved_variant, initialization_inputs = _resolve_variant_inputs(vehicle, initialization, request)
    compiled_initialization = CompiledInitialization(
        id=initialization.id,
        status=initialization.status,
        description=initialization.description,
        inputs=_resolve_inputs(initialization.parameters, initialization_inputs, f"initialization:{initialization.id}"),
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

    compiled_graph = _compile_mission_graph(request.mission_graph, tuple(compiled_segments))
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
        variant=resolved_variant,
        initialization=compiled_initialization,
        mission=mission.id,
        backing_templates=mission.backing_templates,
        qualification_missions=mission.qualification_missions,
        segments=tuple(compiled_segments),
        mission_graph=compiled_graph,
        observation=compiled_observation,
        native_adapter_handoff=adapter_handoff,
        identity_sha256="0" * 64,
    )
    digest = hashlib.sha256(
        json.dumps(draft.canonical_payload(), sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()
    return draft.model_copy(update={"identity_sha256": digest})
    ####


def _compile_mission_graph(
    selection: MissionGraphSelection | None,
    segments: tuple[CompiledSegment, ...],
) -> CompiledMissionGraph:
    """Validate a typed acyclic graph over resolved segment instances.

    Existing template-owned requests remain a linear sequence. An explicitly
    authored graph can be composed and inspected now. A caller-authored graph
    may execute only when it is exactly the template's existing success chain;
    all branches and alternate state handoffs retain a distinct blocked status.
    """

    instance_ids = tuple(segment.instance_id for segment in segments)
    known_ids = set(instance_ids)
    if selection is None:
        nodes = tuple(
            CompiledMissionGraphNode(
                instance_id=segment.instance_id,
                segment_id=segment.id,
                success_transition=(
                    None
                    if index + 1 == len(segments)
                    else CompiledMissionTransition(target_instance_id=segments[index + 1].instance_id)
                ),
            )
            for index, segment in enumerate(segments)
        )
        return CompiledMissionGraph(
            status="linear_sequence_only",
            entry_instance_id=segments[0].instance_id,
            nodes=nodes,
            claim_boundary=(
                "This graph is the exact template-owned success sequence accepted by current native translators. "
                "No caller-configurable branch, fallback, abort, resource, envelope, or timeout semantics are active."
            ),
        )

    selected_nodes = {node.instance_id: node for node in selection.nodes}
    if len(selected_nodes) != len(selection.nodes):
        raise VehicleCompositionError("duplicate-mission-graph-node", "mission graph has duplicate instance IDs", field="mission_graph.nodes")
    if set(selected_nodes) != known_ids:
        missing = sorted(known_ids - set(selected_nodes))
        unknown = sorted(set(selected_nodes) - known_ids)
        raise VehicleCompositionError(
            "mission-graph-node-set-mismatch",
            f"graph nodes must exactly match selected segments; missing={missing!r}, unknown={unknown!r}",
            field="mission_graph.nodes",
        )
    if selection.entry_instance_id not in known_ids:
        raise VehicleCompositionError(
            "unknown-mission-graph-entry",
            f"unknown entry instance {selection.entry_instance_id!r}",
            field="mission_graph.entry_instance_id",
        )

    def resolve_transition(
        transition: MissionTransitionSelection | None,
        *,
        field: str,
    ) -> CompiledMissionTransition | None:
        if transition is None:
            return None
        if transition.target_instance_id is not None and transition.target_instance_id not in known_ids:
            raise VehicleCompositionError(
                "unknown-mission-graph-target",
                f"unknown target instance {transition.target_instance_id!r}",
                field=field,
            )
        return CompiledMissionTransition(
            target_instance_id=transition.target_instance_id,
            state_transfer=transition.state_transfer,
        )

    compiled_nodes: list[CompiledMissionGraphNode] = []
    for segment in segments:
        node = selected_nodes[segment.instance_id]
        prefix = f"mission_graph.nodes.{node.instance_id}"
        compiled_nodes.append(
            CompiledMissionGraphNode(
                instance_id=segment.instance_id,
                segment_id=segment.id,
                success_transition=resolve_transition(node.success_transition, field=f"{prefix}.success_transition"),
                abort_transition=resolve_transition(node.abort_transition, field=f"{prefix}.abort_transition"),
                resource_limit_transition=resolve_transition(node.resource_limit_transition, field=f"{prefix}.resource_limit_transition"),
                envelope_limit_transition=resolve_transition(node.envelope_limit_transition, field=f"{prefix}.envelope_limit_transition"),
                timeout_transition=resolve_transition(node.timeout_transition, field=f"{prefix}.timeout_transition"),
            )
        )

    adjacency = {
        node.instance_id: tuple(
            transition.target_instance_id
            for transition in (
                node.success_transition,
                node.abort_transition,
                node.resource_limit_transition,
                node.envelope_limit_transition,
                node.timeout_transition,
            )
            if transition is not None and transition.target_instance_id is not None
        )
        for node in compiled_nodes
    }
    reachable: set[str] = set()
    stack = [selection.entry_instance_id]
    while stack:
        current = stack.pop()
        if current in reachable:
            continue
        reachable.add(current)
        stack.extend(adjacency[current])
    if reachable != known_ids:
        raise VehicleCompositionError(
            "unreachable-mission-graph-node",
            f"mission graph has unreachable selected segment(s): {sorted(known_ids - reachable)!r}",
            field="mission_graph",
        )
    active: set[str] = set()
    visited: set[str] = set()

    def visit(node_id: str) -> None:
        if node_id in active:
            raise VehicleCompositionError(
                "cyclic-mission-graph",
                "caller-authored mission graphs must be acyclic until a bounded loop contract is declared",
                field="mission_graph",
            )
        if node_id in visited:
            return
        active.add(node_id)
        for target in adjacency[node_id]:
            visit(target)
        active.remove(node_id)
        visited.add(node_id)

    visit(selection.entry_instance_id)
    exact_linear = _is_exact_template_success_sequence(
        selection.entry_instance_id,
        tuple(compiled_nodes),
        segments,
    )
    return CompiledMissionGraph(
        status="authored_linear_sequence_lowered" if exact_linear else "authored_graph_not_lowered",
        entry_instance_id=selection.entry_instance_id,
        nodes=tuple(compiled_nodes),
        claim_boundary=(
            "This caller-authored graph is an exact projection of the template-owned success sequence and adds "
            "no branch, fallback, timeout, or alternate state-transfer semantics. Current native translators may "
            "execute that sequence."
            if exact_linear
            else "This caller-authored graph passed semantic node, transition, reachability, and acyclicity validation. "
            "No current native translator has opted in to execute its branch or alternate state-transfer semantics; "
            "preflight and lowering must remain blocked."
        ),
    )
    ####


def _is_exact_template_success_sequence(
    entry_instance_id: str,
    nodes: tuple[CompiledMissionGraphNode, ...],
    segments: tuple[CompiledSegment, ...],
) -> bool:
    """Return whether a caller graph adds no executable semantics to a template.

    Native adapters currently consume the ordered segment sequence and always
    transfer the previous committed truth state. A graph is lowerable only if
    it describes exactly that already-supported behavior.
    """

    if entry_instance_id != segments[0].instance_id:
        return False
    for index, (node, segment) in enumerate(zip(nodes, segments, strict=True)):
        if node.instance_id != segment.instance_id or node.segment_id != segment.id:
            return False
        if any(
            transition is not None
            for transition in (
                node.abort_transition,
                node.resource_limit_transition,
                node.envelope_limit_transition,
                node.timeout_transition,
            )
        ):
            return False
        expected_target = None if index + 1 == len(segments) else segments[index + 1].instance_id
        transition = node.success_transition
        if expected_target is None:
            if transition is not None:
                return False
        elif (
            transition is None
            or transition.target_instance_id != expected_target
            or transition.state_transfer != "previous_terminal_truth_state"
        ):
            return False
    return True
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
        item
        for item in bindings_for_family(composition.family_id)
        if item.mission == composition.mission and item.fidelity == composition.fidelity
    )
    contract = _gate_interface_to_composition_execution(
        contract,
        mission_id=composition.mission,
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
    mission_id: str,
    episode_runnable: bool,
    batch_runnable: bool,
    execution_records: tuple[VehicleExecutionBinding, ...],
) -> VehicleInterfaceContract:
    """Scope family/fidelity channels to one exact mission execution binding.

    The catalog-level contract remains useful for discovery, but it may have a
    runnable episode for a different mission at the same family/fidelity. A
    compiled composition must not inherit that neighboring endpoint. This is
    especially important for direct-wrench bridge tiers, where a local screen
    and an end-to-end mission have intentionally different authority claims.
    """

    batch_internal_controller_trace = batch_runnable and any(
        item.execution_mode == "local_direct_wrench_screen" for item in execution_records
    )

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
        if episode_runnable:
            return channel
        if batch_internal_controller_trace and bool(channel.binding.get("internal_controller_trace")):
            return replace(channel, availability="available_in_batch")
        return replace(channel, availability="unavailable_at_runtime")
        ####

    def authority_profile(profile: AuthorityProfile) -> AuthorityProfile:
        if profile.availability != "available":
            return profile
        if episode_runnable:
            return profile
        if batch_internal_controller_trace and profile.id == "direct_wrench":
            return replace(profile, availability="available_in_batch")
        return replace(profile, availability="unavailable_at_runtime")
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
            f"episode={episode_runnable} and batch={batch_runnable} execution bindings for mission {mission_id!r}."
        ),
        schema=contract.schema,
    )
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


def _resolve_variant_numeric_value(
    binding: VariantParameterBinding,
    requested_value: float,
) -> tuple[float, CompiledVariantProjection | None]:
    """Apply one declared hard-bound policy without hiding any repair.

    ``project_to_valid`` is intentionally scalar and hard-bound limited for
    this first variant resolver.  It is a useful policy for optimizers that
    occasionally propose a slightly invalid continuous candidate, but it may
    never repair a non-finite value, a wrong unit, or a topology violation that
    lacks a declared finite bound.
    """

    lower = binding.hard_lower
    upper = binding.hard_upper
    resolved = requested_value
    constraints: list[str] = []
    if lower is not None and resolved < lower:
        constraints.append("hard_lower")
        resolved = lower
    if upper is not None and resolved > upper:
        constraints.append("hard_upper")
        resolved = upper
    if not constraints:
        return (resolved, None)
    if binding.resolution_policy == "reject_invalid":
        raise VehicleCompositionError(
            "variant-hard-bound-violation",
            f"value {requested_value!r} is outside [{lower!r}, {upper!r}]",
            field=f"variant.inputs.{binding.id}",
        )
    if lower is not None and upper is not None and upper > lower:
        normalized_distance = abs(resolved - requested_value) / (upper - lower)
    else:
        scale = max(1.0, abs(resolved))
        normalized_distance = abs(resolved - requested_value) / scale
    return (
        resolved,
        CompiledVariantProjection(
            requested_value=requested_value,
            resolved_value=resolved,
            normalized_distance=normalized_distance,
            constraints=tuple(constraints),
        ),
    )
    ####


def _resolve_variant_inputs(
    vehicle: ResolvedVehicleComposition,
    initialization: InitializationContract,
    request: VehicleCompositionRequest,
) -> tuple[CompiledVariantResolution, dict[str, CompositionValue]]:
    """Apply only declared runtime-bound variant values to initialization.

    A variant is not a generic mapping merge. Each supplied ID must identify a
    binding owned by the selected family, point at the selected initialization,
    satisfy hard bounds, and be declared runnable. The resulting value is
    still validated against the target initialization parameter below.
    """

    supplied = request.variant.inputs
    bindings = {item.id: item for item in vehicle.declaration.variant_parameters}
    unknown = sorted(set(supplied) - set(bindings))
    if unknown:
        raise VehicleCompositionError("unknown-variant-input", f"undeclared variant input(s): {', '.join(unknown)}", field="variant.inputs")
    values = dict(request.initialization.inputs)
    applied: dict[str, ResolvedCompositionValue] = {}
    runtime_bindings: dict[str, CompiledVariantRuntimeBinding] = {}
    projections: dict[str, CompiledVariantProjection] = {}
    invalidations: set[str] = set()
    qualification_findings: list[str] = []
    qualification: Literal["baseline", "qualified", "extended"] = "baseline"
    resolution_policies: set[Literal["reject_invalid", "project_to_valid"]] = set()
    active_coupling_groups: dict[str, tuple[str, Literal["exclusive", "composable"]]] = {}
    for identifier, value in supplied.items():
        binding: VariantParameterBinding = bindings[identifier]
        if binding.status != "runnable":
            raise VehicleCompositionError("variant-not-runnable", f"variant {identifier!r} is not bound to an executable runtime adapter", field=f"variant.inputs.{identifier}")
        if request.fidelity not in binding.compatible_fidelities:
            raise VehicleCompositionError(
                "variant-fidelity-incompatible",
                f"variant {identifier!r} is not declared for fidelity {request.fidelity!r}; "
                f"supported={list(binding.compatible_fidelities)!r}",
                field=f"variant.inputs.{identifier}",
            )
        if binding.target_initialization_id != initialization.id:
            raise VehicleCompositionError(
                "variant-initialization-incompatible",
                f"variant {identifier!r} targets initialization {binding.target_initialization_id!r}, not {initialization.id!r}",
                field=f"variant.inputs.{identifier}",
            )
        coupling_group = binding.coupling_group
        if coupling_group is not None:
            prior = active_coupling_groups.get(coupling_group)
            if prior is not None and (prior[1] == "exclusive" or binding.coupling_policy == "exclusive"):
                raise VehicleCompositionError(
                    "variant-coupling-conflict",
                    f"variant {identifier!r} conflicts with {prior[0]!r} in exclusive coupling group {coupling_group!r}",
                    field=f"variant.inputs.{identifier}",
                )
            active_coupling_groups[coupling_group] = (identifier, binding.coupling_policy)
        if binding.canonical_unit is None and value.unit is not None:
            raise VehicleCompositionError("variant-unit-mismatch", "variant is unitless but a unit was supplied", field=f"variant.inputs.{identifier}")
        if binding.canonical_unit is not None and value.unit not in {None, binding.canonical_unit}:
            raise VehicleCompositionError(
                "variant-unit-mismatch",
                f"expected canonical unit {binding.canonical_unit!r}, received {value.unit!r}",
                field=f"variant.inputs.{identifier}",
            )
        if isinstance(value.value, bool) or not isinstance(value.value, int | float):
            raise VehicleCompositionError("variant-type-mismatch", "variant input must be a finite numeric scalar", field=f"variant.inputs.{identifier}")
        numeric = float(value.value)
        if not math.isfinite(numeric):
            raise VehicleCompositionError("variant-type-mismatch", "variant input must be finite", field=f"variant.inputs.{identifier}")
        resolved_numeric, projection = _resolve_variant_numeric_value(binding, numeric)
        try:
            validate_value_space_value(binding.value_space, resolved_numeric, context=f"variant.inputs.{identifier}")
        except ValueError as error:
            raise VehicleCompositionError("variant-value-space-mismatch", str(error), field=f"variant.inputs.{identifier}") from error
        target = binding.target_parameter_id
        if target in values:
            raise VehicleCompositionError(
                "variant-target-conflict",
                f"variant {identifier!r} and initialization both supply {target!r}",
                field=f"variant.inputs.{identifier}",
            )
        resolved_input = CompositionValue(value=resolved_numeric, unit=value.unit)
        values[target] = resolved_input
        applied[identifier] = ResolvedCompositionValue(value=resolved_numeric, canonical_unit=binding.canonical_unit, input_unit=value.unit)
        resolution_policies.add(binding.resolution_policy)
        if projection is not None:
            projections[identifier] = projection
        if binding.transform not in {"identity", "log", "logit"}:
            raise VehicleCompositionError(
                "variant-transform-not-runtime-bindable",
                f"runtime-bound numeric variant {identifier!r} cannot use transform {binding.transform!r}",
                field=f"variant.inputs.{identifier}",
            )
        runtime_bindings[identifier] = CompiledVariantRuntimeBinding(
            runtime_adapter_id=binding.runtime_adapter_id or "",
            runtime_input_path=binding.runtime_input_path or "",
            target_initialization_id=binding.target_initialization_id,
            target_parameter_id=target,
            compatible_fidelities=binding.compatible_fidelities,
            value_space=binding.value_space.as_dict(),
            transform=binding.transform,
            hard_lower=binding.hard_lower,
            hard_upper=binding.hard_upper,
            coupling_group=binding.coupling_group or "",
            coupling_policy=binding.coupling_policy,
            derived_status_channels=binding.derived_status_channels,
            status_derivation_relation=binding.status_derivation_relation,
            resource_derivation=binding.resource_derivation,
            derivation_claim_boundary=binding.derivation_claim_boundary,
        )
        if binding.requires_retrim:
            invalidations.add(f"retrim:{target}")
        if binding.requires_requalification:
            invalidations.add(f"requalification:{target}")
        if binding.qualified_lower is None or binding.qualified_upper is None:
            qualification = "extended"
            qualification_findings.append(
                f"{identifier}: hard-valid runtime binding has no declared qualified range"
            )
        elif resolved_numeric < binding.qualified_lower or resolved_numeric > binding.qualified_upper:
            qualification = "extended"
            qualification_findings.append(
                f"{identifier}: resolved value {resolved_numeric!r} is outside qualified range "
                f"[{binding.qualified_lower!r}, {binding.qualified_upper!r}]"
            )
    if applied and qualification == "baseline":
        qualification = "qualified"
    return (
        CompiledVariantResolution(
            inputs=applied,
            runtime_bindings=runtime_bindings,
            projections=projections,
            invalidations=tuple(sorted(invalidations)),
            resolution_policy=(
                "reject_invalid"
                if not resolution_policies
                else next(iter(resolution_policies))
                if len(resolution_policies) == 1
                else "mixed"
            ),
            qualification=qualification,
            qualification_findings=tuple(qualification_findings),
        ),
        values,
    )
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
        try:
            validate_value_space_value(
                parameter.value_space,
                supplied.value,
                context=f"{scope}.{parameter.id}",
            )
        except ValueError as error:
            raise VehicleCompositionError(
                "value-space-mismatch",
                str(error),
                field=f"{scope}.{parameter.id}",
            ) from error
        if parameter.options and supplied.value not in parameter.options:
            raise VehicleCompositionError(
                "option-mismatch",
                f"expected one of {list(parameter.options)!r}, received {supplied.value!r}",
                field=f"{scope}.{parameter.id}",
            )
        lower, upper = parameter.hard_bounds
        if lower is not None or upper is not None:
            if isinstance(supplied.value, bool) or not isinstance(supplied.value, int | float):
                raise VehicleCompositionError(
                    "bounds-type-mismatch",
                    "a bounded parameter requires a finite scalar",
                    field=f"{scope}.{parameter.id}",
                )
            numeric = float(supplied.value)
            if not math.isfinite(numeric) or (lower is not None and numeric < lower) or (upper is not None and numeric > upper):
                raise VehicleCompositionError(
                    "hard-bound-violation",
                    f"value {numeric!r} is outside [{lower!r}, {upper!r}]",
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
    "CompiledMissionGraph",
    "CompiledMissionGraphNode",
    "CompiledMissionTransition",
    "CompiledObservation",
    "CompositionValue",
    "DeclaredSensorChannelError",
    "DeclaredSensorSelection",
    "InitializationSelection",
    "MissionSelection",
    "MissionGraphNodeSelection",
    "MissionGraphSelection",
    "MissionTransitionSelection",
    "ObservationSelection",
    "ResolvedCompositionValue",
    "SegmentSelection",
    "VariantSelection",
    "CompiledVariantRuntimeBinding",
    "CompiledVariantResolution",
    "VehicleCompositionError",
    "VehicleCompositionRequest",
    "compile_vehicle_composition",
    "load_compiled_vehicle_composition",
    "load_vehicle_composition_request",
    "resolve_vehicle_composition_interface_contract",
]
