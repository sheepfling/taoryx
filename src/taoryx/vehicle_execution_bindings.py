"""Explicit execution bindings for immutable vehicle compositions.

The composition registry answers what a user may request.  This companion
catalog answers the separate operational question: which selected requests
currently have a source-owned batch runner or accepted-truth episode factory?
It intentionally has no family fallback.  A missing binding is actionable
onboarding work, not permission to run a neighboring vehicle model.
"""

from __future__ import annotations

from collections.abc import Iterable, Iterator, Mapping
from functools import lru_cache
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .fidelity_contracts import FidelityTier
from .vehicle_catalog_resources import vehicle_catalog_resources
from .vehicle_composition import CompiledVehicleComposition

if TYPE_CHECKING:
    from .plugins.discovery import PluginCatalog

# Materialized lazily by ``__getattr__`` only for compatibility consumers.
VEHICLE_EXECUTION_BINDINGS: Path
VEHICLE_EXECUTION_PARITY: Path

ExecutionOperation = Literal["batch", "episode"]
ExecutionBindingStatus = Literal["runnable", "planned"]
ExecutionMode = Literal[
    "closed_loop_controller",
    "source_native_autonomous",
    "native_autonomous",
    "source_history_replay",
    "source_scheduled_replay",
    "open_loop_witness",
    "local_direct_wrench_screen",
    "local_native_coordinate_lqi_screen",
    "source_surface_authority_screen",
    "passive_uncontrolled",
    "planned",
]
BatchEpisodeParityAvailability = Literal["registered", "not_registered", "not_available"]
BatchActionTraceDisposition = Literal[
    "emits_committed_interval_trace",
    "committed_interval_history_missing",
    "not_emitted",
    "not_applicable",
    "planned",
]


class VehicleExecutionBinding(BaseModel):
    """One exact composition-to-executable-factory declaration."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    mission: str = Field(min_length=1)
    fidelity: FidelityTier
    operation: ExecutionOperation
    status: ExecutionBindingStatus
    execution_mode: ExecutionMode
    factory_id: str | None = None
    batch_action_trace: BatchActionTraceDisposition
    description: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    blockers: tuple[str, ...] = ()

    @model_validator(mode="after")
    def validate_factory_contract(self) -> VehicleExecutionBinding:
        if self.status == "runnable" and self.factory_id is None:
            raise ValueError("a runnable execution binding requires factory_id")
        if self.status == "planned" and not self.blockers:
            raise ValueError("a planned execution binding requires at least one blocker")
        if self.status == "planned" and self.execution_mode != "planned":
            raise ValueError("a planned execution binding must declare execution_mode planned")
        if self.status == "runnable" and self.execution_mode == "planned":
            raise ValueError("a runnable execution binding cannot declare execution_mode planned")
        if self.status == "planned" and self.batch_action_trace != "planned":
            raise ValueError("a planned execution binding must declare planned batch action-trace support")
        if self.operation == "episode" and self.batch_action_trace != "not_applicable":
            raise ValueError("an episode execution binding must declare batch action tracing not_applicable")
        if self.operation == "batch" and self.status == "runnable" and self.batch_action_trace == "planned":
            raise ValueError("a runnable batch execution binding cannot declare planned batch action tracing")
        if (
            self.execution_mode
            in {
                "source_native_autonomous",
                "native_autonomous",
                "source_history_replay",
                "source_scheduled_replay",
                "passive_uncontrolled",
            }
            and self.operation != "batch"
        ):
            raise ValueError(f"execution_mode {self.execution_mode!r} supports batch execution only")
        if self.execution_mode == "local_direct_wrench_screen" and self.fidelity != "rigid_body_6dof_direct_wrench":
            raise ValueError("local_direct_wrench_screen requires rigid_body_6dof_direct_wrench fidelity")
        if self.execution_mode == "local_native_coordinate_lqi_screen" and self.operation != "batch":
            raise ValueError("local_native_coordinate_lqi_screen supports batch execution only")
        if self.execution_mode == "source_surface_authority_screen" and self.operation != "batch":
            raise ValueError("source_surface_authority_screen supports batch execution only")
        return self
        ####


####


class VehicleExecutionBindingCatalog(BaseModel):
    """Versioned execution-binding authority."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    bindings: tuple[VehicleExecutionBinding, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_unique_bindings(self) -> VehicleExecutionBindingCatalog:
        keys = tuple((item.family_id, item.mission, item.fidelity, item.operation) for item in self.bindings)
        duplicates = sorted({item for item in keys if keys.count(item) > 1})
        if duplicates:
            raise ValueError(f"execution binding catalog has duplicate keys: {duplicates}")
        return self
        ####


####


class VehicleBatchEpisodeParityBinding(BaseModel):
    """One exact, independently registered batch/episode parity witness."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    mission: str = Field(min_length=1)
    fidelity: FidelityTier
    batch_factory_id: str = Field(min_length=1)
    episode_factory_id: str = Field(min_length=1)
    adapter_id: str = Field(min_length=1)
    evidence_scope: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)


class VehicleBatchEpisodeParityCatalog(BaseModel):
    """Versioned registry of exact parity evidence; absence is meaningful."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_id: str = Field(alias="schema", min_length=1)
    version: int = Field(ge=1)
    description: str = Field(min_length=1)
    bindings: tuple[VehicleBatchEpisodeParityBinding, ...] = ()

    @model_validator(mode="after")
    def validate_unique_bindings(self) -> VehicleBatchEpisodeParityCatalog:
        keys = tuple((item.family_id, item.mission, item.fidelity) for item in self.bindings)
        duplicates = sorted({item for item in keys if keys.count(item) > 1})
        if duplicates:
            raise ValueError(f"batch/episode parity catalog has duplicate keys: {duplicates}")
        return self
        ####


####


class VehicleBatchEpisodeParityAdvertisement(BaseModel, Mapping[str, object]):
    """Typed non-promotional status of one exact batch/episode pair.

    This is intentionally separate from the checked-in witness binding.  It
    represents the result of joining the execution and parity catalogues, so
    discovery and authoring clients can retain a validated object until their
    final JSON serialization boundary.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    family_id: str = Field(min_length=1)
    mission: str = Field(min_length=1)
    fidelity: FidelityTier
    availability: BatchEpisodeParityAvailability
    runnable_operations: tuple[ExecutionOperation, ...] = ()
    reason: str | None = None
    adapter_id: str | None = None
    evidence_scope: str | None = None
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_advertisement(self) -> VehicleBatchEpisodeParityAdvertisement:
        if len(set(self.runnable_operations)) != len(self.runnable_operations):
            raise ValueError("batch/episode parity advertisement has duplicate runnable operations")
        complete_pair = set(self.runnable_operations) == {"batch", "episode"}
        if self.availability == "registered":
            if not complete_pair:
                raise ValueError("registered batch/episode parity requires runnable batch and episode operations")
            if self.adapter_id is None or self.evidence_scope is None:
                raise ValueError("registered batch/episode parity requires adapter and evidence scope")
            if self.reason is not None:
                raise ValueError("registered batch/episode parity cannot retain an unavailable reason")
        else:
            if self.reason is None:
                raise ValueError("unregistered batch/episode parity requires an explicit reason")
            if self.adapter_id is not None or self.evidence_scope is not None:
                raise ValueError("unregistered batch/episode parity cannot advertise witness metadata")
            if self.availability == "not_registered" and not complete_pair:
                raise ValueError("not_registered batch/episode parity requires runnable batch and episode operations")
            if self.availability == "not_available" and complete_pair:
                raise ValueError("not_available batch/episode parity must not hide a complete runnable pair")
        return self
        ####

    def as_dict(self) -> dict[str, object]:
        """Serialize the compatibility/public projection without null placeholders."""

        return self.model_dump(mode="json", exclude_none=True)
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


class VehicleExecutionBindingError(ValueError):
    """Fail-closed diagnostic for unavailable composition execution."""

    def __init__(
        self,
        composition: CompiledVehicleComposition,
        operation: ExecutionOperation,
        reason: str,
    ) -> None:
        self.composition = composition
        self.operation = operation
        self.reason = reason
        super().__init__(f"no {operation} execution binding for {composition.family_id!r}/{composition.mission!r}/{composition.fidelity!r}: {reason}")
        ####

    ####


def load_vehicle_execution_binding_catalog(
    path: str | Path | None = None,
    *,
    plugins: PluginCatalog | None = None,
) -> VehicleExecutionBindingCatalog:
    """Load execution bindings from one explicit plug-in resource scope."""

    if path is not None:
        return _load_vehicle_execution_binding_catalog_sources((str(Path(path)),))
    sources = vehicle_catalog_resources("verification/vehicle_execution_bindings.yaml", plugins=plugins)
    return _load_vehicle_execution_binding_catalog_sources(tuple(str(source) for source in sources))
    ####


@lru_cache(maxsize=32)
def _load_vehicle_execution_binding_catalog_sources(
    source_names: tuple[str, ...],
) -> VehicleExecutionBindingCatalog:
    """Parse one cacheable, exact set of execution-binding fragments."""

    payloads = _catalog_payloads_from_source_names(source_names)
    merged = _merge_list_catalog(payloads, "bindings")
    return VehicleExecutionBindingCatalog.model_validate(merged)
    ####


def load_vehicle_batch_episode_parity_catalog(
    path: str | Path | None = None,
    *,
    plugins: PluginCatalog | None = None,
) -> VehicleBatchEpisodeParityCatalog:
    """Load parity declarations from one explicit plug-in resource scope."""

    if path is not None:
        return _load_vehicle_batch_episode_parity_catalog_sources((str(Path(path)),))
    sources = vehicle_catalog_resources("verification/vehicle_execution_parity.yaml", plugins=plugins)
    return _load_vehicle_batch_episode_parity_catalog_sources(tuple(str(source) for source in sources))
    ####


@lru_cache(maxsize=32)
def _load_vehicle_batch_episode_parity_catalog_sources(
    source_names: tuple[str, ...],
) -> VehicleBatchEpisodeParityCatalog:
    """Parse one cacheable, exact set of parity-declaration fragments."""

    payloads = _catalog_payloads_from_source_names(source_names)
    merged = _merge_list_catalog(payloads, "bindings")
    return VehicleBatchEpisodeParityCatalog.model_validate(merged)
    ####


def _catalog_payloads_from_source_names(source_names: tuple[str, ...]) -> list[Mapping[str, object]]:
    """Read a fixed resource-path set without widening its owning scope."""

    if not source_names:
        raise ValueError("vehicle execution catalog has no installed fragments")
    payloads: list[Mapping[str, object]] = []
    for source_name in source_names:
        source = Path(source_name)
        payload = yaml.safe_load(source.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError(f"{source} must contain a mapping")
        payloads.append(payload)
    return payloads
    ####


def _merge_list_catalog(payloads: list[Mapping[str, object]], key: str) -> dict[str, object]:
    """Join same-schema list fragments before typed validation."""

    merged = dict(payloads[0])
    rows: list[object] = []
    for payload in payloads:
        value = payload.get(key)
        if not isinstance(value, list):
            raise ValueError(f"vehicle execution catalog {key!r} must contain a list")
        rows.extend(value)
    merged[key] = rows
    return merged
    ####
    ####


def bindings_for_family(
    family_id: str,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> tuple[VehicleExecutionBinding, ...]:
    """Return all explicit runnable and planned bindings for one family."""

    selected = catalog or load_vehicle_execution_binding_catalog(plugins=plugins)
    return tuple(item for item in selected.bindings if item.family_id == family_id)
    ####


def resolve_vehicle_execution_binding(
    composition: CompiledVehicleComposition,
    operation: ExecutionOperation,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> VehicleExecutionBinding:
    """Resolve an exact runnable factory binding or return its declared gap."""

    selected = catalog or load_vehicle_execution_binding_catalog(plugins=plugins)
    matches = tuple(
        item
        for item in selected.bindings
        if (
            item.family_id == composition.family_id
            and item.mission == composition.mission
            and item.fidelity == composition.fidelity
            and item.operation == operation
        )
    )
    if not matches:
        raise VehicleExecutionBindingError(composition, operation, "the catalog has no binding for this exact composition")
    binding = matches[0]
    if binding.status != "runnable":
        raise VehicleExecutionBindingError(composition, operation, "; ".join(binding.blockers))
    return binding
    ####


def execution_binding_records(
    family_id: str,
    *,
    catalog: VehicleExecutionBindingCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> list[dict[str, object]]:
    """Serialize bindings for discovery without constructing any runtime."""

    return [item.model_dump(mode="json") for item in bindings_for_family(family_id, catalog=catalog, plugins=plugins)]
    ####


def resolve_batch_episode_parity_advertisement(
    family_id: str,
    mission: str,
    fidelity: FidelityTier,
    *,
    execution_catalog: VehicleExecutionBindingCatalog | None = None,
    parity_catalog: VehicleBatchEpisodeParityCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> VehicleBatchEpisodeParityAdvertisement:
    """Resolve typed parity status for one exact composition.

    This is intentionally narrower than runnable operations. It never infers
    equivalence from shared source code, matching factory names, or two green
    runs. ``registered`` means a named witness replays a declared action trace
    through both paths; all other states are explicitly non-promotional.
    """

    selected_execution = execution_catalog or load_vehicle_execution_binding_catalog(plugins=plugins)
    selected_parity = parity_catalog or load_vehicle_batch_episode_parity_catalog(plugins=plugins)
    matching = tuple(item for item in selected_execution.bindings if item.family_id == family_id and item.mission == mission and item.fidelity == fidelity)
    runnable = {item.operation: item for item in matching if item.status == "runnable" and item.factory_id is not None}
    required = {"batch", "episode"}
    if set(runnable) != required:
        return VehicleBatchEpisodeParityAdvertisement(
            family_id=family_id,
            mission=mission,
            fidelity=fidelity,
            availability="not_available",
            runnable_operations=tuple(sorted(runnable)),
            reason="both runnable batch and episode bindings are required before parity can be witnessed",
            claim_boundary=(
                "No batch/episode equivalence is claimed. A missing runtime operation is an execution capability gap, "
                "not permission to substitute another family or fidelity."
            ),
        )
    witnesses = tuple(item for item in selected_parity.bindings if item.family_id == family_id and item.mission == mission and item.fidelity == fidelity)
    if not witnesses:
        return VehicleBatchEpisodeParityAdvertisement(
            family_id=family_id,
            mission=mission,
            fidelity=fidelity,
            availability="not_registered",
            runnable_operations=tuple(sorted(runnable)),
            reason="both paths are runnable but no exact committed-boundary parity witness is registered",
            claim_boundary=("Runnable batch and episode paths do not establish equivalent state, status, event, or sensor behavior."),
        )
    witness = witnesses[0]
    if witness.batch_factory_id != runnable["batch"].factory_id or witness.episode_factory_id != runnable["episode"].factory_id:
        raise ValueError(f"batch/episode parity witness factory IDs disagree with runnable execution bindings: {family_id}/{mission}/{fidelity}")
    return VehicleBatchEpisodeParityAdvertisement(
        family_id=family_id,
        mission=mission,
        fidelity=fidelity,
        availability="registered",
        runnable_operations=tuple(sorted(runnable)),
        adapter_id=witness.adapter_id,
        evidence_scope=witness.evidence_scope,
        claim_boundary=witness.claim_boundary,
    )
    ####


def batch_episode_parity_record(
    family_id: str,
    mission: str,
    fidelity: FidelityTier,
    *,
    execution_catalog: VehicleExecutionBindingCatalog | None = None,
    parity_catalog: VehicleBatchEpisodeParityCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> VehicleBatchEpisodeParityAdvertisement:
    """Return the typed parity advertisement at the producer/consumer seam."""

    return resolve_batch_episode_parity_advertisement(
        family_id,
        mission,
        fidelity,
        execution_catalog=execution_catalog,
        parity_catalog=parity_catalog,
        plugins=plugins,
    )
    ####


def batch_episode_parity_records(
    family_id: str,
    *,
    execution_catalog: VehicleExecutionBindingCatalog | None = None,
    parity_catalog: VehicleBatchEpisodeParityCatalog | None = None,
    plugins: PluginCatalog | None = None,
) -> list[VehicleBatchEpisodeParityAdvertisement]:
    """Return typed non-inferred parity state for every declared execution tuple."""

    selected_execution = execution_catalog or load_vehicle_execution_binding_catalog(plugins=plugins)
    keys = sorted({(item.mission, item.fidelity) for item in selected_execution.bindings if item.family_id == family_id})
    return [
        batch_episode_parity_record(
            family_id,
            mission,
            fidelity,
            execution_catalog=selected_execution,
            parity_catalog=parity_catalog,
            plugins=plugins,
        )
        for mission, fidelity in keys
    ]
    ####


def validate_batch_episode_parity_bindings(
    parity_bindings: Iterable[VehicleBatchEpisodeParityBinding],
    *,
    execution_bindings: Iterable[VehicleExecutionBinding],
) -> tuple[str, ...]:
    """Reject stale parity claims against the authoritative factory bindings."""

    indexed = {(item.family_id, item.mission, item.fidelity, item.operation): item for item in execution_bindings}
    errors: list[str] = []
    for parity in parity_bindings:
        batch = indexed.get((parity.family_id, parity.mission, parity.fidelity, "batch"))
        episode = indexed.get((parity.family_id, parity.mission, parity.fidelity, "episode"))
        label = f"{parity.family_id}/{parity.mission}/{parity.fidelity}"
        if batch is None or episode is None:
            errors.append(f"batch/episode parity binding has no paired execution bindings: {label}")
            continue
        if batch.status != "runnable" or episode.status != "runnable":
            errors.append(f"batch/episode parity binding requires two runnable operations: {label}")
            continue
        if batch.factory_id != parity.batch_factory_id or episode.factory_id != parity.episode_factory_id:
            errors.append(f"batch/episode parity factory mismatch: {label}")
    return tuple(errors)
    ####


def validate_execution_bindings(
    bindings: Iterable[VehicleExecutionBinding],
    *,
    declarations: Mapping[str, object],
) -> tuple[str, ...]:
    """Check bindings against a compact family/mission/fidelity declaration.

    ``declarations`` uses the simple structure emitted by the composition
    registry: ``family_id -> {mission_id: supported_fidelity_names}``.  Keeping
    this validator independent prevents a registry/catalog import cycle while
    still making stale execution declarations fail in tests and release checks.
    """

    errors: list[str] = []
    for binding in bindings:
        family = declarations.get(binding.family_id)
        if not isinstance(family, Mapping):
            errors.append(f"unknown execution-binding family: {binding.family_id}")
            continue
        supported = family.get(binding.mission)
        if not isinstance(supported, set | frozenset | tuple | list):
            errors.append(f"unknown execution-binding mission: {binding.family_id}/{binding.mission}")
            continue
        if binding.fidelity not in supported:
            errors.append(f"execution binding declares unsupported fidelity: {binding.family_id}/{binding.mission}/{binding.fidelity}")
    return tuple(errors)
    ####


def __getattr__(name: str) -> object:
    """Resolve historical aggregate path constants only on explicit access."""

    relative_paths = {
        "VEHICLE_EXECUTION_BINDINGS": "verification/vehicle_execution_bindings.yaml",
        "VEHICLE_EXECUTION_PARITY": "verification/vehicle_execution_parity.yaml",
    }
    try:
        relative_path = relative_paths[name]
    except KeyError as error:
        raise AttributeError(name) from error
    from .compatibility.vehicle_catalog_resources import legacy_vehicle_catalog_resource

    value = legacy_vehicle_catalog_resource(relative_path)
    globals()[name] = value
    return value
    ####


__all__ = [
    "BatchActionTraceDisposition",
    "BatchEpisodeParityAvailability",
    "ExecutionBindingStatus",
    "ExecutionMode",
    "ExecutionOperation",
    "VEHICLE_EXECUTION_BINDINGS",
    "VEHICLE_EXECUTION_PARITY",
    "VehicleBatchEpisodeParityBinding",
    "VehicleBatchEpisodeParityAdvertisement",
    "VehicleBatchEpisodeParityCatalog",
    "VehicleExecutionBinding",
    "VehicleExecutionBindingCatalog",
    "VehicleExecutionBindingError",
    "batch_episode_parity_record",
    "batch_episode_parity_records",
    "bindings_for_family",
    "execution_binding_records",
    "load_vehicle_batch_episode_parity_catalog",
    "load_vehicle_execution_binding_catalog",
    "resolve_vehicle_execution_binding",
    "resolve_batch_episode_parity_advertisement",
    "validate_batch_episode_parity_bindings",
    "validate_execution_bindings",
]
