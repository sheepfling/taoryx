"""Project Simulation Runtime artifacts into the Mission Composition result."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..output_catalog import output_channel_spec
from ..outputs import RunArtifact, VehicleTelemetry
from .configuration_contract import TrajectoryOutputChannelMetadata, TrajectoryOutputSchema
from .execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionOutputSelection,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEntityRelationship,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectoryObjectStatus,
    TrajectoryOutcome,
    TrajectorySample,
    TrajectorySegmentResult,
    TrajectoryStateSnapshot,
    resolve_output_selection,
)


class RuntimeArtifactProjection(BaseModel):
    """Identity and evidence context not carried by a generic runtime artifact."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    request_id: str = Field(min_length=1)
    configuration_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    primary_model_id: str = Field(min_length=1)
    primary_object_id: str = Field(min_length=1)
    default_fidelity: str = Field(min_length=1)
    default_realization_id: str | None = None
    operation: Literal["batch", "step"] = "batch"
    status: TrajectoryOutcome
    model_ids: dict[str, str] = Field(default_factory=dict)
    realization_ids: dict[str, str] = Field(default_factory=dict)
    fidelities: dict[str, str] = Field(default_factory=dict)
    roles: dict[str, str] = Field(default_factory=dict)
    deployment_ids_by_event: dict[str, str] = Field(default_factory=dict)
    object_statuses: dict[str, TrajectoryObjectStatus] = Field(default_factory=dict)
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()
    provenance: str = "Simulation Runtime RunArtifact"
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_status(self) -> RuntimeArtifactProjection:
        has_error = any(item.severity == "error" for item in self.diagnostics)
        if self.status in {"partial", "failed"} and not has_error:
            raise ValueError("partial or failed runtime projection requires an error diagnostic")
        if self.status in {"in_progress", "completed", "terminated"} and has_error:
            raise ValueError("successful runtime projection cannot contain error diagnostics")
        return self
        ####

    ####


def trajectory_result_from_run_artifact(
    artifact: RunArtifact,
    projection: RuntimeArtifactProjection,
    *,
    output_schema: TrajectoryOutputSchema | None = None,
    output_selection: MissionCompositionOutputSelection | None = None,
) -> MissionCompositionTrajectoryResult:
    """Convert every runtime vehicle history, including spawned children."""

    if (output_schema is None) != (output_selection is None):
        raise ValueError("runtime artifact projection requires both output_schema and output_selection, or neither")
    if projection.primary_object_id not in artifact.vehicles:
        raise ValueError(f"primary object {projection.primary_object_id!r} is absent from the runtime artifact")
    selected_primary_channels = (
        resolve_output_selection(
            output_schema,
            output_selection,
            fidelity=projection.fidelities.get(projection.primary_object_id, projection.default_fidelity),
            operation=projection.operation,
        )
        if output_schema is not None and output_selection is not None
        else None
    )
    advertised_core_ids = {item.id for item in output_schema.core_channels} if output_schema is not None else set()
    primary_core_ids = (
        {item.id for item in selected_primary_channels if item.id in advertised_core_ids}
        if selected_primary_channels is not None
        else set()
    )
    primary_telemetry_group_by_channel = (
        {
            channel_id: group.id
            for group in output_schema.telemetry_groups
            for channel_id in group.channel_ids
        }
        if output_schema is not None
        else {}
    )
    model_to_object = _model_to_object(artifact, projection)
    events, spawn_by_child, event_diagnostics = _artifact_events(artifact, projection)
    included_object_ids = _selected_object_ids(artifact, projection, output_selection)
    spawn_by_child = {
        child_id: event
        for child_id, event in spawn_by_child.items()
        if child_id in included_object_ids and event.parent_object_id in included_object_ids
    }
    lifecycle_event_ids = {item.id for item in spawn_by_child.values()}
    events = tuple(
        item
        for item in events
        if item.object_id in included_object_ids
        and (item.parent_object_id is None or item.parent_object_id in included_object_ids)
        and (
            output_selection is None
            or output_selection.include_events
            or item.id in lifecycle_event_ids
        )
    )
    diagnostics = [*projection.diagnostics, *event_diagnostics]
    objects: list[TrajectoryObject] = []
    for object_id, telemetry in artifact.vehicles.items():
        if object_id not in included_object_ids:
            continue
        _validate_telemetry(object_id, telemetry)
        channels, channel_diagnostics = _channels(
            object_id,
            telemetry,
            projection,
            selected=selected_primary_channels if object_id == projection.primary_object_id else None,
            core_ids=primary_core_ids if object_id == projection.primary_object_id else set(),
            telemetry_group_by_channel=(
                primary_telemetry_group_by_channel if object_id == projection.primary_object_id else {}
            ),
        )
        diagnostics.extend(channel_diagnostics)
        parent_object_id = _parent_object_id(telemetry, model_to_object)
        spawn = spawn_by_child.get(object_id)
        if parent_object_id is not None and spawn is None:
            raise ValueError(f"runtime child {object_id!r} has no committed spawn event")
        object_status = _object_status(object_id, projection)
        active_to_s = None if object_status == "active" else telemetry.times[-1]
        segments = (
            _segments(object_id, telemetry, object_status)
            if output_selection is None or output_selection.include_segments
            else ()
        )
        objects.append(
            TrajectoryObject(
                object_id=object_id,
                model_id=projection.model_ids.get(object_id) or telemetry.model_id or object_id,
                realization_id=(
                    projection.realization_ids.get(object_id)
                    or projection.default_realization_id
                    or projection.fidelities.get(object_id, projection.default_fidelity)
                ),
                name=telemetry.name,
                role=projection.roles.get(object_id, "deployed_child" if parent_object_id is not None else "mission_participant"),
                fidelity=projection.fidelities.get(object_id, projection.default_fidelity),
                status=object_status,
                parent_object_id=parent_object_id,
                deployment_id=spawn.deployment_id if spawn is not None else None,
                spawn_event_id=spawn.id if spawn is not None else None,
                active_from_s=telemetry.times[0],
                active_to_s=active_to_s,
                terminal_disposition=None if object_status == "active" else object_status,
                channels=channels,
                samples=_samples(telemetry, channels),
                segments=segments,
                provenance=projection.provenance,
                claim_boundary=projection.claim_boundary,
            )
        )
    object_by_id = {item.object_id: item for item in objects}
    relationships = tuple(
        _spawn_relationship(object_by_id[child_id], event, projection)
        for child_id, event in sorted(spawn_by_child.items())
    )
    return MissionCompositionTrajectoryResult(
        provider_id=projection.provider_id,
        provider_version=projection.provider_version,
        request_id=projection.request_id,
        configuration_fingerprint=projection.configuration_fingerprint,
        primary_model_id=projection.primary_model_id,
        primary_object_id=projection.primary_object_id,
        status=projection.status,
        objects=tuple(objects),
        events=events,
        relationships=relationships,
        diagnostics=tuple(diagnostics),
        claim_boundary=projection.claim_boundary,
    )
    ####


def _selected_object_ids(
    artifact: RunArtifact,
    projection: RuntimeArtifactProjection,
    selection: MissionCompositionOutputSelection | None,
) -> set[str]:
    if selection is None:
        return set(artifact.vehicles)
    selected = [projection.primary_object_id]
    if selection.include_spawned_objects:
        remaining = [item for item in artifact.vehicles if item != projection.primary_object_id]
        while remaining:
            accepted = False
            for object_id in tuple(remaining):
                parent_model_id = artifact.vehicles[object_id].parent_model_id
                parent_object_id = None if parent_model_id is None else _object_for_model_id(artifact, projection, parent_model_id)
                if parent_object_id is None or parent_object_id in selected:
                    selected.append(object_id)
                    remaining.remove(object_id)
                    accepted = True
            if not accepted:
                raise ValueError("runtime artifact contains an unresolved spawned-entity graph")
    if selection.maximum_objects is not None:
        selected = selected[: selection.maximum_objects]
    return set(selected)
    ####


def _object_for_model_id(
    artifact: RunArtifact,
    projection: RuntimeArtifactProjection,
    model_id: str,
) -> str | None:
    matches = [
        object_id
        for object_id, telemetry in artifact.vehicles.items()
        if (projection.model_ids.get(object_id) or telemetry.model_id) == model_id
    ]
    if len(matches) > 1:
        raise ValueError(f"runtime artifact model_id {model_id!r} is not unique")
    return matches[0] if matches else None
    ####


def _validate_telemetry(object_id: str, telemetry: VehicleTelemetry) -> None:
    if not telemetry.times:
        raise ValueError(f"runtime object {object_id!r} has no accepted samples")
    if any(not math.isfinite(item) or item < 0.0 for item in telemetry.times):
        raise ValueError(f"runtime object {object_id!r} has invalid accepted times")
    if any(later < earlier for earlier, later in zip(telemetry.times, telemetry.times[1:], strict=False)):
        raise ValueError(f"runtime object {object_id!r} accepted times are not monotonic")
    ####


def _model_to_object(artifact: RunArtifact, projection: RuntimeArtifactProjection) -> dict[str, str]:
    result: dict[str, str] = {}
    for object_id, telemetry in artifact.vehicles.items():
        model_id = projection.model_ids.get(object_id) or telemetry.model_id
        if model_id is None:
            continue
        if model_id in result:
            raise ValueError(f"runtime artifact repeats model_id {model_id!r}; supply unambiguous object lineage")
        result[model_id] = object_id
    return result
    ####


def _parent_object_id(telemetry: VehicleTelemetry, model_to_object: Mapping[str, str]) -> str | None:
    if telemetry.parent_model_id is None:
        return None
    try:
        return model_to_object[telemetry.parent_model_id]
    except KeyError as error:
        raise ValueError(
            f"runtime object {telemetry.vehicle_id!r} references unknown parent model {telemetry.parent_model_id!r}"
        ) from error
    ####


def _object_status(object_id: str, projection: RuntimeArtifactProjection) -> TrajectoryObjectStatus:
    explicit = projection.object_statuses.get(object_id)
    if explicit is not None:
        return explicit
    statuses: dict[TrajectoryOutcome, TrajectoryObjectStatus] = {
        "in_progress": "active",
        "completed": "completed",
        "terminated": "terminated",
        "partial": "failed",
        "failed": "failed",
    }
    return statuses[projection.status]
    ####


def _channels(
    object_id: str,
    telemetry: VehicleTelemetry,
    projection: RuntimeArtifactProjection,
    *,
    selected: tuple[TrajectoryOutputChannelMetadata, ...] | None,
    core_ids: set[str],
    telemetry_group_by_channel: Mapping[str, str],
) -> tuple[tuple[TrajectoryChannelMetadata, ...], tuple[MissionCompositionDiagnostic, ...]]:
    result: list[TrajectoryChannelMetadata] = []
    diagnostics: list[MissionCompositionDiagnostic] = []
    selected_by_id = {item.id: item for item in selected or ()}
    if selected is not None:
        missing_core = sorted(core_ids - set(telemetry.channels))
        if missing_core:
            raise ValueError(f"runtime object {object_id!r} is missing requested core channels {missing_core!r}")
        missing_telemetry = sorted(set(selected_by_id) - core_ids - set(telemetry.channels))
        for channel_id in missing_telemetry:
            diagnostics.append(
                MissionCompositionDiagnostic(
                    severity="warning",
                    code="conditional-telemetry-unavailable",
                    message=f"Requested telemetry channel {channel_id!r} was not emitted by the runtime.",
                    phase="projection",
                    recoverability="degraded",
                    provider_id=projection.provider_id,
                    model_id=projection.model_ids.get(object_id) or telemetry.model_id,
                    object_id=object_id,
                    details={"channel_id": channel_id},
                )
            )
    for channel_id, channel in telemetry.channels.items():
        if selected is not None and channel_id not in selected_by_id:
            continue
        if any(value is None or not math.isfinite(value) for value in channel.values):
            diagnostics.append(
                MissionCompositionDiagnostic(
                    severity="warning",
                    code="sparse-channel-omitted",
                    message=f"Channel {channel_id!r} contains null or non-finite values and was omitted.",
                    phase="projection",
                    recoverability="degraded",
                    provider_id=projection.provider_id,
                    model_id=projection.model_ids.get(object_id) or telemetry.model_id,
                    object_id=object_id,
                    details={"channel_id": channel_id},
                )
            )
            continue
        spec = output_channel_spec(channel.source_name)
        interpolation = "periodic" if channel.interpolation == "angle" else channel.interpolation
        result.append(
            TrajectoryChannelMetadata(
                id=channel_id,
                channel_class=(
                    "core_state"
                    if channel_id in core_ids or (selected is None and _runtime_channel_is_core(channel_id))
                    else "telemetry"
                ),
                telemetry_group=(
                    None
                    if channel_id in core_ids or (selected is None and _runtime_channel_is_core(channel_id))
                    else telemetry_group_by_channel.get(channel_id, _runtime_telemetry_group(channel_id))
                ),
                quantity=(selected_by_id[channel_id].quantity if channel_id in selected_by_id else None)
                or (spec.quantity if spec is not None else None),
                unit=channel.unit,
                data_type=selected_by_id[channel_id].data_type if channel_id in selected_by_id else "float64",
                shape=selected_by_id[channel_id].shape if channel_id in selected_by_id else (),
                frame=selected_by_id[channel_id].frame if channel_id in selected_by_id else None,
                sampling_semantics=(
                    selected_by_id[channel_id].sampling_semantics
                    if channel_id in selected_by_id
                    else "continuous_sample"
                ),
                interpolation=interpolation,
                description=(selected_by_id[channel_id].description if channel_id in selected_by_id else None)
                or f"Runtime channel projected from {channel.source_name!r}.",
            )
        )
    if not result:
        raise ValueError(f"runtime object {object_id!r} has no complete finite numeric channels")
    return tuple(result), tuple(diagnostics)
    ####


def _runtime_channel_is_core(channel_id: str) -> bool:
    return channel_id.startswith(("position.", "velocity.", "attitude.", "angular_rate."))
    ####


def _runtime_telemetry_group(channel_id: str) -> str:
    prefix = channel_id.split(".", maxsplit=1)[0]
    return {
        "mass": "resources",
        "propulsion": "propulsion",
        "aerodynamics": "aerodynamics",
        "guidance": "guidance",
        "control": "controls",
        "actuator": "actuators",
        "phase": "stage",
    }.get(prefix, "model_specific")
    ####


def _samples(
    telemetry: VehicleTelemetry,
    channels: tuple[TrajectoryChannelMetadata, ...],
) -> tuple[TrajectorySample, ...]:
    channel_ids = tuple(item.id for item in channels)
    return tuple(
        TrajectorySample(
            time_s=time_s,
            values={channel_id: _required_float(telemetry.channels[channel_id].values[index]) for channel_id in channel_ids},
            segment_instance_id=_segment_at_time(telemetry, time_s),
        )
        for index, time_s in enumerate(telemetry.times)
    )
    ####


def _required_float(value: float | None) -> float:
    if value is None:
        raise ValueError("a selected runtime channel contains a null sample")
    return float(value)
    ####


def _spawn_relationship(
    child: TrajectoryObject,
    event: TrajectoryEvent,
    projection: RuntimeArtifactProjection,
) -> TrajectoryEntityRelationship:
    if child.parent_object_id is None or child.deployment_id is None or child.spawn_event_id is None:
        raise ValueError(f"runtime child {child.object_id!r} lacks complete spawn lineage")
    if event.category not in {"release", "separation", "deployment"}:
        raise ValueError(f"runtime child {child.object_id!r} has a non-spawn lifecycle event")
    return TrajectoryEntityRelationship(
        id=f"relationship-{event.id}",
        kind=event.category,
        parent_object_id=child.parent_object_id,
        child_object_id=child.object_id,
        deployment_id=child.deployment_id,
        event_id=child.spawn_event_id,
        created_at_s=child.active_from_s,
        state_transfer=str(event.data.get("state_transfer") or "provider_defined_at_accepted_boundary"),
        initial_state=TrajectoryStateSnapshot(
            time_s=child.active_from_s,
            values=child.samples[0].values,
        ),
        provenance=projection.provenance,
    )
    ####


def _segments(
    object_id: str,
    telemetry: VehicleTelemetry,
    object_status: TrajectoryObjectStatus,
) -> tuple[TrajectorySegmentResult, ...]:
    return tuple(
        TrajectorySegmentResult(
            id=f"segment-{item.number}",
            instance_id=f"{object_id}:segment-{item.number}",
            object_id=object_id,
            start_time_s=item.start_time,
            end_time_s=item.end_time,
            status="terminated" if object_status in {"terminated", "failed"} and index == len(telemetry.segments) - 1 else "completed",
        )
        for index, item in enumerate(telemetry.segments)
    )
    ####


def _segment_at_time(telemetry: VehicleTelemetry, time_s: float) -> str | None:
    matches = [item for item in telemetry.segments if item.start_time - 1.0e-9 <= time_s <= item.end_time + 1.0e-9]
    if not matches:
        return None
    selected = matches[-1]
    return f"{telemetry.vehicle_id}:segment-{selected.number}"
    ####


def _artifact_events(
    artifact: RunArtifact,
    projection: RuntimeArtifactProjection,
) -> tuple[
    tuple[TrajectoryEvent, ...],
    dict[str, TrajectoryEvent],
    tuple[MissionCompositionDiagnostic, ...],
]:
    events: list[TrajectoryEvent] = []
    spawn_by_child: dict[str, TrajectoryEvent] = {}
    diagnostics: list[MissionCompositionDiagnostic] = []
    for event_index, payload in enumerate(artifact.events, start=1):
        parent = _string(payload.get("vehicle"))
        event_id = _string(payload.get("event_id")) or _string(payload.get("name")) or f"runtime-event-{event_index}"
        time_s = _finite_time(payload.get("time"), event_id)
        if payload.get("action") == "spawn" and payload.get("status") == "committed":
            children = _string_list(payload.get("child_names"))
            deployment_id = projection.deployment_ids_by_event.get(event_id, event_id)
            for child_index, child in enumerate(children, start=1):
                stable_id = f"event-{event_index:04d}-{_slug(event_id)}-{child_index:02d}"
                spawn_event = TrajectoryEvent(
                    id=stable_id,
                    time_s=time_s,
                    category=_deployment_category(event_id),
                    kind=_string(payload.get("name")) or event_id,
                    object_id=child,
                    parent_object_id=parent,
                    deployment_id=deployment_id,
                    detail="Committed accepted-boundary child deployment.",
                    data={
                        "source": payload.get("source"),
                        "signal": payload.get("signal"),
                        "status": payload.get("status"),
                    },
                )
                if child in spawn_by_child:
                    raise ValueError(f"runtime child {child!r} has multiple committed spawn events")
                spawn_by_child[child] = spawn_event
                events.append(spawn_event)
            continue
        if parent is None or parent not in artifact.vehicles:
            diagnostics.append(
                MissionCompositionDiagnostic(
                    severity="warning",
                    code="unscoped-runtime-event-omitted",
                    message=f"Runtime event {event_id!r} does not identify a returned object and was omitted.",
                    phase="projection",
                    recoverability="degraded",
                    provider_id=projection.provider_id,
                    details={"event_id": event_id},
                )
            )
            continue
        events.append(
            TrajectoryEvent(
                id=f"event-{event_index:04d}-{_slug(event_id)}",
                time_s=time_s,
                category=_event_category(event_id, payload),
                kind=_string(payload.get("name")) or event_id,
                object_id=parent,
                segment_instance_id=_runtime_event_segment(parent, payload),
                detail=_string(payload.get("status")) or "Runtime event.",
                data={
                    "action": payload.get("action"),
                    "signal": payload.get("signal"),
                    "source": payload.get("source"),
                },
            )
        )
    events.sort(key=lambda item: (item.time_s, item.id))
    return tuple(events), spawn_by_child, tuple(diagnostics)
    ####


def _runtime_event_segment(object_id: str, payload: Mapping[str, object]) -> str | None:
    raw = payload.get("segment_to", payload.get("segment_from"))
    if isinstance(raw, bool) or not isinstance(raw, int | float):
        return None
    return f"{object_id}:segment-{int(raw)}"
    ####


def _event_category(event_id: str, payload: Mapping[str, object]) -> Literal[
    "segment",
    "burnout",
    "release",
    "separation",
    "deployment",
    "impact",
    "termination",
    "resource",
    "custom",
]:
    lowered = event_id.casefold()
    if "impact" in lowered:
        return "impact"
    if "burnout" in lowered or "cutoff" in lowered:
        return "burnout"
    if "separation" in lowered:
        return "separation"
    if "release" in lowered:
        return "release"
    if "deploy" in lowered or payload.get("action") == "spawn":
        return "deployment"
    if "segment" in lowered:
        return "segment"
    if "stop" in lowered or payload.get("action") == "stop":
        return "termination"
    return "custom"
    ####


def _deployment_category(event_id: str) -> Literal["release", "separation", "deployment"]:
    lowered = event_id.casefold()
    if "separation" in lowered:
        return "separation"
    if "release" in lowered:
        return "release"
    return "deployment"
    ####


def _finite_time(value: object, event_id: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float) or not math.isfinite(float(value)):
        raise ValueError(f"runtime event {event_id!r} has invalid time {value!r}")
    return float(value)
    ####


def _string(value: object) -> str | None:
    return value if isinstance(value, str) and value else None
    ####


def _string_list(value: object) -> tuple[str, ...]:
    if not isinstance(value, list):
        return ()
    return tuple(item for item in value if isinstance(item, str) and item)
    ####


def _slug(value: str) -> str:
    normalized = "".join(character.lower() if character.isalnum() else "-" for character in value)
    return "-".join(part for part in normalized.split("-") if part) or "runtime-event"
    ####


__all__ = ["RuntimeArtifactProjection", "trajectory_result_from_run_artifact"]
