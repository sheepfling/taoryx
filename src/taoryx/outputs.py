"""Output-evaluation and structured telemetry contracts."""

from __future__ import annotations

import csv
import json
import math
import sqlite3
from collections.abc import Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from io import StringIO
from pathlib import Path
from typing import TYPE_CHECKING, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

from taoryx.output_catalog import canonical_output_name, output_channel_spec

if TYPE_CHECKING:
    from taoryx.runtime.common import RuntimeProblem
    from taoryx.runtime.engine import ExecutionResult


InterpolationKind = Literal["linear", "angle", "step", "slerp", "event"]


class VehicleKind(StrEnum):
    """Vehicle-family hint used by visualization profiles."""

    GENERIC = "generic"
    ROCKET = "rocket"
    GLIDER = "glider"
    AIRBREATHER = "airbreather"


class DynamicsKind(StrEnum):
    """State interpretation, independent of vehicle family."""

    POINT_MASS_3DOF = "point_mass_3dof"
    KINEMATIC_3_PLUS_3_DOF = "kinematic_3_plus_3_dof"
    RIGID_BODY_6DOF = "rigid_body_6dof"


class TelemetryChannel(BaseModel):
    """One semantically named, time-aligned vehicle channel."""

    model_config = ConfigDict(frozen=True)

    source_name: str
    semantic_name: str
    unit: str | None = None
    interpolation: InterpolationKind = "linear"
    values: list[float | None]


class SegmentSpan(BaseModel):
    """A contiguous segment interval in a vehicle history."""

    model_config = ConfigDict(frozen=True)

    number: int
    title: str
    start_time: float
    end_time: float
    stage: str | None = None


class EventRecord(BaseModel):
    """A discrete event aligned to the telemetry timeline."""

    model_config = ConfigDict(frozen=True)

    time: float
    vehicle: str
    name: str
    kind: str
    segment_from: int | None = None
    segment_to: int | None = None
    value_changes: dict[str, tuple[float | None, float | None]] = Field(default_factory=dict)


class VehicleTelemetry(BaseModel):
    """Structured history and metadata for one executed vehicle."""

    model_config = ConfigDict(frozen=True)

    vehicle_id: str
    name: str
    model_id: str | None = None
    parent_model_id: str | None = None
    kind: VehicleKind
    dynamics: DynamicsKind
    attitude_source: str | None = None
    times: list[float]
    channels: dict[str, TelemetryChannel]
    segments: list[SegmentSpan] = Field(default_factory=list)
    events: list[EventRecord] = Field(default_factory=list)

    @model_validator(mode="after")
    def aligned_channels(self) -> VehicleTelemetry:
        for channel in self.channels.values():
            if len(channel.values) != len(self.times):
                raise ValueError(f"channel {channel.semantic_name!r} is not aligned with vehicle times")
        return self


class _PortableRunMetadata(BaseModel, Mapping[str, object]):
    """Typed additive run metadata that preserves mapping-style consumers."""

    model_config = ConfigDict(frozen=True, extra="allow")

    def as_dict(self) -> dict[str, object]:
        """Return a JSON-safe compatibility projection."""

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

    def __bool__(self) -> bool:
        return bool(self.as_dict())
        ####

    def __eq__(self, other: object) -> bool:
        if isinstance(other, Mapping):
            return self.as_dict() == dict(other)
        return super().__eq__(other)
        ####

    ####


class RunTermination(_PortableRunMetadata):
    """Typed completion disposition shared by batch and interactive runs."""

    completed: bool | None = None
    stop_reason: str | None = None
    status: str | None = None

    @model_validator(mode="before")
    @classmethod
    def normalize_reason_alias(cls, value: object) -> object:
        """Accept older ``reason`` producers while publishing ``stop_reason``."""

        if isinstance(value, Mapping):
            normalized = dict(value)
            legacy_reason = normalized.pop("reason", None)
            if normalized.get("stop_reason") is None and legacy_reason is not None:
                normalized["stop_reason"] = legacy_reason
            elif legacy_reason is not None and legacy_reason != normalized.get("stop_reason"):
                raise ValueError("termination reason and stop_reason must agree when both are present")
            return normalized
        return value
        ####

    @field_validator("stop_reason", "status")
    @classmethod
    def nonblank_optional_text(cls, value: str | None) -> str | None:
        """Reject empty lifecycle labels that cannot help a diagnostic client."""

        if value is not None and not value.strip():
            raise ValueError("run lifecycle text must not be blank")
        return value
        ####

    @model_validator(mode="after")
    def incomplete_runs_have_a_reason(self) -> RunTermination:
        """Ensure a failed/incomplete run does not lose its terminal cause."""

        if self.completed is False and self.stop_reason is None:
            raise ValueError("an incomplete run artifact requires stop_reason")
        return self
        ####

    ####


class RunCommandRecord(_PortableRunMetadata):
    """One held-command frame retained in an interactive run artifact."""

    duration: float | None = Field(default=None, ge=0.0)
    commands: dict[str, float | str | bool] | None = None

    @field_validator("duration")
    @classmethod
    def finite_duration(cls, value: float | None) -> float | None:
        """Reject non-finite command intervals before serialization."""

        if value is not None and not math.isfinite(value):
            raise ValueError("command duration must be finite")
        return value
        ####

    @field_validator("commands")
    @classmethod
    def valid_command_values(cls, value: dict[str, float | str | bool] | None) -> dict[str, float | str | bool] | None:
        """Make each command channel addressable and JSON-safe."""

        if value is None:
            return None
        if any(not name.strip() for name in value):
            raise ValueError("command names must not be blank")
        for command in value.values():
            if isinstance(command, float) and not math.isfinite(command):
                raise ValueError("command values must be finite")
        return value
        ####

    ####


class RunLifecycleEvent(_PortableRunMetadata):
    """A typed top-level runtime event with provider-specific details additive."""

    name: str = Field(min_length=1)
    action: str = Field(min_length=1)
    event_id: str | None = None
    vehicle: str | None = None
    model_id: str | None = None
    time: float | None = None
    status: str | None = None
    signal: str | None = None
    source: str | None = None
    segment_from: int | None = None
    segment_to: int | None = None

    @field_validator("name", "action", "event_id", "vehicle", "model_id", "status", "signal", "source")
    @classmethod
    def nonblank_event_text(cls, value: str | None) -> str | None:
        """Reject labels that downstream event grouping cannot use."""

        if value is not None and not value.strip():
            raise ValueError("event text must not be blank")
        return value
        ####

    @field_validator("time")
    @classmethod
    def finite_event_time(cls, value: float | None) -> float | None:
        """Events may omit time, but a supplied timestamp must be usable."""

        if value is not None and not math.isfinite(value):
            raise ValueError("event time must be finite")
        return value
        ####

    ####


class RunSensorMeasurementSummary(BaseModel):
    """Portable counters from an accepted-truth sensor execution."""

    model_config = ConfigDict(frozen=True, extra="allow")

    emitted: int = Field(ge=0)
    valid: int = Field(ge=0)
    invalid: int = Field(ge=0)
    dropped: int = Field(ge=0)
    delivered: int = Field(ge=0)
    queued: int = Field(ge=0)
    timeout: bool
    last_accepted_truth_time_s: float | None = None
    next_requested_sensor_time_s: float | None = None

    @field_validator("last_accepted_truth_time_s", "next_requested_sensor_time_s")
    @classmethod
    def finite_optional_time(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("sensor execution time must be finite")
        return value
        ####

    ####


class RunSensorExecution(_PortableRunMetadata):
    """Typed sensor-execution summary with provider detail retained additively."""

    schema_version: int | None = Field(default=None, ge=1)
    execution: str | None = None
    scenario_identity: str | None = None
    provider: str | None = None
    spec: Mapping[str, object] | None = None
    measurement_summary: RunSensorMeasurementSummary | None = None
    termination: RunTermination | None = None

    @model_validator(mode="before")
    @classmethod
    def derive_provider_from_spec(cls, value: object) -> object:
        """Promote the common provider identity while preserving the full spec."""

        if isinstance(value, Mapping):
            normalized = dict(value)
            spec = normalized.get("spec")
            if normalized.get("provider") is None and isinstance(spec, Mapping):
                provider = spec.get("provider")
                if isinstance(provider, str):
                    normalized["provider"] = provider
            return normalized
        return value
        ####

    @field_validator("execution", "scenario_identity", "provider")
    @classmethod
    def nonblank_sensor_text(cls, value: str | None) -> str | None:
        if value is not None and not value.strip():
            raise ValueError("sensor execution text must not be blank")
        return value
        ####

    ####


class RunOutputSampling(_PortableRunMetadata):
    """Typed sampling policy advertised to artifact readers."""

    sample_interval: float | None = Field(default=None, gt=0.0)
    channels: list[str] = Field(default_factory=list)
    include_events: bool = True

    @field_validator("sample_interval")
    @classmethod
    def finite_interval(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("output sample interval must be finite")
        return value
        ####

    @field_validator("channels")
    @classmethod
    def nonblank_channels(cls, value: list[str]) -> list[str]:
        if any(not channel.strip() for channel in value):
            raise ValueError("output sampling channels must not be blank")
        return value
        ####

    ####


class RunVisualizationMetadata(_PortableRunMetadata):
    """Renderer/runtime metadata exposed to visualizers and diagnostics."""

    source: str | None = None
    schema_version: int | None = Field(default=None, ge=1)
    runtime: Mapping[str, object] | None = None
    output_sampling: RunOutputSampling | None = None

    @field_validator("source")
    @classmethod
    def nonblank_visualization_source(cls, value: str | None) -> str | None:
        if value is not None and not value.strip():
            raise ValueError("visualization source must not be blank")
        return value
        ####

    ####


class RunArtifact(BaseModel):
    """Persistable simulation-data contract for reports and visualizers."""

    # A run artifact is the public runtime handoff, so a misspelled top-level
    # field must fail at construction rather than disappear under Pydantic's
    # default ``extra='ignore'`` behavior. Provider-specific details belong in
    # the typed additive metadata records below, where they remain visible to
    # their owner without weakening this envelope.
    model_config = ConfigDict(frozen=True, extra="forbid")

    schema_version: int = 1
    problem: str
    vehicles: dict[str, VehicleTelemetry]
    parameters: dict[str, float | str | bool] = Field(default_factory=dict)
    scenario_identity: str | None = None
    composition: list[dict[str, object]] = Field(default_factory=list)
    resolution_records: list[dict[str, object]] = Field(default_factory=list)
    commands: list[RunCommandRecord] = Field(default_factory=list)
    termination: RunTermination = Field(default_factory=RunTermination)
    events: list[RunLifecycleEvent] = Field(default_factory=list)
    sensor_execution: RunSensorExecution = Field(default_factory=RunSensorExecution)
    visualization: RunVisualizationMetadata = Field(default_factory=RunVisualizationMetadata)

    def write_json(self, path: str | Path) -> Path:
        """Persist this artifact as human-readable JSON."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(self.model_dump_json(indent=2) + "\n", encoding="utf-8")
        return destination

    def format_text(self, *, max_rows: int | None = None) -> str:
        """Return a deterministic, human-readable view of the artifact."""

        output = StringIO()
        output.write(f"Run: {self.problem}\n")
        output.write(f"Schema version: {self.schema_version}\n")
        if self.scenario_identity is not None:
            output.write(f"Scenario identity: {self.scenario_identity}\n")
        if self.resolution_records:
            output.write(f"Resolution records: {len(self.resolution_records)}\n")
        if self.termination:
            output.write(f"Termination: {self.termination.as_dict()}\n")
        if self.parameters:
            output.write("Parameters:\n")
            for name, value in sorted(self.parameters.items()):
                formatted = f"{value:g}" if isinstance(value, (int, float)) and not isinstance(value, bool) else str(value)
                output.write(f"  {name} = {formatted}\n")
        for vehicle_id, vehicle in self.vehicles.items():
            output.write(f"\nVehicle {vehicle_id}: {vehicle.name}\n")
            output.write(f"  kind={vehicle.kind.value} dynamics={vehicle.dynamics.value}\n")
            output.write("  channels: " + ", ".join(vehicle.channels) + "\n")
            headers = ["time", *vehicle.channels]
            output.write("  " + " | ".join(headers) + "\n")
            row_count = len(vehicle.times) if max_rows is None else min(len(vehicle.times), max_rows)
            for index in range(row_count):
                values = [vehicle.times[index]] + [vehicle.channels[name].values[index] for name in vehicle.channels]
                output.write("  " + " | ".join(_format_text_value(value) for value in values) + "\n")
            if row_count < len(vehicle.times):
                output.write(f"  ... {len(vehicle.times) - row_count} rows omitted\n")
            if vehicle.segments:
                output.write("  segments: " + ", ".join(str(segment.number) for segment in vehicle.segments) + "\n")
            if vehicle.events:
                output.write(f"  events: {len(vehicle.events)}\n")
        return output.getvalue()

    def write_csv(
        self,
        path: str | Path,
        *,
        vehicle_id: str | None = None,
        channels: Sequence[str] = (),
    ) -> Path:
        """Write selected telemetry in deterministic long-form CSV."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        selected = (vehicle_id,) if vehicle_id is not None else tuple(sorted(self.vehicles))
        requested = tuple(channels)
        with destination.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            linked = self.scenario_identity is not None
            header = ("vehicle_id", "time", "semantic_name", "source_name", "unit", "value", "schema_version", "scenario_identity") if linked else ("vehicle_id", "time", "semantic_name", "source_name", "unit", "value")
            writer.writerow(header)
            for selected_id in selected:
                vehicle = self.vehicles[selected_id]
                names = requested or tuple(sorted(vehicle.channels))
                for index, time in enumerate(vehicle.times):
                    for name in names:
                        channel = vehicle.channels.get(name)
                        if channel is None:
                            continue
                        row = (selected_id, time, name, channel.source_name, channel.unit or "", channel.values[index])
                        writer.writerow((*row, self.schema_version, self.scenario_identity) if linked else row)
        return destination

    def print_text(self, *, max_rows: int | None = None) -> None:
        """Print the deterministic text view to standard output."""

        print(self.format_text(max_rows=max_rows), end="")

    def write_sqlite(self, path: str | Path, *, run_id: str = "run-1", replace: bool = True) -> Path:
        """Write this artifact to a normalized, dependency-free SQLite database."""

        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(destination) as connection:
            self.dump_sqlite(connection, run_id=run_id, replace=replace)
        return destination

    def dump_sqlite(self, connection: sqlite3.Connection, *, run_id: str = "run-1", replace: bool = True) -> None:
        """Write this artifact into an existing SQLite/DB-API connection."""

        connection.executescript(_SQLITE_SCHEMA)
        if replace:
            _delete_sqlite_run(connection, run_id)
        connection.execute(
            "INSERT INTO taoryx_runs(run_id, schema_version, problem) VALUES (?, ?, ?)",
            (run_id, self.schema_version, self.problem),
        )
        connection.executemany(
            "INSERT INTO taoryx_run_metadata(run_id, name, value_json) VALUES (?, ?, ?)",
            [
                (run_id, "scenario_identity", json.dumps(self.scenario_identity)),
                (run_id, "composition", json.dumps(self.composition, sort_keys=True)),
                (run_id, "resolution_records", json.dumps(self.resolution_records, sort_keys=True)),
                (run_id, "commands", json.dumps([item.as_dict() for item in self.commands], sort_keys=True)),
                (run_id, "termination", json.dumps(self.termination.as_dict(), sort_keys=True)),
                (run_id, "events", json.dumps([item.as_dict() for item in self.events], sort_keys=True)),
                (run_id, "sensor_execution", json.dumps(self.sensor_execution.as_dict(), sort_keys=True)),
                (run_id, "visualization", json.dumps(self.visualization.as_dict(), sort_keys=True)),
            ],
        )
        connection.executemany(
            "INSERT INTO taoryx_parameters(run_id, name, value) VALUES (?, ?, ?)",
            [(run_id, name, value) for name, value in sorted(self.parameters.items())],
        )
        event_index = 0
        for vehicle_id, vehicle in self.vehicles.items():
            connection.execute(
                """INSERT INTO taoryx_vehicles
                   (run_id, vehicle_id, name, kind, dynamics, attitude_source)
                   VALUES (?, ?, ?, ?, ?, ?)""",
                (run_id, vehicle_id, vehicle.name, vehicle.kind.value, vehicle.dynamics.value, vehicle.attitude_source),
            )
            connection.executemany(
                """INSERT INTO taoryx_channels
                   (run_id, vehicle_id, semantic_name, source_name, unit, interpolation)
                   VALUES (?, ?, ?, ?, ?, ?)""",
                [
                    (run_id, vehicle_id, name, channel.source_name, channel.unit, channel.interpolation)
                    for name, channel in vehicle.channels.items()
                ],
            )
            samples = [
                (run_id, vehicle_id, index, vehicle.times[index], name, channel.values[index])
                for index in range(len(vehicle.times))
                for name, channel in vehicle.channels.items()
            ]
            connection.executemany(
                """INSERT INTO taoryx_samples
                   (run_id, vehicle_id, sample_index, time, semantic_name, value)
                   VALUES (?, ?, ?, ?, ?, ?)""",
                samples,
            )
            connection.executemany(
                """INSERT INTO taoryx_segments
                   (run_id, vehicle_id, ordinal, number, title, start_time, end_time, stage)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?)""",
                [
                    (run_id, vehicle_id, ordinal, segment.number, segment.title, segment.start_time, segment.end_time, segment.stage)
                    for ordinal, segment in enumerate(vehicle.segments)
                ],
            )
            connection.executemany(
                """INSERT INTO taoryx_events
                   (run_id, event_index, vehicle_id, time, name, kind, segment_from, segment_to, value_changes_json)
                   VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)""",
                [
                    (run_id, event_index + ordinal, event.vehicle, event.time, event.name, event.kind, event.segment_from, event.segment_to, json.dumps(event.value_changes, sort_keys=True))
                    for ordinal, event in enumerate(vehicle.events)
                ],
            )
            event_index += len(vehicle.events)


_SQLITE_SCHEMA = """
PRAGMA foreign_keys = ON;
CREATE TABLE IF NOT EXISTS taoryx_runs (
    run_id TEXT PRIMARY KEY,
    schema_version INTEGER NOT NULL,
    problem TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS taoryx_parameters (
    run_id TEXT NOT NULL REFERENCES taoryx_runs(run_id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    value REAL NOT NULL,
    PRIMARY KEY (run_id, name)
);
CREATE TABLE IF NOT EXISTS taoryx_run_metadata (
    run_id TEXT NOT NULL REFERENCES taoryx_runs(run_id) ON DELETE CASCADE,
    name TEXT NOT NULL,
    value_json TEXT NOT NULL,
    PRIMARY KEY (run_id, name)
);
CREATE TABLE IF NOT EXISTS taoryx_vehicles (
    run_id TEXT NOT NULL REFERENCES taoryx_runs(run_id) ON DELETE CASCADE,
    vehicle_id TEXT NOT NULL,
    name TEXT NOT NULL,
    kind TEXT NOT NULL,
    dynamics TEXT NOT NULL,
    attitude_source TEXT,
    PRIMARY KEY (run_id, vehicle_id)
);
CREATE TABLE IF NOT EXISTS taoryx_channels (
    run_id TEXT NOT NULL,
    vehicle_id TEXT NOT NULL,
    semantic_name TEXT NOT NULL,
    source_name TEXT NOT NULL,
    unit TEXT,
    interpolation TEXT NOT NULL,
    PRIMARY KEY (run_id, vehicle_id, semantic_name),
    FOREIGN KEY (run_id, vehicle_id) REFERENCES taoryx_vehicles(run_id, vehicle_id) ON DELETE CASCADE
);
CREATE TABLE IF NOT EXISTS taoryx_samples (
    run_id TEXT NOT NULL,
    vehicle_id TEXT NOT NULL,
    sample_index INTEGER NOT NULL,
    time REAL NOT NULL,
    semantic_name TEXT NOT NULL,
    value REAL,
    PRIMARY KEY (run_id, vehicle_id, sample_index, semantic_name),
    FOREIGN KEY (run_id, vehicle_id, semantic_name) REFERENCES taoryx_channels(run_id, vehicle_id, semantic_name) ON DELETE CASCADE
);
CREATE TABLE IF NOT EXISTS taoryx_segments (
    run_id TEXT NOT NULL,
    vehicle_id TEXT NOT NULL,
    ordinal INTEGER NOT NULL,
    number INTEGER NOT NULL,
    title TEXT NOT NULL,
    start_time REAL NOT NULL,
    end_time REAL NOT NULL,
    stage TEXT,
    PRIMARY KEY (run_id, vehicle_id, ordinal),
    FOREIGN KEY (run_id, vehicle_id) REFERENCES taoryx_vehicles(run_id, vehicle_id) ON DELETE CASCADE
);
CREATE TABLE IF NOT EXISTS taoryx_events (
    run_id TEXT NOT NULL,
    event_index INTEGER NOT NULL,
    vehicle_id TEXT NOT NULL,
    time REAL NOT NULL,
    name TEXT NOT NULL,
    kind TEXT NOT NULL,
    segment_from INTEGER,
    segment_to INTEGER,
    value_changes_json TEXT NOT NULL,
    PRIMARY KEY (run_id, event_index),
    FOREIGN KEY (run_id, vehicle_id) REFERENCES taoryx_vehicles(run_id, vehicle_id) ON DELETE CASCADE
);
"""


def _delete_sqlite_run(connection: sqlite3.Connection, run_id: str) -> None:
    connection.execute("DELETE FROM taoryx_runs WHERE run_id = ?", (run_id,))


def _format_text_value(value: float | None) -> str:
    return "" if value is None else f"{value:g}"


def build_run_artifact(
    problem_name: str,
    problem: RuntimeProblem,
    result: ExecutionResult,
    *,
    vehicle_kinds: Mapping[str, VehicleKind] | None = None,
    scenario_identity: str | None = None,
    composition: Sequence[Mapping[str, object]] = (),
    resolution_records: Sequence[Mapping[str, object]] = (),
    commands: Sequence[Mapping[str, object]] = (),
    termination: Mapping[str, object] | None = None,
    events: Sequence[Mapping[str, object]] = (),
    visualization: Mapping[str, object] | None = None,
) -> RunArtifact:
    """Build telemetry directly from runtime histories, never from report text."""

    kinds = vehicle_kinds or {}
    vehicles = {
        vehicle_id: _build_vehicle_telemetry(
            vehicle_id,
            problem.vehicles.get(vehicle_id),
            history,
            kinds.get(vehicle_id, VehicleKind.GENERIC),
        )
        for vehicle_id, history in result.states.items()
        if history
    }
    raw_parameters = problem.metadata.get("parameters", {})
    parameters = {
        str(name): (float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else value)
        for name, value in raw_parameters.items()
        if isinstance(value, (bool, int, float, str))
    } if isinstance(raw_parameters, Mapping) else {}
    return RunArtifact(
        problem=problem_name,
        vehicles=vehicles,
        parameters=parameters,
        scenario_identity=scenario_identity,
        composition=[dict(item) for item in composition],
        resolution_records=[dict(item) for item in resolution_records],
        commands=[RunCommandRecord.model_validate(item) for item in commands],
        termination=RunTermination.model_validate(termination or {"completed": result.completed, "stop_reason": result.stop_reason}),
        events=[RunLifecycleEvent.model_validate(item) for item in events],
        visualization=RunVisualizationMetadata.model_validate(visualization or {}),
    )


def apply_output_subscriptions(artifact: RunArtifact, subscriptions: Sequence[object]) -> RunArtifact:
    """Filter and deterministically sample an artifact from runtime subscriptions."""

    if not subscriptions:
        return artifact
    intervals = [float(getattr(item, "sample_interval")) for item in subscriptions if getattr(item, "sample_interval", None) is not None]
    interval = min(intervals) if intervals else None
    requested = {
        str(channel).casefold()
        for item in subscriptions
        for channel in getattr(item, "channels", ())
    }
    include_events = any(bool(getattr(item, "include_events", True)) for item in subscriptions)
    vehicles: dict[str, VehicleTelemetry] = {}
    for vehicle_id, vehicle in artifact.vehicles.items():
        indices = _sample_indices(vehicle.times, interval)
        channels = {
            name: channel.model_copy(update={"values": [channel.values[index] for index in indices]})
            for name, channel in vehicle.channels.items()
            if not requested or name.casefold() in requested or channel.source_name.casefold() in requested
        }
        vehicles[vehicle_id] = vehicle.model_copy(
            update={
                "times": [vehicle.times[index] for index in indices],
                "channels": channels,
                "events": vehicle.events if include_events else [],
            }
        )
    visualization = artifact.visualization.as_dict()
    visualization["output_sampling"] = {
        "sample_interval": interval,
        "channels": sorted(requested),
        "include_events": include_events,
    }
    return artifact.model_copy(
        update={
            "vehicles": vehicles,
            "events": artifact.events if include_events else [],
            "visualization": RunVisualizationMetadata.model_validate(visualization),
        }
    )


def _sample_indices(times: Sequence[float], interval: float | None) -> list[int]:
    if not times:
        return []
    if interval is None:
        return list(range(len(times)))
    if not math.isfinite(interval) or interval <= 0.0:
        raise ValueError("output sample interval must be positive and finite")
    indices = [0]
    next_time = times[0] + interval
    for index, time in enumerate(times[1:], start=1):
        if time + 1.0e-12 >= next_time:
            indices.append(index)
            next_time = times[0] + (len(indices)) * interval
    if indices[-1] != len(times) - 1:
        indices.append(len(times) - 1)
    return indices


def _build_vehicle_telemetry(vehicle_id: str, vehicle: object, history: Sequence[object], kind: VehicleKind) -> VehicleTelemetry:
    times = [float(getattr(state, "time")) for state in history]
    source_names: list[str] = []
    seen_source_names: set[str] = set()
    for state in history:
        for name in (*getattr(state, "value_names", ()), *getattr(state, "named", {}).keys()):
            if name != "time" and (not name.startswith("_") or name == "_segment") and name not in seen_source_names:
                source_names.append(name)
                seen_source_names.add(name)
    value_indices = tuple(
        {name: index for index, name in enumerate(getattr(state, "value_names", ())) }
        for state in history
    )
    channels: dict[str, TelemetryChannel] = {}
    for source_name in source_names:
        public_source_name = canonical_output_name(source_name)
        spec = output_channel_spec(public_source_name)
        semantic_name = spec.semantic_name if spec is not None else f"taos.{public_source_name.casefold()}"
        if semantic_name in channels:
            continue
        channels[semantic_name] = TelemetryChannel(
            source_name=public_source_name,
            semantic_name=semantic_name,
            interpolation=spec.interpolation if spec is not None else _interpolation_for(public_source_name),
            values=[
                _state_value(state, public_source_name, value_index=index)
                for state, index in zip(history, value_indices, strict=True)
            ],
        )
    segments, events = _segment_metadata(vehicle_id, history, times)
    dynamics = _dynamics_kind(getattr(vehicle, "dynamics_mode", None))
    attitude_source = {
        DynamicsKind.POINT_MASS_3DOF: "commanded_body_basis",
        DynamicsKind.KINEMATIC_3_PLUS_3_DOF: "controller_body_rates",
        DynamicsKind.RIGID_BODY_6DOF: "integrated_quaternion",
    }[dynamics]
    return VehicleTelemetry(
        vehicle_id=vehicle_id,
        name=str(getattr(vehicle, "name", vehicle_id)),
        model_id=getattr(vehicle, "model_id", None),
        parent_model_id=getattr(vehicle, "parent_model_id", None),
        kind=kind,
        dynamics=dynamics,
        attitude_source=attitude_source,
        times=times,
        channels=channels,
        segments=segments,
        events=events,
    )


def _state_value(
    state: object,
    source_name: str,
    *,
    value_index: Mapping[str, int] | None = None,
) -> float | None:
    named = getattr(state, "named", {})
    value = named.get(source_name)
    if value is None and source_name.casefold() == "segment":
        value = named.get("_segment")
    if value is None:
        names = getattr(state, "value_names", ())
        values = getattr(state, "values", ())
        index = value_index.get(source_name) if value_index is not None else (names.index(source_name) if source_name in names else None)
        if index is not None:
            value = values[index]
    return None if value is None else float(value)


def _interpolation_for(source_name: str) -> InterpolationKind:
    spec = output_channel_spec(source_name)
    if spec is not None:
        return spec.interpolation
    lowered = source_name.casefold()
    return "angle" if lowered in {"alphat", "betae", "bankgc", "bankgd", "gamgc", "gamgd", "yawgc", "yawgd"} else "linear"


def _dynamics_kind(mode: object) -> DynamicsKind:
    value = getattr(mode, "value", mode)
    return {
        "point-mass": DynamicsKind.POINT_MASS_3DOF,
        "kinematic-6dof": DynamicsKind.KINEMATIC_3_PLUS_3_DOF,
        "rigid-body-6dof": DynamicsKind.RIGID_BODY_6DOF,
    }.get(str(value), DynamicsKind.POINT_MASS_3DOF)


def _segment_metadata(vehicle_id: str, history: Sequence[object], times: list[float]) -> tuple[list[SegmentSpan], list[EventRecord]]:
    numbers = [_state_value(state, "_segment") or _state_value(state, "segment") for state in history]
    spans: list[SegmentSpan] = []
    events: list[EventRecord] = []
    current: int | None = None
    start_index = 0
    for index, value in enumerate(numbers):
        number = None if value is None else int(value)
        if number == current:
            continue
        if current is not None:
            spans.append(SegmentSpan(number=current, title=f"segment {current}", start_time=times[start_index], end_time=times[index]))
            events.append(EventRecord(time=times[index], vehicle=vehicle_id, name="segment-transition", kind="segment_transition", segment_from=current, segment_to=number))
        current = number
        start_index = index
    if current is not None:
        spans.append(SegmentSpan(number=current, title=f"segment {current}", start_time=times[start_index], end_time=times[-1]))
    return spans, events


@dataclass(frozen=True, slots=True)
class OutputEvaluationPlan:
    """Ordered set of derived evaluators required by a problem."""

    required: tuple[str, ...]
    unavailable: tuple[str, ...]
####


def build_output_evaluation_plan(
    references: Iterable[str],
    final_conditions: Iterable[str] = (),
    expressions: Iterable[str] = (),
    searches: Iterable[str] = (),
    optimization_constraints: Iterable[str] = (),
    available_evaluators: Mapping[str, object] | Iterable[str] = (),
) -> OutputEvaluationPlan:
    """Evaluate TAOS-ALG-OUT-001's demand-driven output dependency plan."""

    required = tuple(
        dict.fromkeys(
            name.casefold()
            for source in (references, final_conditions, expressions, searches, optimization_constraints)
            for name in source
        )
    )
    available = {name.casefold() for name in available_evaluators} if not isinstance(available_evaluators, Mapping) else {name.casefold() for name in available_evaluators}
    return OutputEvaluationPlan(required, tuple(name for name in required if name not in available))
####
