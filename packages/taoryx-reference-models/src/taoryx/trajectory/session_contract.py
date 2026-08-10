"""Stateful Mission Composition session contract and native episode adapter."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..composition_episode import (
    EpisodeChannel,
    EpisodeObservation,
    VehicleCompositionEpisode,
    open_vehicle_composition_episode,
)
from .configuration_contract import (
    PreparedTrajectoryConfiguration,
    TrajectoryModelMetadata,
    TrajectoryOutputDataType,
    TrajectorySamplingSemantics,
)
from .execution_contract import MissionCompositionDiagnostic, MissionCompositionExecutionError
from .native_mission_composition import compile_prepared_vehicle_composition
from .registry_mission_composition import RegistryMissionCompositionProvider

SessionLifecycle = Literal["ready", "active", "completed", "closed", "failed"]


class MissionCompositionSessionChannel(BaseModel):
    """One action or committed observation channel exposed by an open session."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    direction: Literal["action", "observation"]
    description: str = Field(min_length=1)
    quantity: str | None = None
    unit: str | None = None
    data_type: TrajectoryOutputDataType = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    sampling_semantics: TrajectorySamplingSemantics = "continuous_sample"
    minimum: float | None = None
    maximum: float | None = None
    value_space: dict[str, Any]

    @model_validator(mode="after")
    def validate_bounds(self) -> MissionCompositionSessionChannel:
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"session channel {self.id!r} has inverted bounds")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"session channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.unit is not None:
            raise ValueError(f"non-numeric session channel {self.id!r} cannot advertise units")
        return self
        ####

    ####


class MissionCompositionSessionObservation(BaseModel):
    """One inspectable committed state at a session boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-session-observation/v1"] = Field(
        default="taoryx.mission-composition-session-observation/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    sequence: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    lifecycle: SessionLifecycle
    values: dict[str, Any]
    events: tuple[str, ...] = ()
    spawned_entity_ids: tuple[str, ...] = ()
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()

    @model_validator(mode="after")
    def validate_time(self) -> MissionCompositionSessionObservation:
        if not math.isfinite(self.time_s):
            raise ValueError("session observation time must be finite")
        return self
        ####

    ####


class MissionCompositionOpenSessionRequest(BaseModel):
    """Create one provider-owned stateful execution session."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-open-session/v1"] = Field(
        default="taoryx.mission-composition-open-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    prepared_configuration: PreparedTrajectoryConfiguration
    seed: int | None = None
    integration_step_s: float = Field(default=0.02, gt=0.0)

    @property
    def model_id(self) -> str:
        return self.prepared_configuration.configuration.model_id
        ####

    ####


class MissionCompositionSessionDescriptor(BaseModel):
    """Immutable session identity, lifecycle semantics, and dynamic I/O schema."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-session/v1"] = Field(
        default="taoryx.mission-composition-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str
    provider_id: str
    provider_version: str
    model_id: str
    mission_template_id: str
    realization_id: str
    fidelity: str
    configuration_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    state_owner: Literal["provider_session"] = "provider_session"
    seed: int | None = None
    deterministic_reset: bool = True
    reset_semantics: Literal["reconstruct_prepared_initial_state"] = "reconstruct_prepared_initial_state"
    timestep_semantics: Literal["caller_duration_held_across_native_substeps"] = "caller_duration_held_across_native_substeps"
    integration_step_s: float = Field(gt=0.0)
    supports_spawned_entities: bool = False
    action_schema: tuple[MissionCompositionSessionChannel, ...]
    observation_schema: tuple[MissionCompositionSessionChannel, ...] = Field(min_length=1)
    initial_observation: MissionCompositionSessionObservation
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="after")
    def validate_descriptor(self) -> MissionCompositionSessionDescriptor:
        action_ids = tuple(item.id for item in self.action_schema)
        observation_ids = tuple(item.id for item in self.observation_schema)
        if len(action_ids) != len(set(action_ids)):
            raise ValueError("session descriptor contains duplicate action channel IDs")
        if len(observation_ids) != len(set(observation_ids)):
            raise ValueError("session descriptor contains duplicate observation channel IDs")
        if self.initial_observation.session_id != self.session_id:
            raise ValueError("session descriptor initial observation names another session")
        if self.initial_observation.sequence != 0:
            raise ValueError("session descriptor initial observation must have sequence zero")
        if set(self.initial_observation.values) != set(observation_ids):
            raise ValueError("session descriptor initial observation does not match its observation schema")
        return self
        ####

    ####


class MissionCompositionSessionStepRequest(BaseModel):
    """Apply one action for an explicit external hold duration."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-session-step-request/v1"] = Field(
        default="taoryx.mission-composition-session-step-request/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    action: dict[str, Any] = Field(default_factory=dict)
    duration_s: float = Field(gt=0.0)
    expected_sequence: int | None = Field(default=None, ge=0)


class MissionCompositionSessionStepResult(BaseModel):
    """Auditable action-to-observation transition at committed truth boundaries."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-session-step-result/v1"] = Field(
        default="taoryx.mission-composition-session-step-result/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str
    sequence: int = Field(ge=1)
    time_start_s: float = Field(ge=0.0)
    time_end_s: float = Field(ge=0.0)
    requested_action: dict[str, Any]
    applied_action: dict[str, Any]
    observation: MissionCompositionSessionObservation
    events: tuple[str, ...] = ()
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = ()

    @model_validator(mode="after")
    def validate_interval(self) -> MissionCompositionSessionStepResult:
        if self.time_end_s < self.time_start_s:
            raise ValueError("session step ends before it starts")
        if self.observation.sequence != self.sequence or abs(self.observation.time_s - self.time_end_s) > 1.0e-9:
            raise ValueError("session step observation is not aligned to the committed end boundary")
        return self
        ####

    ####


class MissionCompositionResetSessionRequest(BaseModel):
    """Deterministically reconstruct one session from its prepared initial state."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-reset-session/v1"] = Field(
        default="taoryx.mission-composition-reset-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    seed: int | None = None


class MissionCompositionInspectSessionRequest(BaseModel):
    """Read one session without advancing native state."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-inspect-session/v1"] = Field(
        default="taoryx.mission-composition-inspect-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)


class MissionCompositionCloseSessionRequest(BaseModel):
    """Release provider-owned state for one session."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-close-session/v1"] = Field(
        default="taoryx.mission-composition-close-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)


class MissionCompositionClosedSession(BaseModel):
    """Terminal close acknowledgement retained for lifecycle inspection."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-closed-session/v1"] = Field(
        default="taoryx.mission-composition-closed-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str
    lifecycle: Literal["closed"] = "closed"
    final_sequence: int = Field(ge=0)
    final_time_s: float = Field(ge=0.0)


@dataclass(slots=True)
class _SessionRecord:
    descriptor: MissionCompositionSessionDescriptor
    episode: VehicleCompositionEpisode
    sequence: int
    observation: MissionCompositionSessionObservation
    closed: bool = False


class MissionCompositionSessionManager:
    """Own stateful native episodes behind one provider-neutral lifecycle API."""

    def __init__(self, provider: RegistryMissionCompositionProvider | None = None) -> None:
        self.provider = provider or RegistryMissionCompositionProvider()
        self._sessions: dict[str, _SessionRecord] = {}
        ####

    def open(self, request: MissionCompositionOpenSessionRequest) -> MissionCompositionSessionDescriptor:
        """Create a session and return its exact action/observation schema."""

        if request.session_id in self._sessions:
            raise _session_error("session-already-exists", "The requested session ID is already in use.", request.model_id)
        if request.provider_id != self.provider.metadata.id or request.provider_version != self.provider.metadata.version:
            raise _session_error("provider-version-mismatch", "The session request names another provider version.", request.model_id)
        prepared = self.provider.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise _session_error("prepared-configuration-stale", "The prepared configuration is stale.", request.model_id)
        model = self.provider.model(prepared.configuration.model_id)
        if model.model_kind != "canonical_vehicle_family":
            # Workflows do not pass through the vehicle-composition compiler.
            # Check their advertised interactive tuple first so a caller gets
            # its declared blocker rather than an implementation-shaped
            # "native-model-not-supported" failure.
            _require_session_operation(
                model,
                prepared.configuration.mission_template_id or "",
                prepared.configuration.fidelity,
                prepared.configuration.realization_id or prepared.configuration.fidelity,
            )
        composition = compile_prepared_vehicle_composition(self.provider, prepared)
        model = self.provider.model(composition.vehicle_id)
        realization_id = prepared.configuration.realization_id or composition.fidelity
        _require_session_operation(model, composition.mission, composition.fidelity, realization_id)
        try:
            episode = open_vehicle_composition_episode(
                composition,
                seed=request.seed,
                integration_step_s=request.integration_step_s,
            )
            action_schema = tuple(_channel(item, "action") for item in episode.action_schema)
            observation_schema = tuple(_channel(item, "observation") for item in episode.observation_schema)
            observation = _observation(
                request.session_id,
                0,
                episode.observe(),
                observation_schema,
            )
        except MissionCompositionExecutionError:
            raise
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise _session_error(
                "session-open-failed",
                str(error),
                request.model_id,
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        supports_spawned = any(item.common_runner_status == "registered" and "step" in item.operations for item in model.deployments)
        descriptor = MissionCompositionSessionDescriptor(
            session_id=request.session_id,
            provider_id=self.provider.metadata.id,
            provider_version=self.provider.metadata.version,
            model_id=model.id,
            mission_template_id=composition.mission,
            realization_id=realization_id,
            fidelity=composition.fidelity,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=request.integration_step_s,
            supports_spawned_entities=supports_spawned,
            action_schema=action_schema,
            observation_schema=observation_schema,
            initial_observation=observation,
            claim_boundary=episode.claim_boundary,
        )
        self._sessions[request.session_id] = _SessionRecord(descriptor, episode, 0, observation)
        return descriptor
        ####

    create = open

    def inspect(
        self,
        request: MissionCompositionInspectSessionRequest | str,
    ) -> MissionCompositionSessionObservation:
        """Read the current committed state without advancing time."""

        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        record = self._record(session_id)
        if record.closed:
            return record.observation
        try:
            record.observation = _observation(
                session_id,
                record.sequence,
                record.episode.observe(),
                record.descriptor.observation_schema,
            )
            return record.observation
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise _session_error("session-inspect-failed", str(error), record.descriptor.model_id, phase="execution") from error
        ####

    def reset(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reset to the prepared initial state and sequence zero."""

        record = self._record(request.session_id)
        self._require_open(record)
        try:
            observed = record.episode.reset(seed=request.seed)
            record.sequence = 0
            record.observation = _observation(
                request.session_id,
                0,
                observed,
                record.descriptor.observation_schema,
            )
            return record.observation
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise _session_error("session-reset-failed", str(error), record.descriptor.model_id, phase="execution") from error
        ####

    def step(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance one session using a caller-duration action hold."""

        record = self._record(request.session_id)
        self._require_open(record)
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise _session_error(
                "stale-session-sequence",
                f"Expected sequence {request.expected_sequence}, current sequence is {record.sequence}.",
                record.descriptor.model_id,
            )
        allowed = {item.id for item in record.descriptor.action_schema}
        unknown = sorted(set(request.action) - allowed)
        if unknown:
            raise _session_error(
                "unknown-action-channel",
                f"The session does not advertise action channels {unknown!r}.",
                record.descriptor.model_id,
            )
        action_by_id = {item.id: item for item in record.descriptor.action_schema}
        for channel_id, value in request.action.items():
            try:
                _validate_session_value(
                    action_by_id[channel_id],
                    value,
                    path=f"session action channel {channel_id!r}",
                )
            except ValueError as error:
                raise _session_error(
                    "invalid-action-value",
                    str(error),
                    record.descriptor.model_id,
                    details={"channel_id": channel_id},
                ) from error
        try:
            native = record.episode.step(request.action, request.duration_s)
            record.sequence += 1
            diagnostics = tuple(
                MissionCompositionDiagnostic(
                    severity="warning",
                    code=_diagnostic_code(item),
                    message=item,
                    phase="execution",
                    recoverability="degraded",
                    provider_id=record.descriptor.provider_id,
                    model_id=record.descriptor.model_id,
                )
                for item in native.diagnostics
            )
            observation = _observation(
                request.session_id,
                record.sequence,
                native.observation,
                record.descriptor.observation_schema,
                events=native.events,
                diagnostics=diagnostics,
            )
            record.observation = observation
            return MissionCompositionSessionStepResult(
                session_id=request.session_id,
                sequence=record.sequence,
                time_start_s=native.time_start_s,
                time_end_s=native.time_end_s,
                requested_action=dict(native.requested_action),
                applied_action=dict(native.applied_action),
                observation=observation,
                events=native.events,
                diagnostics=diagnostics,
            )
        except MissionCompositionExecutionError:
            raise
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise _session_error(
                "session-step-failed",
                str(error),
                record.descriptor.model_id,
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        ####

    def close(
        self,
        request: MissionCompositionCloseSessionRequest | str,
    ) -> MissionCompositionClosedSession:
        """Close native state ownership and retain a terminal acknowledgement."""

        session_id = request.session_id if isinstance(request, MissionCompositionCloseSessionRequest) else request
        record = self._record(session_id)
        if not record.closed:
            try:
                record.episode.close()
            except (RuntimeError, TypeError, ValueError) as error:
                raise _session_error("session-close-failed", str(error), record.descriptor.model_id, phase="execution") from error
            record.closed = True
            record.observation = record.observation.model_copy(update={"lifecycle": "closed"})
        return MissionCompositionClosedSession(
            session_id=session_id,
            final_sequence=record.sequence,
            final_time_s=record.observation.time_s,
        )
        ####

    def active_session_ids(self) -> tuple[str, ...]:
        return tuple(sorted(identifier for identifier, record in self._sessions.items() if not record.closed))
        ####

    def _record(self, session_id: str) -> _SessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise _session_error("unknown-session", f"Session {session_id!r} does not exist.", "unknown") from error
        ####

    @staticmethod
    def _require_open(record: _SessionRecord) -> None:
        if record.closed:
            raise _session_error("session-closed", "The session is closed.", record.descriptor.model_id)
        ####

    ####


def _channel(channel: EpisodeChannel, direction: Literal["action", "observation"]) -> MissionCompositionSessionChannel:
    if channel.value_space is None:
        raise ValueError(f"episode channel {channel.name!r} has no value-space metadata")
    data_type, shape = _session_channel_representation(channel)
    return MissionCompositionSessionChannel(
        id=channel.name,
        direction=direction,
        description=channel.description,
        quantity="boolean" if data_type == "boolean" else _quantity(channel.unit),
        unit=None if data_type in {"boolean", "string", "json"} else channel.unit,
        data_type=data_type,
        shape=shape,
        sampling_semantics="discrete_sample" if data_type in {"boolean", "string", "json", "int64"} else "continuous_sample",
        minimum=None if data_type in {"boolean", "string", "json"} else channel.lower,
        maximum=None if data_type in {"boolean", "string", "json"} else channel.upper,
        value_space=channel.value_space.as_dict(),
    )
    ####


def _session_channel_representation(
    channel: EpisodeChannel,
) -> tuple[TrajectoryOutputDataType, tuple[int | Literal["variable"], ...]]:
    """Resolve primitive type and shape from the episode's explicit value space."""

    assert channel.value_space is not None
    topology = channel.value_space.topology
    representation = channel.value_space.representation
    if topology == "boolean":
        return "boolean", ()
    if topology in {"finite_set", "event"}:
        return ("float64", ()) if "code" in representation else ("string", ())
    if representation.startswith("vector"):
        size = int(representation.removeprefix("vector").split(maxsplit=1)[0])
        return "float64", (size,)
    if topology == "product":
        return "float64", (len(channel.value_space.components),)
    return "float64", ()
    ####


def _observation(
    session_id: str,
    sequence: int,
    native: EpisodeObservation,
    schema: tuple[MissionCompositionSessionChannel, ...],
    *,
    events: tuple[str, ...] = (),
    diagnostics: tuple[MissionCompositionDiagnostic, ...] = (),
) -> MissionCompositionSessionObservation:
    lifecycle: SessionLifecycle = cast_lifecycle(native.status)
    values = {item.id: _observation_value(native, item.id) for item in schema}
    for item in schema:
        _validate_session_value(
            item,
            values[item.id],
            path=f"session observation channel {item.id!r}",
        )
    return MissionCompositionSessionObservation(
        session_id=session_id,
        sequence=sequence,
        time_s=native.time_s,
        lifecycle=lifecycle,
        values=values,
        events=events,
        diagnostics=diagnostics,
    )
    ####


def _validate_session_value(
    metadata: MissionCompositionSessionChannel,
    value: Any,
    *,
    path: str,
) -> None:
    """Validate a session action or observation against advertised metadata."""

    def visit(item: Any, dimensions: tuple[int | Literal["variable"], ...], item_path: str) -> None:
        if dimensions:
            if not isinstance(item, list | tuple):
                raise ValueError(f"{item_path} must be an array with shape {metadata.shape!r}")
            expected = dimensions[0]
            if expected != "variable" and len(item) != expected:
                raise ValueError(f"{item_path} must contain {expected} items")
            for index, child in enumerate(item):
                visit(child, dimensions[1:], f"{item_path}[{index}]")
            return
        if metadata.data_type == "float64":
            if isinstance(item, bool) or not isinstance(item, int | float) or not math.isfinite(float(item)):
                raise ValueError(f"{item_path} must be a finite number")
            numeric = float(item)
            if metadata.minimum is not None and numeric < metadata.minimum:
                raise ValueError(f"{item_path} must be at least {metadata.minimum}")
            if metadata.maximum is not None and numeric > metadata.maximum:
                raise ValueError(f"{item_path} must be at most {metadata.maximum}")
        elif metadata.data_type == "int64":
            if isinstance(item, bool) or not isinstance(item, int):
                raise ValueError(f"{item_path} must be an integer")
            if metadata.minimum is not None and item < metadata.minimum:
                raise ValueError(f"{item_path} must be at least {metadata.minimum}")
            if metadata.maximum is not None and item > metadata.maximum:
                raise ValueError(f"{item_path} must be at most {metadata.maximum}")
        elif metadata.data_type == "boolean":
            if not isinstance(item, bool):
                raise ValueError(f"{item_path} must be boolean")
        elif metadata.data_type == "string":
            if not isinstance(item, str):
                raise ValueError(f"{item_path} must be text")
        else:
            _validate_json_session_value(item, path=item_path)
        ####

    visit(value, metadata.shape, path)
    ####


def _validate_json_session_value(value: Any, *, path: str) -> None:
    if value is None or isinstance(value, bool | str):
        return
    if isinstance(value, int | float):
        if not math.isfinite(float(value)):
            raise ValueError(f"{path} must not contain non-finite numbers")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise ValueError(f"{path} object keys must be strings")
            _validate_json_session_value(item, path=f"{path}.{key}")
        return
    if isinstance(value, list | tuple):
        for index, item in enumerate(value):
            _validate_json_session_value(item, path=f"{path}[{index}]")
        return
    raise ValueError(f"{path} has unsupported value type {type(value).__name__!r}")
    ####


def _observation_value(observation: EpisodeObservation, channel_id: str) -> Any:
    values = observation.values
    if isinstance(values, dict) and channel_id in values:
        return values[channel_id]
    aliases = {
        "execution.time": observation.time_s,
        "execution.status": observation.status,
        "position.north": values.get("north_m") if isinstance(values, dict) else None,
        "position.east": values.get("east_m") if isinstance(values, dict) else None,
        "position.altitude": values.get("altitude_m") if isinstance(values, dict) else None,
        "velocity.speed": values.get("speed_m_s") if isinstance(values, dict) else None,
        "flight.path_angle": values.get("flight_path_angle_rad") if isinstance(values, dict) else None,
        "flight.heading": values.get("heading_rad") if isinstance(values, dict) else None,
        "aero.dynamic_pressure": values.get("dynamic_pressure_pa") if isinstance(values, dict) else None,
        "attitude.euler": values.get("attitude_euler_deg") if isinstance(values, dict) else None,
        "body_rate": values.get("body_rate_rad_s") if isinstance(values, dict) else None,
        "propulsion.thrust": values.get("thrust_n") if isinstance(values, dict) else None,
        "control.throttle.realized": values.get("throttle_ratio") if isinstance(values, dict) else None,
        "resources.mass.total": values.get("mass_kg") if isinstance(values, dict) else None,
        "resources.mass.fuel_flow": values.get("fuel_flow_kg_s") if isinstance(values, dict) else None,
        "control.realization": values.get("control_realization") if isinstance(values, dict) else None,
        # The common interface always advertises the controller-method
        # diagnostic.  Source-backed response-law episodes may execute
        # without a reusable LQR/LQI controller, so expose that boundary
        # explicitly rather than omitting an advertised observation.
        "control.controller.method": values.get("controller_method", "not_applicable") if isinstance(values, dict) else "not_applicable",
    }
    if channel_id in aliases and aliases[channel_id] is not None:
        return aliases[channel_id]
    current = values
    for part in channel_id.split("."):
        if not isinstance(current, dict) or part not in current:
            raise KeyError(f"native observation omitted advertised channel {channel_id!r}")
        current = current[part]
    return current
    ####


def cast_lifecycle(status: str) -> SessionLifecycle:
    if status not in {"ready", "active", "completed", "closed"}:
        return "failed"
    return status  # type: ignore[return-value]
    ####


def _quantity(unit: str | None) -> str | None:
    if unit is None:
        return None
    return {
        "m": "length",
        "m/s": "speed",
        "m/s^2": "acceleration",
        "s": "time",
        "rad": "angle",
        "deg": "angle",
        "rad/s": "angular_rate",
        "deg/s": "angular_rate",
        "N": "force",
        "N*m": "moment",
        "kg": "mass",
        "boolean": "boolean",
        "dimensionless": "dimensionless",
    }.get(unit)
    ####


def _require_session_operation(
    model: TrajectoryModelMetadata,
    mission_id: str,
    fidelity: str,
    realization_id: str,
) -> None:
    mission = next((item for item in model.mission_templates if item.id == mission_id), None)
    exact = (
        None
        if mission is None
        else next(
            (item for item in mission.operations if item.operation == "step" and item.fidelity == fidelity and item.realization_id in {None, realization_id}),
            None,
        )
    )
    if exact is None or exact.status != "available" or exact.common_runner_status != "registered":
        raise _session_error(
            "interactive-operation-not-available",
            f"The exact {model.id}/{mission_id}/{fidelity}/{realization_id}/step tuple is not available.",
            model.id,
            details={"blockers": [] if exact is None else list(exact.blockers)},
        )
    ####


def _diagnostic_code(message: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "-", message.casefold()).strip("-")
    return normalized[:64].rstrip("-") or "native-episode-diagnostic"
    ####


def _session_error(
    code: str,
    message: str,
    model_id: str,
    *,
    phase: Literal["preflight", "execution"] = "preflight",
    details: dict[str, Any] | None = None,
) -> MissionCompositionExecutionError:
    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message or "Mission Composition session operation failed.",
            phase=phase,
            recoverability="correctable" if phase == "preflight" else "fatal",
            model_id=model_id,
            details=details or {},
        ),
        category="unsupported" if phase == "preflight" else "execution_failed",
    )
    ####


__all__ = [
    "MissionCompositionCloseSessionRequest",
    "MissionCompositionClosedSession",
    "MissionCompositionInspectSessionRequest",
    "MissionCompositionOpenSessionRequest",
    "MissionCompositionResetSessionRequest",
    "MissionCompositionSessionChannel",
    "MissionCompositionSessionDescriptor",
    "MissionCompositionSessionManager",
    "MissionCompositionSessionObservation",
    "MissionCompositionSessionStepRequest",
    "MissionCompositionSessionStepResult",
]
