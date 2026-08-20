"""Stateful control-streaming contracts for capable trajectory providers."""

from __future__ import annotations

import json
import math
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .composition import ContractDiagnostic, PreparedCompositionConfiguration
from .standard import StandardEcefState

SessionLifecycle = Literal["ready", "active", "completed", "closed", "failed"]
ChannelDirection = Literal["action", "observation"]
StreamingDataType = Literal["float64", "int64", "boolean", "string", "json"]


def _validate_json_value(value: object, *, path: str) -> None:
    try:
        json.dumps(value, allow_nan=False)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{path} is not JSON-compatible: {error}") from error
    ####


class StreamingChannel(BaseModel):
    """One action or accepted-truth observation channel in a session."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    direction: ChannelDirection
    description: str = Field(min_length=1)
    data_type: StreamingDataType = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    unit: str | None = None
    frame: str | None = None
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    sampling: Literal["held_action", "truth_boundary", "event", "provider_defined"] = "truth_boundary"
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_channel(self) -> StreamingChannel:
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"streaming channel {self.id!r} has inverted bounds")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"streaming channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.unit is not None:
            raise ValueError(f"non-numeric streaming channel {self.id!r} cannot name a unit")
        if self.choices and self.data_type != "string":
            raise ValueError(f"streaming channel {self.id!r} choices require string data")
        if len(self.choices) != len(set(self.choices)):
            raise ValueError(f"streaming channel {self.id!r} contains duplicate choices")
        _validate_json_value(self.metadata, path=f"streaming channel {self.id!r} metadata")
        return self
        ####

    ####


class ControlAuthorityState(BaseModel):
    """Current caller/provider control ownership at a committed boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    active_profile_id: str | None = None
    command_source_id: str | None = None
    command_owner: Literal["caller", "source_program", "provider_controller", "open_loop"] | None = None
    available_action_ids: tuple[str, ...] = ()
    unavailable_action_reasons: dict[str, tuple[str, ...]] = Field(default_factory=dict)
    selection_scope: Literal["batch", "session", "phase", "step", "provider"] | None = None
    switching_policy: Literal["locked", "explicit_bumpless", "provider_managed"] | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_authority(self) -> ControlAuthorityState:
        if self.active_profile_id is None and any(
            item is not None
            for item in (self.command_source_id, self.command_owner, self.selection_scope, self.switching_policy)
        ):
            raise ValueError("authority state without an active profile cannot claim control ownership")
        if len(self.available_action_ids) != len(set(self.available_action_ids)):
            raise ValueError("authority state contains duplicate available action IDs")
        overlap = set(self.available_action_ids) & set(self.unavailable_action_reasons)
        if overlap:
            raise ValueError(f"authority state marks actions both available and unavailable: {sorted(overlap)!r}")
        _validate_json_value(self.metadata, path="authority-state metadata")
        return self
        ####

    ####


class StreamingObservation(BaseModel):
    """One committed state boundary, including the common ECEF minimum."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-stream-observation/v1"] = Field(
        default="taoryx.trajectory-composition-stream-observation/v1",
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
    diagnostics: tuple[ContractDiagnostic, ...] = ()
    control_authority: ControlAuthorityState | None = None
    standard_ecef: StandardEcefState
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_observation(self) -> StreamingObservation:
        if not math.isfinite(self.time_s):
            raise ValueError("streaming observation time must be finite")
        _validate_json_value(self.values, path="streaming observation values")
        _validate_json_value(self.extensions, path="streaming observation extensions")
        return self
        ####

    ####


class StreamingAuthorityProfile(BaseModel):
    """One advertised semantic control surface a provider can select."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    authority: str = Field(min_length=1)
    action_ids: tuple[str, ...]
    command_owner: Literal["caller", "source_program", "provider_controller", "open_loop"]
    selection_scope: Literal["batch", "session", "phase", "step", "provider"]
    switching_policy: Literal["locked", "explicit_bumpless", "provider_managed"]
    description: str = Field(min_length=1)
    claim_boundary: str = Field(min_length=1)
    metadata: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_profile(self) -> StreamingAuthorityProfile:
        if len(self.action_ids) != len(set(self.action_ids)):
            raise ValueError(f"authority profile {self.id!r} contains duplicate action IDs")
        if self.command_owner == "caller" and self.selection_scope == "provider":
            raise ValueError("caller-owned authority cannot be provider-selected")
        if self.command_owner == "caller" and self.switching_policy == "provider_managed":
            raise ValueError("caller-owned authority cannot use provider-managed switching")
        _validate_json_value(self.metadata, path=f"authority profile {self.id!r} metadata")
        return self
        ####

    ####


class OpenStreamingSessionRequest(BaseModel):
    """Open a provider-owned streaming session from a prepared configuration."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-open-session/v1"] = Field(
        default="taoryx.trajectory-composition-open-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    prepared_configuration: PreparedCompositionConfiguration
    seed: int | None = None
    integration_step_s: float = Field(default=0.02, gt=0.0)
    authority_profile_id: str | None = Field(default=None, min_length=1)
    command_source_id: str | None = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def validate_selection(self) -> OpenStreamingSessionRequest:
        if self.command_source_id is not None and self.authority_profile_id is None:
            raise ValueError("command_source_id requires an authority_profile_id")
        return self
        ####

    ####


class StreamingSessionDescriptor(BaseModel):
    """Immutable session identity plus its exact action and observation schema."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-stream-session/v1"] = Field(
        default="taoryx.trajectory-composition-stream-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    provider_id: str = Field(min_length=1)
    provider_version: str = Field(min_length=1)
    model_id: str = Field(min_length=1)
    mission_template_id: str | None = None
    realization_id: str | None = None
    fidelity: str = Field(min_length=1)
    configuration_fingerprint: str = Field(pattern=r"^[0-9a-f]{64}$")
    state_owner: Literal["provider_session", "core_batch_replay_session", "provider_defined"] = "provider_session"
    integration_step_s: float = Field(gt=0.0)
    action_channels: tuple[StreamingChannel, ...] = ()
    observation_channels: tuple[StreamingChannel, ...] = Field(min_length=1)
    authority_profiles: tuple[StreamingAuthorityProfile, ...] = ()
    active_authority_profile_id: str | None = None
    initial_observation: StreamingObservation
    claim_boundary: str = Field(min_length=1)
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_descriptor(self) -> StreamingSessionDescriptor:
        action_ids = tuple(item.id for item in self.action_channels)
        observation_ids = tuple(item.id for item in self.observation_channels)
        if len(action_ids) != len(set(action_ids)):
            raise ValueError("streaming descriptor contains duplicate action channel IDs")
        if len(observation_ids) != len(set(observation_ids)):
            raise ValueError("streaming descriptor contains duplicate observation channel IDs")
        if any(item.direction != "action" for item in self.action_channels):
            raise ValueError("streaming descriptor action_channels must all be action channels")
        if any(item.direction != "observation" for item in self.observation_channels):
            raise ValueError("streaming descriptor observation_channels must all be observation channels")
        profile_ids = tuple(item.id for item in self.authority_profiles)
        if len(profile_ids) != len(set(profile_ids)):
            raise ValueError("streaming descriptor contains duplicate authority-profile IDs")
        if self.active_authority_profile_id is not None and self.active_authority_profile_id not in profile_ids:
            raise ValueError("streaming descriptor names an unknown active authority profile")
        if self.initial_observation.session_id != self.session_id or self.initial_observation.sequence != 0:
            raise ValueError("streaming descriptor initial observation is not the session zero boundary")
        _validate_json_value(self.extensions, path="streaming descriptor extensions")
        return self
        ####

    ####


class StreamingStepRequest(BaseModel):
    """Apply one caller action for an explicit held duration."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-stream-step-request/v1"] = Field(
        default="taoryx.trajectory-composition-stream-step-request/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    action: dict[str, Any] = Field(default_factory=dict)
    duration_s: float = Field(gt=0.0)
    expected_sequence: int | None = Field(default=None, ge=0)
    authority_profile_id: str | None = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def validate_action(self) -> StreamingStepRequest:
        _validate_json_value(self.action, path="streaming step action")
        return self
        ####

    ####


class ControlFeedback(BaseModel):
    """Per-channel action acceptance and achieved-response readback."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    channel_id: str = Field(min_length=1)
    availability: Literal["available", "unavailable"]
    request_present: bool
    requested_value: Any = None
    applied_present: bool
    applied_value: Any = None
    disposition: Literal["not_commanded", "held", "applied_as_requested", "limited", "withheld", "unavailable"]
    reason_codes: tuple[str, ...] = ()
    feedback_channel_id: str | None = None
    achievement_status: Literal["observed", "not_observed"] = "not_observed"
    achieved_value: Any = None

    @model_validator(mode="after")
    def validate_feedback(self) -> ControlFeedback:
        if self.request_present != (self.disposition in {"applied_as_requested", "limited", "withheld", "unavailable"}):
            raise ValueError("control-feedback request flag disagrees with disposition")
        if self.disposition in {"applied_as_requested", "limited", "held"} and not self.applied_present:
            raise ValueError("an applied control-feedback disposition requires an applied value")
        if self.disposition in {"not_commanded", "withheld", "unavailable"} and self.applied_present:
            raise ValueError("a non-applied control-feedback disposition cannot claim an applied value")
        if self.achievement_status == "observed" and self.feedback_channel_id is None:
            raise ValueError("observed control achievement requires a feedback channel")
        for field, value in (
            ("requested_value", self.requested_value),
            ("applied_value", self.applied_value),
            ("achieved_value", self.achieved_value),
        ):
            _validate_json_value(value, path=f"control-feedback {field}")
        return self
        ####

    ####


class StreamingStepResult(BaseModel):
    """Auditable action-to-observation transition at a truth boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-stream-step-result/v1"] = Field(
        default="taoryx.trajectory-composition-stream-step-result/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    sequence: int = Field(ge=1)
    time_start_s: float = Field(ge=0.0)
    time_end_s: float = Field(ge=0.0)
    requested_action: dict[str, Any]
    applied_action: dict[str, Any]
    observation: StreamingObservation
    diagnostics: tuple[ContractDiagnostic, ...] = ()
    control_feedback: tuple[ControlFeedback, ...] = ()
    authority_profile_id: str | None = None
    command_source_id: str | None = None
    lowering_evidence: dict[str, Any] = Field(default_factory=dict)
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_step(self) -> StreamingStepResult:
        if self.time_end_s < self.time_start_s:
            raise ValueError("streaming step ends before it starts")
        if self.observation.session_id != self.session_id:
            raise ValueError("streaming step observation names another session")
        if self.observation.sequence != self.sequence or abs(self.observation.time_s - self.time_end_s) > 1.0e-9:
            raise ValueError("streaming step observation is not aligned to its end boundary")
        feedback_ids = tuple(item.channel_id for item in self.control_feedback)
        if len(feedback_ids) != len(set(feedback_ids)):
            raise ValueError("streaming step contains duplicate control-feedback channels")
        _validate_json_value(self.requested_action, path="streaming requested action")
        _validate_json_value(self.applied_action, path="streaming applied action")
        _validate_json_value(self.lowering_evidence, path="streaming lowering evidence")
        _validate_json_value(self.extensions, path="streaming step extensions")
        return self
        ####

    ####


class SwitchAuthorityRequest(BaseModel):
    """Request a state-continuous control-authority transfer."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-switch-authority/v1"] = Field(
        default="taoryx.trajectory-composition-switch-authority/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    authority_profile_id: str = Field(min_length=1)
    expected_sequence: int | None = Field(default=None, ge=0)
    command_source_id: str | None = Field(default=None, min_length=1)


class AuthorityTransition(BaseModel):
    """Acknowledged authority switch that does not advance plant time."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-authority-transition/v1"] = Field(
        default="taoryx.trajectory-composition-authority-transition/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    sequence: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    previous_authority_profile_id: str = Field(min_length=1)
    active_authority_profile_id: str = Field(min_length=1)
    command_source_id: str | None = None
    action_channels: tuple[StreamingChannel, ...] = ()
    observation: StreamingObservation
    extensions: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_transition(self) -> AuthorityTransition:
        if not math.isfinite(self.time_s):
            raise ValueError("authority-transition time must be finite")
        if self.previous_authority_profile_id == self.active_authority_profile_id:
            raise ValueError("authority transition must change profiles")
        if self.observation.session_id != self.session_id:
            raise ValueError("authority transition observation names another session")
        if self.observation.sequence != self.sequence or abs(self.observation.time_s - self.time_s) > 1.0e-9:
            raise ValueError("authority transition observation is not aligned to the transition boundary")
        _validate_json_value(self.extensions, path="authority transition extensions")
        return self
        ####

    ####


class ResetStreamingSessionRequest(BaseModel):
    """Request deterministic reconstruction of a streaming session."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-reset-session/v1"] = Field(
        default="taoryx.trajectory-composition-reset-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    seed: int | None = None


class InspectStreamingSessionRequest(BaseModel):
    """Inspect a session without advancing its native state."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-inspect-session/v1"] = Field(
        default="taoryx.trajectory-composition-inspect-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)


class CloseStreamingSessionRequest(BaseModel):
    """Release one provider-owned streaming session."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-close-session/v1"] = Field(
        default="taoryx.trajectory-composition-close-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)


class ClosedStreamingSession(BaseModel):
    """Terminal close acknowledgement retained for lifecycle inspection."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.trajectory-composition-closed-session/v1"] = Field(
        default="taoryx.trajectory-composition-closed-session/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    lifecycle: Literal["closed"] = "closed"
    final_sequence: int = Field(ge=0)
    final_time_s: float = Field(ge=0.0)


__all__ = [
    "AuthorityTransition",
    "ChannelDirection",
    "CloseStreamingSessionRequest",
    "ClosedStreamingSession",
    "ControlAuthorityState",
    "ControlFeedback",
    "InspectStreamingSessionRequest",
    "OpenStreamingSessionRequest",
    "ResetStreamingSessionRequest",
    "SessionLifecycle",
    "StreamingAuthorityProfile",
    "StreamingChannel",
    "StreamingDataType",
    "StreamingObservation",
    "StreamingSessionDescriptor",
    "StreamingStepRequest",
    "StreamingStepResult",
    "SwitchAuthorityRequest",
]
