"""Stateful Mission Composition session contract and native episode adapter."""

from __future__ import annotations

import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Literal, cast

from pydantic import BaseModel, ConfigDict, Field, model_validator

from ..composition_episode import (
    ActionFrame,
    EpisodeChannel,
    EpisodeObservation,
    MissionCompositionEpisode,
    open_vehicle_composition_episode,
)
from ..control_schemes import (
    ControlSchemeConsumerRole,
    ControlSchemeLayer,
    ControlSchemeStreamingPreference,
    control_scheme_definition,
)
from ..vehicle_interface import AuthorityProfile, InterfaceChannel
from .configuration_contract import (
    ConfigurableTrajectoryProvider,
    PreparedTrajectoryConfiguration,
    TrajectoryControlSamplingSemantics,
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
    frame: str | None = None
    control_sampling_semantics: TrajectoryControlSamplingSemantics | None = None
    data_type: TrajectoryOutputDataType = "float64"
    shape: tuple[int | Literal["variable"], ...] = ()
    sampling_semantics: TrajectorySamplingSemantics = "continuous_sample"
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    value_space: dict[str, Any]

    @model_validator(mode="after")
    def validate_bounds(self) -> MissionCompositionSessionChannel:
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError(f"session channel {self.id!r} has inverted bounds")
        if any(item != "variable" and item <= 0 for item in self.shape):
            raise ValueError(f"session channel {self.id!r} has a non-positive shape dimension")
        if self.data_type in {"boolean", "string", "json"} and self.unit is not None:
            raise ValueError(f"non-numeric session channel {self.id!r} cannot advertise units")
        if self.choices and self.data_type != "string":
            raise ValueError(f"session channel {self.id!r} choices require string data")
        if len(self.choices) != len(set(self.choices)):
            raise ValueError(f"session channel {self.id!r} contains duplicate choices")
        return self
        ####

    ####


class MissionCompositionSessionAuthorityProfile(BaseModel):
    """One selectable semantic action surface advertised by an open session."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    id: str = Field(min_length=1)
    authority: str = Field(min_length=1)
    availability: str = Field(min_length=1)
    action_ids: tuple[str, ...]
    description: str = Field(min_length=1)
    command_owner: str = Field(min_length=1)
    selection_scope: str = Field(min_length=1)
    switching_policy: str = Field(min_length=1)
    applicable_phase_ids: tuple[str, ...] = ()
    lowering_chain: tuple[str, ...] = ()
    scheme_id: str | None = Field(default=None, pattern=r"^[a-z0-9]+(?:[._-][a-z0-9]+)*$")
    scheme_layer: ControlSchemeLayer | None = None
    consumer_roles: tuple[ControlSchemeConsumerRole, ...] = ()
    streaming_preference: ControlSchemeStreamingPreference | None = None
    ui_order: int | None = Field(default=None, ge=0)
    claim_boundary: str = Field(min_length=1)

    @model_validator(mode="before")
    @classmethod
    def populate_scheme_defaults(cls, payload: object) -> object:
        """Retain canonical UI grouping in each open-session descriptor."""

        if not isinstance(payload, Mapping):
            return payload
        scheme_id = payload.get("scheme_id")
        if not isinstance(scheme_id, str):
            return payload
        definition = control_scheme_definition(scheme_id)
        if definition is None:
            return payload
        values = dict(payload)
        if values.get("scheme_layer") is None:
            values["scheme_layer"] = definition.layer
        if not values.get("consumer_roles"):
            values["consumer_roles"] = definition.consumer_roles
        if values.get("streaming_preference") is None:
            values["streaming_preference"] = definition.streaming_preference
        if values.get("ui_order") is None:
            values["ui_order"] = definition.ui_order
        return values
        ####

    @model_validator(mode="after")
    def validate_profile(self) -> MissionCompositionSessionAuthorityProfile:
        if len(self.action_ids) != len(set(self.action_ids)):
            raise ValueError(f"session authority profile {self.id!r} contains duplicate action IDs")
        if len(self.applicable_phase_ids) != len(set(self.applicable_phase_ids)):
            raise ValueError(f"session authority profile {self.id!r} contains duplicate phase IDs")
        if len(self.lowering_chain) != len(set(self.lowering_chain)):
            raise ValueError(f"session authority profile {self.id!r} contains duplicate lowering stages")
        if len(self.consumer_roles) != len(set(self.consumer_roles)):
            raise ValueError(f"session authority profile {self.id!r} contains duplicate consumer roles")
        if any(not item.strip() for item in (*self.action_ids, *self.applicable_phase_ids, *self.lowering_chain)):
            raise ValueError(f"session authority profile {self.id!r} contains an empty identifier")
        if self.command_owner not in {"caller", "source_program", "provider_controller", "open_loop"}:
            raise ValueError(f"session authority profile {self.id!r} has an unknown command owner")
        if self.selection_scope not in {"batch", "session", "phase", "step", "provider"}:
            raise ValueError(f"session authority profile {self.id!r} has an unknown selection scope")
        if self.switching_policy not in {"locked", "explicit_bumpless", "provider_managed"}:
            raise ValueError(f"session authority profile {self.id!r} has an unknown switching policy")
        if self.command_owner == "caller" and self.selection_scope == "provider":
            raise ValueError(f"caller-owned session authority profile {self.id!r} cannot be provider-selected")
        if self.command_owner == "caller" and self.switching_policy == "provider_managed":
            raise ValueError(f"caller-owned session authority profile {self.id!r} cannot use provider-managed switching")
        if self.scheme_id is None and any(
            item is not None
            for item in (self.scheme_layer, self.streaming_preference, self.ui_order)
        ):
            raise ValueError(f"session authority profile {self.id!r} scheme metadata requires scheme_id")
        if self.scheme_id is None and self.consumer_roles:
            raise ValueError(f"session authority profile {self.id!r} consumer roles require scheme_id")
        if self.scheme_id is not None:
            definition = control_scheme_definition(self.scheme_id)
            if definition is None and self.scheme_layer is None:
                raise ValueError(
                    f"provider-specific control scheme {self.scheme_id!r} requires scheme_layer"
                )
            if definition is not None and self.scheme_layer not in {None, definition.layer}:
                raise ValueError(
                    f"control scheme {self.scheme_id!r} belongs to {definition.layer!r}, "
                    f"not {self.scheme_layer!r}"
                )
        return self
        ####

    @classmethod
    def from_interface(cls, profile: AuthorityProfile) -> MissionCompositionSessionAuthorityProfile:
        """Project the immutable vehicle-interface profile without losing semantics."""

        return cls.model_validate(profile.as_dict())
        ####

    ####


class MissionCompositionControlAuthorityState(BaseModel):
    """Inspectable ownership and lowering state at a committed session boundary."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    active_profile_id: str | None = None
    command_source_id: str | None = None
    command_owner: str | None = None
    lowering_chain: tuple[str, ...] = ()
    selection_scope: str | None = None
    switching_policy: str | None = None

    @model_validator(mode="after")
    def validate_state(self) -> MissionCompositionControlAuthorityState:
        if self.active_profile_id is None and any(
            item is not None
            for item in (
                self.command_source_id,
                self.command_owner,
                self.selection_scope,
                self.switching_policy,
            )
        ):
            raise ValueError("control-authority state without an active profile cannot claim ownership")
        if self.active_profile_id is None and self.lowering_chain:
            raise ValueError("control-authority state without an active profile cannot claim a lowering chain")
        return self
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
    control_authority: MissionCompositionControlAuthorityState | None = None

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
    authority_profile_id: str | None = Field(default=None, min_length=1)
    command_source_id: str | None = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def validate_authority_selection(self) -> MissionCompositionOpenSessionRequest:
        if self.command_source_id is not None and self.authority_profile_id is None:
            raise ValueError("session command_source_id requires an authority_profile_id")
        return self
        ####

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
    action_schema_projection: Literal["native_union", "selected_semantic_profile"] = "native_union"
    authority_profiles: tuple[MissionCompositionSessionAuthorityProfile, ...] = ()
    default_authority_profile_id: str | None = None
    active_authority_profile_id: str | None = None
    command_source_id: str | None = None
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
        authority_ids = tuple(item.id for item in self.authority_profiles)
        if len(authority_ids) != len(set(authority_ids)):
            raise ValueError("session descriptor contains duplicate authority-profile IDs")
        if self.default_authority_profile_id is not None and self.default_authority_profile_id not in authority_ids:
            raise ValueError("session descriptor names an unknown default authority profile")
        if self.active_authority_profile_id is not None and self.active_authority_profile_id not in authority_ids:
            raise ValueError("session descriptor names an unknown active authority profile")
        if self.action_schema_projection == "selected_semantic_profile":
            if self.active_authority_profile_id is None:
                raise ValueError("semantic action schema requires an active authority profile")
            active = next(item for item in self.authority_profiles if item.id == self.active_authority_profile_id)
            if set(action_ids) != set(active.action_ids):
                raise ValueError("session semantic action schema does not match its active authority profile")
            authority_state = self.initial_observation.control_authority
            if authority_state is None or authority_state.active_profile_id != self.active_authority_profile_id:
                raise ValueError("session initial observation does not identify its active authority profile")
            if authority_state.command_source_id != self.command_source_id:
                raise ValueError("session initial observation command source disagrees with its descriptor")
            if authority_state.command_owner != active.command_owner:
                raise ValueError("session initial observation command owner disagrees with its active profile")
        elif self.active_authority_profile_id is not None:
            raise ValueError("native-union session cannot claim an active semantic authority profile")
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
    authority_profile_id: str | None = Field(default=None, min_length=1)


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
    authority_profile_id: str | None = None
    command_source_id: str | None = None
    lowered_action: dict[str, Any] = Field(default_factory=dict)
    lowering_evidence: dict[str, Any] = Field(default_factory=dict)

    @model_validator(mode="after")
    def validate_interval(self) -> MissionCompositionSessionStepResult:
        if self.time_end_s < self.time_start_s:
            raise ValueError("session step ends before it starts")
        if self.observation.sequence != self.sequence or abs(self.observation.time_s - self.time_end_s) > 1.0e-9:
            raise ValueError("session step observation is not aligned to the committed end boundary")
        authority_state = self.observation.control_authority
        if self.authority_profile_id is None:
            if authority_state is not None:
                raise ValueError("session step without an authority profile cannot publish active authority state")
        else:
            if authority_state is None or authority_state.active_profile_id != self.authority_profile_id:
                raise ValueError("session step observation disagrees with its active authority profile")
            if authority_state.command_source_id != self.command_source_id:
                raise ValueError("session step observation disagrees with its command source")
        return self
        ####

    ####


class MissionCompositionSwitchAuthorityRequest(BaseModel):
    """Request an in-stream, state-continuous transfer to another profile."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-switch-authority/v1"] = Field(
        default="taoryx.mission-composition-switch-authority/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str = Field(min_length=1)
    authority_profile_id: str = Field(min_length=1)
    expected_sequence: int | None = Field(default=None, ge=0)
    command_source_id: str | None = Field(default=None, min_length=1)


class MissionCompositionAuthorityTransition(BaseModel):
    """Acknowledged authority transfer without advancing plant time."""

    model_config = ConfigDict(frozen=True, extra="forbid", populate_by_name=True)

    schema_id: Literal["taoryx.mission-composition-authority-transition/v1"] = Field(
        default="taoryx.mission-composition-authority-transition/v1",
        alias="schema",
        serialization_alias="schema",
    )
    session_id: str
    sequence: int = Field(ge=0)
    time_s: float = Field(ge=0.0)
    previous_authority_profile_id: str = Field(min_length=1)
    active_authority_profile_id: str = Field(min_length=1)
    command_source_id: str | None = None
    transfer_semantics: Literal["state_continuous_reference_handoff"] = "state_continuous_reference_handoff"
    action_schema: tuple[MissionCompositionSessionChannel, ...]
    observation: MissionCompositionSessionObservation

    @model_validator(mode="after")
    def validate_transition(self) -> MissionCompositionAuthorityTransition:
        if not math.isfinite(self.time_s):
            raise ValueError("authority transition time must be finite")
        if self.previous_authority_profile_id == self.active_authority_profile_id:
            raise ValueError("authority transition must change profiles")
        if self.observation.session_id != self.session_id:
            raise ValueError("authority transition observation names another session")
        if self.observation.sequence != self.sequence or abs(self.observation.time_s - self.time_s) > 1.0e-9:
            raise ValueError("authority transition observation is not aligned to its committed boundary")
        authority_state = self.observation.control_authority
        if authority_state is None or authority_state.active_profile_id != self.active_authority_profile_id:
            raise ValueError("authority transition observation disagrees with its active profile")
        if authority_state.command_source_id != self.command_source_id:
            raise ValueError("authority transition observation disagrees with its command source")
        action_ids = tuple(item.id for item in self.action_schema)
        if len(action_ids) != len(set(action_ids)):
            raise ValueError("authority transition action schema contains duplicate channel IDs")
        return self
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
    episode: MissionCompositionEpisode
    sequence: int
    observation: MissionCompositionSessionObservation
    closed: bool = False


class MissionCompositionSessionManager:
    """Own stateful native episodes behind one provider-neutral lifecycle API."""

    def __init__(self, provider: ConfigurableTrajectoryProvider | None = None) -> None:
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
        model = _provider_model(self.provider, prepared.configuration.model_id)
        mission_template_id = prepared.configuration.mission_template_id or ""
        fidelity = prepared.configuration.fidelity
        realization_id = prepared.configuration.realization_id or fidelity
        _require_session_operation(model, mission_template_id, fidelity, realization_id)
        try:
            if model.model_kind == "canonical_vehicle_family":
                if not isinstance(self.provider, RegistryMissionCompositionProvider):
                    raise TypeError("canonical vehicle sessions require the registry provider compiler")
                composition = compile_prepared_vehicle_composition(self.provider, prepared)
                episode: MissionCompositionEpisode = open_vehicle_composition_episode(
                    composition,
                    seed=request.seed,
                    integration_step_s=request.integration_step_s,
                )
                mission_template_id = composition.mission
                fidelity = composition.fidelity
                realization_id = prepared.configuration.realization_id or composition.fidelity
            else:
                opener = getattr(self.provider, "open_session_episode", None)
                if not callable(opener):
                    raise TypeError(
                        f"provider {self.provider.metadata.id!r} advertises step but has no session episode factory"
                    )
                episode = opener(
                    prepared,
                    seed=request.seed,
                    integration_step_s=request.integration_step_s,
                )
            contract = episode.interface_contract
            authority_profiles = tuple(
                MissionCompositionSessionAuthorityProfile.from_interface(item)
                for item in contract.authority_profiles
            )
            active_authority_profile_id = request.authority_profile_id
            if active_authority_profile_id is None and getattr(episode, "auto_select_default_authority", False):
                active_authority_profile_id = contract.default_authority_profile_id
            active_profile = (
                None
                if active_authority_profile_id is None
                else _require_session_authority_profile(contract.authority_profile(active_authority_profile_id))
            )
            command_source_id = _authority_command_source(
                active_profile,
                request.command_source_id,
                fallback="external_client",
            )
            if active_profile is None:
                action_schema = tuple(_channel(item, "action") for item in episode.action_schema)
                action_schema_projection: Literal["native_union", "selected_semantic_profile"] = "native_union"
            else:
                _select_episode_authority(episode, active_profile.id)
                action_schema = _semantic_action_schema(contract.action_channels, active_profile)
                action_schema_projection = "selected_semantic_profile"
            observation_schema = tuple(_channel(item, "observation") for item in episode.observation_schema)
            observation = _observation(
                request.session_id,
                0,
                episode.observe(),
                observation_schema,
                control_authority=_authority_state(active_profile, command_source_id),
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
            mission_template_id=mission_template_id,
            realization_id=realization_id,
            fidelity=fidelity,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=request.integration_step_s,
            supports_spawned_entities=supports_spawned,
            action_schema=action_schema,
            action_schema_projection=action_schema_projection,
            authority_profiles=authority_profiles,
            default_authority_profile_id=contract.default_authority_profile_id,
            active_authority_profile_id=None if active_profile is None else active_profile.id,
            command_source_id=command_source_id,
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
                control_authority=_descriptor_authority_state(record.descriptor),
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
                control_authority=_descriptor_authority_state(record.descriptor),
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
        active_authority_profile_id = record.descriptor.active_authority_profile_id
        if request.authority_profile_id is not None and request.authority_profile_id != active_authority_profile_id:
            raise _session_error(
                "authority-profile-mismatch",
                f"The frame names authority profile {request.authority_profile_id!r}; the session selected {active_authority_profile_id!r}.",
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
            if active_authority_profile_id is None:
                native = record.episode.step(request.action, request.duration_s)
                requested_action = dict(native.requested_action)
                applied_action = dict(native.applied_action)
                lowered_action: dict[str, Any] = {}
                lowering_evidence: dict[str, Any] = {}
            else:
                frame = ActionFrame(
                    record.episode.interface_contract.id,
                    record.episode.interface_contract.fingerprint,
                    active_authority_profile_id,
                    dict(request.action),
                    request.duration_s,
                )
                native = record.episode.step_frame(frame)
                requested_action = dict(request.action)
                applied_action = dict(
                    request.action
                    if native.applied_semantic_action is None
                    else native.applied_semantic_action
                )
                lowered_action = dict(native.applied_action)
                raw_lowering = (
                    None
                    if native.status_frame is None
                    else native.status_frame.raw_values.get("control_lowering")
                )
                lowering_evidence = dict(raw_lowering) if isinstance(raw_lowering, dict) else {}
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
                control_authority=_descriptor_authority_state(record.descriptor),
            )
            record.observation = observation
            return MissionCompositionSessionStepResult(
                session_id=request.session_id,
                sequence=record.sequence,
                time_start_s=native.time_start_s,
                time_end_s=native.time_end_s,
                requested_action=requested_action,
                applied_action=applied_action,
                observation=observation,
                events=native.events,
                diagnostics=diagnostics,
                authority_profile_id=active_authority_profile_id,
                command_source_id=record.descriptor.command_source_id,
                lowered_action=lowered_action,
                lowering_evidence=lowering_evidence,
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

    def switch_authority(
        self,
        request: MissionCompositionSwitchAuthorityRequest,
    ) -> MissionCompositionAuthorityTransition:
        """Transfer a selected semantic stream without replacing its transport session."""

        record = self._record(request.session_id)
        self._require_open(record)
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise _session_error(
                "stale-session-sequence",
                f"Expected sequence {request.expected_sequence}, current sequence is {record.sequence}.",
                record.descriptor.model_id,
            )
        previous_id = record.descriptor.active_authority_profile_id
        if previous_id is None:
            raise _session_error(
                "native-union-authority-not-switchable",
                "A legacy native-union session must be reopened with an authority_profile_id before switching profiles.",
                record.descriptor.model_id,
            )
        contract = record.episode.interface_contract
        try:
            previous = _require_session_authority_profile(contract.authority_profile(previous_id))
            selected = _require_session_authority_profile(contract.authority_profile(request.authority_profile_id))
        except (KeyError, ValueError) as error:
            raise _session_error(
                "authority-profile-unavailable",
                str(error),
                record.descriptor.model_id,
                details={"authority_profile_id": request.authority_profile_id},
            ) from error
        if selected.id == previous.id:
            raise _session_error(
                "authority-profile-already-active",
                f"Authority profile {selected.id!r} is already active.",
                record.descriptor.model_id,
            )
        if previous.switching_policy != "explicit_bumpless" or selected.switching_policy != "explicit_bumpless":
            raise _session_error(
                "authority-transfer-not-declared",
                f"Authority transfer {previous.id!r} -> {selected.id!r} is not declared bumpless.",
                record.descriptor.model_id,
            )
        try:
            _select_episode_authority(record.episode, selected.id, require_transition_hook=True)
            action_schema = _semantic_action_schema(contract.action_channels, selected)
            command_source_id = _authority_command_source(
                selected,
                request.command_source_id,
                fallback=record.descriptor.command_source_id or "external_client",
            )
            record.descriptor = record.descriptor.model_copy(
                update={
                    "action_schema": action_schema,
                    "active_authority_profile_id": selected.id,
                    "command_source_id": command_source_id,
                }
            )
            record.observation = _observation(
                request.session_id,
                record.sequence,
                record.episode.observe(),
                record.descriptor.observation_schema,
                events=(f"authority_profile_changed:{previous.id}->{selected.id}",),
                control_authority=_authority_state(selected, command_source_id),
            )
        except MissionCompositionExecutionError:
            raise
        except (KeyError, RuntimeError, TypeError, ValueError) as error:
            raise _session_error(
                "authority-transfer-failed",
                str(error),
                record.descriptor.model_id,
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return MissionCompositionAuthorityTransition(
            session_id=request.session_id,
            sequence=record.sequence,
            time_s=record.observation.time_s,
            previous_authority_profile_id=previous.id,
            active_authority_profile_id=selected.id,
            command_source_id=command_source_id,
            action_schema=action_schema,
            observation=record.observation,
        )
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
        sampling_semantics=(
            channel.sampling_semantics
            or ("discrete_sample" if data_type in {"boolean", "string", "json", "int64"} else "continuous_sample")
        ),
        minimum=None if data_type in {"boolean", "string", "json"} else channel.lower,
        maximum=None if data_type in {"boolean", "string", "json"} else channel.upper,
        value_space=channel.value_space.as_dict(),
    )
    ####


def _semantic_action_schema(
    channels: tuple[InterfaceChannel, ...],
    profile: AuthorityProfile,
) -> tuple[MissionCompositionSessionChannel, ...]:
    """Expose only the semantic channels owned by one selected profile."""

    by_id = {item.id: item for item in channels}
    return tuple(_semantic_channel(by_id[identifier]) for identifier in profile.action_ids)
    ####


def _semantic_channel(channel: InterfaceChannel) -> MissionCompositionSessionChannel:
    """Project one semantic interface action into the common stream schema."""

    representation = {
        "scalar": ("float64", ()),
        "vector3": ("float64", (3,)),
        "vector4": ("float64", (4,)),
        "boolean": ("boolean", ()),
        "enum": ("string", ()),
        "event": ("string", ()),
    }[channel.value_type]
    data_type, shape = representation
    if channel.value_space is None:
        raise ValueError(f"semantic action channel {channel.id!r} has no value-space metadata")
    if channel.sampling not in {"held_action", "not_sampled", "event"}:
        raise ValueError(
            f"semantic action channel {channel.id!r} has non-control sampling semantics {channel.sampling!r}"
        )
    binding_choices = channel.binding.get("choices", ())
    choices = (
        tuple(str(item) for item in binding_choices if isinstance(item, str))
        if isinstance(binding_choices, (list, tuple))
        else ()
    )
    return MissionCompositionSessionChannel(
        id=channel.id,
        direction="action",
        description=channel.description,
        quantity="boolean" if data_type == "boolean" else _quantity(channel.canonical_unit),
        unit=None if data_type in {"boolean", "string", "json"} else channel.canonical_unit,
        frame=channel.frame,
        control_sampling_semantics=cast(TrajectoryControlSamplingSemantics, channel.sampling),
        data_type=data_type,  # type: ignore[arg-type]
        shape=shape,
        sampling_semantics="discrete_sample" if data_type in {"boolean", "string", "json", "int64"} else "continuous_sample",
        minimum=None if data_type in {"boolean", "string", "json"} else channel.lower,
        maximum=None if data_type in {"boolean", "string", "json"} else channel.upper,
        choices=choices,
        value_space=channel.value_space.as_dict(),
    )
    ####


def _require_session_authority_profile(profile: AuthorityProfile) -> AuthorityProfile:
    if profile.availability != "available":
        raise ValueError(f"authority profile {profile.id!r} is {profile.availability}, not executable")
    if profile.command_owner != "caller" and profile.action_ids:
        raise ValueError(
            f"authority profile {profile.id!r} is owned by {profile.command_owner} "
            "and cannot expose caller action channels"
        )
    return profile
    ####


def _authority_command_source(
    profile: AuthorityProfile | None,
    requested: str | None,
    *,
    fallback: str,
) -> str | None:
    """Attribute caller commands while leaving provider/open-loop profiles source-free."""

    if profile is None:
        return None
    if profile.command_owner == "caller":
        return requested or fallback
    if requested is not None:
        raise ValueError(
            f"authority profile {profile.id!r} is owned by {profile.command_owner}; "
            "command_source_id must be omitted"
        )
    return None
    ####


def _select_episode_authority(
    episode: MissionCompositionEpisode,
    authority_profile_id: str,
    *,
    require_transition_hook: bool = False,
) -> None:
    selector = getattr(episode, "select_authority_profile", None)
    if callable(selector):
        selector(authority_profile_id)
        return
    if require_transition_hook:
        raise ValueError("the selected episode has no state-continuous authority-transfer hook")
    ####


def _authority_state(
    profile: AuthorityProfile | None,
    command_source_id: str | None,
) -> MissionCompositionControlAuthorityState | None:
    if profile is None:
        return None
    return MissionCompositionControlAuthorityState(
        active_profile_id=profile.id,
        command_source_id=command_source_id,
        command_owner=profile.command_owner,
        lowering_chain=profile.lowering_chain,
        selection_scope=profile.selection_scope,
        switching_policy=profile.switching_policy,
    )
    ####


def _descriptor_authority_state(
    descriptor: MissionCompositionSessionDescriptor,
) -> MissionCompositionControlAuthorityState | None:
    identifier = descriptor.active_authority_profile_id
    if identifier is None:
        return None
    session_profile = next(item for item in descriptor.authority_profiles if item.id == identifier)
    return MissionCompositionControlAuthorityState(
        active_profile_id=session_profile.id,
        command_source_id=descriptor.command_source_id,
        command_owner=session_profile.command_owner,
        lowering_chain=session_profile.lowering_chain,
        selection_scope=session_profile.selection_scope,
        switching_policy=session_profile.switching_policy,
    )
    ####


def _session_channel_representation(
    channel: EpisodeChannel,
) -> tuple[TrajectoryOutputDataType, tuple[int | Literal["variable"], ...]]:
    """Resolve primitive type and shape from the episode's explicit value space."""

    assert channel.value_space is not None
    if channel.data_type is not None:
        return channel.data_type, channel.shape
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
    control_authority: MissionCompositionControlAuthorityState | None = None,
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
        control_authority=control_authority,
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
            if metadata.choices and item not in metadata.choices:
                raise ValueError(f"{item_path} must be one of {list(metadata.choices)!r}")
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


def _provider_model(
    provider: ConfigurableTrajectoryProvider,
    model_id: str,
) -> TrajectoryModelMetadata:
    """Resolve metadata through the provider-neutral discovery contract."""

    model_getter = getattr(provider, "model", None)
    if callable(model_getter):
        return cast(TrajectoryModelMetadata, model_getter(model_id))
    model = next((item for item in provider.list_models() if item.id == model_id), None)
    if model is None:
        raise KeyError(f"unknown trajectory model {model_id!r}")
    return model
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
    "MissionCompositionAuthorityTransition",
    "MissionCompositionCloseSessionRequest",
    "MissionCompositionClosedSession",
    "MissionCompositionControlAuthorityState",
    "MissionCompositionInspectSessionRequest",
    "MissionCompositionOpenSessionRequest",
    "MissionCompositionResetSessionRequest",
    "MissionCompositionSessionChannel",
    "MissionCompositionSessionAuthorityProfile",
    "MissionCompositionSessionDescriptor",
    "MissionCompositionSessionManager",
    "MissionCompositionSessionObservation",
    "MissionCompositionSessionStepRequest",
    "MissionCompositionSessionStepResult",
    "MissionCompositionSwitchAuthorityRequest",
]
