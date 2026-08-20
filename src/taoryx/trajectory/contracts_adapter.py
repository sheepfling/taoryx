"""TAORYX implementation of the standalone trajectory-contracts protocol.

The public ``taoryx_trajectory_contracts`` package deliberately does not
import TAORYX.  This adapter is the inverse boundary: it translates a
configuration, batch request, and optional streaming lifecycle into the
existing TAORYX Mission Composition runtime without asking a host to know its
internal Pydantic models.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Literal, cast

from pydantic import BaseModel, ValidationError
from taoryx_trajectory_contracts.composition import (
    BatchRunRequest,
    BatchRunResult,
    CompositionConfiguration,
    CompositionOperationDescriptor,
    ContractDiagnostic,
    PreparedCompositionConfiguration,
    TrajectoryChannel,
    TrajectoryEntity,
    TrajectoryEvent,
    TrajectoryModelDescriptor,
    TrajectoryProviderDescriptor,
    TrajectorySample,
    canonical_json_sha256,
)
from taoryx_trajectory_contracts.streaming import (
    AuthorityTransition,
    ClosedStreamingSession,
    CloseStreamingSessionRequest,
    ControlAuthorityState,
    ControlFeedback,
    InspectStreamingSessionRequest,
    OpenStreamingSessionRequest,
    ResetStreamingSessionRequest,
    StreamingAuthorityProfile,
    StreamingChannel,
    StreamingObservation,
    StreamingSessionDescriptor,
    StreamingStepRequest,
    StreamingStepResult,
    SwitchAuthorityRequest,
)

from ..model_authoring import build_model_default_configuration, model_default_configuration_id
from .configuration_contract import (
    ConfigurableTrajectoryProvider,
    ConfigurableTrajectoryProviderRegistry,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelMetadata,
)
from .execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionFailure,
    MissionCompositionFailureResponse,
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResponse,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryObject,
)
from .execution_contract import (
    TrajectoryEvent as NativeTrajectoryEvent,
)
from .session_contract import (
    MissionCompositionAuthorityTransition,
    MissionCompositionClosedSession,
    MissionCompositionCloseSessionRequest,
    MissionCompositionControlAuthorityState,
    MissionCompositionInspectSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionResetSessionRequest,
    MissionCompositionSessionAuthorityProfile,
    MissionCompositionSessionChannel,
    MissionCompositionSessionDescriptor,
    MissionCompositionSessionManager,
    MissionCompositionSessionObservation,
    MissionCompositionSessionStepRequest,
    MissionCompositionSessionStepResult,
    MissionCompositionSwitchAuthorityRequest,
)


class TaoryxTrajectoryContractsAdapterError(RuntimeError):
    """A TAORYX boundary error expressed with portable contract diagnostics."""

    def __init__(self, diagnostics: tuple[ContractDiagnostic, ...]) -> None:
        if not diagnostics:
            raise ValueError("a trajectory-contracts adapter error requires diagnostics")
        self.diagnostics = diagnostics
        super().__init__("; ".join(f"{item.code}: {item.message}" for item in diagnostics))
        ####

    ####


def composition_configuration_from_taoryx(
    provider: ConfigurableTrajectoryProvider,
    configuration: TrajectoryConfigurationInstance,
) -> CompositionConfiguration:
    """Wrap a TAORYX-authored configuration in the public opaque payload.

    The payload remains lossless and JSON-safe.  The duplicated identity fields
    let a host select and cache it without importing TAORYX's configuration
    language; the adapter verifies that they agree before it runs anything.
    """

    metadata = provider.metadata
    schema = provider.get_model_schema(configuration.model_id)
    return CompositionConfiguration(
        configuration_id=configuration.configuration_id,
        provider_id=metadata.id,
        provider_version=metadata.version,
        model_id=configuration.model_id,
        model_version=configuration.model_version,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelity=configuration.fidelity,
        realization_id=configuration.realization_id,
        mission_template_id=configuration.mission_template_id,
        payload=configuration.model_dump(mode="json", by_alias=True),
    )
    ####


class TaoryxTrajectoryContractsAdapter:
    """Expose one existing TAORYX provider through the versioned public API.

    TAORYX's ``taoryx_universal`` profile is intentionally retained in its
    native advertisement gate.  This adapter faithfully reports the exact
    batch and streaming availability it receives; it does not require an
    independent provider to claim streaming merely because TAORYX supports it.
    """

    def __init__(
        self,
        provider: ConfigurableTrajectoryProvider,
        *,
        runner: MissionCompositionRunnerRegistry | None = None,
    ) -> None:
        self._provider = provider
        self._runner = runner
        self._sessions = MissionCompositionSessionManager(provider)
        self._descriptor: TrajectoryProviderDescriptor | None = None
        ####

    @property
    def descriptor(self) -> TrajectoryProviderDescriptor:
        """Return a cacheable public discovery projection for this provider."""

        if self._descriptor is None:
            metadata = self._provider.metadata
            self._descriptor = TrajectoryProviderDescriptor(
                id=metadata.id,
                version=metadata.version,
                name=metadata.name,
                description=metadata.description,
                models=tuple(_model_descriptor(model) for model in self._provider.list_models()),
                claim_boundary=metadata.claim_boundary,
            )
        return self._descriptor
        ####

    def wrap_configuration(self, configuration: TrajectoryConfigurationInstance) -> CompositionConfiguration:
        """Return the public configuration envelope for one TAORYX selection."""

        return composition_configuration_from_taoryx(self._provider, configuration)
        ####

    def build_default_configuration(self, model_id: str) -> CompositionConfiguration:
        """Return TAORYX's deterministic provider-selected runnable default.

        Unlike a schema scaffold, this never asks a generic host to invent
        required launch or mission values. Its values come from a provider-owned
        runnable default and remain subject to the normal public prepare handoff
        before execution.
        """

        try:
            configuration = build_model_default_configuration(
                ConfigurableTrajectoryProviderRegistry((self._provider,)),
                self._provider.metadata.id,
                model_id,
            )
        except Exception as error:  # Preserve a portable failure at the external boundary.
            raise _adapter_error(
                "default-configuration-unavailable",
                str(error),
                phase="configuration",
                details={"model_id": model_id, "exception_type": type(error).__name__},
            ) from error
        return self.wrap_configuration(configuration)
        ####

    def prepare_configuration(self, configuration: CompositionConfiguration) -> PreparedCompositionConfiguration:
        """Validate a public opaque configuration through the selected provider."""

        native = self._native_configuration(configuration)
        try:
            prepared = self._provider.validate_configuration(native)
        except Exception as error:  # Preserve a portable error at the implementation boundary.
            raise _adapter_error(
                "configuration-validation-failed",
                str(error),
                phase="configuration",
                details={"exception_type": type(error).__name__},
            ) from error
        payload = {"taoryx_prepared_configuration": _json_model(prepared)}
        fingerprint = canonical_json_sha256(
            {
                "configuration": configuration.model_dump(mode="json", by_alias=True),
                "provider_payload": payload,
            }
        )
        return PreparedCompositionConfiguration(
            configuration=configuration,
            provider_payload=payload,
            fingerprint=fingerprint,
        )
        ####

    def run_batch(self, request: BatchRunRequest) -> BatchRunResult:
        """Execute one prepared configuration through TAORYX's common runner."""

        native_prepared = self._native_prepared_configuration(request.prepared_configuration)
        metadata = self._provider.metadata
        response = self._runner_for(native_prepared.configuration.model_id).run(
            MissionCompositionRunRequest(
                request_id=request.request_id,
                provider_id=metadata.id,
                provider_version=metadata.version,
                prepared_configuration=native_prepared,
                output=MissionCompositionOutputSelection(
                    mode=request.output.mode,
                    cadence_s=request.output.cadence_s,
                    channels=request.output.channels,
                    telemetry_groups=request.output.telemetry_groups,
                    include_events=request.output.include_events,
                    include_segments=request.output.include_segments,
                    include_spawned_objects=request.output.include_spawned_entities,
                    maximum_samples_per_object=request.output.maximum_samples_per_entity,
                    maximum_objects=request.output.maximum_entities,
                ),
            )
        )
        if isinstance(response, MissionCompositionTrajectoryResponse):
            return _batch_result(response.result, request.prepared_configuration.fingerprint)
        if isinstance(response, MissionCompositionFailureResponse):
            raise _failure_error(response.failure)
        raise _adapter_error(
            "unknown-batch-response",
            f"TAORYX runner returned unsupported response type {type(response).__name__}.",
            phase="execution",
        )
        ####

    def open_stream(self, request: OpenStreamingSessionRequest) -> StreamingSessionDescriptor:
        """Open a stateful TAORYX session from a prepared public configuration."""

        prepared = self._native_prepared_configuration(request.prepared_configuration)
        metadata = self._provider.metadata
        try:
            descriptor = self._sessions.open(
                MissionCompositionOpenSessionRequest(
                    session_id=request.session_id,
                    provider_id=metadata.id,
                    provider_version=metadata.version,
                    prepared_configuration=prepared,
                    seed=request.seed,
                    integration_step_s=request.integration_step_s,
                    authority_profile_id=request.authority_profile_id,
                    command_source_id=request.command_source_id,
                )
            )
        except Exception as error:
            raise _adapter_error(
                "stream-open-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _stream_descriptor(descriptor, request.prepared_configuration.fingerprint)
        ####

    def inspect_stream(self, request: InspectStreamingSessionRequest) -> StreamingObservation:
        """Inspect a TAORYX session without advancing its plant state."""

        try:
            observation = self._sessions.inspect(MissionCompositionInspectSessionRequest(session_id=request.session_id))
        except Exception as error:
            raise _adapter_error(
                "stream-inspect-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _stream_observation(observation)
        ####

    def step_stream(self, request: StreamingStepRequest) -> StreamingStepResult:
        """Advance a TAORYX session for one explicit held action duration."""

        try:
            result = self._sessions.step(
                MissionCompositionSessionStepRequest(
                    session_id=request.session_id,
                    action=request.action,
                    duration_s=request.duration_s,
                    expected_sequence=request.expected_sequence,
                    authority_profile_id=request.authority_profile_id,
                )
            )
        except Exception as error:
            raise _adapter_error(
                "stream-step-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _stream_step_result(result)
        ####

    def switch_stream_authority(self, request: SwitchAuthorityRequest) -> AuthorityTransition:
        """Switch a TAORYX session's declared authority without advancing time."""

        try:
            transition = self._sessions.switch_authority(
                MissionCompositionSwitchAuthorityRequest(
                    session_id=request.session_id,
                    authority_profile_id=request.authority_profile_id,
                    expected_sequence=request.expected_sequence,
                    command_source_id=request.command_source_id,
                )
            )
        except Exception as error:
            raise _adapter_error(
                "stream-authority-switch-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _authority_transition(transition)
        ####

    def reset_stream(self, request: ResetStreamingSessionRequest) -> StreamingObservation:
        """Reset a TAORYX session and return its new zero-boundary state."""

        try:
            observation = self._sessions.reset(
                MissionCompositionResetSessionRequest(session_id=request.session_id, seed=request.seed)
            )
        except Exception as error:
            raise _adapter_error(
                "stream-reset-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _stream_observation(observation)
        ####

    def close_stream(self, request: CloseStreamingSessionRequest) -> ClosedStreamingSession:
        """Close a TAORYX session and return its portable terminal acknowledgement."""

        try:
            closed = self._sessions.close(MissionCompositionCloseSessionRequest(session_id=request.session_id))
        except Exception as error:
            raise _adapter_error(
                "stream-close-failed",
                str(error),
                phase="execution",
                details={"exception_type": type(error).__name__},
            ) from error
        return _closed_stream(closed)
        ####

    def _native_configuration(self, configuration: CompositionConfiguration) -> TrajectoryConfigurationInstance:
        """Parse and cross-check the opaque TAORYX configuration payload."""

        metadata = self._provider.metadata
        if configuration.provider_id != metadata.id or configuration.provider_version != metadata.version:
            raise _adapter_error(
                "provider-identity-mismatch",
                "The configuration selects a different provider identity or version.",
                phase="configuration",
                details={
                    "expected_provider_id": metadata.id,
                    "expected_provider_version": metadata.version,
                    "received_provider_id": configuration.provider_id,
                    "received_provider_version": configuration.provider_version,
                },
            )
        try:
            native = TrajectoryConfigurationInstance.model_validate(configuration.payload)
        except ValidationError as error:
            raise _adapter_error(
                "invalid-taoryx-configuration-payload",
                "The public configuration payload is not a TAORYX configuration instance.",
                phase="configuration",
                details={"validation_error": error.errors(include_url=False)},
            ) from error
        if (
            native.configuration_id != configuration.configuration_id
            or native.model_id != configuration.model_id
            or native.model_version != configuration.model_version
            or native.fidelity != configuration.fidelity
            or native.realization_id != configuration.realization_id
            or native.mission_template_id != configuration.mission_template_id
        ):
            raise _adapter_error(
                "configuration-envelope-mismatch",
                "The public configuration identity does not match its opaque TAORYX payload.",
                phase="configuration",
            )
        try:
            schema = self._provider.get_model_schema(native.model_id)
        except Exception as error:
            raise _adapter_error(
                "unknown-model-configuration",
                str(error),
                phase="configuration",
                details={"model_id": native.model_id},
            ) from error
        if (
            configuration.configuration_schema_id != schema.schema_id
            or configuration.configuration_schema_fingerprint != schema.fingerprint
            or native.schema_fingerprint != schema.fingerprint
        ):
            raise _adapter_error(
                "configuration-schema-mismatch",
                "The configuration does not match the selected provider schema revision.",
                phase="configuration",
                details={"model_id": native.model_id, "expected_schema_fingerprint": schema.fingerprint},
            )
        return native
        ####

    def _native_prepared_configuration(
        self,
        prepared: PreparedCompositionConfiguration,
    ) -> PreparedTrajectoryConfiguration:
        """Revalidate a portable prepared artifact before native execution."""

        native_configuration = self._native_configuration(prepared.configuration)
        payload = prepared.provider_payload.get("taoryx_prepared_configuration")
        if not isinstance(payload, Mapping):
            raise _adapter_error(
                "missing-taoryx-prepared-payload",
                "The prepared configuration was not produced by the TAORYX adapter.",
                phase="preflight",
            )
        try:
            serialized = PreparedTrajectoryConfiguration.model_validate(payload)
        except ValidationError as error:
            raise _adapter_error(
                "invalid-taoryx-prepared-payload",
                "The portable prepared payload is not a TAORYX prepared configuration.",
                phase="preflight",
                details={"validation_error": error.errors(include_url=False)},
            ) from error
        if serialized.configuration != native_configuration:
            raise _adapter_error(
                "prepared-configuration-mismatch",
                "The portable prepared payload names a different authored configuration.",
                phase="preflight",
            )
        try:
            fresh = self._provider.validate_configuration(native_configuration)
        except Exception as error:
            raise _adapter_error(
                "prepared-configuration-revalidation-failed",
                str(error),
                phase="preflight",
                details={"exception_type": type(error).__name__},
            ) from error
        if serialized.fingerprint != fresh.fingerprint:
            raise _adapter_error(
                "stale-or-tampered-prepared-configuration",
                "The TAORYX prepared payload no longer agrees with current provider validation.",
                phase="preflight",
            )
        return fresh
        ####

    def _runner_for(self, model_id: str) -> MissionCompositionRunnerRegistry:
        """Resolve a provider-owned batch registry only when execution is requested."""

        if self._runner is None:
            build_runner = getattr(self._provider, "build_runner", None)
            if not callable(build_runner):
                raise _adapter_error(
                    "batch-runner-not-implemented",
                    f"TAORYX provider {self._provider.metadata.id!r} does not expose build_runner().",
                    phase="preflight",
                    details={"model_id": model_id},
                )
            try:
                runner = build_runner()
            except Exception as error:
                raise _adapter_error(
                    "batch-runner-construction-failed",
                    str(error),
                    phase="preflight",
                    details={"model_id": model_id, "exception_type": type(error).__name__},
                ) from error
            if not isinstance(runner, MissionCompositionRunnerRegistry):
                raise _adapter_error(
                    "invalid-batch-runner",
                    "TAORYX provider build_runner() did not return a MissionCompositionRunnerRegistry.",
                    phase="preflight",
                    details={"model_id": model_id, "actual_type": type(runner).__name__},
                )
            self._runner = runner
        return self._runner
        ####

    ####


def _model_descriptor(model: TrajectoryModelMetadata) -> TrajectoryModelDescriptor:
    """Translate TAORYX's rich metadata into the independent discovery core."""

    operations = tuple(
        _operation_descriptor(mission, operation)
        for mission in model.mission_templates
        for operation in mission.operations
    )
    return TrajectoryModelDescriptor(
        id=model.id,
        version=model.version,
        name=model.name,
        description=model.description,
        model_kind=model.model_kind,
        status=model.status,
        configuration_schema_id=model.configuration_schema_id,
        output_schema_id=model.output_schema_id,
        default_configuration_id=model_default_configuration_id(model.id),
        operations=operations,
        tags=model.tags,
        claim_boundary=model.claim_boundary,
    )
    ####


def _operation_descriptor(
    mission: TrajectoryMissionTemplateMetadata,
    operation: TrajectoryMissionOperationMetadata,
) -> CompositionOperationDescriptor:
    """Project exact TAORYX availability without applying a universal policy."""

    execution_kind: Literal["native", "adapter", "replay", "provider_defined"]
    if operation.common_runner_status == "registered":
        execution_kind = "replay" if operation.execution_mode == "core_batch_replay" else "native"
    elif operation.common_runner_status == "adapter_required":
        execution_kind = "adapter"
    else:
        execution_kind = "provider_defined"
    return CompositionOperationDescriptor(
        mission_template_id=mission.id,
        fidelity=operation.fidelity,
        realization_id=operation.realization_id,
        operation=operation.operation,
        availability="available" if operation.status == "available" else "blocked",
        execution_kind=execution_kind,
        executor_id=operation.executor_id,
        blockers=operation.blockers,
        claim_boundary=operation.claim_boundary,
    )
    ####


def _batch_result(result: MissionCompositionTrajectoryResult, configuration_fingerprint: str) -> BatchRunResult:
    """Map a native multi-object batch result without losing extensions."""

    return BatchRunResult(
        provider_id=result.provider_id,
        provider_version=result.provider_version,
        request_id=result.request_id,
        configuration_fingerprint=configuration_fingerprint,
        primary_entity_id=result.primary_object_id,
        status=cast(Any, result.status),
        entities=tuple(_trajectory_entity(item) for item in result.objects),
        events=tuple(_trajectory_event(item) for item in result.events),
        diagnostics=tuple(_diagnostic(item) for item in result.diagnostics),
        claim_boundary=result.claim_boundary,
        extensions={
            "taoryx_native_configuration_fingerprint": result.configuration_fingerprint,
            "relationships": [_json_model(item) for item in result.relationships],
        },
    )
    ####


def _trajectory_entity(entity: TrajectoryObject) -> TrajectoryEntity:
    """Map a native entity and retain non-common lineage/segment evidence."""

    return TrajectoryEntity(
        id=entity.object_id,
        model_id=entity.model_id,
        realization_id=entity.realization_id,
        fidelity=entity.fidelity,
        status=cast(Any, entity.status),
        role=entity.role,
        parent_id=entity.parent_object_id,
        channels=tuple(_trajectory_channel(item) for item in entity.channels),
        samples=tuple(_trajectory_sample(item) for item in entity.samples),
        claim_boundary=entity.claim_boundary,
        extensions={
            "name": entity.name,
            "deployment_id": entity.deployment_id,
            "spawn_event_id": entity.spawn_event_id,
            "death_event_id": entity.death_event_id,
            "active_from_s": entity.active_from_s,
            "active_to_s": entity.active_to_s,
            "terminal_disposition": entity.terminal_disposition,
            "segments": [_json_model(item) for item in entity.segments],
            "provenance": entity.provenance,
        },
    )
    ####


def _trajectory_channel(channel: TrajectoryChannelMetadata) -> TrajectoryChannel:
    """Map one returned native channel plus its full TAORYX semantics."""

    return TrajectoryChannel(
        id=channel.id,
        unit=channel.unit,
        frame=channel.frame,
        data_type=cast(Any, channel.data_type),
        shape=channel.shape,
        description=channel.description,
        extensions={
            "channel_class": channel.channel_class,
            "telemetry_group": channel.telemetry_group,
            "quantity": channel.quantity,
            "sampling_semantics": channel.sampling_semantics,
            "interpolation": channel.interpolation,
        },
    )
    ####


def _trajectory_sample(sample: Any) -> TrajectorySample:
    """Map a native sample while asserting the mandatory shared state exists."""

    if sample.standard_ecef is None:
        raise _adapter_error(
            "missing-standard-ecef-state",
            "TAORYX attempted to emit a native sample without standard ECEF kinematics and attitude.",
            phase="projection",
        )
    return TrajectorySample(
        time_s=sample.time_s,
        values=sample.values,
        segment_instance_id=sample.segment_instance_id,
        standard_ecef=sample.standard_ecef,
    )
    ####


def _trajectory_event(event: NativeTrajectoryEvent) -> TrajectoryEvent:
    """Map a native event and retain TAORYX deployment/segment extension fields."""

    return TrajectoryEvent(
        id=event.id,
        time_s=event.time_s,
        category=event.category,
        kind=event.kind,
        entity_id=event.object_id,
        parent_entity_id=event.parent_object_id,
        detail=event.detail,
        data=event.data,
        extensions={
            "deployment_id": event.deployment_id,
            "segment_instance_id": event.segment_instance_id,
        },
    )
    ####


def _stream_descriptor(
    descriptor: MissionCompositionSessionDescriptor,
    configuration_fingerprint: str,
) -> StreamingSessionDescriptor:
    """Map a TAORYX session descriptor into the optional streaming extension."""

    return StreamingSessionDescriptor(
        session_id=descriptor.session_id,
        provider_id=descriptor.provider_id,
        provider_version=descriptor.provider_version,
        model_id=descriptor.model_id,
        mission_template_id=descriptor.mission_template_id,
        realization_id=descriptor.realization_id,
        fidelity=descriptor.fidelity,
        configuration_fingerprint=configuration_fingerprint,
        state_owner=cast(Any, descriptor.state_owner),
        integration_step_s=descriptor.integration_step_s,
        action_channels=tuple(_stream_channel(item) for item in descriptor.action_schema),
        observation_channels=tuple(_stream_channel(item) for item in descriptor.observation_schema),
        authority_profiles=tuple(_authority_profile(item) for item in descriptor.authority_profiles),
        active_authority_profile_id=descriptor.active_authority_profile_id,
        initial_observation=_stream_observation(descriptor.initial_observation),
        claim_boundary=descriptor.claim_boundary,
        extensions={
            "taoryx_native_configuration_fingerprint": descriptor.configuration_fingerprint,
            "deterministic_reset": descriptor.deterministic_reset,
            "reset_semantics": descriptor.reset_semantics,
            "timestep_semantics": descriptor.timestep_semantics,
            "supports_spawned_entities": descriptor.supports_spawned_entities,
            "action_schema_projection": descriptor.action_schema_projection,
            "default_authority_profile_id": descriptor.default_authority_profile_id,
            "command_source_id": descriptor.command_source_id,
            "agent_action_space": None
            if descriptor.agent_action_space is None
            else _json_model(descriptor.agent_action_space),
        },
    )
    ####


def _stream_channel(channel: MissionCompositionSessionChannel) -> StreamingChannel:
    """Map a native session channel while retaining its detailed value-space."""

    return StreamingChannel(
        id=channel.id,
        direction=channel.direction,
        description=channel.description,
        data_type=cast(Any, channel.data_type),
        shape=channel.shape,
        unit=channel.unit,
        frame=channel.frame,
        minimum=channel.minimum,
        maximum=channel.maximum,
        choices=channel.choices,
        sampling="held_action" if channel.direction == "action" else "truth_boundary",
        metadata=_json_model(channel),
    )
    ####


def _authority_profile(profile: MissionCompositionSessionAuthorityProfile) -> StreamingAuthorityProfile:
    """Map a declared TAORYX semantic authority profile."""

    return StreamingAuthorityProfile(
        id=profile.id,
        authority=profile.authority,
        action_ids=profile.action_ids,
        command_owner=cast(Any, profile.command_owner),
        selection_scope=cast(Any, profile.selection_scope),
        switching_policy=cast(Any, profile.switching_policy),
        description=profile.description,
        claim_boundary=profile.claim_boundary,
        metadata=_json_model(profile),
    )
    ####


def _stream_observation(observation: MissionCompositionSessionObservation) -> StreamingObservation:
    """Map one native committed boundary with its shared ECEF state."""

    if observation.standard_ecef is None:
        raise _adapter_error(
            "missing-standard-ecef-state",
            "TAORYX attempted to emit a session observation without standard ECEF state.",
            phase="projection",
        )
    return StreamingObservation(
        session_id=observation.session_id,
        sequence=observation.sequence,
        time_s=observation.time_s,
        lifecycle=observation.lifecycle,
        values=observation.values,
        events=observation.events,
        spawned_entity_ids=observation.spawned_entity_ids,
        diagnostics=tuple(_diagnostic(item) for item in observation.diagnostics),
        control_authority=_control_authority(observation.control_authority),
        standard_ecef=observation.standard_ecef,
    )
    ####


def _control_authority(
    state: MissionCompositionControlAuthorityState | None,
) -> ControlAuthorityState | None:
    """Map runtime authority availability without hiding TAORYX-specific detail."""

    if state is None:
        return None
    return ControlAuthorityState(
        active_profile_id=state.active_profile_id,
        command_source_id=state.command_source_id,
        command_owner=cast(Any, state.command_owner),
        available_action_ids=state.available_action_ids,
        unavailable_action_reasons=state.unavailable_action_reasons,
        selection_scope=cast(Any, state.selection_scope),
        switching_policy=cast(Any, state.switching_policy),
        metadata={
            "lowering_chain": list(state.lowering_chain),
            "scheme_id": state.scheme_id,
            "runtime_availability": state.runtime_availability,
            "phase_id": state.phase_id,
            "availability_reason_codes": list(state.availability_reason_codes),
        },
    )
    ####


def _stream_step_result(result: MissionCompositionSessionStepResult) -> StreamingStepResult:
    """Map a native action-to-observation transition."""

    return StreamingStepResult(
        session_id=result.session_id,
        sequence=result.sequence,
        time_start_s=result.time_start_s,
        time_end_s=result.time_end_s,
        requested_action=result.requested_action,
        applied_action=result.applied_action,
        observation=_stream_observation(result.observation),
        diagnostics=tuple(_diagnostic(item) for item in result.diagnostics),
        control_feedback=tuple(_control_feedback(item) for item in result.control_feedback),
        authority_profile_id=result.authority_profile_id,
        command_source_id=result.command_source_id,
        lowering_evidence=result.lowering_evidence,
        extensions={"events": list(result.events), "lowered_action": result.lowered_action},
    )
    ####


def _control_feedback(feedback: Any) -> ControlFeedback:
    """Map one detailed control acceptance/readback record."""

    return ControlFeedback(
        channel_id=feedback.channel_id,
        availability=feedback.availability,
        request_present=feedback.request_present,
        requested_value=feedback.requested_value,
        applied_present=feedback.applied_present,
        applied_value=feedback.applied_value,
        disposition=feedback.disposition,
        reason_codes=feedback.reason_codes,
        feedback_channel_id=feedback.feedback_channel_id,
        achievement_status=feedback.achievement_status,
        achieved_value=feedback.achieved_value,
    )
    ####


def _authority_transition(transition: MissionCompositionAuthorityTransition) -> AuthorityTransition:
    """Map a non-advancing native authority handoff."""

    return AuthorityTransition(
        session_id=transition.session_id,
        sequence=transition.sequence,
        time_s=transition.time_s,
        previous_authority_profile_id=transition.previous_authority_profile_id,
        active_authority_profile_id=transition.active_authority_profile_id,
        command_source_id=transition.command_source_id,
        action_channels=tuple(_stream_channel(item) for item in transition.action_schema),
        observation=_stream_observation(transition.observation),
        extensions={
            "transfer_semantics": transition.transfer_semantics,
            "agent_action_space": None
            if transition.agent_action_space is None
            else _json_model(transition.agent_action_space),
        },
    )
    ####


def _closed_stream(closed: MissionCompositionClosedSession) -> ClosedStreamingSession:
    """Map TAORYX's terminal acknowledgement exactly."""

    return ClosedStreamingSession(
        session_id=closed.session_id,
        final_sequence=closed.final_sequence,
        final_time_s=closed.final_time_s,
    )
    ####


def _diagnostic(diagnostic: MissionCompositionDiagnostic) -> ContractDiagnostic:
    """Translate a native diagnostic and preserve its richer recovery context."""

    return ContractDiagnostic(
        severity=cast(Any, diagnostic.severity),
        code=diagnostic.code,
        message=diagnostic.message,
        phase=cast(Any, diagnostic.phase),
        path=diagnostic.path,
        retryable=diagnostic.recoverability == "retryable",
        details={
            "recoverability": diagnostic.recoverability,
            "hint": diagnostic.hint,
            "provider_id": diagnostic.provider_id,
            "model_id": diagnostic.model_id,
            "object_id": diagnostic.object_id,
            "segment_instance_id": diagnostic.segment_instance_id,
            "taoryx_details": diagnostic.details,
        },
    )
    ####


def _failure_error(failure: MissionCompositionFailure) -> TaoryxTrajectoryContractsAdapterError:
    """Convert a common-runner failure envelope into a portable exception."""

    return TaoryxTrajectoryContractsAdapterError(tuple(_diagnostic(item) for item in failure.diagnostics))
    ####


def _adapter_error(
    code: str,
    message: str,
    *,
    phase: Literal["configuration", "preflight", "execution", "projection"],
    details: dict[str, Any] | None = None,
) -> TaoryxTrajectoryContractsAdapterError:
    """Build one stable diagnostic for an adapter-owned boundary failure."""

    return TaoryxTrajectoryContractsAdapterError(
        (
            ContractDiagnostic(
                severity="error",
                code=code,
                message=message or "TAORYX adapter operation failed without a detailed message.",
                phase=phase,
                details=details or {},
            ),
        )
    )
    ####


def _json_model(value: BaseModel) -> dict[str, Any]:
    """Return JSON-compatible Pydantic content for an extension payload."""

    payload = value.model_dump(mode="json", by_alias=True)
    if not isinstance(payload, dict):
        raise TypeError(f"expected Pydantic model {type(value).__name__} to serialize as an object")
    return payload
    ####


__all__ = [
    "TaoryxTrajectoryContractsAdapter",
    "TaoryxTrajectoryContractsAdapterError",
    "composition_configuration_from_taoryx",
]
