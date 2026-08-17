"""Taoryx Mission Composition bridge for the source-compatible SRAAM6 plug-in."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any, Literal

from taoryx.runtime import SensorBinding, SensorBus, SensorClockSpec
from taoryx.sensor_api import MeasurementPacket, SensorBuildContext
from taoryx.sensor_plugins.relative_state import RelativeStateTrackerConfig, RelativeStateTrackerSensor
from taoryx.trajectory.configuration_contract import (
    ConfigurationBound,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationInterval,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationRole,
    ConfigurationValueType,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryEntityOutputMetadata,
    TrajectoryFidelityMetadata,
    TrajectoryMissionOperationMetadata,
    TrajectoryMissionTemplateMetadata,
    TrajectoryModelCapabilities,
    TrajectoryModelMetadata,
    TrajectoryModelPresentationMetadata,
    TrajectoryModelPropertyMetadata,
    TrajectoryOutputChannelMetadata,
    TrajectoryOutputSchema,
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionOutputSelection,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    resolve_output_selection,
)
from taoryx.trajectory.mission_composition import (
    MissionCompositionClosedSession,
    MissionCompositionCloseSessionRequest,
    MissionCompositionInspectSessionRequest,
    MissionCompositionOpenSessionRequest,
    MissionCompositionResetSessionRequest,
    MissionCompositionSessionChannel,
    MissionCompositionSessionDescriptor,
    MissionCompositionSessionObservation,
    MissionCompositionSessionStepRequest,
    MissionCompositionSessionStepResult,
)

from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import blocked_control_advertisement, source_managed_control_advertisement
from .output_metadata import cadac_output_quantity
from .session_authority import (
    CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
    CADAC_SOURCE_PROGRAM_PROFILE_ID,
    source_managed_session_authority_fields,
    source_managed_session_authority_state,
    validate_source_managed_session_request,
)
from .sraam6 import Sraam6RunResult, Sraam6Sample, Sraam6ScenarioSession, Sraam6SourceDefinition, Sraam6TargetSample
from .sraam6_plugin import Sraam6PluginOverrides, Sraam6VehiclePlugin

CADAC_PROVIDER_ID = "cadac"
SRAAM6_MODEL_ID = "cadac.sraam6.missile"
SRAAM6_TARGET_MODEL_ID = "cadac.sraam6.target"
SRAAM6_MODEL_VERSION = "0.6.0"
SRAAM6_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
SRAAM6_REALIZATION_ID = "cadac-standard-four-fin"
SRAAM6_FIN_REALIZATION_ID = SRAAM6_REALIZATION_ID
SRAAM6_TVC_REALIZATION_ID = "cadac-optional-tvc"
SRAAM6_MISSION_ID = "air_intercept"
SRAAM6_FIN_MISSION_ID = SRAAM6_MISSION_ID
SRAAM6_EXECUTOR_ID = "cadac.sraam6.standard_fin.batch"
SRAAM6_SESSION_EXECUTOR_ID = "cadac.sraam6.standard_fin.session"
SRAAM6_FIN_PHASE_ID = "fin_control"
SRAAM6_TVC_PHASE_ID = "optional_tvc"
_LOCAL_NED_FRAME_ID = "cadac.local_ned"
_BODY_FRAME_ID = "cadac.body_xyz"
_NATIVE_SENSOR_ID = "sraam6-native-relative-state"


@dataclass(slots=True)
class _Sraam6SessionRecord:
    """Provider-owned SRAAM6 source and raw-track state for one open session."""

    descriptor: MissionCompositionSessionDescriptor
    prepared: PreparedTrajectoryConfiguration
    definition: Sraam6SourceDefinition
    source: Sraam6ScenarioSession
    sensor_bus: SensorBus
    seed: int
    sequence: int
    observation: MissionCompositionSessionObservation
    lifecycle: str = "ready"
    closed: bool = False


class CadacSraam6MissionCompositionProvider:
    """Self-describing Mission Composition provider for one installed SRAAM6 case."""

    def __init__(self, plugin: Sraam6VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish SRAAM6 provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)
        self._sessions: dict[str, _Sraam6SessionRecord] = {}
        self._metadata = TrajectoryProviderMetadata(
            id=CADAC_PROVIDER_ID,
            name="CADAC vehicle plug-ins",
            version=SRAAM6_MODEL_VERSION,
            description="Source-ordered CADAC SRAAM6 physical-effector engagement exposed through Taoryx Mission Composition.",
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="CADAC Vehicle Plug-ins",
                short_name="CADAC",
                summary="CADAC source actors exposed through typed Taoryx vehicle plug-in contracts.",
                organization="Taoryx integration layer",
                categories=("aerospace", "reference-models", "vehicle-plugins"),
            ),
            status="development",
            tags=("cadac", "sraam6", "physical-surfaces", "vehicle-plugin"),
            execution_contract=SRAAM6_EXECUTOR_ID,
            model_count=1,
            provenance="missiondesignsolutions/CADAC SRAAM6 source model plus Taoryx source-compatible reconstruction",
            claim_boundary=(
                "The four-fin realization is executable through the exact provider/model runner. The optional TVC source path is "
                "discoverable and validation-only. Compiled-CADAC numerical parity remains a separate promotion gate."
            ),
        )

    ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity and execution claim boundary."""

        return self._metadata

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return the exact installed SRAAM6 model."""

        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return the SRAAM6 portable configuration grammar."""

        if model_id != SRAAM6_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return SRAAM6 core truth and optional telemetry channels."""

        if model_id != SRAAM6_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate exact source phase, fidelity, realization, and mission selection."""

        if configuration.fidelity != SRAAM6_FIDELITY_ID:
            raise ValueError(f"SRAAM6 requires fidelity {SRAAM6_FIDELITY_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        if not isinstance(prepared.resolved, Mapping):
            raise ValueError("SRAAM6 resolved configuration root must be a mapping")
        ####
        phase = prepared.resolved.get("source_phase")
        if phase == SRAAM6_FIN_PHASE_ID:
            if configuration.realization_id not in {None, SRAAM6_FIN_REALIZATION_ID}:
                raise ValueError(f"SRAAM6 fin phase requires realization {SRAAM6_FIN_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id not in {None, SRAAM6_FIN_MISSION_ID}:
                raise ValueError(f"SRAAM6 fin phase requires mission {SRAAM6_FIN_MISSION_ID!r}")
            ####
        elif phase == SRAAM6_TVC_PHASE_ID:
            if configuration.realization_id not in {None, SRAAM6_TVC_REALIZATION_ID}:
                raise ValueError(f"SRAAM6 TVC phase requires realization {SRAAM6_TVC_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id is not None:
                raise ValueError("SRAAM6 optional TVC is validation-only and has no executable mission template")
            ####
        else:
            raise ValueError(f"unsupported SRAAM6 source phase {phase!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Execute one validated four-fin engagement through the installed runtime."""

        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != SRAAM6_MODEL_ID:
            raise ValueError("SRAAM6 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("SRAAM6 prepared configuration does not match provider validation")
        ####
        if not isinstance(prepared.resolved, Mapping) or prepared.resolved.get("source_phase") != SRAAM6_FIN_PHASE_ID:
            raise ValueError("SRAAM6 optional TVC is discoverable but not registered for batch execution")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####

    def open_session(
        self,
        request: MissionCompositionOpenSessionRequest,
        *,
        advertised_provider_version: str | None = None,
    ) -> MissionCompositionSessionDescriptor:
        """Open a persistent standard-fin SRAAM6/TARGET3 source engagement."""

        if request.session_id in self._sessions:
            raise ValueError(f"SRAAM6 session {request.session_id!r} already exists")
        ####
        expected_version = SRAAM6_MODEL_VERSION if advertised_provider_version is None else advertised_provider_version
        if request.provider_id != CADAC_PROVIDER_ID or request.provider_version != expected_version:
            raise ValueError("SRAAM6 session request names another provider version")
        if request.model_id != SRAAM6_MODEL_ID:
            raise ValueError("SRAAM6 session request names another model")
        validate_source_managed_session_request(request.authority_profile_id, request.command_source_id)
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("SRAAM6 session prepared configuration is stale")
        if not isinstance(prepared.resolved, Mapping) or prepared.resolved.get("source_phase") != SRAAM6_FIN_PHASE_ID:
            raise ValueError("SRAAM6 persistent sessions support only the standard executable four-fin phase")
        ####
        definition = self._plugin.prepare_definition(_overrides_from_resolved(prepared.resolved))
        if not math.isclose(request.integration_step_s, definition.integration_step_s, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                "SRAAM6 session integration_step_s must equal the source compatibility step "
                f"{definition.integration_step_s:.17g} s"
            )
        ####
        seed = 0 if request.seed is None else request.seed
        source = Sraam6ScenarioSession(definition)
        sensor_bus = _sraam6_sensor_bus(seed, definition.integration_step_s)
        initial_context = source.native_sensor_context()
        sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        observation = _sraam6_session_observation(
            session_id=request.session_id,
            sequence=0,
            source=source,
            sensor_bus=sensor_bus,
            lifecycle="ready",
        )
        descriptor = MissionCompositionSessionDescriptor(
            session_id=request.session_id,
            provider_id=CADAC_PROVIDER_ID,
            provider_version=request.provider_version,
            model_id=SRAAM6_MODEL_ID,
            mission_template_id=SRAAM6_FIN_MISSION_ID,
            realization_id=SRAAM6_FIN_REALIZATION_ID,
            fidelity=SRAAM6_FIDELITY_ID,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=definition.integration_step_s,
            supports_spawned_entities=False,
            action_schema=(),
            **source_managed_session_authority_fields(),
            observation_schema=_sraam6_session_observation_schema(),
            initial_observation=observation,
            claim_boundary=(
                "The provider owns persistent SRAAM6 missile, TARGET3, seeker, controller, actuator, source-event, "
                "and previous-target-pass communication state. A native raw relative-state SensorBus packet is emitted "
                "at every committed source substep and does not replace source seeker filtering or lock state."
            ),
        )
        self._sessions[request.session_id] = _Sraam6SessionRecord(
            descriptor=descriptor,
            prepared=prepared,
            definition=definition,
            source=source,
            sensor_bus=sensor_bus,
            seed=seed,
            sequence=0,
            observation=observation,
        )
        return descriptor
        ####

    create_session = open_session

    def inspect_session(
        self,
        request: MissionCompositionInspectSessionRequest | str,
    ) -> MissionCompositionSessionObservation:
        """Inspect a committed SRAAM6 source boundary without advancing it."""

        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        return self._session(session_id).observation
        ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance a persistent SRAAM6 engagement across exact source substeps."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("SRAAM6 session is closed")
        if record.lifecycle in {"completed", "failed"}:
            raise ValueError("SRAAM6 session is already terminal")
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise ValueError(f"SRAAM6 expected sequence {request.expected_sequence}, current sequence is {record.sequence}")
        if request.action:
            raise ValueError("SRAAM6 source-managed session does not accept external action channels")
        validate_source_managed_session_request(request.authority_profile_id, None)
        ####
        substeps = _sraam6_session_substeps(request.duration_s, record.definition.integration_step_s)
        time_start_s = record.source.sim_time_s
        previous_context = record.source.native_sensor_context()
        events: list[str] = []
        for _ in range(substeps):
            traces = record.source.advance(record.definition.integration_step_s)
            current_context = record.source.native_sensor_context()
            record.sensor_bus.accepted_context(
                _NATIVE_SENSOR_ID,
                current_context.host,
                current_context,
                previous_truth=previous_context.host,
                previous_context=previous_context,
            )
            previous_context = current_context
            events.extend(_sraam6_session_events(traces, record.source))
            if record.source.completed:
                break
            ####
        ####
        record.sequence += 1
        record.lifecycle = _sraam6_session_lifecycle(record.source)
        observation = _sraam6_session_observation(
            session_id=request.session_id,
            sequence=record.sequence,
            source=record.source,
            sensor_bus=record.sensor_bus,
            lifecycle=record.lifecycle,
            events=tuple(events),
        )
        record.observation = observation
        return MissionCompositionSessionStepResult(
            session_id=request.session_id,
            sequence=record.sequence,
            time_start_s=time_start_s,
            time_end_s=record.source.sim_time_s,
            requested_action={},
            applied_action={},
            observation=observation,
            events=observation.events,
            diagnostics=observation.diagnostics,
            authority_profile_id=CADAC_SOURCE_PROGRAM_PROFILE_ID,
            command_source_id=CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
            lowering_evidence=source_managed_session_authority_state().model_dump(mode="json"),
        )
        ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reconstruct source and raw sensor state from the prepared configuration."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("SRAAM6 session is closed")
        selected_seed = record.seed if request.seed is None else request.seed
        record.source.reset()
        record.sensor_bus = _sraam6_sensor_bus(selected_seed, record.definition.integration_step_s)
        initial_context = record.source.native_sensor_context()
        record.sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        record.seed = selected_seed
        record.sequence = 0
        record.lifecycle = "ready"
        record.observation = _sraam6_session_observation(
            session_id=request.session_id,
            sequence=0,
            source=record.source,
            sensor_bus=record.sensor_bus,
            lifecycle="ready",
        )
        return record.observation
        ####

    def close_session(
        self,
        request: MissionCompositionCloseSessionRequest | str,
    ) -> MissionCompositionClosedSession:
        """Close a persistent SRAAM6 session without dropping its acknowledgement."""

        session_id = request.session_id if isinstance(request, MissionCompositionCloseSessionRequest) else request
        record = self._session(session_id)
        if not record.closed:
            record.closed = True
            record.lifecycle = "closed"
            record.observation = record.observation.model_copy(update={"lifecycle": "closed"})
        return MissionCompositionClosedSession(
            session_id=session_id,
            final_sequence=record.sequence,
            final_time_s=record.observation.time_s,
        )
        ####

    def active_session_ids(self) -> tuple[str, ...]:
        return tuple(sorted(session_id for session_id, record in self._sessions.items() if not record.closed))
        ####

    def has_session(self, session_id: str) -> bool:
        return session_id in self._sessions
        ####

    def session_sensor_packets(
        self,
        session_id: str,
        *,
        delivered: bool = True,
    ) -> tuple[MeasurementPacket[Any], ...]:
        """Return the retained standard SensorBus packets for an open or closed session."""

        return self._session(session_id).sensor_bus.packets(_NATIVE_SENSOR_ID, delivered=delivered)
        ####

    def _session(self, session_id: str) -> _Sraam6SessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ValueError(f"SRAAM6 session {session_id!r} does not exist") from error
        ####


####


def _sraam6_session_substeps(duration_s: float, source_step_s: float) -> int:
    """Validate one caller hold against the exact source timestep."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("SRAAM6 session duration_s must be positive and finite")
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"SRAAM6 session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    return steps
    ####


def _sraam6_sensor_bus(seed: int, cadence_s: float) -> SensorBus:
    """Build the native raw target-track SensorBus for one source engagement."""

    tracker = RelativeStateTrackerSensor(
        RelativeStateTrackerConfig(target_id="sraam6-target"),
        SensorBuildContext(_NATIVE_SENSOR_ID, seed),
    )
    bus = SensorBus()
    bus.register(
        SensorBinding(
            _NATIVE_SENSOR_ID,
            "sraam6-missile-1",
            SensorClockSpec(_NATIVE_SENSOR_ID, "tracking", cadence_s=cadence_s),
            tracker,
            provenance={
                "provider": "relative-state-track",
                "frame": _LOCAL_NED_FRAME_ID,
                "execution": "persistent-source-session",
                "claim_boundary": "Raw committed target geometry only; CADAC source seeker gimbal, acquisition, lock, filtering, and previous-target-pass lag remain source behavior.",
            },
            rng_seed=seed,
        )
    )
    return bus
    ####


def _sraam6_session_observation_schema() -> tuple[MissionCompositionSessionChannel, ...]:
    """Declare source-owned missile, target, controller, and raw sensor ports."""

    scalar = {"topology": "continuous", "representation": "scalar"}
    vector3 = {"topology": "product", "representation": "vector3", "components": 3}
    vector4 = {"topology": "product", "representation": "quaternion", "components": 4}
    fin_vector4 = {"topology": "product", "representation": "vector4", "components": 4}
    opaque = {"topology": "opaque", "representation": "json"}
    return (
        MissionCompositionSessionChannel(
            id="position_ned_m", direction="observation", description="Committed SRAAM6 local-NED missile position.",
            quantity=cadac_output_quantity("position_ned_m", "m"), unit="m", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="velocity_ned_mps", direction="observation", description="Committed SRAAM6 local-NED missile velocity.",
            quantity=cadac_output_quantity("velocity_ned_mps", "m/s"), unit="m/s", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="quaternion_wxyz", direction="observation", description="Committed scalar-first body-from-local attitude quaternion.",
            shape=(4,), value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="body_rates_rad_s", direction="observation", description="Committed missile body rates.",
            quantity=cadac_output_quantity("body_rates_rad_s", "rad/s"), unit="rad/s", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="target_position_ned_m", direction="observation", description="Committed TARGET3 local-NED position.",
            quantity=cadac_output_quantity("target_position_ned_m", "m"), unit="m", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="target_velocity_ned_mps", direction="observation", description="Committed TARGET3 local-NED velocity.",
            quantity=cadac_output_quantity("target_velocity_ned_mps", "m/s"), unit="m/s", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_control_deg", direction="observation", description="Source-controller requested roll, pitch, and yaw control.",
            unit="deg", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="achieved_control_deg", direction="observation", description="Actuator-achieved roll, pitch, and yaw control equivalents.",
            unit="deg", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_fins_deg", direction="observation", description="Source mixer requested four physical fin angles.",
            unit="deg", shape=(4,), value_space=fin_vector4,
        ),
        MissionCompositionSessionChannel(
            id="achieved_fins_deg", direction="observation", description="Actuator-achieved four physical fin angles driving the airframe.",
            unit="deg", shape=(4,), value_space=fin_vector4,
        ),
        MissionCompositionSessionChannel(
            id="normal_command_g", direction="observation", description="Source-owned normal acceleration command.",
            quantity=cadac_output_quantity("normal_command_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_command_g", direction="observation", description="Source-owned lateral acceleration command.",
            quantity=cadac_output_quantity("lateral_command_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="normal_acceleration_g", direction="observation", description="Realized normal specific-force response.",
            quantity=cadac_output_quantity("normal_acceleration_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_acceleration_g", direction="observation", description="Realized lateral specific-force response.",
            quantity=cadac_output_quantity("lateral_acceleration_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="source_seeker_state", direction="observation", description="Source seeker mode, source LOS/filter state, and target range.",
            data_type="json", sampling_semantics="discrete_sample", value_space=opaque,
        ),
        MissionCompositionSessionChannel(
            id="native_relative_state_track", direction="observation", description="Latest delivered native raw relative-state SensorBus packet.",
            data_type="json", sampling_semantics="discrete_sample", value_space=opaque,
        ),
    )
    ####


def _sraam6_session_observation(
    *,
    session_id: str,
    sequence: int,
    source: Sraam6ScenarioSession,
    sensor_bus: SensorBus,
    lifecycle: str,
    events: tuple[str, ...] = (),
) -> MissionCompositionSessionObservation:
    """Project one committed source boundary into standard session observations."""

    sample = source.sample
    target = source.target_sample
    return MissionCompositionSessionObservation(
        session_id=session_id,
        sequence=sequence,
        time_s=source.sim_time_s,
        lifecycle=lifecycle,
        values={
            "position_ned_m": sample.position_ned_m,
            "velocity_ned_mps": sample.velocity_ned_mps,
            "quaternion_wxyz": sample.quaternion_wxyz,
            "body_rates_rad_s": sample.body_rates_rad_s,
            "target_position_ned_m": target.position_ned_m,
            "target_velocity_ned_mps": target.velocity_ned_mps,
            "requested_control_deg": sample.requested_control_deg,
            "achieved_control_deg": sample.achieved_control_deg,
            "requested_fins_deg": sample.requested_fins_deg,
            "achieved_fins_deg": sample.achieved_fins_deg,
            "normal_command_g": sample.normal_command_g,
            "lateral_command_g": sample.lateral_command_g,
            "normal_acceleration_g": sample.normal_acceleration_g,
            "lateral_acceleration_g": sample.lateral_acceleration_g,
            "source_seeker_state": {
                "seeker_mode": sample.seeker_mode,
                "guidance_mode": sample.guidance_mode,
                "target_range_m": sample.target_range_m,
                "closing_speed_mps": sample.closing_speed_mps,
                "seeker_pointing_pitch_rad": sample.seeker_pointing_pitch_rad,
                "seeker_pointing_yaw_rad": sample.seeker_pointing_yaw_rad,
                "seeker_los_rate_pitch_rad_s": sample.seeker_los_rate_pitch_rad_s,
                "seeker_los_rate_yaw_rad_s": sample.seeker_los_rate_yaw_rad_s,
            },
            "native_relative_state_track": _sraam6_native_track_value(sensor_bus),
        },
        events=events,
        control_authority=source_managed_session_authority_state(),
    )
    ####


def _sraam6_native_track_value(sensor_bus: SensorBus) -> dict[str, object]:
    packet: MeasurementPacket[Any] = sensor_bus.packets(_NATIVE_SENSOR_ID)[-1]
    return {
        "sensor_id": packet.sensor_id,
        "sequence": packet.sequence,
        "sampled_at_s": packet.sampled_at_s,
        "available_at_s": packet.available_at_s,
        "schema_id": packet.schema_id,
        "port": packet.port,
        "valid": packet.valid,
        "invalid_reason": packet.invalid_reason,
        "payload": None if packet.payload is None else asdict(packet.payload),
    }
    ####


def _sraam6_session_lifecycle(source: Sraam6ScenarioSession) -> str:
    if source.terminated_reason == "nonfinite_state":
        return "failed"
    if source.completed:
        return "completed"
    return "active"
    ####


def _sraam6_session_events(
    traces: tuple[object, ...],
    source: Sraam6ScenarioSession,
) -> tuple[str, ...]:
    events = tuple(
        f"cadac-source-event:{trace.actor}:{trace.event_index}:{trace.watch_variable}:{trace.time_s:.17g}"
        for trace in traces
        if hasattr(trace, "actor") and hasattr(trace, "event_index") and hasattr(trace, "watch_variable") and hasattr(trace, "time_s")
    )
    if source.intercept is not None:
        return (*events, f"cadac-source-terminal:intercept:{source.intercept.time_s:.17g}")
    if source.terminated_reason in {"ground_impact", "nonfinite_state"}:
        return (*events, f"cadac-source-terminal:{source.terminated_reason}:{source.sim_time_s:.17g}")
    return events
    ####


def build_default_sraam6_configuration(
    provider: CadacSraam6MissionCompositionProvider,
    *,
    configuration_id: str = "sraam6-default",
    phase_id: str = SRAAM6_FIN_PHASE_ID,
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one caller-owned source-phase configuration with semantic overrides."""

    if phase_id not in {SRAAM6_FIN_PHASE_ID, SRAAM6_TVC_PHASE_ID}:
        raise ValueError(f"unsupported SRAAM6 source phase {phase_id!r}")
    ####
    supplied = dict(overrides or {})
    routes = {
        "missile_position_ned_m": ("missile", "position_ned_m"),
        "missile_speed_mps": ("missile", "speed_mps"),
        "missile_yaw_deg": ("missile", "yaw_deg"),
        "missile_pitch_deg": ("missile", "pitch_deg"),
        "missile_roll_deg": ("missile", "roll_deg"),
        "missile_alpha_deg": ("missile", "alpha_deg"),
        "missile_beta_deg": ("missile", "beta_deg"),
        "missile_body_rates_deg_s": ("missile", "body_rates_deg_s"),
        "target_position_ned_m": ("target", "position_ned_m"),
        "target_speed_mps": ("target", "speed_mps"),
        "target_heading_deg": ("target", "heading_deg"),
        "target_flight_path_deg": ("target", "flight_path_deg"),
        "target_option": ("target", "aircraft_option"),
        "target_turn_g": ("target", "turn_g"),
        "navigation_gain": ("guidance", "navigation_gain"),
        "fin_position_limit_deg": ("actuation", "fin_position_limit_deg"),
        "fin_rate_limit_deg_s": ("actuation", "fin_rate_limit_deg_s"),
        "fin_natural_frequency_rad_s": ("actuation", "fin_natural_frequency_rad_s"),
        "fin_damping_ratio": ("actuation", "fin_damping_ratio"),
        "seeker_acquisition_range_m": ("seeker", "acquisition_range_m"),
        "seeker_filter_gain_per_s": ("seeker", "filter_gain_per_s"),
        "seeker_filter_natural_frequency_rad_s": ("seeker", "filter_natural_frequency_rad_s"),
        "seeker_filter_damping_ratio": ("seeker", "filter_damping_ratio"),
        "structural_limit_g": ("controller", "structural_limit_g"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown SRAAM6 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    root_values: dict[str, object] = {
        "source_phase": ConfigurationParameterValue(value=phase_id),
    }
    for group_id, values in grouped.items():
        root_values[group_id] = ConfigurationGroupValue(
            values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in values.items()}
        )
    ####
    schema = provider.get_model_schema(SRAAM6_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=SRAAM6_MODEL_ID,
        model_version=SRAAM6_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=SRAAM6_FIDELITY_ID,
        realization_id=(SRAAM6_FIN_REALIZATION_ID if phase_id == SRAAM6_FIN_PHASE_ID else SRAAM6_TVC_REALIZATION_ID),
        mission_template_id=SRAAM6_FIN_MISSION_ID if phase_id == SRAAM6_FIN_PHASE_ID else None,
        root=ConfigurationGroupValue(values=root_values),
    )


####


def register_sraam6_mission_composition(
    provider: CadacSraam6MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact four-fin SRAAM6 batch executor without fallback."""

    registry.register(CADAC_PROVIDER_ID, SRAAM6_MODEL_ID, provider.execute_batch)


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    value_type: ConfigurationValueType = "number",
    unit: str | None = None,
    frame: str | None = None,
    role: ConfigurationRole | None = None,
    interval: ConfigurationInterval | None = None,
) -> ConfigurationParameterSchema:
    resolved_role: ConfigurationRole = role or ("constraint" if parameter_id in {"end_time_s", "sample_step_s"} else "initialization")
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        value_type=value_type,
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        interval=interval,
        role=resolved_role,
        compatible_fidelities=(SRAAM6_FIDELITY_ID,),
        frame=frame,
        provenance="lowered from the installed CADAC SRAAM6 source case",
    )


####


def _build_configuration_schema(plugin: Sraam6VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    target = source.target
    root = ConfigurationGroupSchema(
        id="sraam6",
        label="SRAAM6 air-intercept configuration",
        description="Source-default SRAAM6/TARGET3 engagement with explicit physical-effector phase selection.",
        children=(
            ConfigurationParameterSchema(
                id="source_phase",
                label="Source control realization",
                description="Four-fin execution or validation-only optional TVC source realization.",
                value_type="enum",
                choices=(SRAAM6_FIN_PHASE_ID, SRAAM6_TVC_PHASE_ID),
                required=False,
                default=SRAAM6_FIN_PHASE_ID,
                default_declared=True,
                role="variant",
                compatible_fidelities=(SRAAM6_FIDELITY_ID,),
                provenance="missiondesignsolutions/CADAC/SRAAM6 input and actuator/TVC modules",
            ),
            ConfigurationGroupSchema(
                id="missile",
                label="Missile initial state",
                children=(
                    _parameter(
                        "position_ned_m",
                        "Position NED",
                        "Initial missile local-NED position.",
                        default=initial.position_ned_m,
                        value_type="vector3",
                        unit="m",
                        frame=_LOCAL_NED_FRAME_ID,
                    ),
                    _parameter("speed_mps", "Speed", "Initial missile speed.", default=initial.speed_mps, unit="m/s"),
                    _parameter("yaw_deg", "Yaw", "Initial missile body yaw.", default=initial.yaw_deg, unit="deg"),
                    _parameter("pitch_deg", "Pitch", "Initial missile body pitch.", default=initial.pitch_deg, unit="deg"),
                    _parameter("roll_deg", "Roll", "Initial missile body roll.", default=initial.roll_deg, unit="deg"),
                    _parameter("alpha_deg", "Angle of attack", "Initial missile angle of attack.", default=initial.alpha_deg, unit="deg"),
                    _parameter("beta_deg", "Sideslip", "Initial missile sideslip.", default=initial.beta_deg, unit="deg"),
                    _parameter(
                        "body_rates_deg_s",
                        "Body rates",
                        "Initial roll/pitch/yaw body rates.",
                        default=initial.body_rates_deg_s,
                        value_type="vector3",
                        unit="deg/s",
                        frame=_BODY_FRAME_ID,
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="target",
                label="TARGET3 aircraft",
                children=(
                    _parameter(
                        "position_ned_m",
                        "Target position NED",
                        "Initial target local-NED position.",
                        default=target.position_ned_m,
                        value_type="vector3",
                        unit="m",
                        frame=_LOCAL_NED_FRAME_ID,
                    ),
                    _parameter("speed_mps", "Target speed", "Initial TARGET3 speed.", default=target.speed_mps, unit="m/s"),
                    _parameter("heading_deg", "Target heading", "Initial target heading.", default=target.heading_deg, unit="deg"),
                    _parameter("flight_path_deg", "Target flight path", "Initial target flight-path angle.", default=target.flight_path_deg, unit="deg"),
                    _parameter(
                        "aircraft_option",
                        "Target maneuver mode",
                        "TARGET3 straight-flight or horizontal-g-turn mode.",
                        default=target.aircraft_option,
                        value_type="integer",
                    ),
                    _parameter("turn_g", "Target turn command", "TARGET3 horizontal turn command.", default=target.turn_g, unit="g"),
                ),
            ),
            ConfigurationGroupSchema(
                id="guidance",
                label="Guidance",
                children=(_parameter("navigation_gain", "Navigation gain", "SRAAM6 proportional-navigation gain.", default=source.guidance.navigation_gain),),
            ),
            ConfigurationGroupSchema(
                id="actuation",
                label="Physical-fin tuning",
                description=(
                    "Source-backed physical actuator limits and dynamics. These values define a reproducible vehicle "
                    "variant before a run; they are not live fin commands."
                ),
                children=(
                    _parameter(
                        "fin_position_limit_deg",
                        "Fin position limit",
                        "Maximum physical four-fin deflection used by the source actuator.",
                        default=source.actuator.position_limit_deg,
                        unit="deg",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                    _parameter(
                        "fin_rate_limit_deg_s",
                        "Fin rate limit",
                        "Maximum physical four-fin slew rate used by the source actuator.",
                        default=source.actuator.rate_limit_deg_s,
                        unit="deg/s",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                    _parameter(
                        "fin_natural_frequency_rad_s",
                        "Fin natural frequency",
                        "Second-order physical fin-actuator natural frequency.",
                        default=source.actuator.natural_frequency_rad_s,
                        unit="rad/s",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                    _parameter(
                        "fin_damping_ratio",
                        "Fin damping ratio",
                        "Second-order physical fin-actuator damping ratio.",
                        default=source.actuator.damping_ratio,
                        role="variant",
                        interval=_nonnegative_interval(),
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="seeker",
                label="Source seeker tuning",
                description=(
                    "Source dynamic-seeker acquisition and LOS-filter parameters. The native relative-state sensor "
                    "output remains the Taoryx sensor interface; these settings configure the CADAC source model."
                ),
                children=(
                    _parameter(
                        "acquisition_range_m",
                        "Acquisition range",
                        "Maximum source seeker acquisition range.",
                        default=source.seeker.acquisition_range_m,
                        unit="m",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                    _parameter(
                        "filter_gain_per_s",
                        "LOS filter gain",
                        "Source seeker line-of-sight rate-filter gain.",
                        default=source.seeker.filter_gain_per_s,
                        unit="1/s",
                        role="variant",
                        interval=_nonnegative_interval(),
                    ),
                    _parameter(
                        "filter_natural_frequency_rad_s",
                        "LOS filter natural frequency",
                        "Source seeker line-of-sight rate-filter natural frequency.",
                        default=source.seeker.filter_natural_frequency_rad_s,
                        unit="rad/s",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                    _parameter(
                        "filter_damping_ratio",
                        "LOS filter damping ratio",
                        "Source seeker line-of-sight rate-filter damping ratio.",
                        default=source.seeker.filter_damping_ratio,
                        role="variant",
                        interval=_nonnegative_interval(),
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="controller",
                label="Source controller tuning",
                children=(
                    _parameter(
                        "structural_limit_g",
                        "Structural acceleration limit",
                        "Source controller acceleration limit applied before physical fin allocation.",
                        default=source.control.structural_limit_g,
                        unit="g",
                        role="variant",
                        interval=_positive_interval(),
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="runtime",
                label="Runtime",
                children=(
                    _parameter("end_time_s", "End time", "Maximum source simulation time.", default=source.end_time_s, unit="s"),
                    _parameter(
                        "sample_step_s",
                        "Output cadence",
                        "Requested accepted output cadence.",
                        default=source.plot_step_s or max(source.integration_step_s, 0.02),
                        unit="s",
                    ),
                ),
            ),
        ),
    )
    return TrajectoryConfigurationSchema(
        model_id=SRAAM6_MODEL_ID,
        model_version=SRAAM6_MODEL_VERSION,
        supported_fidelities=(SRAAM6_FIDELITY_ID,),
        root=root,
        claim_boundary=(
            "Source initial truth and bounded semantic overrides are portable. The provider owns source module order, "
            "stored-derivative integration, event progression, bus lag, and physical-effector realization."
        ),
    )


####


def _output_channel(
    channel_id: str,
    label: str,
    description: str,
    *,
    unit: str | None = None,
    shape: tuple[int, ...] = (),
    frame: str | None = None,
    data_type: Literal["float64", "int64", "boolean", "string", "json"] = "float64",
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=channel_id,
        label=label,
        description=description,
        quantity=cadac_output_quantity(channel_id, unit, data_type),
        canonical_unit=unit,
        display_unit=unit,
        data_type=data_type,
        shape=shape,
        frame=frame,
        availability=("guaranteed" if channel_id in {"position_ned_m", "velocity_ned_mps", "quaternion_wxyz", "body_rates_rad_s"} else "conditional"),
        compatible_fidelities=(SRAAM6_FIDELITY_ID,),
        compatible_realizations=(SRAAM6_FIN_REALIZATION_ID,),
        compatible_mission_templates=(SRAAM6_FIN_MISSION_ID,),
        operations=("batch", "step"),
        provenance="SRAAM6 source-compatible runner accepted truth",
        claim_boundary="Python reconstruction output; compiled-CADAC numerical parity remains a separate promotion gate.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output_channel("position_ned_m", "Position NED", "Missile local-NED position.", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel("velocity_ned_mps", "Velocity NED", "Missile local-NED velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel("quaternion_wxyz", "Attitude quaternion", "Scalar-first body-from-local attitude quaternion.", shape=(4,)),
        _output_channel("body_rates_rad_s", "Body rates", "Roll, pitch, and yaw body rates.", unit="rad/s", shape=(3,), frame=_BODY_FRAME_ID),
    )
    flight = (
        _output_channel("speed_mps", "Speed", "Missile speed.", unit="m/s"),
        _output_channel("altitude_m", "Altitude", "Flat-Earth altitude.", unit="m"),
        _output_channel("mach", "Mach", "Missile Mach number."),
        _output_channel("dynamic_pressure_pa", "Dynamic pressure", "Missile dynamic pressure.", unit="Pa"),
        _output_channel("alpha_deg", "Angle of attack", "Body-axis angle of attack.", unit="deg"),
        _output_channel("beta_deg", "Sideslip", "Body-axis sideslip angle.", unit="deg"),
        _output_channel("total_alpha_deg", "Total incidence", "Total aeroballistic incidence angle.", unit="deg"),
        _output_channel("aerodynamic_roll_deg", "Aerodynamic roll", "Aeroballistic roll angle.", unit="deg"),
    )
    control = (
        _output_channel("requested_control_deg", "Requested control axes", "Requested roll/pitch/yaw control coordinates.", unit="deg", shape=(3,)),
        _output_channel("requested_fins_deg", "Requested fins", "Four source-mixed requested fin positions.", unit="deg", shape=(4,)),
        _output_channel(
            "achieved_fins_deg", "Achieved fins", "Four achieved physical fin positions after actuator dynamics and limits.", unit="deg", shape=(4,)
        ),
        _output_channel("achieved_control_deg", "Achieved control axes", "Roll/pitch/yaw control reconstructed from achieved fins.", unit="deg", shape=(3,)),
        _output_channel("normal_command_g", "Normal command", "Guidance normal-acceleration command.", unit="g"),
        _output_channel("lateral_command_g", "Lateral command", "Guidance lateral-acceleration command.", unit="g"),
        _output_channel("normal_acceleration_g", "Normal acceleration", "Achieved normal specific force.", unit="g"),
        _output_channel("lateral_acceleration_g", "Lateral acceleration", "Achieved lateral specific force.", unit="g"),
        _output_channel("autopilot_mode", "Autopilot mode", "Source SRAAM6 control mode.", data_type="int64"),
        _output_channel("force_body_n", "Body force", "Total non-gravitational body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output_channel("moment_body_nm", "Body moment", "Total aerodynamic body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
    )
    resources = (
        _output_channel("mass_kg", "Mass", "Source-deck missile mass.", unit="kg"),
        _output_channel("center_of_gravity_m", "Center of gravity", "Source-deck center-of-gravity station from nose.", unit="m"),
        _output_channel("roll_inertia_kg_m2", "Roll inertia", "Source-deck roll moment of inertia.", unit="kg*m^2"),
        _output_channel("pitch_inertia_kg_m2", "Pitch inertia", "Source-deck transverse moment of inertia.", unit="kg*m^2"),
        _output_channel("thrust_n", "Thrust", "Altitude-adjusted source motor thrust.", unit="N"),
        _output_channel("propulsion_mode", "Propulsion mode", "Source propulsion mode.", data_type="int64"),
    )
    seeker = (
        _output_channel("target_range_m", "Target range", "Seeker-time range to the assigned TARGET3 actor.", unit="m"),
        _output_channel("closing_speed_mps", "Closing speed", "Signed relative closing speed.", unit="m/s"),
        _output_channel("seeker_mode", "Seeker mode", "Source seeker acquisition/lock/blind mode.", data_type="int64"),
        _output_channel("guidance_mode", "Guidance mode", "Source midcourse or terminal guidance mode.", data_type="int64"),
        _output_channel("seeker_pointing_pitch_rad", "Seeker pitch", "Seeker pointing pitch angle.", unit="rad"),
        _output_channel("seeker_pointing_yaw_rad", "Seeker yaw", "Seeker pointing yaw angle.", unit="rad"),
        _output_channel("seeker_los_rate_pitch_rad_s", "Pitch LOS rate", "Filtered pitch line-of-sight rate.", unit="rad/s"),
        _output_channel("seeker_los_rate_yaw_rad_s", "Yaw LOS rate", "Filtered yaw line-of-sight rate.", unit="rad/s"),
    )
    telemetry = (*flight, *control, *resources, *seeker)
    return TrajectoryOutputSchema(
        model_id=SRAAM6_MODEL_ID,
        model_version=SRAAM6_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_state",
                label="Flight state",
                description="Atmospheric and aerodynamic state diagnostics.",
                channel_ids=tuple(item.id for item in flight),
                default_selected=False,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="physical_control",
                label="Physical control",
                description="Requested and achieved physical-fin control closure.",
                channel_ids=tuple(item.id for item in control),
                default_selected=False,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="resources",
                label="Resources",
                description="Motor and time-varying mass-property telemetry.",
                channel_ids=tuple(item.id for item in resources),
                default_selected=False,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="seeker_guidance",
                label="Seeker and guidance",
                description="Target geometry, seeker modes, LOS-rate estimates, and guidance mode.",
                channel_ids=tuple(item.id for item in seeker),
                default_selected=False,
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(
            supports_multiple_entities=True,
            supports_dynamic_spawning=False,
            supports_recursive_spawning=False,
            maximum_descendant_depth=0,
            child_output_schema_policy="not_applicable",
            includes_spawn_initial_state=False,
            includes_lifecycle_events=True,
        ),
        claim_boundary="The SRAAM6 missile and independently propagated TARGET3 aircraft are returned as separate root objects.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=SRAAM6_FIDELITY_ID,
        label="CADAC SRAAM6 physical-effector 6-DoF",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="rigid_body_6dof",
        control_realization="physical_effector_allocation",
        promotion_status="development",
        operations=("validate", "batch", "step"),
        profile_id="cadac.sraam6.source-compatibility",
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        required_operations=("rigid-body propagation", "physical fin actuator dynamics", "source-order closed-loop engagement"),
        claim_boundary="The four-fin plant is executable at T4; the optional TVC path remains validation-only.",
    )
    fin_realization = TrajectoryRealizationMetadata(
        id=SRAAM6_FIN_REALIZATION_ID,
        label="CADAC four-fin source realization",
        description="Source-ordered SRAAM6 guidance, autopilot, four-fin mixing, second-order actuators, and aerodynamic closure.",
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        controls=source_managed_control_advertisement(
            mission_ids=(SRAAM6_FIN_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/SRAAM6",),
            operations=("batch", "step"),
            claim_boundary="SRAAM6 source guidance and fin control are internal to the persistent compatibility runtime.",
        ),
        fidelity_aliases=(SRAAM6_FIDELITY_ID,),
        mission_template_ids=(SRAAM6_FIN_MISSION_ID,),
        operations=("validate", "batch", "step"),
        native_factory_ids=(SRAAM6_EXECUTOR_ID, SRAAM6_SESSION_EXECUTOR_ID),
        source_refs=("missiondesignsolutions/CADAC/SRAAM6",),
        claim_boundary="Available means the Python reconstruction is executable; compiled-source numerical parity is pending.",
    )
    tvc_realization = TrajectoryRealizationMetadata(
        id=SRAAM6_TVC_REALIZATION_ID,
        label="CADAC optional TVC source realization",
        description="Source optional thrust-vector-control path with physical nozzle position and rate states.",
        status="blocked",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("thrust_vectoring",),
        controls=blocked_control_advertisement(
            claim_boundary="The optional TVC source path has no installed CADAC control runtime.",
        ),
        fidelity_aliases=(SRAAM6_FIDELITY_ID,),
        operations=("validate",),
        blockers=(
            "optional TVC is not registered as an exact SRAAM6 Mission Composition executor",
            "a dedicated source case and closed-loop parity fixture are not yet installed",
        ),
        source_refs=("missiondesignsolutions/CADAC/SRAAM6/tvc.cpp",),
        claim_boundary="The source phase is discoverable and validates, but no batch execution is implied.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=SRAAM6_FIN_MISSION_ID,
        name="SRAAM6 physical-fin air intercept",
        description="One SRAAM6 missile against one TARGET3 aircraft using source event, bus, seeker, guidance, control, and actuator order.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("rate_control_launch", "acceleration_control_midcourse", "terminal_intercept"),
        compatible_fidelities=(SRAAM6_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=SRAAM6_FIDELITY_ID,
                realization_id=SRAAM6_FIN_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_compatible_validation",
                common_runner_status="not_available",
                claim_boundary="Validation is provider-local and does not execute the engagement.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=SRAAM6_FIDELITY_ID,
                realization_id=SRAAM6_FIN_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatible_batch",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=SRAAM6_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed four-fin SRAAM6 actor runtime.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=SRAAM6_FIDELITY_ID,
                realization_id=SRAAM6_FIN_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="source_compatible_persistent_session",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=SRAAM6_SESSION_EXECUTOR_ID,
                claim_boundary="Step dispatch owns the SRAAM6/TARGET3 source state and accepts only whole source-step holds with source-managed controls.",
            ),
        ),
        provenance="CADAC SRAAM6 input, deck, and module semantics",
        claim_boundary="The recipe covers the standard four-fin missile/TARGET3 engagement; optional TVC is a separate blocked realization.",
    )
    return TrajectoryModelMetadata(
        id=SRAAM6_MODEL_ID,
        name="CADAC SRAAM6 missile",
        version=SRAAM6_MODEL_VERSION,
        description="Physical-fin rigid-body CADAC SRAAM6 air-intercept missile exposed as a Taoryx vehicle plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC SRAAM6",
            short_name="SRAAM6",
            summary="Source-ordered physical-fin 6-DoF missile with an independently propagated TARGET3 aircraft.",
            category="missile",
            subcategory="air_intercept",
            sort_key="cadac-sraam6",
            badges=("CADAC", "6-DoF", "physical fins", "development"),
            default_fidelity_id=SRAAM6_FIDELITY_ID,
            default_mission_template_id=SRAAM6_FIN_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "quaternion_wxyz", "body_rates_rad_s"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="CADAC actor identifier from the upstream package.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="MISSILE6",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/SRAAM6",),
                    provenance="CADAC SRAAM6 source package",
                    claim_boundary="Identity metadata only; it does not establish numerical equivalence.",
                ),
                TrajectoryModelPropertyMetadata(
                    id="physical_effectors",
                    label="Physical effectors",
                    description="Default executable source realization uses four independently lagged aerodynamic fins.",
                    semantic_role="capability",
                    value_type="string",
                    value_kind="declared",
                    value="four aerodynamic fins with fixed mixing and second-order actuator dynamics",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/SRAAM6/actuator.cpp",),
                    provenance="CADAC SRAAM6 actuator module",
                    claim_boundary="Applies to the executable fin realization only.",
                ),
            ),
        ),
        family_id=SRAAM6_MODEL_ID,
        physical_family="air_intercept_missile",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "missile", "rigid_body_6dof", "physical_surfaces"),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("rate_control_launch", "acceleration_control_midcourse", "terminal_intercept"),
            termination_modes=("intercept", "ground_impact", "end_time", "nonfinite_state"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=True,
        ),
        realizations=(fin_realization, tvc_realization),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_NED_FRAME_ID,
                name="CADAC flat-Earth local NED",
                description="Local North-East-Down Cartesian frame used by SRAAM6 Flat6 and TARGET3 Flat3.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source-case local reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/SRAAM6/newton.cpp", "missiondesignsolutions/CADAC/SRAAM6/flat3_modules.cpp"),
                provenance="CADAC flat-Earth convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC missile body XYZ",
                description="Right-handed forward-right-down missile body frame.",
                frame_kind="body",
                axes=("forward", "right", "down"),
                handedness="right",
                origin="missile center of mass",
                orientation="body-fixed forward-right-down",
                source_refs=("missiondesignsolutions/CADAC/SRAAM6/kinematics.cpp",),
                provenance="CADAC SRAAM6 body-axis convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/SRAAM6",),
        provenance="CADAC SRAAM6 source-compatible Python reconstruction",
        claim_boundary=(
            "The physical-fin plug-in is executable at development status. Complete dynamic-seeker optics and compiled-CADAC "
            "numerical parity remain independent promotion gates; optional TVC is validation-only."
        ),
    )


####


def _overrides_from_resolved(resolved: Mapping[str, object]) -> Sraam6PluginOverrides:
    missile = _group(resolved, "missile")
    target = _group(resolved, "target")
    guidance = _group(resolved, "guidance")
    actuation = _group(resolved, "actuation")
    seeker = _group(resolved, "seeker")
    controller = _group(resolved, "controller")
    runtime = _group(resolved, "runtime")
    return Sraam6PluginOverrides(
        missile_position_ned_m=_vector3(missile["position_ned_m"]),
        missile_speed_mps=float(missile["speed_mps"]),
        missile_yaw_deg=float(missile["yaw_deg"]),
        missile_pitch_deg=float(missile["pitch_deg"]),
        missile_roll_deg=float(missile["roll_deg"]),
        missile_alpha_deg=float(missile["alpha_deg"]),
        missile_beta_deg=float(missile["beta_deg"]),
        missile_body_rates_deg_s=_vector3(missile["body_rates_deg_s"]),
        target_position_ned_m=_vector3(target["position_ned_m"]),
        target_speed_mps=float(target["speed_mps"]),
        target_heading_deg=float(target["heading_deg"]),
        target_flight_path_deg=float(target["flight_path_deg"]),
        target_option=int(target["aircraft_option"]),
        target_turn_g=float(target["turn_g"]),
        navigation_gain=float(guidance["navigation_gain"]),
        fin_position_limit_deg=float(actuation["fin_position_limit_deg"]),
        fin_rate_limit_deg_s=float(actuation["fin_rate_limit_deg_s"]),
        fin_natural_frequency_rad_s=float(actuation["fin_natural_frequency_rad_s"]),
        fin_damping_ratio=float(actuation["fin_damping_ratio"]),
        seeker_acquisition_range_m=float(seeker["acquisition_range_m"]),
        seeker_filter_gain_per_s=float(seeker["filter_gain_per_s"]),
        seeker_filter_natural_frequency_rad_s=float(seeker["filter_natural_frequency_rad_s"]),
        seeker_filter_damping_ratio=float(seeker["filter_damping_ratio"]),
        structural_limit_g=float(controller["structural_limit_g"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"SRAAM6 resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("SRAAM6 vector configuration requires exactly three components")
    ####
    return (float(value[0]), float(value[1]), float(value[2]))


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id in {"position_ned_m", "acquisition_range_m"}:
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
        "alpha_deg",
        "beta_deg",
        "heading_deg",
        "flight_path_deg",
        "fin_position_limit_deg",
    }:
        return "deg"
    ####
    if parameter_id == "body_rates_deg_s":
        return "deg/s"
    ####
    if parameter_id == "fin_rate_limit_deg_s":
        return "deg/s"
    ####
    if parameter_id in {"fin_natural_frequency_rad_s", "filter_natural_frequency_rad_s"}:
        return "rad/s"
    ####
    if parameter_id == "filter_gain_per_s":
        return "1/s"
    ####
    if parameter_id in {"turn_g", "structural_limit_g"}:
        return "g"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _positive_interval() -> ConfigurationInterval:
    """Return the source-model domain for a strictly positive scalar."""

    return ConfigurationInterval(minimum=ConfigurationBound(value=0.0, inclusive=False))


####


def _nonnegative_interval() -> ConfigurationInterval:
    """Return the source-model domain for a nonnegative scalar."""

    return ConfigurationInterval(minimum=ConfigurationBound(value=0.0))


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Sraam6RunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=SRAAM6_FIDELITY_ID,
        operation="batch",
        realization_id=SRAAM6_FIN_REALIZATION_ID,
        mission_template_id=SRAAM6_FIN_MISSION_ID,
    )
    missile_channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    target_channels = _target_runtime_channels(request.output)
    missile_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={item.id: _missile_value(item.id, sample) for item in selected},
        )
        for sample in run.samples
    )
    target_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={item.id: _target_value(item.id, sample) for item in target_channels},
        )
        for sample in run.target_samples
    )
    missile_object_id = "sraam6-missile-1"
    target_object_id = "sraam6-target-1"
    events = _trajectory_events(run, missile_object_id, target_object_id)
    missile_terminated = run.terminated_reason != "end_time"
    target_terminated = run.terminated_reason == "intercept"
    missile_object = TrajectoryObject(
        object_id=missile_object_id,
        model_id=SRAAM6_MODEL_ID,
        realization_id=SRAAM6_FIN_REALIZATION_ID,
        name="CADAC SRAAM6 missile",
        role="interceptor",
        fidelity=SRAAM6_FIDELITY_ID,
        status="terminated" if missile_terminated else "completed",
        active_from_s=missile_samples[0].time_s,
        active_to_s=missile_samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=missile_channels,
        samples=missile_samples,
        provenance="CADAC SRAAM6 source-compatible four-fin runner",
        claim_boundary="Missile truth closes through achieved physical fin positions; compiled-CADAC golden parity is pending.",
    )
    target_object = TrajectoryObject(
        object_id=target_object_id,
        model_id=SRAAM6_TARGET_MODEL_ID,
        realization_id=SRAAM6_FIN_REALIZATION_ID,
        name="CADAC TARGET3 aircraft",
        role="target",
        fidelity="point_mass_3dof",
        status="terminated" if target_terminated else "completed",
        active_from_s=target_samples[0].time_s,
        active_to_s=target_samples[-1].time_s,
        terminal_disposition="intercept" if target_terminated else "source_run_end",
        channels=target_channels,
        samples=target_samples,
        provenance="CADAC SRAAM6 TARGET3 source-compatible runner",
        claim_boundary="TARGET3 is an independently propagated root object under the source vehicle-major scheduler.",
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="SRAAM6 is executing through the source-compatible Python reconstruction; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=SRAAM6_MODEL_ID,
            object_id=missile_object_id,
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-dynamic-seeker-partial",
            message="The acquisition/lock/blind state machine and LOS-rate dynamics execute; complete optical-error and aimpoint/gimbal-head geometry remains outside the current reconstruction.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=SRAAM6_MODEL_ID,
            object_id=missile_object_id,
        ),
    )
    result_status = "failed" if run.terminated_reason == "nonfinite_state" else "terminated" if missile_terminated else "completed"
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=SRAAM6_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=SRAAM6_MODEL_ID,
        primary_object_id=missile_object_id,
        status=result_status,
        objects=(missile_object, target_object),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="SRAAM6 and TARGET3 are independent source-scheduled root objects; no parent/child lineage is inferred.",
    )


####


def _runtime_channel(
    channel: TrajectoryOutputChannelMetadata,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryChannelMetadata:
    telemetry_groups = {channel_id: group.id for group in output_schema.telemetry_groups for channel_id in group.channel_ids}
    return TrajectoryChannelMetadata(
        id=channel.id,
        channel_class="core_state" if channel in output_schema.core_channels else "telemetry",
        telemetry_group=telemetry_groups.get(channel.id),
        quantity=channel.quantity,
        unit=channel.canonical_unit,
        data_type=channel.data_type,
        shape=channel.shape,
        frame=channel.frame,
        sampling_semantics=channel.sampling_semantics,
        interpolation=channel.interpolation,
        description=channel.description,
    )


####


def _target_runtime_channels(selection: MissionCompositionOutputSelection) -> tuple[TrajectoryChannelMetadata, ...]:
    core = (
        TrajectoryChannelMetadata(
            id="position_ned_m", channel_class="core_state", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID, description="TARGET3 local-NED position."
        ),
        TrajectoryChannelMetadata(
            id="velocity_ned_mps", channel_class="core_state", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID, description="TARGET3 local-NED velocity."
        ),
    )
    if selection.mode == "core":
        return core
    ####
    telemetry = (
        TrajectoryChannelMetadata(id="speed_mps", channel_class="telemetry", telemetry_group="target_kinematics", unit="m/s", description="Target speed."),
        TrajectoryChannelMetadata(id="heading_deg", channel_class="telemetry", telemetry_group="target_kinematics", unit="deg", description="Target heading."),
        TrajectoryChannelMetadata(
            id="flight_path_deg", channel_class="telemetry", telemetry_group="target_kinematics", unit="deg", description="Target flight-path angle."
        ),
        TrajectoryChannelMetadata(
            id="altitude_m", channel_class="telemetry", telemetry_group="target_kinematics", unit="m", description="Target flat-Earth altitude."
        ),
        TrajectoryChannelMetadata(
            id="bank_deg", channel_class="telemetry", telemetry_group="target_kinematics", unit="deg", description="Target achieved bank angle."
        ),
        TrajectoryChannelMetadata(
            id="normal_load_g", channel_class="telemetry", telemetry_group="target_kinematics", unit="g", description="Target achieved normal load factor."
        ),
        TrajectoryChannelMetadata(
            id="alive",
            channel_class="telemetry",
            telemetry_group="target_kinematics",
            data_type="boolean",
            interpolation="step",
            description="Target lifecycle state.",
        ),
    )
    return (*core, *telemetry)


####


def _missile_value(channel_id: str, sample: Sraam6Sample) -> object:
    mapping: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "quaternion_wxyz": sample.quaternion_wxyz,
        "body_rates_rad_s": sample.body_rates_rad_s,
        "speed_mps": sample.speed_mps,
        "altitude_m": sample.altitude_m,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "alpha_deg": sample.alpha_deg,
        "beta_deg": sample.beta_deg,
        "total_alpha_deg": sample.total_alpha_deg,
        "aerodynamic_roll_deg": sample.aerodynamic_roll_deg,
        "requested_control_deg": sample.requested_control_deg,
        "requested_fins_deg": sample.requested_fins_deg,
        "achieved_fins_deg": sample.achieved_fins_deg,
        "achieved_control_deg": sample.achieved_control_deg,
        "force_body_n": sample.force_body_n,
        "moment_body_nm": sample.moment_body_nm,
        "mass_kg": sample.mass_kg,
        "center_of_gravity_m": sample.center_of_gravity_m,
        "roll_inertia_kg_m2": sample.roll_inertia_kg_m2,
        "pitch_inertia_kg_m2": sample.pitch_inertia_kg_m2,
        "thrust_n": sample.thrust_n,
        "propulsion_mode": sample.propulsion_mode,
        "seeker_mode": sample.seeker_mode,
        "guidance_mode": sample.guidance_mode,
        "autopilot_mode": sample.autopilot_mode,
        "target_range_m": sample.target_range_m,
        "closing_speed_mps": sample.closing_speed_mps,
        "seeker_pointing_pitch_rad": sample.seeker_pointing_pitch_rad,
        "seeker_pointing_yaw_rad": sample.seeker_pointing_yaw_rad,
        "seeker_los_rate_pitch_rad_s": sample.seeker_los_rate_pitch_rad_s,
        "seeker_los_rate_yaw_rad_s": sample.seeker_los_rate_yaw_rad_s,
        "normal_command_g": sample.normal_command_g,
        "lateral_command_g": sample.lateral_command_g,
        "normal_acceleration_g": sample.normal_acceleration_g,
        "lateral_acceleration_g": sample.lateral_acceleration_g,
    }
    return mapping[channel_id]


####


def _target_value(channel_id: str, sample: Sraam6TargetSample) -> object:
    mapping: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "altitude_m": sample.altitude_m,
        "bank_deg": sample.bank_deg,
        "normal_load_g": sample.normal_load_g,
        "alive": sample.alive,
    }
    return mapping[channel_id]


####


def _trajectory_events(
    run: Sraam6RunResult,
    missile_object_id: str,
    target_object_id: str,
) -> tuple[TrajectoryEvent, ...]:
    events: list[TrajectoryEvent] = []
    for trace in run.event_trace:
        object_id = missile_object_id if trace.actor == "MISSILE6" else target_object_id
        events.append(
            TrajectoryEvent(
                id=f"source-event-{trace.actor.casefold()}-{trace.event_index}-{len(events)}",
                time_s=trace.time_s,
                category="custom",
                kind="cadac-source-event",
                object_id=object_id,
                detail=f"{trace.watch_variable} {trace.operator} {trace.criterion}",
                data={"source_line": trace.source_line, "updated_values": dict(trace.updated_values)},
            )
        )
    ####
    if run.intercept is not None:
        events.append(
            TrajectoryEvent(
                id=f"intercept-{missile_object_id}-{target_object_id}",
                time_s=run.intercept.time_s,
                category="termination",
                kind="intercept",
                object_id=missile_object_id,
                detail="CADAC SRAAM6 closest-approach gate terminated the engagement.",
                data={
                    "target_object_id": target_object_id,
                    "miss_distance_m": run.intercept.miss_distance_m,
                    "miss_vector_ned_m": run.intercept.miss_vector_ned_m,
                    "differential_speed_mps": run.intercept.differential_speed_mps,
                },
            )
        )
    elif run.terminated_reason == "ground_impact":
        events.append(
            TrajectoryEvent(
                id=f"ground-impact-{missile_object_id}",
                time_s=run.samples[-1].time_s,
                category="impact",
                kind="ground-impact",
                object_id=missile_object_id,
                detail="SRAAM6 reached non-positive flat-Earth altitude.",
            )
        )
    ####
    return tuple(sorted(events, key=lambda item: item.time_s))


####


__all__ = [
    "CADAC_PROVIDER_ID",
    "SRAAM6_EXECUTOR_ID",
    "SRAAM6_SESSION_EXECUTOR_ID",
    "SRAAM6_FIDELITY_ID",
    "SRAAM6_FIN_MISSION_ID",
    "SRAAM6_MISSION_ID",
    "SRAAM6_FIN_PHASE_ID",
    "SRAAM6_FIN_REALIZATION_ID",
    "SRAAM6_REALIZATION_ID",
    "SRAAM6_MODEL_ID",
    "SRAAM6_MODEL_VERSION",
    "SRAAM6_TARGET_MODEL_ID",
    "SRAAM6_TVC_PHASE_ID",
    "SRAAM6_TVC_REALIZATION_ID",
    "CadacSraam6MissionCompositionProvider",
    "build_default_sraam6_configuration",
    "register_sraam6_mission_composition",
]
