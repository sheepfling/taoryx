"""Taoryx Mission Composition bridge for the source-compatible AIM5 plug-in."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any

from taoryx.runtime import SensorBinding, SensorBus, SensorClockSpec
from taoryx.sensor_api import MeasurementPacket, SensorBuildContext
from taoryx.sensor_plugins.relative_state import RelativeStateTrackerConfig, RelativeStateTrackerSensor
from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
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

from .aim5 import Aim5Sample
from .aim5_plugin import Aim5PluginOverrides, Aim5VehiclePlugin
from .aim5_scenario import (
    Aim5EngagementRun,
    Aim5ScenarioRunResult,
    Aim5ScenarioSession,
    Aim5ScenarioSourceDefinition,
    Aim5TargetSample,
)
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import source_managed_control_advertisement
from .output_metadata import cadac_output_quantity
from .sensor_adapter import cadac_local_ned_sensor_context

CADAC_PROVIDER_ID = "cadac"
AIM5_MODEL_ID = "cadac.aim5.missile"
AIM5_TARGET_MODEL_ID = "cadac.aim5.target"
AIM5_MODEL_VERSION = "0.5.0"
AIM5_REALIZATION_ID = "cadac-source-compatibility"
AIM5_MISSION_ID = "air_intercept"
AIM5_EXECUTOR_ID = "cadac.aim5.source_compatibility.batch"
AIM5_SESSION_EXECUTOR_ID = "cadac.aim5.source_compatibility.session"
_LOCAL_NED_FRAME_ID = "cadac.local_ned"
_AIM5_NATIVE_SENSOR_ID = "aim5-native-relative-state"


@dataclass(slots=True)
class _Aim5SessionRecord:
    """One provider-owned AIM5 source loop plus its native sensor delivery state."""

    descriptor: MissionCompositionSessionDescriptor
    prepared: PreparedTrajectoryConfiguration
    definition: Aim5ScenarioSourceDefinition
    source: Aim5ScenarioSession
    sensor_bus: SensorBus
    seed: int
    sequence: int
    observation: MissionCompositionSessionObservation
    lifecycle: str = "ready"
    closed: bool = False


class CadacAim5MissionCompositionProvider:
    """Self-describing AIM5 provider with exact batch and persistent source sessions."""

    def __init__(self, plugin: Aim5VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish AIM5 provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)
        self._sessions: dict[str, _Aim5SessionRecord] = {}
        self._metadata = TrajectoryProviderMetadata(
            id=CADAC_PROVIDER_ID,
            name="CADAC vehicle plug-ins",
            version=AIM5_MODEL_VERSION,
            description="Source-compatible CADAC actor plug-ins exposed through Taoryx Mission Composition.",
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="CADAC Vehicle Plug-ins",
                short_name="CADAC",
                summary="CADAC source actors exposed through typed Taoryx vehicle plug-in contracts.",
                organization="Taoryx integration layer",
                categories=("aerospace", "reference-models"),
            ),
            status="development",
            tags=("cadac", "source-compatible", "vehicle-plugin"),
            execution_contract=AIM5_EXECUTOR_ID,
            model_count=1,
            provenance="missiondesignsolutions/CADAC AIM5 source model plus Taoryx compatibility reconstruction",
            claim_boundary=(
                "This provider advertises the executable Python reconstruction and source scheduling semantics. "
                "Numerical equivalence to a compiled CADAC executable remains unpromoted until golden parity is registered."
            ),
        )

    ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity and execution claim boundary."""

        return self._metadata

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return the exact models installed through this provider instance."""

        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return the AIM5 portable configuration grammar."""

        if model_id != AIM5_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return AIM5 core truth and optional telemetry channels."""

        if model_id != AIM5_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate fidelity, realization, mission recipe, and semantic run inputs."""

        if configuration.realization_id not in {None, AIM5_REALIZATION_ID}:
            raise ValueError(f"AIM5 supports only realization {AIM5_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, AIM5_MISSION_ID}:
            raise ValueError(f"AIM5 supports only mission template {AIM5_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        return validate_configuration_instance(self._schema, configuration)

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Execute one validated common-runner request through the installed AIM5 runtime."""

        if request.provider_id != self.metadata.id or request.model_id != AIM5_MODEL_ID:
            raise ValueError("AIM5 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("AIM5 prepared configuration does not match provider validation")
        ####
        overrides = _overrides_from_resolved(prepared.resolved)
        run = self._plugin.run_batch(overrides)
        return _project_run_result(request, run, self._output_schema)

    ####

    def open_session(
        self,
        request: MissionCompositionOpenSessionRequest,
        *,
        advertised_provider_version: str | None = None,
    ) -> MissionCompositionSessionDescriptor:
        """Open an AIM5 source loop behind the standard persistent-session contract."""

        if request.session_id in self._sessions:
            raise ValueError(f"AIM5 session {request.session_id!r} already exists")
        expected_provider_version = self.metadata.version if advertised_provider_version is None else advertised_provider_version
        if request.provider_id != self.metadata.id or request.provider_version != expected_provider_version:
            raise ValueError("AIM5 session request names another provider version")
        if request.model_id != AIM5_MODEL_ID:
            raise ValueError("AIM5 session request names another model")
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("AIM5 session prepared configuration is stale")
        overrides = _overrides_from_resolved(prepared.resolved)
        definition = self._plugin.prepare_definition(overrides)
        if not math.isclose(request.integration_step_s, definition.integration_step_s, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                "AIM5 session integration_step_s must equal the source compatibility step "
                f"{definition.integration_step_s:.17g} s"
            )
        seed = 0 if request.seed is None else request.seed
        source = Aim5ScenarioSession(definition)
        sensor_bus = _aim5_sensor_bus(seed, definition.integration_step_s)
        initial_context = _aim5_sensor_context(source)
        sensor_bus.accepted_context(_AIM5_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        observation_schema = _aim5_session_observation_schema()
        observation = _aim5_session_observation(
            session_id=request.session_id,
            sequence=0,
            source=source,
            sensor_bus=sensor_bus,
            lifecycle="ready",
        )
        descriptor = MissionCompositionSessionDescriptor(
            session_id=request.session_id,
            provider_id=self.metadata.id,
            provider_version=request.provider_version,
            model_id=AIM5_MODEL_ID,
            mission_template_id=AIM5_MISSION_ID,
            realization_id=AIM5_REALIZATION_ID,
            fidelity="pseudo_6dof",
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=definition.integration_step_s,
            supports_spawned_entities=False,
            action_schema=(),
            observation_schema=observation_schema,
            initial_observation=observation,
            claim_boundary=(
                "The provider owns the persistent AIM5/AIRCRAFT3 source state, event cursors, and source communication bus. "
                "It additionally publishes a separate native relative-state SensorBus packet at committed source boundaries; "
                "that packet does not replace source seeker lag, acquisition, or gimbal behavior."
            ),
        )
        self._sessions[request.session_id] = _Aim5SessionRecord(
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
        """Return the latest committed state without advancing the source loop."""

        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        return self._session(session_id).observation
        ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance AIM5 exactly over the caller's integral source-step hold."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("AIM5 session is closed")
        if record.lifecycle == "completed":
            raise ValueError("AIM5 session is already completed")
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise ValueError(f"AIM5 session expected sequence {request.expected_sequence}, current sequence is {record.sequence}")
        if request.action:
            raise ValueError("AIM5 source-managed session does not accept external action channels")
        time_start_s = record.source.sim_time
        previous_context = _aim5_sensor_context(record.source)
        substeps = _aim5_session_substeps(request.duration_s, record.definition.integration_step_s)
        traces: list[Any] = []
        for _ in range(substeps):
            before_time_s = record.source.sim_time
            traces.extend(record.source.advance(record.definition.integration_step_s))
            if record.source.sim_time <= before_time_s:
                break
            ####
            current_context = _aim5_sensor_context(record.source)
            record.sensor_bus.accepted_context(
                _AIM5_NATIVE_SENSOR_ID,
                current_context.host,
                current_context,
                previous_truth=previous_context.host,
                previous_context=previous_context,
            )
            previous_context = current_context
            if record.source.completed:
                break
            ####
        ####
        record.sequence += 1
        record.lifecycle = "completed" if record.source.completed else "active"
        observation = _aim5_session_observation(
            session_id=request.session_id,
            sequence=record.sequence,
            source=record.source,
            sensor_bus=record.sensor_bus,
            lifecycle=record.lifecycle,
            events=_aim5_session_event_ids(tuple(traces)),
        )
        record.observation = observation
        return MissionCompositionSessionStepResult(
            session_id=request.session_id,
            sequence=record.sequence,
            time_start_s=time_start_s,
            time_end_s=record.source.sim_time,
            requested_action={},
            applied_action={},
            observation=observation,
            events=observation.events,
            diagnostics=observation.diagnostics,
        )
        ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reconstruct source and native-sensor state from the prepared configuration."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("AIM5 session is closed")
        selected_seed = record.seed if request.seed is None else request.seed
        record.source.reset()
        record.sensor_bus = _aim5_sensor_bus(selected_seed, record.definition.integration_step_s)
        initial_context = _aim5_sensor_context(record.source)
        record.sensor_bus.accepted_context(_AIM5_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        record.seed = selected_seed
        record.sequence = 0
        record.lifecycle = "ready"
        record.observation = _aim5_session_observation(
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
        """Release the provider-owned source and sensor state for one session."""

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
        """Return open AIM5 session identifiers in deterministic order."""

        return tuple(sorted(session_id for session_id, record in self._sessions.items() if not record.closed))
        ####

    def session_sensor_packets(
        self,
        session_id: str,
        *,
        delivered: bool = True,
    ) -> tuple[MeasurementPacket[Any], ...]:
        """Return the session's full native raw-track packet history.

        This is intentionally the standard :class:`SensorBus` history rather
        than a CADAC-specific parallel record.  The descriptor/observation
        surface carries the latest delivered packet for ordinary composition
        consumers; this method serves tooling that needs each packet's timing
        and sequence evidence.
        """

        return self._session(session_id).sensor_bus.packets(_AIM5_NATIVE_SENSOR_ID, delivered=delivered)
        ####

    def has_session(self, session_id: str) -> bool:
        """Return whether this provider owns a session, including a closed acknowledgement."""

        return session_id in self._sessions
        ####

    def _session(self, session_id: str) -> _Aim5SessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ValueError(f"AIM5 session {session_id!r} does not exist") from error
        ####


####


def _aim5_session_substeps(duration_s: float, source_step_s: float) -> int:
    """Validate a caller hold and return its exact count of source boundaries."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("AIM5 session duration_s must be positive and finite")
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"AIM5 session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    return steps
    ####


def _aim5_sensor_bus(seed: int, cadence_s: float) -> SensorBus:
    """Build the native raw-track bus that accompanies one persistent AIM5 session."""

    tracker = RelativeStateTrackerSensor(
        RelativeStateTrackerConfig(target_id="aim5-target"),
        SensorBuildContext(_AIM5_NATIVE_SENSOR_ID, seed),
    )
    bus = SensorBus()
    bus.register(
        SensorBinding(
            _AIM5_NATIVE_SENSOR_ID,
            "m1",
            SensorClockSpec(_AIM5_NATIVE_SENSOR_ID, "tracking", cadence_s=cadence_s),
            tracker,
            provenance={
                "provider": "relative-state-track",
                "frame": _LOCAL_NED_FRAME_ID,
                "execution": "persistent-source-session",
                "claim_boundary": "Raw committed geometry only; source seeker lag and specialised state remain in the CADAC module.",
            },
            rng_seed=seed,
        )
    )
    return bus
    ####


def _aim5_sensor_context(source: Aim5ScenarioSession):
    """Project accepted AIM5/AIRCRAFT3 state into the native relative-track context."""

    aim = source.missiles[0]
    target_index = aim.config.target_number - 1
    target = source.targets[target_index]
    return cadac_local_ned_sensor_context(
        time_s=source.sim_time,
        host_position_ned_m=aim.flat.position_ned_m,
        host_velocity_ned_mps=aim.flat.velocity_ned_mps,
        target_id="aim5-target",
        target_position_ned_m=target.flat.position_ned_m,
        target_velocity_ned_mps=target.flat.velocity_ned_mps,
        body_from_local=aim.flat.vehicle_to_local,
    )
    ####


def _aim5_session_observation_schema() -> tuple[MissionCompositionSessionChannel, ...]:
    """Declare the stable source state, controller, and raw-sensor observation ports."""

    scalar = {"topology": "continuous", "representation": "scalar"}
    vector3 = {"topology": "product", "representation": "vector3", "components": 3}
    opaque = {"topology": "opaque", "representation": "json"}
    return (
        MissionCompositionSessionChannel(
            id="position_ned_m",
            direction="observation",
            description="Committed AIM5 local-NED position.",
            quantity=cadac_output_quantity("position_ned_m", "m"),
            unit="m",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="velocity_ned_mps",
            direction="observation",
            description="Committed AIM5 local-NED velocity.",
            quantity=cadac_output_quantity("velocity_ned_mps", "m/s"),
            unit="m/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="speed_mps",
            direction="observation",
            description="Committed AIM5 speed.",
            quantity=cadac_output_quantity("speed_mps", "m/s"),
            unit="m/s",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="heading_deg",
            direction="observation",
            description="Committed AIM5 heading in CADAC local-NED convention.",
            quantity=cadac_output_quantity("heading_deg", "deg"),
            unit="deg",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="flight_path_deg",
            direction="observation",
            description="Committed AIM5 flight-path angle.",
            quantity=cadac_output_quantity("flight_path_deg", "deg"),
            unit="deg",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="altitude_m",
            direction="observation",
            description="Committed AIM5 flat-Earth altitude.",
            quantity=cadac_output_quantity("altitude_m", "m"),
            unit="m",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="mass_kg",
            direction="observation",
            description="Committed source-deck AIM5 mass.",
            quantity=cadac_output_quantity("mass_kg", "kg"),
            unit="kg",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="normal_command_g",
            direction="observation",
            description="Source-owned AIM5 normal acceleration command.",
            quantity=cadac_output_quantity("normal_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_command_g",
            direction="observation",
            description="Source-owned AIM5 lateral acceleration command.",
            quantity=cadac_output_quantity("lateral_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="source_seeker_state",
            direction="observation",
            description="AIM5 source seeker state, including its lagged source communication-bus observation.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
        MissionCompositionSessionChannel(
            id="native_relative_state_track",
            direction="observation",
            description="Latest delivered Taoryx relative-state SensorBus packet from the committed session boundary.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
    )
    ####


def _aim5_session_observation(
    *,
    session_id: str,
    sequence: int,
    source: Aim5ScenarioSession,
    sensor_bus: SensorBus,
    lifecycle: str,
    events: tuple[str, ...] = (),
) -> MissionCompositionSessionObservation:
    """Produce one finite, inspectable session boundary from the source-owned state."""

    aim = source.missiles[0]
    values = {
        "position_ned_m": tuple(float(value) for value in aim.flat.position_ned_m),
        "velocity_ned_mps": tuple(float(value) for value in aim.flat.velocity_ned_mps),
        "speed_mps": float(aim.flat.speed_mps),
        "heading_deg": float(aim.flat.heading_rad * 57.2957795130823),
        "flight_path_deg": float(aim.flat.flight_path_rad * 57.2957795130823),
        "altitude_m": float(aim.flat.altitude_m),
        "mass_kg": float(aim.mass_kg),
        "normal_command_g": float(aim.normal_command_g),
        "lateral_command_g": float(aim.lateral_command_g),
        "source_seeker_state": _aim5_source_seeker_value(source),
        "native_relative_state_track": _aim5_native_track_value(sensor_bus),
    }
    return MissionCompositionSessionObservation(
        session_id=session_id,
        sequence=sequence,
        time_s=source.sim_time,
        lifecycle=lifecycle,
        values=values,
        events=events,
    )
    ####


def _aim5_source_seeker_value(source: Aim5ScenarioSession) -> dict[str, object]:
    """Serialize source-owned seeker state without turning it into a second sensor model."""

    aim = source.missiles[0]
    return {
        "source_schedule": "previous_target_vehicle_pass",
        "initialized": math.isfinite(aim.range_m),
        "range_m": float(aim.range_m) if math.isfinite(aim.range_m) else None,
        "closing_speed_mps": float(aim.closing_speed_mps),
        "target_relative_position_ned_m": [float(value) for value in aim.target_relative_position_ned_m],
        "unit_los_vehicle": [float(value) for value in aim.unit_los_vehicle],
        "line_of_sight_rate_vehicle_rad_s": [float(value) for value in aim.los_rate_vehicle_rps],
    }
    ####


def _aim5_native_track_value(sensor_bus: SensorBus) -> dict[str, object]:
    """Serialize the latest standard packet with its timing and validity facts intact."""

    packets = sensor_bus.packets(_AIM5_NATIVE_SENSOR_ID)
    packet: MeasurementPacket[Any] = packets[-1]
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


def _aim5_session_event_ids(traces: tuple[Any, ...]) -> tuple[str, ...]:
    """Project source event traces into stable session event identifiers."""

    return tuple(
        f"cadac-source-event:{trace.vehicle_model}:{trace.object_id or 'actor'}:{trace.source_line}:{trace.time_s:.17g}"
        for trace in traces
    )
    ####


def build_default_aim5_configuration(
    provider: CadacAim5MissionCompositionProvider,
    *,
    configuration_id: str = "aim5-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create a caller-owned configuration using source defaults plus explicit overrides."""

    supplied = dict(overrides or {})
    grouped: dict[str, dict[str, object]] = {
        "missile": {},
        "target": {},
        "guidance": {},
        "runtime": {},
    }
    routes = {
        "missile_position_ned_m": ("missile", "position_ned_m"),
        "missile_speed_mps": ("missile", "speed_mps"),
        "missile_heading_deg": ("missile", "heading_deg"),
        "missile_flight_path_deg": ("missile", "flight_path_deg"),
        "missile_alpha_deg": ("missile", "alpha_deg"),
        "missile_beta_deg": ("missile", "beta_deg"),
        "target_position_ned_m": ("target", "position_ned_m"),
        "target_speed_mps": ("target", "speed_mps"),
        "target_heading_deg": ("target", "heading_deg"),
        "target_flight_path_deg": ("target", "flight_path_deg"),
        "target_aircraft_option": ("target", "aircraft_option"),
        "target_turn_g": ("target", "turn_g"),
        "navigation_gain": ("guidance", "navigation_gain"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown AIM5 configuration overrides: {unknown!r}")
    ####
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped[group_id][parameter_id] = value
    ####
    root_values = {
        group_id: ConfigurationGroupValue(
            values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in values.items()}
        )
        for group_id, values in grouped.items()
        if values
    }
    schema = provider.get_model_schema(AIM5_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=AIM5_MODEL_ID,
        model_version=AIM5_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity="pseudo_6dof",
        realization_id=AIM5_REALIZATION_ID,
        mission_template_id=AIM5_MISSION_ID,
        root=ConfigurationGroupValue(values=root_values),
    )


####


def register_aim5_mission_composition(
    provider: CadacAim5MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model batch executor without family fallback."""

    registry.register(provider.metadata.id, AIM5_MODEL_ID, provider.execute_batch)


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
) -> ConfigurationParameterSchema:
    role: ConfigurationRole = "constraint" if parameter_id in {"end_time_s", "sample_step_s"} else "initialization"
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
        role=role,
        compatible_fidelities=("pseudo_6dof",),
        frame=frame,
        provenance="lowered from the installed CADAC AIM5 source case",
    )


####


def _build_configuration_schema(plugin: Aim5VehiclePlugin) -> TrajectoryConfigurationSchema:
    definition = plugin.source_definition()
    missile = definition.missiles[0].config
    target = definition.targets[0].config
    root = ConfigurationGroupSchema(
        id="aim5",
        label="AIM5 air-intercept configuration",
        description="Source-default AIM5/AIRCRAFT3 engagement with bounded semantic overrides.",
        children=(
            ConfigurationGroupSchema(
                id="missile",
                label="Missile initial state",
                children=(
                    _parameter(
                        "position_ned_m",
                        "Position NED",
                        "Initial missile local-NED position.",
                        default=missile.position_ned_m,
                        value_type="vector3",
                        unit="m",
                        frame=_LOCAL_NED_FRAME_ID,
                    ),
                    _parameter("speed_mps", "Speed", "Initial missile speed.", default=missile.speed_mps, unit="m/s"),
                    _parameter("heading_deg", "Heading", "Initial missile heading.", default=missile.heading_deg, unit="deg"),
                    _parameter("flight_path_deg", "Flight path", "Initial missile flight-path angle.", default=missile.flight_path_deg, unit="deg"),
                    _parameter("alpha_deg", "Alpha", "Initial AIM5 angle of attack.", default=missile.alpha_deg, unit="deg"),
                    _parameter("beta_deg", "Beta", "Initial AIM5 sideslip angle.", default=missile.beta_deg, unit="deg"),
                ),
            ),
            ConfigurationGroupSchema(
                id="target",
                label="AIRCRAFT3 target",
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
                    _parameter("speed_mps", "Target speed", "Initial AIRCRAFT3 speed.", default=target.speed_mps, unit="m/s"),
                    _parameter("heading_deg", "Target heading", "Initial AIRCRAFT3 heading.", default=target.heading_deg, unit="deg"),
                    _parameter("flight_path_deg", "Target flight path", "Initial AIRCRAFT3 flight-path angle.", default=target.flight_path_deg, unit="deg"),
                    _parameter(
                        "aircraft_option", "Target maneuver mode", "CADAC AIRCRAFT3 maneuver option.", default=target.aircraft_option, value_type="integer"
                    ),
                    _parameter("turn_g", "Target turn g", "AIRCRAFT3 horizontal turn command.", default=target.turn_g),
                ),
            ),
            ConfigurationGroupSchema(
                id="guidance",
                label="Guidance",
                children=(_parameter("navigation_gain", "Navigation gain", "AIM5 proportional-navigation gain.", default=missile.navigation_gain),),
            ),
            ConfigurationGroupSchema(
                id="runtime",
                label="Runtime",
                children=(
                    _parameter("end_time_s", "End time", "Maximum source simulation time.", default=definition.end_time_s, unit="s"),
                    _parameter(
                        "sample_step_s",
                        "Output cadence",
                        "Requested accepted output cadence.",
                        default=definition.plot_step_s or max(definition.integration_step_s, 0.02),
                        unit="s",
                    ),
                ),
            ),
        ),
    )
    return TrajectoryConfigurationSchema(
        model_id=AIM5_MODEL_ID,
        model_version=AIM5_MODEL_VERSION,
        supported_fidelities=("pseudo_6dof",),
        root=root,
        claim_boundary="Runtime fields expose bounded semantic overrides; CADAC module order and compatibility integration remain provider-owned.",
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
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=channel_id,
        label=label,
        description=description,
        quantity=cadac_output_quantity(channel_id, unit),
        canonical_unit=unit,
        display_unit=unit,
        shape=shape,
        frame=frame,
        availability="guaranteed" if channel_id in {"position_ned_m", "velocity_ned_mps"} else "conditional",
        compatible_fidelities=("pseudo_6dof",),
        compatible_realizations=(AIM5_REALIZATION_ID,),
        compatible_mission_templates=(AIM5_MISSION_ID,),
        operations=("batch", "step"),
        provenance="AIM5 compatibility runner accepted truth",
        claim_boundary="Python reconstruction output; compiled-CADAC golden parity remains a separate promotion gate.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output_channel("position_ned_m", "Position NED", "Missile local-NED position.", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel("velocity_ned_mps", "Velocity NED", "Missile local-NED velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
    )
    telemetry = (
        _output_channel("speed_mps", "Speed", "Missile speed.", unit="m/s"),
        _output_channel("heading_deg", "Heading", "Missile heading.", unit="deg"),
        _output_channel("flight_path_deg", "Flight path", "Missile flight-path angle.", unit="deg"),
        _output_channel("altitude_m", "Altitude", "Flat-Earth altitude.", unit="m"),
        _output_channel("range_m", "Target range", "Seeker-time range to assigned AIRCRAFT3 target.", unit="m"),
        _output_channel("closing_speed_mps", "Closing speed", "Signed seeker-time relative closing speed.", unit="m/s"),
        _output_channel(
            "target_relative_position_ned_m",
            "Target relative position NED",
            "Source seeker target-relative position in CADAC local-NED coordinates.",
            unit="m",
            shape=(3,),
            frame=_LOCAL_NED_FRAME_ID,
        ),
        _output_channel(
            "unit_los_vehicle",
            "Unit line of sight",
            "Source seeker unit line-of-sight vector in the AIM5 vehicle frame.",
            shape=(3,),
            frame="cadac.aim5.vehicle_body",
        ),
        _output_channel(
            "line_of_sight_rate_vehicle_rad_s",
            "Line-of-sight rate",
            "Source seeker line-of-sight angular-rate vector in the AIM5 vehicle frame.",
            unit="rad/s",
            shape=(3,),
            frame="cadac.aim5.vehicle_body",
        ),
        _output_channel("mach", "Mach", "Missile Mach number."),
        _output_channel("dynamic_pressure_pa", "Dynamic pressure", "Missile dynamic pressure.", unit="Pa"),
        _output_channel("alpha_deg", "Alpha", "AIM5 reduced-order angle of attack.", unit="deg"),
        _output_channel("beta_deg", "Beta", "AIM5 reduced-order sideslip angle.", unit="deg"),
        _output_channel("normal_command_g", "Normal command", "Guidance normal-acceleration command.", unit="g"),
        _output_channel("lateral_command_g", "Lateral command", "Guidance lateral-acceleration command.", unit="g"),
        _output_channel("normal_acceleration_g", "Normal acceleration", "Realized AIM5 normal acceleration.", unit="g"),
        _output_channel("lateral_acceleration_g", "Lateral acceleration", "Realized AIM5 lateral acceleration.", unit="g"),
        _output_channel("mass_kg", "Mass", "Source-deck missile mass.", unit="kg"),
        _output_channel("thrust_n", "Thrust", "Altitude-adjusted motor thrust.", unit="N"),
    )
    return TrajectoryOutputSchema(
        model_id=AIM5_MODEL_ID,
        model_version=AIM5_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_diagnostics",
                label="Flight diagnostics",
                description="AIM5 seeker, aerodynamic, guidance, and resource telemetry.",
                channel_ids=tuple(item.id for item in telemetry),
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
        claim_boundary="The primary missile and its independently propagated target are returned as separate root trajectory objects.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id="pseudo_6dof",
        label="CADAC AIM5 pseudo-6-DoF",
        rank=1,
        declared=True,
        dynamics_fidelity="pseudo_6dof",
        input_realization="provider_defined",
        runtime_fidelity="pseudo_6dof",
        control_realization="response_law",
        promotion_status="development",
        operations=("validate", "batch", "step"),
        profile_id="cadac.aim5.source-compatibility",
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        required_operations=("state propagation", "source-order guidance/control scheduling"),
        claim_boundary="Pseudo-6-DoF dynamics are implemented; compiled-source equivalence is not yet a promoted claim.",
    )
    realization = TrajectoryRealizationMetadata(
        id=AIM5_REALIZATION_ID,
        label="CADAC source compatibility",
        description="Python execution retaining AIM5 source module order, bus lag, table policy, and stored-derivative integration.",
        status="available",
        dynamics_fidelities=("pseudo_6dof",),
        input_realization="provider_defined",
        controls=source_managed_control_advertisement(
            mission_ids=(AIM5_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/AIM5",),
            operations=("batch", "step"),
            claim_boundary="AIM5 source guidance and control are internal to the persistent source-compatible runtime; sessions expose no caller action channel.",
        ),
        fidelity_aliases=("pseudo_6dof",),
        mission_template_ids=(AIM5_MISSION_ID,),
        operations=("validate", "batch", "step"),
            native_factory_ids=(AIM5_EXECUTOR_ID, AIM5_SESSION_EXECUTOR_ID),
        source_refs=("missiondesignsolutions/CADAC/AIM5",),
        claim_boundary="Available means the Python reconstruction is executable, not that C++ numerical parity is promoted.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=AIM5_MISSION_ID,
        name="AIM5 air intercept",
        description="One AIM5 missile against one AIRCRAFT3 target using source-compatible scheduling.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("air_intercept",),
        compatible_fidelities=("pseudo_6dof",),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity="pseudo_6dof",
                realization_id=AIM5_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_compatible_validation",
                common_runner_status="not_available",
                claim_boundary="Validation is provider-local and does not execute the model.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity="pseudo_6dof",
                realization_id=AIM5_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatible_batch",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=AIM5_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact AIM5 source-compatible actor runtime.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity="pseudo_6dof",
                realization_id=AIM5_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="source_compatible_persistent_session",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=AIM5_SESSION_EXECUTOR_ID,
                claim_boundary="Step dispatch owns AIM5/AIRCRAFT3 source state and accepts only whole source-step holds with source-managed control.",
            ),
        ),
        provenance="CADAC AIM5 input/deck semantics",
        claim_boundary="This mission recipe covers one AIM5/AIRCRAFT3 engagement; multi-engagement source cases remain a separate scenario composition.",
    )
    return TrajectoryModelMetadata(
        id=AIM5_MODEL_ID,
        name="CADAC AIM5 missile",
        version=AIM5_MODEL_VERSION,
        description="Pseudo-6-DoF CADAC AIM5 air-intercept missile exposed as a Taoryx vehicle plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC AIM5",
            short_name="AIM5",
            summary="Source-compatible pseudo-6-DoF air-intercept missile with an independently propagated AIRCRAFT3 target.",
            category="missile",
            subcategory="air_intercept",
            sort_key="cadac-aim5",
            badges=("CADAC", "pseudo-6DoF", "development"),
            default_fidelity_id="pseudo_6dof",
            default_mission_template_id=AIM5_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="CADAC actor identifier from the upstream source package.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="AIM5",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/AIM5",),
                    provenance="CADAC AIM5 source package",
                    claim_boundary="Identity metadata only; it does not establish numerical equivalence.",
                ),
            ),
        ),
        family_id=AIM5_MODEL_ID,
        physical_family="air_intercept_missile",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "missile", "pseudo_6dof"),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("air_intercept",),
            termination_modes=("intercept", "end_time"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=True,
        ),
        realizations=(realization,),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_NED_FRAME_ID,
                name="CADAC flat-Earth local NED",
                description="Local North-East-Down Cartesian frame used by AIM5 Flat3.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source-case local reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/AIM5/flat3_modules.cpp",),
                provenance="CADAC Flat3 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id="cadac.aim5.vehicle_body",
                name="CADAC AIM5 vehicle body",
                description="Instantaneous AIM5 body frame used by the source seeker and guidance law.",
                frame_kind="body",
                axes=("forward", "right", "down"),
                handedness="right",
                origin="AIM5 source reference point",
                orientation="source Flat3 vehicle-to-local direction cosine matrix",
                source_refs=("missiondesignsolutions/CADAC/AIM5/flat3_modules.cpp",),
                provenance="CADAC AIM5 source seeker convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/AIM5",),
        provenance="CADAC AIM5 source-compatible Python reconstruction",
        claim_boundary="The plugin is executable at development status; source-executable numerical parity remains an independent promotion gate.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Aim5PluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("AIM5 resolved configuration root must be a mapping")
    ####
    missile = _group(resolved, "missile")
    target = _group(resolved, "target")
    guidance = _group(resolved, "guidance")
    runtime = _group(resolved, "runtime")
    return Aim5PluginOverrides(
        missile_position_ned_m=_vector3(missile["position_ned_m"]),
        missile_speed_mps=float(missile["speed_mps"]),
        missile_heading_deg=float(missile["heading_deg"]),
        missile_flight_path_deg=float(missile["flight_path_deg"]),
        missile_alpha_deg=float(missile["alpha_deg"]),
        missile_beta_deg=float(missile["beta_deg"]),
        target_position_ned_m=_vector3(target["position_ned_m"]),
        target_speed_mps=float(target["speed_mps"]),
        target_heading_deg=float(target["heading_deg"]),
        target_flight_path_deg=float(target["flight_path_deg"]),
        target_aircraft_option=int(target["aircraft_option"]),
        target_turn_g=float(target["turn_g"]),
        navigation_gain=float(guidance["navigation_gain"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"AIM5 resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("AIM5 vector configuration requires exactly three components")
    ####
    return (float(value[0]), float(value[1]), float(value[2]))


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id == "position_ned_m":
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {"heading_deg", "flight_path_deg", "alpha_deg", "beta_deg"}:
        return "deg"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Aim5ScenarioRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    engagement = run.engagements[0]
    target = run.targets[0]
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity="pseudo_6dof",
        operation="batch",
        realization_id=AIM5_REALIZATION_ID,
        mission_template_id=AIM5_MISSION_ID,
    )
    missile_channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    target_channels = _target_runtime_channels(request.output)
    missile_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={item.id: _missile_value(item.id, sample) for item in selected},
        )
        for sample in engagement.samples
    )
    target_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={item.id: _target_value(item.id, sample) for item in target_channels},
        )
        for sample in target.samples
    )
    events = _trajectory_events(run, engagement)
    missile_terminated = engagement.intercept is not None
    target_terminated = engagement.intercept is not None
    missile_object = TrajectoryObject(
        object_id=engagement.missile_object_id,
        model_id=AIM5_MODEL_ID,
        realization_id=AIM5_REALIZATION_ID,
        name="CADAC AIM5 missile",
        role="interceptor",
        fidelity="pseudo_6dof",
        status="terminated" if missile_terminated else "completed",
        active_from_s=missile_samples[0].time_s,
        active_to_s=missile_samples[-1].time_s,
        terminal_disposition="intercept" if missile_terminated else "end_time",
        channels=missile_channels,
        samples=missile_samples,
        provenance="CADAC AIM5 source-compatible runner",
        claim_boundary="Missile truth is produced by the Python source-compatibility reconstruction; C++ golden parity is pending.",
    )
    target_object = TrajectoryObject(
        object_id=target.target_object_id,
        model_id=AIM5_TARGET_MODEL_ID,
        realization_id=AIM5_REALIZATION_ID,
        name="CADAC AIRCRAFT3 target",
        role="target",
        fidelity="point_mass_3dof",
        status="terminated" if target_terminated else "completed",
        active_from_s=target_samples[0].time_s,
        active_to_s=target_samples[-1].time_s,
        terminal_disposition="intercept" if target_terminated else "end_time",
        channels=target_channels,
        samples=target_samples,
        provenance="CADAC AIM5 AIRCRAFT3 source-compatible runner",
        claim_boundary="Target truth is an independently propagated root object from the source vehicle scheduler.",
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="AIM5 is executing through the source-compatible Python reconstruction; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=AIM5_MODEL_ID,
            object_id=engagement.missile_object_id,
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=AIM5_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=AIM5_MODEL_ID,
        primary_object_id=engagement.missile_object_id,
        status="terminated" if missile_terminated else "completed",
        objects=(missile_object, target_object),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="AIM5 and AIRCRAFT3 are independent source-scheduled trajectory objects; no dynamic parent/child relationship is inferred.",
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
            id="position_ned_m", channel_class="core_state", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID, description="Target local-NED position."
        ),
        TrajectoryChannelMetadata(
            id="velocity_ned_mps", channel_class="core_state", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID, description="Target local-NED velocity."
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
    )
    return (*core, *telemetry)


####


def _missile_value(channel_id: str, sample: Aim5Sample) -> object:
    mapping = {
        "position_ned_m": sample.missile_position_ned_m,
        "velocity_ned_mps": sample.missile_velocity_ned_mps,
        "speed_mps": sample.missile_speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "altitude_m": sample.altitude_m,
        "range_m": sample.range_m,
        "closing_speed_mps": sample.closing_speed_mps,
        "target_relative_position_ned_m": sample.target_relative_position_ned_m,
        "unit_los_vehicle": sample.unit_los_vehicle,
        "line_of_sight_rate_vehicle_rad_s": sample.line_of_sight_rate_vehicle_rad_s,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "alpha_deg": sample.alpha_deg,
        "beta_deg": sample.beta_deg,
        "normal_command_g": sample.normal_command_g,
        "lateral_command_g": sample.lateral_command_g,
        "normal_acceleration_g": sample.normal_acceleration_g,
        "lateral_acceleration_g": sample.lateral_acceleration_g,
        "mass_kg": sample.mass_kg,
        "thrust_n": sample.thrust_n,
    }
    return mapping[channel_id]


####


def _target_value(channel_id: str, sample: Aim5TargetSample) -> object:
    mapping = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "altitude_m": sample.altitude_m,
    }
    return mapping[channel_id]


####


def _trajectory_events(
    run: Aim5ScenarioRunResult,
    engagement: Aim5EngagementRun,
) -> tuple[TrajectoryEvent, ...]:
    events: list[TrajectoryEvent] = []
    for trace in run.event_trace:
        events.append(
            TrajectoryEvent(
                id=f"source-event-{trace.object_id or trace.vehicle_model}-{trace.event_index}-{len(events)}",
                time_s=trace.time_s,
                category="custom",
                kind="cadac-source-event",
                object_id=trace.object_id or engagement.missile_object_id,
                detail=f"{trace.watch_variable} {trace.operator} {trace.criterion}",
                data={"source_line": trace.source_line, "updated_values": dict(trace.updated_values)},
            )
        )
    ####
    if engagement.intercept is not None:
        intercept = engagement.intercept
        events.append(
            TrajectoryEvent(
                id=f"intercept-{engagement.missile_object_id}-{engagement.target_object_id}",
                time_s=intercept.time_s,
                category="termination",
                kind="intercept",
                object_id=engagement.missile_object_id,
                detail="CADAC AIM5 closest-approach intercept gate terminated the engagement.",
                data={
                    "target_object_id": engagement.target_object_id,
                    "miss_distance_m": intercept.miss_distance_m,
                    "differential_speed_mps": intercept.differential_speed_mps,
                    "aspect_azimuth_deg": intercept.aspect_azimuth_deg,
                    "aspect_elevation_deg": intercept.aspect_elevation_deg,
                },
            )
        )
    ####
    return tuple(sorted(events, key=lambda item: item.time_s))


####


__all__ = [
    "AIM5_EXECUTOR_ID",
    "AIM5_MISSION_ID",
    "AIM5_MODEL_ID",
    "AIM5_MODEL_VERSION",
    "AIM5_REALIZATION_ID",
    "AIM5_SESSION_EXECUTOR_ID",
    "AIM5_TARGET_MODEL_ID",
    "CADAC_PROVIDER_ID",
    "CadacAim5MissionCompositionProvider",
    "build_default_aim5_configuration",
    "register_aim5_mission_composition",
]
