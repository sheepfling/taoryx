"""Taoryx Mission Composition bridge for the source-compatible AGM6 plug-in."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any, Literal

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

from .agm6 import Agm6PointMassSample, Agm6RunResult, Agm6Sample, Agm6ScenarioSession, Agm6SourceDefinition
from .agm6_plugin import Agm6PluginOverrides, Agm6VehiclePlugin
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import source_managed_control_advertisement
from .output_metadata import cadac_output_quantity

CADAC_PROVIDER_ID = "cadac"
AGM6_MODEL_ID = "cadac.agm6.missile"
AGM6_TARGET_MODEL_ID = "cadac.agm6.ground_target"
AGM6_AIRCRAFT_MODEL_ID = "cadac.agm6.aircraft"
AGM6_MODEL_VERSION = "0.6.0"
AGM6_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
AGM6_REALIZATION_ID = "cadac-standard-four-fin"
AGM6_MISSION_ID = "air_to_ground_engagement"
AGM6_EXECUTOR_ID = "cadac.agm6.standard_fin.batch"
AGM6_SESSION_EXECUTOR_ID = "cadac.agm6.standard_fin.session"
AGM6_PHASE_ID = "fin_control"
AGM6_TARGET_REALIZATION_ID = "cadac-moving-ground-target"
AGM6_AIRCRAFT_REALIZATION_ID = "cadac-tracking-aircraft"
_LOCAL_NED_FRAME_ID = "cadac.local_ned"
_BODY_FRAME_ID = "cadac.body_xyz"
_NATIVE_SENSOR_ID = "agm6-native-relative-state"


@dataclass(slots=True)
class _Agm6SessionRecord:
    """Provider-owned AGM6 source and raw-track state for one open session."""

    descriptor: MissionCompositionSessionDescriptor
    prepared: PreparedTrajectoryConfiguration
    definition: Agm6SourceDefinition
    source: Agm6ScenarioSession
    sensor_bus: SensorBus
    seed: int
    sequence: int
    observation: MissionCompositionSessionObservation
    lifecycle: str = "ready"
    closed: bool = False


class CadacAgm6MissionCompositionProvider:
    """Self-describing Mission Composition provider for one installed AGM6 case."""

    def __init__(self, plugin: Agm6VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish AGM6 provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)
        self._sessions: dict[str, _Agm6SessionRecord] = {}
        self._metadata = TrajectoryProviderMetadata(
            id=CADAC_PROVIDER_ID,
            name="CADAC vehicle plug-ins",
            version=AGM6_MODEL_VERSION,
            description=("Source-ordered CADAC AGM6 physical-fin air-to-ground engagement exposed through Taoryx Mission Composition."),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="CADAC Vehicle Plug-ins",
                short_name="CADAC",
                summary="CADAC source actors exposed through typed Taoryx vehicle plug-in contracts.",
                organization="Taoryx integration layer",
                categories=("aerospace", "reference-models", "vehicle-plugins"),
            ),
            status="development",
            tags=("cadac", "agm6", "physical-surfaces", "air-to-ground", "vehicle-plugin"),
            execution_contract=AGM6_EXECUTOR_ID,
            model_count=1,
            provenance=("missiondesignsolutions/CADAC AGM6 source model plus Taoryx source-compatible reconstruction"),
            claim_boundary=(
                "The standard four-fin AGM6 missile, moving TARGET3, and tracking AIRCRAFT3 "
                "execute through one exact source-order batch binding. Real-INS error propagation, "
                "complete IIR optical geometry, C-rand stochastic parity, and compiled-CADAC "
                "numerical parity remain outside the promoted claim."
            ),
        )

    ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return provider identity and execution claim boundary."""

        return self._metadata

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return the exact installed AGM6 primary model."""

        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return the AGM6 portable configuration grammar."""

        if model_id != AGM6_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return AGM6 core truth and optional telemetry channels."""

        if model_id != AGM6_MODEL_ID:
            raise KeyError(f"unknown CADAC Mission Composition model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate the exact standard-fin fidelity, realization, and mission selection."""

        if configuration.fidelity != AGM6_FIDELITY_ID:
            raise ValueError(f"AGM6 requires fidelity {AGM6_FIDELITY_ID!r}")
        ####
        if configuration.realization_id not in {None, AGM6_REALIZATION_ID}:
            raise ValueError(f"AGM6 supports only realization {AGM6_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, AGM6_MISSION_ID}:
            raise ValueError(f"AGM6 supports only mission template {AGM6_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        if not isinstance(prepared.resolved, Mapping):
            raise ValueError("AGM6 resolved configuration root must be a mapping")
        ####
        if prepared.resolved.get("source_phase") != AGM6_PHASE_ID:
            raise ValueError(f"AGM6 supports only source phase {AGM6_PHASE_ID!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Execute one validated three-actor AGM6 engagement."""

        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != AGM6_MODEL_ID:
            raise ValueError("AGM6 executor received a request for another provider/model")
        ####
        if request.operation != "batch":
            raise ValueError("AGM6 currently supports only batch execution")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("AGM6 prepared configuration does not match provider validation")
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
        """Open a persistent standard-fin AGM6/TARGET3/AIRCRAFT3 engagement."""

        if request.session_id in self._sessions:
            raise ValueError(f"AGM6 session {request.session_id!r} already exists")
        ####
        expected_version = self.metadata.version if advertised_provider_version is None else advertised_provider_version
        if request.provider_id != self.metadata.id or request.provider_version != expected_version:
            raise ValueError("AGM6 session request names another provider version")
        if request.model_id != AGM6_MODEL_ID:
            raise ValueError("AGM6 session request names another model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("AGM6 session prepared configuration is stale")
        ####
        definition = self._plugin.prepare_definition(_overrides_from_resolved(prepared.resolved))
        if not math.isclose(request.integration_step_s, definition.integration_step_s, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                "AGM6 session integration_step_s must equal the source compatibility step "
                f"{definition.integration_step_s:.17g} s"
            )
        ####
        seed = 0 if request.seed is None else request.seed
        source = Agm6ScenarioSession(definition)
        sensor_bus = _agm6_sensor_bus(seed, definition.integration_step_s)
        initial_context = source.native_sensor_context()
        sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        observation = _agm6_session_observation(
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
            model_id=AGM6_MODEL_ID,
            mission_template_id=AGM6_MISSION_ID,
            realization_id=AGM6_REALIZATION_ID,
            fidelity=AGM6_FIDELITY_ID,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=definition.integration_step_s,
            supports_spawned_entities=False,
            action_schema=(),
            observation_schema=_agm6_session_observation_schema(),
            initial_observation=observation,
            claim_boundary=(
                "The provider owns persistent AGM6 missile, TARGET3, AIRCRAFT3, source-event, IIR, datalink, "
                "tracking, controller, and actuator state. A raw relative-state SensorBus packet is emitted at every "
                "committed source substep; it does not replace the source IIR or datalink paths."
            ),
        )
        self._sessions[request.session_id] = _Agm6SessionRecord(
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
        """Inspect a committed AGM6 source boundary without advancing it."""

        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        return self._session(session_id).observation
    ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance a persistent AGM6 engagement across exact source substeps."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("AGM6 session is closed")
        if record.lifecycle in {"completed", "failed"}:
            raise ValueError("AGM6 session is already terminal")
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise ValueError(f"AGM6 expected sequence {request.expected_sequence}, current sequence is {record.sequence}")
        if request.action:
            raise ValueError("AGM6 source-managed session does not accept external action channels")
        ####
        substeps = _agm6_session_substeps(request.duration_s, record.definition.integration_step_s)
        time_start_s = record.source.sim_time_s
        previous_context = record.source.native_sensor_context()
        events: list[str] = []
        for _ in range(substeps):
            previous_track_sequence = record.source.tracking.update_sequence
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
            events.extend(_agm6_session_event_ids(traces))
            if record.source.tracking.update_sequence != previous_track_sequence:
                events.append(f"cadac-target-track-update-{record.source.tracking.update_sequence}")
            ####
            if record.source.completed:
                break
            ####
        ####
        record.sequence += 1
        record.lifecycle = _agm6_session_lifecycle(record.source)
        record.observation = _agm6_session_observation(
            session_id=request.session_id,
            sequence=record.sequence,
            source=record.source,
            sensor_bus=record.sensor_bus,
            lifecycle=record.lifecycle,
            events=tuple(events),
        )
        return MissionCompositionSessionStepResult(
            session_id=request.session_id,
            sequence=record.sequence,
            time_start_s=time_start_s,
            time_end_s=record.source.sim_time_s,
            requested_action={},
            applied_action={},
            observation=record.observation,
            events=record.observation.events,
            diagnostics=record.observation.diagnostics,
        )
    ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reconstruct AGM6 source and native-sensor state from configuration."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("AGM6 session is closed")
        selected_seed = record.seed if request.seed is None else request.seed
        record.source.reset()
        record.sensor_bus = _agm6_sensor_bus(selected_seed, record.definition.integration_step_s)
        initial_context = record.source.native_sensor_context()
        record.sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        record.seed = selected_seed
        record.sequence = 0
        record.lifecycle = "ready"
        record.observation = _agm6_session_observation(
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
        """Close a persistent AGM6 session while retaining its acknowledgement."""

        session_id = request.session_id if isinstance(request, MissionCompositionCloseSessionRequest) else request
        record = self._session(session_id)
        if not record.closed:
            record.closed = True
            record.lifecycle = "closed"
            record.observation = record.observation.model_copy(update={"lifecycle": "closed"})
        ####
        return MissionCompositionClosedSession(
            session_id=session_id,
            final_sequence=record.sequence,
            final_time_s=record.observation.time_s,
        )
    ####

    def active_session_ids(self) -> tuple[str, ...]:
        """Return deterministic IDs for currently open source sessions."""

        return tuple(sorted(session_id for session_id, record in self._sessions.items() if not record.closed))
    ####

    def has_session(self, session_id: str) -> bool:
        """Return whether the provider owns a session, including closed acknowledgements."""

        return session_id in self._sessions
    ####

    def session_sensor_packets(
        self,
        session_id: str,
        *,
        delivered: bool = True,
    ) -> tuple[MeasurementPacket[Any], ...]:
        """Return retained standard SensorBus packets for one source session."""

        return self._session(session_id).sensor_bus.packets(_NATIVE_SENSOR_ID, delivered=delivered)
    ####

    def _session(self, session_id: str) -> _Agm6SessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ValueError(f"AGM6 session {session_id!r} does not exist") from error
    ####


####


def _agm6_session_substeps(duration_s: float, source_step_s: float) -> int:
    """Validate one composition hold against the AGM6 source timestep."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("AGM6 session duration_s must be positive and finite")
    ####
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"AGM6 session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    ####
    return steps


####


def _agm6_sensor_bus(seed: int, cadence_s: float) -> SensorBus:
    """Build the native raw missile-to-target SensorBus for one AGM6 session."""

    tracker = RelativeStateTrackerSensor(
        RelativeStateTrackerConfig(target_id="agm6-target"),
        SensorBuildContext(_NATIVE_SENSOR_ID, seed),
    )
    bus = SensorBus()
    bus.register(
        SensorBinding(
            _NATIVE_SENSOR_ID,
            "agm6-missile-1",
            SensorClockSpec(_NATIVE_SENSOR_ID, "tracking", cadence_s=cadence_s),
            tracker,
            provenance={
                "provider": "relative-state-track",
                "frame": _LOCAL_NED_FRAME_ID,
                "execution": "persistent-source-session",
                "claim_boundary": (
                    "Raw committed missile/TARGET3 geometry only; AGM6 source IIR acquisition, filtering, "
                    "AIRCRAFT3 tracking, and datalink behavior remain source-owned."
                ),
            },
            rng_seed=seed,
        )
    )
    return bus
    ####


####


def _agm6_session_observation_schema() -> tuple[MissionCompositionSessionChannel, ...]:
    """Declare source state, controller, tracking, and native sensor output ports."""

    scalar = {"topology": "continuous", "representation": "scalar"}
    vector3 = {"topology": "product", "representation": "vector3", "components": 3}
    vector4 = {"topology": "product", "representation": "vector4", "components": 4}
    opaque = {"topology": "opaque", "representation": "json"}
    return (
        MissionCompositionSessionChannel(
            id="position_ned_m",
            direction="observation",
            description="Committed AGM6 local-NED position.",
            quantity=cadac_output_quantity("position_ned_m", "m"),
            unit="m",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="velocity_ned_mps",
            direction="observation",
            description="Committed AGM6 local-NED velocity.",
            quantity=cadac_output_quantity("velocity_ned_mps", "m/s"),
            unit="m/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="quaternion_wxyz",
            direction="observation",
            description="Committed AGM6 body-from-local quaternion.",
            unit="1",
            shape=(4,),
            value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="body_rates_rad_s",
            direction="observation",
            description="Committed AGM6 body angular rates.",
            quantity=cadac_output_quantity("body_rates_rad_s", "rad/s"),
            unit="rad/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="target_position_ned_m",
            direction="observation",
            description="Committed independent TARGET3 local-NED position.",
            quantity=cadac_output_quantity("target_position_ned_m", "m"),
            unit="m",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="target_velocity_ned_mps",
            direction="observation",
            description="Committed independent TARGET3 local-NED velocity.",
            quantity=cadac_output_quantity("target_velocity_ned_mps", "m/s"),
            unit="m/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="aircraft_position_ned_m",
            direction="observation",
            description="Committed independent AIRCRAFT3 local-NED position.",
            quantity=cadac_output_quantity("aircraft_position_ned_m", "m"),
            unit="m",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="aircraft_velocity_ned_mps",
            direction="observation",
            description="Committed independent AIRCRAFT3 local-NED velocity.",
            quantity=cadac_output_quantity("aircraft_velocity_ned_mps", "m/s"),
            unit="m/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_control_deg",
            direction="observation",
            description="Source controller requested roll, pitch, and yaw control commands.",
            unit="deg",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="achieved_control_deg",
            direction="observation",
            description="Actuator-achieved roll, pitch, and yaw control equivalents.",
            unit="deg",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_fins_deg",
            direction="observation",
            description="Source mixer requested four physical fin angles.",
            unit="deg",
            shape=(4,),
            value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="achieved_fins_deg",
            direction="observation",
            description="Actuator-achieved four physical fin angles driving the airframe.",
            unit="deg",
            shape=(4,),
            value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="normal_command_g",
            direction="observation",
            description="Source-owned normal acceleration command.",
            quantity=cadac_output_quantity("normal_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_command_g",
            direction="observation",
            description="Source-owned lateral acceleration command.",
            quantity=cadac_output_quantity("lateral_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="normal_acceleration_g",
            direction="observation",
            description="Achieved body-normal specific-force response.",
            quantity=cadac_output_quantity("normal_acceleration_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_acceleration_g",
            direction="observation",
            description="Achieved body-lateral specific-force response.",
            quantity=cadac_output_quantity("lateral_acceleration_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="source_sensor_and_datalink_state",
            direction="observation",
            description="Source IIR, AIRCRAFT3 track, and datalink state; remains separate from the raw Taoryx track.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
        MissionCompositionSessionChannel(
            id="native_relative_state_track",
            direction="observation",
            description="Latest delivered Taoryx relative-state SensorBus packet from this committed boundary.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
    )
    ####


####


def _agm6_session_observation(
    *,
    session_id: str,
    sequence: int,
    source: Agm6ScenarioSession,
    sensor_bus: SensorBus,
    lifecycle: str,
    events: tuple[str, ...] = (),
) -> MissionCompositionSessionObservation:
    """Produce one inspectable composition boundary from source-owned state."""

    sample = source.sample()
    target = source.target_sample()
    aircraft = source.aircraft_sample()
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
            "aircraft_position_ned_m": aircraft.position_ned_m,
            "aircraft_velocity_ned_mps": aircraft.velocity_ned_mps,
            "requested_control_deg": sample.requested_control_deg,
            "achieved_control_deg": sample.achieved_control_deg,
            "requested_fins_deg": sample.requested_fins_deg,
            "achieved_fins_deg": sample.achieved_fins_deg,
            "normal_command_g": sample.normal_command_g,
            "lateral_command_g": sample.lateral_command_g,
            "normal_acceleration_g": sample.normal_acceleration_g,
            "lateral_acceleration_g": sample.lateral_acceleration_g,
            "source_sensor_and_datalink_state": _agm6_source_sensor_value(source, sample),
            "native_relative_state_track": _agm6_native_track_value(sensor_bus),
        },
        events=events,
    )
    ####


####


def _agm6_source_sensor_value(source: Agm6ScenarioSession, sample: Agm6Sample) -> dict[str, object]:
    """Serialize source sensor and track state without creating a duplicate sensor."""

    track = source.tracking.track
    return {
        "source_schedule": "missile_then_target_then_tracking_aircraft",
        "iir_mode": sample.sensor_mode,
        "iir_initialized": source.missile.sensor_initialized,
        "target_range_m": sample.target_range_m,
        "closing_speed_mps": sample.closing_speed_mps,
        "sensor_pointing_pitch_rad": sample.sensor_pointing_pitch_rad,
        "sensor_pointing_yaw_rad": sample.sensor_pointing_yaw_rad,
        "sensor_los_rate_pitch_rad_s": sample.sensor_los_rate_pitch_rad_s,
        "sensor_los_rate_yaw_rad_s": sample.sensor_los_rate_yaw_rad_s,
        "datalink_update_mode": sample.datalink_update_mode,
        "datalink_track_sequence": sample.datalink_track_sequence,
        "aircraft_track": {
            "sequence": track.update_sequence,
            "time_s": track.time_s,
            "position_ned_m": [float(value) for value in track.position_ned_m],
            "velocity_ned_mps": [float(value) for value in track.velocity_ned_mps],
        },
    }
    ####


####


def _agm6_native_track_value(sensor_bus: SensorBus) -> dict[str, object]:
    """Serialize the latest standard SensorBus packet with timing facts intact."""

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


####


def _agm6_session_event_ids(traces: tuple[Any, ...]) -> tuple[str, ...]:
    """Expose source event applications as stable composition event identifiers."""

    return tuple(f"cadac-source-event-{trace.actor.casefold()}-{trace.event_index}" for trace in traces)
    ####


####


def _agm6_session_lifecycle(source: Agm6ScenarioSession) -> str:
    """Map source termination to the standard session lifecycle vocabulary."""

    if source.terminated_reason == "nonfinite_state":
        return "failed"
    ####
    return "completed" if source.completed else "active"
    ####


####


def build_default_agm6_configuration(
    provider: CadacAgm6MissionCompositionProvider,
    *,
    configuration_id: str = "agm6-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one caller-owned AGM6 configuration from source defaults plus overrides."""

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
        "target_longitudinal_acceleration_g": ("target", "longitudinal_acceleration_g"),
        "target_lateral_acceleration_g": ("target", "lateral_acceleration_g"),
        "aircraft_position_ned_m": ("aircraft", "position_ned_m"),
        "aircraft_speed_mps": ("aircraft", "speed_mps"),
        "aircraft_heading_deg": ("aircraft", "heading_deg"),
        "aircraft_flight_path_deg": ("aircraft", "flight_path_deg"),
        "aircraft_option": ("aircraft", "aircraft_option"),
        "aircraft_turn_g": ("aircraft", "turn_g"),
        "aircraft_longitudinal_acceleration_g": (
            "aircraft",
            "longitudinal_acceleration_g",
        ),
        "navigation_gain": ("guidance", "navigation_gain"),
        "random_seed": ("stochastic", "random_seed"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown AGM6 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    root_values: dict[str, object] = {
        "source_phase": ConfigurationParameterValue(value=AGM6_PHASE_ID),
    }
    for group_id, values in grouped.items():
        root_values[group_id] = ConfigurationGroupValue(
            values={
                parameter_id: ConfigurationParameterValue(
                    value=value,
                    unit=_configuration_unit(parameter_id),
                )
                for parameter_id, value in values.items()
            }
        )
    ####
    schema = provider.get_model_schema(AGM6_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=AGM6_MODEL_ID,
        model_version=AGM6_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=AGM6_FIDELITY_ID,
        realization_id=AGM6_REALIZATION_ID,
        mission_template_id=AGM6_MISSION_ID,
        root=ConfigurationGroupValue(values=root_values),
    )


####


def register_agm6_mission_composition(
    provider: CadacAgm6MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact AGM6 provider/model batch executor without fallback."""

    registry.register(CADAC_PROVIDER_ID, AGM6_MODEL_ID, provider.execute_batch)


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
        role=resolved_role,
        compatible_fidelities=(AGM6_FIDELITY_ID,),
        frame=frame,
        provenance="lowered from the installed CADAC AGM6 source case",
    )


####


def _build_configuration_schema(plugin: Agm6VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    target = source.target
    aircraft = source.aircraft
    root = ConfigurationGroupSchema(
        id="agm6",
        label="AGM6 air-to-ground engagement configuration",
        description=("Source-default AGM6 missile, moving ground target, and tracking aircraft with bounded semantic overrides."),
        children=(
            ConfigurationParameterSchema(
                id="source_phase",
                label="Source control realization",
                description="The installed physical four-fin AGM6 realization.",
                value_type="enum",
                choices=(AGM6_PHASE_ID,),
                required=False,
                default=AGM6_PHASE_ID,
                default_declared=True,
                role="variant",
                compatible_fidelities=(AGM6_FIDELITY_ID,),
                provenance="missiondesignsolutions/CADAC/AGM6 input and actuator modules",
            ),
            ConfigurationGroupSchema(
                id="missile",
                label="MISSILE6 initial state",
                children=(
                    _parameter(
                        "position_ned_m",
                        "Missile position NED",
                        "Initial missile local-NED position.",
                        default=initial.position_ned_m,
                        value_type="vector3",
                        unit="m",
                        frame=_LOCAL_NED_FRAME_ID,
                    ),
                    _parameter("speed_mps", "Missile speed", "Initial missile speed.", default=initial.speed_mps, unit="m/s"),
                    _parameter("yaw_deg", "Missile yaw", "Initial missile body yaw.", default=initial.yaw_deg, unit="deg"),
                    _parameter("pitch_deg", "Missile pitch", "Initial missile body pitch.", default=initial.pitch_deg, unit="deg"),
                    _parameter("roll_deg", "Missile roll", "Initial missile body roll.", default=initial.roll_deg, unit="deg"),
                    _parameter("alpha_deg", "Angle of attack", "Initial missile angle of attack.", default=initial.alpha_deg, unit="deg"),
                    _parameter("beta_deg", "Sideslip", "Initial missile sideslip.", default=initial.beta_deg, unit="deg"),
                    _parameter(
                        "body_rates_deg_s",
                        "Missile body rates",
                        "Initial roll, pitch, and yaw body rates.",
                        default=initial.body_rates_deg_s,
                        value_type="vector3",
                        unit="deg/s",
                        frame=_BODY_FRAME_ID,
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="target",
                label="TARGET3 moving ground target",
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
                        "longitudinal_acceleration_g",
                        "Target longitudinal acceleration",
                        "Source TARGET3 axial acceleration command.",
                        default=target.longitudinal_acceleration_g,
                        unit="g",
                    ),
                    _parameter(
                        "lateral_acceleration_g",
                        "Target lateral acceleration",
                        "Source TARGET3 lateral acceleration command.",
                        default=target.lateral_acceleration_g,
                        unit="g",
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="aircraft",
                label="AIRCRAFT3 tracking platform",
                children=(
                    _parameter(
                        "position_ned_m",
                        "Aircraft position NED",
                        "Initial carrier/tracker local-NED position.",
                        default=aircraft.position_ned_m,
                        value_type="vector3",
                        unit="m",
                        frame=_LOCAL_NED_FRAME_ID,
                    ),
                    _parameter("speed_mps", "Aircraft speed", "Initial AIRCRAFT3 speed.", default=aircraft.speed_mps, unit="m/s"),
                    _parameter("heading_deg", "Aircraft heading", "Initial aircraft heading.", default=aircraft.heading_deg, unit="deg"),
                    _parameter("flight_path_deg", "Aircraft flight path", "Initial aircraft flight-path angle.", default=aircraft.flight_path_deg, unit="deg"),
                    _parameter(
                        "aircraft_option",
                        "Aircraft maneuver option",
                        "Source AIRCRAFT3 straight, horizontal-turn, or escape option.",
                        default=aircraft.aircraft_option,
                        value_type="integer",
                    ),
                    _parameter("turn_g", "Aircraft turn command", "AIRCRAFT3 horizontal turn command.", default=aircraft.turn_g, unit="g"),
                    _parameter(
                        "longitudinal_acceleration_g",
                        "Aircraft longitudinal acceleration",
                        "AIRCRAFT3 longitudinal acceleration command.",
                        default=aircraft.longitudinal_acceleration_g,
                        unit="g",
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="guidance",
                label="Missile guidance",
                children=(
                    _parameter(
                        "navigation_gain",
                        "Navigation gain",
                        "AGM6 proportional-navigation gain.",
                        default=source.guidance.navigation_gain,
                    ),
                ),
            ),
            ConfigurationGroupSchema(
                id="stochastic",
                label="Stochastic execution",
                children=(
                    _parameter(
                        "random_seed",
                        "Random seed",
                        "Seed for deterministic wind, turbulence, and carrier track corruption.",
                        default=source.monte_carlo_seed,
                        value_type="integer",
                        role="variant",
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
        model_id=AGM6_MODEL_ID,
        model_version=AGM6_MODEL_VERSION,
        supported_fidelities=(AGM6_FIDELITY_ID,),
        root=root,
        claim_boundary=(
            "Source initial truth and bounded semantic overrides are portable. The provider owns "
            "module order, stored-derivative integration, event progression, carrier-track bus lag, "
            "and the physical four-fin realization."
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
    guaranteed = {
        "position_ned_m",
        "velocity_ned_mps",
        "quaternion_wxyz",
        "body_rates_rad_s",
    }
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
        interpolation="step" if data_type in {"int64", "boolean", "string", "json"} else "linear",
        availability="guaranteed" if channel_id in guaranteed else "conditional",
        compatible_fidelities=(AGM6_FIDELITY_ID,),
        compatible_realizations=(AGM6_REALIZATION_ID,),
        compatible_mission_templates=(AGM6_MISSION_ID,),
        operations=("batch", "step"),
        provenance="CADAC AGM6 source-compatible runtime",
        claim_boundary="Runtime channel from the Python reconstruction; compiled-source golden parity is pending.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output_channel("position_ned_m", "Position NED", "Missile local-NED position.", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel("velocity_ned_mps", "Velocity NED", "Missile local-NED velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel("quaternion_wxyz", "Quaternion", "Scalar-first body-to-local quaternion.", shape=(4,)),
        _output_channel("body_rates_rad_s", "Body rates", "Missile body roll, pitch, and yaw rates.", unit="rad/s", shape=(3,), frame=_BODY_FRAME_ID),
    )
    flight = (
        _output_channel("speed_mps", "Ground speed", "Missile speed relative to flat Earth.", unit="m/s"),
        _output_channel("airspeed_mps", "Airspeed", "Missile speed relative to the modeled air mass.", unit="m/s"),
        _output_channel("altitude_m", "Altitude", "Flat-Earth altitude.", unit="m"),
        _output_channel("wind_ned_mps", "Wind NED", "Smoothed modeled wind velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output_channel(
            "turbulence_ned_mps", "Turbulence NED", "Source-shaped Dryden turbulence contribution.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID
        ),
        _output_channel("mach", "Mach", "Missile Mach number."),
        _output_channel("dynamic_pressure_pa", "Dynamic pressure", "Missile dynamic pressure.", unit="Pa"),
        _output_channel("alpha_deg", "Angle of attack", "Missile angle of attack.", unit="deg"),
        _output_channel("beta_deg", "Sideslip", "Missile sideslip angle.", unit="deg"),
        _output_channel("total_alpha_deg", "Total incidence", "Missile total incidence angle.", unit="deg"),
        _output_channel("aerodynamic_roll_deg", "Aerodynamic roll", "Aeroballistic roll angle.", unit="deg"),
    )
    effector = (
        _output_channel(
            "requested_control_deg", "Requested control", "Requested roll, pitch, and yaw control coordinates.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID
        ),
        _output_channel("requested_fins_deg", "Requested fins", "Four requested cruciform-fin positions.", unit="deg", shape=(4,)),
        _output_channel(
            "achieved_fins_deg", "Achieved fins", "Four achieved physical fin positions after actuator dynamics and limits.", unit="deg", shape=(4,)
        ),
        _output_channel(
            "achieved_control_deg",
            "Achieved control",
            "Control coordinates reconstructed from achieved fin positions.",
            unit="deg",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
    )
    wrench = (
        _output_channel("force_body_n", "Body force", "Total non-gravitational body force.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output_channel("moment_body_nm", "Body moment", "Total aerodynamic body moment.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
    )
    resources = (
        _output_channel("mass_kg", "Mass", "Current missile mass.", unit="kg"),
        _output_channel("fuel_remaining_kg", "Fuel remaining", "Remaining source rocket fuel mass.", unit="kg"),
        _output_channel("roll_inertia_kg_m2", "Roll inertia", "Source roll moment of inertia.", unit="kg*m^2"),
        _output_channel("pitch_inertia_kg_m2", "Pitch/yaw inertia", "Source transverse moment of inertia.", unit="kg*m^2"),
        _output_channel("thrust_n", "Thrust", "Pressure-corrected rocket thrust.", unit="N"),
        _output_channel("propulsion_mode", "Propulsion mode", "Source propulsion mode.", data_type="int64"),
    )
    engagement = (
        _output_channel("sensor_mode", "Sensor mode", "Source IIR sensor mode.", data_type="int64"),
        _output_channel("guidance_mode", "Guidance mode", "Encoded source midcourse and terminal guidance mode.", data_type="int64"),
        _output_channel("autopilot_mode", "Autopilot mode", "Source rate or acceleration controller mode.", data_type="int64"),
        _output_channel("datalink_update_mode", "Datalink update", "Source datalink target-update flag.", data_type="int64"),
        _output_channel(
            "datalink_track_sequence", "Datalink track sequence", "Carrier target-track update sequence consumed by the missile.", data_type="int64"
        ),
        _output_channel("target_range_m", "Target range", "True missile-to-target distance.", unit="m"),
        _output_channel("closing_speed_mps", "Closing speed", "Missile-target closing-speed magnitude.", unit="m/s"),
        _output_channel("sensor_pointing_pitch_rad", "Sensor pitch", "IIR sensor pitch pointing angle.", unit="rad"),
        _output_channel("sensor_pointing_yaw_rad", "Sensor yaw", "IIR sensor yaw pointing angle.", unit="rad"),
        _output_channel("sensor_los_rate_pitch_rad_s", "Pitch LOS rate", "Filtered pitch sight-line rate.", unit="rad/s"),
        _output_channel("sensor_los_rate_yaw_rad_s", "Yaw LOS rate", "Filtered yaw sight-line rate.", unit="rad/s"),
        _output_channel("normal_command_g", "Normal command", "Normal acceleration command.", unit="g"),
        _output_channel("lateral_command_g", "Lateral command", "Lateral acceleration command.", unit="g"),
        _output_channel("normal_acceleration_g", "Normal acceleration", "Achieved normal specific force.", unit="g"),
        _output_channel("lateral_acceleration_g", "Lateral acceleration", "Achieved lateral specific force.", unit="g"),
    )
    navigation = (
        _output_channel("ins_mode_requested", "Requested INS mode", "Source requested ideal or error-state INS mode.", data_type="int64"),
        _output_channel(
            "ins_model_effective", "Effective INS model", "Effective navigation model participating in the current reconstruction.", data_type="string"
        ),
    )
    telemetry = (*flight, *effector, *wrench, *resources, *engagement, *navigation)
    return TrajectoryOutputSchema(
        model_id=AGM6_MODEL_ID,
        model_version=AGM6_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_conditions",
                label="Flight conditions",
                description="Air-data and incidence telemetry.",
                channel_ids=tuple(item.id for item in flight),
                default_selected=True,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="physical_effectors",
                label="Physical fins",
                description="Requested and achieved four-fin realization.",
                channel_ids=tuple(item.id for item in effector),
                default_selected=True,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="body_wrench", label="Body wrench", description="Total force and moment closure.", channel_ids=tuple(item.id for item in wrench)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="resources", label="Resources", description="Propulsion and mass-property telemetry.", channel_ids=tuple(item.id for item in resources)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="engagement",
                label="Engagement",
                description="Datalink, sensor, guidance, and control telemetry.",
                channel_ids=tuple(item.id for item in engagement),
                default_selected=True,
            ),
            TrajectoryTelemetryGroupMetadata(
                id="navigation", label="Navigation", description="Requested and effective INS boundary.", channel_ids=tuple(item.id for item in navigation)
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(
            supports_multiple_entities=True,
            supports_dynamic_spawning=False,
            supports_recursive_spawning=False,
            maximum_descendant_depth=0,
            relationship_kinds=(),
            child_output_schema_policy="not_applicable",
            includes_spawn_initial_state=False,
            includes_lifecycle_events=True,
        ),
        claim_boundary=(
            "The AGM6 MISSILE6, moving TARGET3, and tracking AIRCRAFT3 are returned as three "
            "independent source-scheduled root objects. Carrier track files are emitted as events."
        ),
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=AGM6_FIDELITY_ID,
        label="CADAC AGM6 physical-effector 6-DoF",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="rigid_body_6dof",
        control_realization="effector_allocated",
        promotion_status="development",
        operations=("validate", "batch", "step"),
        profile_id="cadac.agm6.source-compatibility",
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        required_operations=(
            "rigid-body propagation",
            "physical fin actuator dynamics",
            "carrier track and datalink scheduling",
            "source-order closed-loop engagement",
        ),
        claim_boundary=("T4 applies because achieved physical fin positions participate in aerodynamic force-and-moment closure."),
    )
    realization = TrajectoryRealizationMetadata(
        id=AGM6_REALIZATION_ID,
        label="CADAC four-fin source realization",
        description=("Source-ordered AGM6 datalink, IIR sensor, guidance, autopilot, cruciform-fin mixing, second-order actuators, and aerodynamic closure."),
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        controls=source_managed_control_advertisement(
            mission_ids=(AGM6_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/AGM6",),
            claim_boundary="AGM6 source guidance and control remain internal to the persistent source runtime.",
        ),
        fidelity_aliases=(AGM6_FIDELITY_ID,),
        mission_template_ids=(AGM6_MISSION_ID,),
        operations=("validate", "batch", "step"),
        native_factory_ids=(AGM6_EXECUTOR_ID, AGM6_SESSION_EXECUTOR_ID),
        source_refs=("missiondesignsolutions/CADAC/AGM6",),
        claim_boundary=("Available means the Python reconstruction is executable; compiled-source numerical parity is pending."),
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=AGM6_MISSION_ID,
        name="AGM6 carrier-supported air-to-ground engagement",
        description=("One physical-fin MISSILE6, one moving TARGET3, and one tracking AIRCRAFT3 under the source vehicle-major scheduler."),
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=(
            "source_release",
            "carrier_datalink_midcourse",
            "terminal_sensor_guidance",
        ),
        compatible_fidelities=(AGM6_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=AGM6_FIDELITY_ID,
                realization_id=AGM6_REALIZATION_ID,
                operation="validate",
                status="available",
                claim_boundary="The installed source case and semantic overrides validate.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=AGM6_FIDELITY_ID,
                realization_id=AGM6_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="native_provider",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=AGM6_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed AGM6 three-actor runtime.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=AGM6_FIDELITY_ID,
                realization_id=AGM6_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="native_provider",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=AGM6_SESSION_EXECUTOR_ID,
                claim_boundary="Persistent stepping retains the source scheduler, IIR, datalink, tracker, controller, and actuator state.",
            ),
        ),
        provenance="CADAC AGM6 input, deck, module, carrier-track, and datalink semantics",
        claim_boundary=(
            "The mission recipe covers the standard physical-fin air-to-ground source composition; "
            "it does not claim real-INS or complete optical-sensor parity."
        ),
    )
    return TrajectoryModelMetadata(
        id=AGM6_MODEL_ID,
        name="CADAC AGM6 missile",
        version=AGM6_MODEL_VERSION,
        description=("Physical-fin rigid-body CADAC AGM6 air-to-ground missile with a moving target and carrier tracking/datalink support."),
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC AGM6",
            short_name="AGM6",
            summary=("Source-ordered physical-fin 6-DoF strike missile with independent moving target and tracking-aircraft roots."),
            category="missile",
            subcategory="air_to_ground",
            sort_key="cadac-agm6",
            badges=("CADAC", "6-DoF", "physical fins", "three actors", "development"),
            default_fidelity_id=AGM6_FIDELITY_ID,
            default_mission_template_id=AGM6_MISSION_ID,
            default_output_channel_ids=(
                "position_ned_m",
                "velocity_ned_mps",
                "quaternion_wxyz",
                "body_rates_rad_s",
            ),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor/model identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="MISSILE6",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/AGM6",),
                    provenance="CADAC AGM6 source package",
                    claim_boundary="Identity metadata only; it does not establish numerical equivalence.",
                ),
                TrajectoryModelPropertyMetadata(
                    id="source_composition",
                    label="Source composition",
                    description="Independent source actors participating in one engagement.",
                    semantic_role="implementation",
                    value_type="string",
                    value_kind="declared",
                    value="MISSILE6 + TARGET3 + AIRCRAFT3",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/AGM6/input.asc",),
                    provenance="CADAC AGM6 source case",
                    claim_boundary="The three actors remain independent roots; no lineage is inferred.",
                ),
                TrajectoryModelPropertyMetadata(
                    id="physical_effectors",
                    label="Physical effectors",
                    description="Source physical control realization.",
                    semantic_role="implementation",
                    value_type="string",
                    value_kind="declared",
                    value="four aerodynamic fins with fixed cruciform mixing and second-order dynamics",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/AGM6/actuator.cpp",),
                    provenance="CADAC AGM6 actuator module",
                    claim_boundary="Applies to the executable standard-fin realization.",
                ),
            ),
        ),
        family_id=AGM6_MODEL_ID,
        physical_family="air_to_ground_missile",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "agm6", "air-to-ground", "rigid-body", "physical-fins"),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=(
                "source_release",
                "carrier_datalink_midcourse",
                "terminal_sensor_guidance",
            ),
            termination_modes=("target_plane_intercept", "ground_impact", "source_stop", "end_time"),
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
                description="Local North-East-Down Cartesian frame used by AGM6 Flat6 and both Flat3 actors.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/AGM6",),
                provenance="CADAC Flat6/Flat3 local-level convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC body axes",
                description="Forward-right-down body axes used for missile forces, moments, and rates.",
                frame_kind="body",
                axes=("forward", "right", "down"),
                handedness="right",
                origin="missile center of mass",
                orientation="vehicle body",
                source_refs=("missiondesignsolutions/CADAC/AGM6",),
                provenance="CADAC AGM6 body-axis convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/AGM6",),
        provenance="CADAC AGM6 source model projected into Taoryx Mission Composition",
        claim_boundary=(
            "This model advertises the executable Python reconstruction and source scheduling "
            "semantics. Compiled-CADAC numerical equivalence remains unpromoted."
        ),
    )


####


def _overrides_from_resolved(resolved: Mapping[str, object]) -> Agm6PluginOverrides:
    missile = _group(resolved, "missile")
    target = _group(resolved, "target")
    aircraft = _group(resolved, "aircraft")
    guidance = _group(resolved, "guidance")
    stochastic = _group(resolved, "stochastic")
    runtime = _group(resolved, "runtime")
    return Agm6PluginOverrides(
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
        target_longitudinal_acceleration_g=float(target["longitudinal_acceleration_g"]),
        target_lateral_acceleration_g=float(target["lateral_acceleration_g"]),
        aircraft_position_ned_m=_vector3(aircraft["position_ned_m"]),
        aircraft_speed_mps=float(aircraft["speed_mps"]),
        aircraft_heading_deg=float(aircraft["heading_deg"]),
        aircraft_flight_path_deg=float(aircraft["flight_path_deg"]),
        aircraft_option=int(aircraft["aircraft_option"]),
        aircraft_turn_g=float(aircraft["turn_g"]),
        aircraft_longitudinal_acceleration_g=float(aircraft["longitudinal_acceleration_g"]),
        navigation_gain=float(guidance["navigation_gain"]),
        random_seed=int(stochastic["random_seed"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"AGM6 resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("AGM6 vector configuration requires exactly three components")
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
    if parameter_id in {
        "yaw_deg",
        "pitch_deg",
        "roll_deg",
        "alpha_deg",
        "beta_deg",
        "heading_deg",
        "flight_path_deg",
    }:
        return "deg"
    ####
    if parameter_id == "body_rates_deg_s":
        return "deg/s"
    ####
    if parameter_id in {"longitudinal_acceleration_g", "lateral_acceleration_g", "turn_g"}:
        return "g"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Agm6RunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=AGM6_FIDELITY_ID,
        operation="batch",
        realization_id=AGM6_REALIZATION_ID,
        mission_template_id=AGM6_MISSION_ID,
    )
    missile_channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    target_channels = _point_runtime_channels(
        request.output,
        telemetry_group="target_kinematics",
        actor_label="TARGET3",
    )
    aircraft_channels = _point_runtime_channels(
        request.output,
        telemetry_group="aircraft_kinematics",
        actor_label="AIRCRAFT3",
    )
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
            values={item.id: _point_value(item.id, sample) for item in target_channels},
        )
        for sample in run.target_samples
    )
    aircraft_samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={item.id: _point_value(item.id, sample) for item in aircraft_channels},
        )
        for sample in run.aircraft_samples
    )
    missile_object_id = "agm6-missile-1"
    target_object_id = "agm6-ground-target-1"
    aircraft_object_id = "agm6-tracking-aircraft-1"
    events = (
        _trajectory_events(
            run,
            missile_object_id=missile_object_id,
            target_object_id=target_object_id,
            aircraft_object_id=aircraft_object_id,
        )
        if request.output.include_events
        else ()
    )
    missile_terminated = run.terminated_reason != "end_time"
    missile_object = TrajectoryObject(
        object_id=missile_object_id,
        model_id=AGM6_MODEL_ID,
        realization_id=AGM6_REALIZATION_ID,
        name="CADAC AGM6 missile",
        role="strike_vehicle",
        fidelity=AGM6_FIDELITY_ID,
        status="failed" if run.terminated_reason == "nonfinite_state" else "terminated" if missile_terminated else "completed",
        active_from_s=missile_samples[0].time_s,
        active_to_s=missile_samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=missile_channels,
        samples=missile_samples,
        provenance="CADAC AGM6 source-compatible four-fin runner",
        claim_boundary=("Missile truth closes through achieved physical fin positions; compiled-CADAC golden parity is pending."),
    )
    target_object = TrajectoryObject(
        object_id=target_object_id,
        model_id=AGM6_TARGET_MODEL_ID,
        realization_id=AGM6_TARGET_REALIZATION_ID,
        name="CADAC TARGET3 moving ground target",
        role="target",
        fidelity="point_mass_3dof",
        status="completed",
        active_from_s=target_samples[0].time_s,
        active_to_s=target_samples[-1].time_s,
        terminal_disposition="source_run_end",
        channels=target_channels,
        samples=target_samples,
        provenance="CADAC AGM6 TARGET3 source-compatible runner",
        claim_boundary=("TARGET3 is an independently propagated point-mass root object under the source vehicle-major scheduler."),
    )
    aircraft_object = TrajectoryObject(
        object_id=aircraft_object_id,
        model_id=AGM6_AIRCRAFT_MODEL_ID,
        realization_id=AGM6_AIRCRAFT_REALIZATION_ID,
        name="CADAC AIRCRAFT3 tracking platform",
        role="support_tracker",
        fidelity="point_mass_3dof",
        status="completed",
        active_from_s=aircraft_samples[0].time_s,
        active_to_s=aircraft_samples[-1].time_s,
        terminal_disposition="source_run_end",
        channels=aircraft_channels,
        samples=aircraft_samples,
        provenance="CADAC AGM6 AIRCRAFT3 source-compatible runner",
        claim_boundary=("AIRCRAFT3 is an independent point-mass root that produces target-track updates; it is not a spawned child of the missile."),
    )
    diagnostics = _diagnostics(run, missile_object_id)
    result_status = "failed" if run.terminated_reason == "nonfinite_state" else "terminated" if missile_terminated else "completed"
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=AGM6_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=AGM6_MODEL_ID,
        primary_object_id=missile_object_id,
        status=result_status,
        objects=(missile_object, target_object, aircraft_object),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary=(
            "AGM6, TARGET3, and AIRCRAFT3 are independent source-scheduled root objects. Carrier "
            "track measurements are events, not lineage or hidden missile state."
        ),
    )


####


def _diagnostics(
    run: Agm6RunResult,
    missile_object_id: str,
) -> tuple[MissionCompositionDiagnostic, ...]:
    diagnostics: list[MissionCompositionDiagnostic] = [
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message=("AGM6 is executing through the source-compatible Python reconstruction; compiled-CADAC numerical parity is not yet promoted."),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=AGM6_MODEL_ID,
            object_id=missile_object_id,
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-iir-sensor-partial",
            message=(
                "The sensor mode machine and LOS-rate filter execute; complete focal-plane "
                "corruption, aimpoint modulation, and gimbal-head geometry remain outside the claim."
            ),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=AGM6_MODEL_ID,
            object_id=missile_object_id,
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-stochastic-generator-substitution",
            message=("Seeded NumPy stochastic draws reproduce deterministic source-shaped behavior but do not claim C rand()/Box-Muller sequence parity."),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=AGM6_MODEL_ID,
            object_id=missile_object_id,
        ),
    ]
    if any(sample.ins_mode_requested == 1 for sample in run.samples):
        diagnostics.append(
            MissionCompositionDiagnostic(
                severity="info",
                code="cadac-ins-truth-aligned-substitution",
                message=(
                    "The source requested real-INS mode, but this tranche feeds the controller and "
                    "guidance with the disclosed truth-aligned navigation solution."
                ),
                phase="execution",
                recoverability="degraded",
                provider_id=CADAC_PROVIDER_ID,
                model_id=AGM6_MODEL_ID,
                object_id=missile_object_id,
            )
        )
    ####
    return tuple(diagnostics)


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


def _point_runtime_channels(
    selection: MissionCompositionOutputSelection,
    *,
    telemetry_group: str,
    actor_label: str,
) -> tuple[TrajectoryChannelMetadata, ...]:
    core = (
        TrajectoryChannelMetadata(
            id="position_ned_m",
            channel_class="core_state",
            unit="m",
            shape=(3,),
            frame=_LOCAL_NED_FRAME_ID,
            description=f"{actor_label} local-NED position.",
        ),
        TrajectoryChannelMetadata(
            id="velocity_ned_mps",
            channel_class="core_state",
            unit="m/s",
            shape=(3,),
            frame=_LOCAL_NED_FRAME_ID,
            description=f"{actor_label} local-NED velocity.",
        ),
    )
    telemetry = (
        TrajectoryChannelMetadata(id="speed_mps", channel_class="telemetry", telemetry_group=telemetry_group, unit="m/s", description=f"{actor_label} speed."),
        TrajectoryChannelMetadata(
            id="heading_deg", channel_class="telemetry", telemetry_group=telemetry_group, unit="deg", description=f"{actor_label} heading."
        ),
        TrajectoryChannelMetadata(
            id="flight_path_deg", channel_class="telemetry", telemetry_group=telemetry_group, unit="deg", description=f"{actor_label} flight-path angle."
        ),
        TrajectoryChannelMetadata(
            id="altitude_m", channel_class="telemetry", telemetry_group=telemetry_group, unit="m", description=f"{actor_label} flat-Earth altitude."
        ),
        TrajectoryChannelMetadata(
            id="bank_deg", channel_class="telemetry", telemetry_group=telemetry_group, unit="deg", description=f"{actor_label} achieved bank angle."
        ),
        TrajectoryChannelMetadata(
            id="normal_load_g", channel_class="telemetry", telemetry_group=telemetry_group, unit="g", description=f"{actor_label} achieved normal load factor."
        ),
        TrajectoryChannelMetadata(
            id="alive",
            channel_class="telemetry",
            telemetry_group=telemetry_group,
            data_type="boolean",
            interpolation="step",
            description=f"{actor_label} source lifecycle state.",
        ),
    )
    if selection.mode == "core":
        return core
    ####
    if selection.mode == "all":
        return (*core, *telemetry)
    ####
    requested = set(selection.channels)
    if telemetry_group in selection.telemetry_groups:
        requested.update(item.id for item in telemetry)
    ####
    return (*core, *(item for item in telemetry if item.id in requested))


####


def _missile_value(channel_id: str, sample: Agm6Sample) -> object:
    mapping: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "quaternion_wxyz": sample.quaternion_wxyz,
        "body_rates_rad_s": sample.body_rates_rad_s,
        "speed_mps": sample.speed_mps,
        "airspeed_mps": sample.airspeed_mps,
        "altitude_m": sample.altitude_m,
        "wind_ned_mps": sample.wind_ned_mps,
        "turbulence_ned_mps": sample.turbulence_ned_mps,
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
        "fuel_remaining_kg": sample.fuel_remaining_kg,
        "roll_inertia_kg_m2": sample.roll_inertia_kg_m2,
        "pitch_inertia_kg_m2": sample.pitch_inertia_kg_m2,
        "thrust_n": sample.thrust_n,
        "propulsion_mode": sample.propulsion_mode,
        "sensor_mode": sample.sensor_mode,
        "guidance_mode": sample.guidance_mode,
        "autopilot_mode": sample.autopilot_mode,
        "datalink_update_mode": sample.datalink_update_mode,
        "datalink_track_sequence": sample.datalink_track_sequence,
        "target_range_m": sample.target_range_m,
        "closing_speed_mps": sample.closing_speed_mps,
        "sensor_pointing_pitch_rad": sample.sensor_pointing_pitch_rad,
        "sensor_pointing_yaw_rad": sample.sensor_pointing_yaw_rad,
        "sensor_los_rate_pitch_rad_s": sample.sensor_los_rate_pitch_rad_s,
        "sensor_los_rate_yaw_rad_s": sample.sensor_los_rate_yaw_rad_s,
        "normal_command_g": sample.normal_command_g,
        "lateral_command_g": sample.lateral_command_g,
        "normal_acceleration_g": sample.normal_acceleration_g,
        "lateral_acceleration_g": sample.lateral_acceleration_g,
        "ins_mode_requested": sample.ins_mode_requested,
        "ins_model_effective": sample.ins_model_effective,
    }
    return mapping[channel_id]


####


def _point_value(channel_id: str, sample: Agm6PointMassSample) -> object:
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
    run: Agm6RunResult,
    *,
    missile_object_id: str,
    target_object_id: str,
    aircraft_object_id: str,
) -> tuple[TrajectoryEvent, ...]:
    object_by_actor = {
        "MISSILE6": missile_object_id,
        "TARGET3": target_object_id,
        "AIRCRAFT3": aircraft_object_id,
    }
    events: list[TrajectoryEvent] = []
    for trace in run.event_trace:
        events.append(
            TrajectoryEvent(
                id=f"source-event-{trace.actor.casefold()}-{trace.event_index}-{len(events)}",
                time_s=trace.time_s,
                category="custom",
                kind="cadac-source-event",
                object_id=object_by_actor[trace.actor],
                detail=f"{trace.watch_variable} {trace.operator} {trace.criterion}",
                data={
                    "source_line": trace.source_line,
                    "previous_values": dict(trace.previous_values),
                    "updated_values": dict(trace.updated_values),
                },
            )
        )
    ####
    for track in run.track_samples:
        if track.update_sequence == 0:
            continue
        ####
        events.append(
            TrajectoryEvent(
                id=f"track-update-{track.update_sequence}",
                time_s=track.time_s,
                category="custom",
                kind="cadac-target-track-update",
                object_id=aircraft_object_id,
                detail="AIRCRAFT3 published a measured TARGET3 position/velocity track.",
                data={
                    "target_object_id": target_object_id,
                    "update_sequence": track.update_sequence,
                    "measured_position_ned_m": track.position_ned_m,
                    "measured_velocity_ned_mps": track.velocity_ned_mps,
                },
            )
        )
    ####
    if run.intercept is not None:
        events.append(
            TrajectoryEvent(
                id=f"target-plane-intercept-{missile_object_id}-{target_object_id}",
                time_s=run.intercept.time_s,
                category="termination",
                kind="target-plane-intercept",
                object_id=missile_object_id,
                detail="CADAC AGM6 target-plane crossing terminated the missile trajectory.",
                data={
                    "target_object_id": target_object_id,
                    "miss_distance_m": run.intercept.miss_distance_m,
                    "miss_vector_target_plane_m": run.intercept.miss_vector_target_plane_m,
                    "target_range_m": run.intercept.target_range_m,
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
                detail="AGM6 reached non-positive flat-Earth altitude.",
            )
        )
    elif run.terminated_reason == "source_stop":
        events.append(
            TrajectoryEvent(
                id=f"source-stop-{missile_object_id}",
                time_s=run.samples[-1].time_s,
                category="termination",
                kind="cadac-source-stop",
                object_id=missile_object_id,
                detail="AGM6 source stopping criteria terminated the missile trajectory.",
            )
        )
    ####
    return tuple(sorted(events, key=lambda item: (item.time_s, item.id)))


####


__all__ = [
    "AGM6_AIRCRAFT_MODEL_ID",
    "AGM6_AIRCRAFT_REALIZATION_ID",
    "AGM6_EXECUTOR_ID",
    "AGM6_FIDELITY_ID",
    "AGM6_MISSION_ID",
    "AGM6_MODEL_ID",
    "AGM6_MODEL_VERSION",
    "AGM6_PHASE_ID",
    "AGM6_REALIZATION_ID",
    "AGM6_TARGET_MODEL_ID",
    "AGM6_TARGET_REALIZATION_ID",
    "CADAC_PROVIDER_ID",
    "CadacAgm6MissionCompositionProvider",
    "build_default_agm6_configuration",
    "register_agm6_mission_composition",
]
