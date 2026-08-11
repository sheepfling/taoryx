"""Taoryx Mission Composition bridge for the ADS6 ``ROCKET5`` SRBM plug-in."""

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
    TrajectoryRealizationMetadata,
    TrajectoryReferenceFrameMetadata,
    TrajectoryTelemetryGroupMetadata,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionDiagnostic,
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

from .ads6_srbm import (
    Ads6SrbmRunResult,
    Ads6SrbmSample,
    Ads6SrbmSession,
    Ads6SrbmSourceDefinition,
)
from .ads6_srbm_plugin import Ads6SrbmPluginOverrides, Ads6SrbmVehiclePlugin
from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import source_managed_control_advertisement
from .output_metadata import cadac_output_quantity

ADS6_SRBM_MODEL_ID = "cadac.ads6.srbm"
ADS6_SRBM_MODEL_VERSION = "0.9.0"
ADS6_SRBM_FIDELITY_ID = "pseudo_6dof"
ADS6_SRBM_REALIZATION_ID = "cadac-ads6-srbm-response-law"
ADS6_SRBM_PHASE_ID = "source_model"
ADS6_SRBM_MISSION_ID = "ads6_srbm_flight"
ADS6_SRBM_EXECUTOR_ID = "cadac.ads6.srbm.source_compatibility.batch"
ADS6_SRBM_SESSION_EXECUTOR_ID = "cadac.ads6.srbm.source_compatibility.session"
_LOCAL_FRAME_ID = "cadac.ads6.local_ned"
_BODY_FRAME_ID = "cadac.ads6.srbm.body"
_NATIVE_SENSOR_ID = "ads6-srbm-native-relative-state"


@dataclass(slots=True)
class _Ads6SrbmSessionRecord:
    """Provider-owned source and native-sensor state for one SRBM session."""

    descriptor: MissionCompositionSessionDescriptor
    prepared: PreparedTrajectoryConfiguration
    definition: Ads6SrbmSourceDefinition
    source: Ads6SrbmSession
    sensor_bus: SensorBus
    seed: int
    sequence: int
    observation: MissionCompositionSessionObservation
    lifecycle: str = "ready"
    closed: bool = False


class CadacAds6SrbmMissionCompositionProvider:
    """Self-describing ADS6 ROCKET5 provider with batch and persistent source sessions."""

    def __init__(self, plugin: Ads6SrbmVehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish ADS6 SRBM provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)
        self._sessions: dict[str, _Ads6SrbmSessionRecord] = {}

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != ADS6_SRBM_MODEL_ID:
            raise KeyError(f"unknown ADS6 SRBM model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != ADS6_SRBM_MODEL_ID:
            raise KeyError(f"unknown ADS6 SRBM model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != ADS6_SRBM_FIDELITY_ID:
            raise ValueError(f"ADS6 SRBM source model requires fidelity {ADS6_SRBM_FIDELITY_ID!r}")
        ####
        if configuration.realization_id not in {None, ADS6_SRBM_REALIZATION_ID}:
            raise ValueError(f"ADS6 SRBM supports only realization {ADS6_SRBM_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, ADS6_SRBM_MISSION_ID}:
            raise ValueError(f"ADS6 SRBM supports only mission template {ADS6_SRBM_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        resolved = prepared.resolved
        if not isinstance(resolved, Mapping) or resolved.get("source_phase") != ADS6_SRBM_PHASE_ID:
            raise ValueError(f"ADS6 SRBM requires source phase {ADS6_SRBM_PHASE_ID!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != ADS6_SRBM_MODEL_ID:
            raise ValueError("ADS6 SRBM executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 SRBM prepared configuration does not match provider validation")
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
        """Open a persistent ROCKET5 source loop under the common session contract."""

        if request.session_id in self._sessions:
            raise ValueError(f"ADS6 SRBM session {request.session_id!r} already exists")
        expected_version = ADS6_SRBM_MODEL_VERSION if advertised_provider_version is None else advertised_provider_version
        if request.provider_id != CADAC_PROVIDER_ID or request.provider_version != expected_version:
            raise ValueError("ADS6 SRBM session request names another provider version")
        if request.model_id != ADS6_SRBM_MODEL_ID:
            raise ValueError("ADS6 SRBM session request names another model")
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 SRBM session prepared configuration is stale")
        definition = self._plugin.prepare_definition(_overrides_from_resolved(prepared.resolved))
        if not math.isclose(request.integration_step_s, definition.integration_step_s, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                "ADS6 SRBM session integration_step_s must equal the source compatibility step "
                f"{definition.integration_step_s:.17g} s"
            )
        seed = 0 if request.seed is None else request.seed
        source = Ads6SrbmSession(definition)
        sensor_bus = _ads6_srbm_sensor_bus(seed, definition.integration_step_s)
        initial_context = source.native_sensor_context()
        sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        observation = _ads6_srbm_session_observation(
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
            model_id=ADS6_SRBM_MODEL_ID,
            mission_template_id=ADS6_SRBM_MISSION_ID,
            realization_id=ADS6_SRBM_REALIZATION_ID,
            fidelity=ADS6_SRBM_FIDELITY_ID,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=definition.integration_step_s,
            supports_spawned_entities=False,
            action_schema=(),
            observation_schema=_ads6_srbm_session_observation_schema(),
            initial_observation=observation,
            claim_boundary=(
                "The provider owns persistent ROCKET5 translation, response-law, propulsion, phase, and terminal state. "
                "A native fixed-target relative-state SensorBus packet is emitted at every committed source substep; "
                "it is additional to source seeker enable/phase gating."
            ),
        )
        self._sessions[request.session_id] = _Ads6SrbmSessionRecord(
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
        """Inspect a committed source state without advancing it."""

        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        return self._session(session_id).observation
        ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance one source-owned ROCKET5 session across declared source substeps."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("ADS6 SRBM session is closed")
        if record.lifecycle in {"completed", "failed"}:
            raise ValueError("ADS6 SRBM session is already terminal")
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise ValueError(f"ADS6 SRBM expected sequence {request.expected_sequence}, current sequence is {record.sequence}")
        if request.action:
            raise ValueError("ADS6 SRBM source-managed session does not accept external action channels")
        substeps = _ads6_srbm_session_substeps(request.duration_s, record.definition.integration_step_s)
        time_start_s = record.source.sim_time_s
        previous_context = record.source.native_sensor_context()
        transitions: list[Any] = []
        for _ in range(substeps):
            before_time_s = record.source.sim_time_s
            transitions.extend(record.source.advance(record.definition.integration_step_s))
            if record.source.sim_time_s <= before_time_s:
                break
            ####
            current_context = record.source.native_sensor_context()
            record.sensor_bus.accepted_context(
                _NATIVE_SENSOR_ID,
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
        record.lifecycle = _ads6_srbm_session_lifecycle(record.source)
        observation = _ads6_srbm_session_observation(
            session_id=request.session_id,
            sequence=record.sequence,
            source=record.source,
            sensor_bus=record.sensor_bus,
            lifecycle=record.lifecycle,
            events=_ads6_srbm_session_events(tuple(transitions), record.source),
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
        )
        ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        """Reconstruct the source and native sensor state from its prepared configuration."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("ADS6 SRBM session is closed")
        selected_seed = record.seed if request.seed is None else request.seed
        record.source.reset()
        record.sensor_bus = _ads6_srbm_sensor_bus(selected_seed, record.definition.integration_step_s)
        initial_context = record.source.native_sensor_context()
        record.sensor_bus.accepted_context(_NATIVE_SENSOR_ID, initial_context.host, initial_context)
        record.seed = selected_seed
        record.sequence = 0
        record.lifecycle = "ready"
        record.observation = _ads6_srbm_session_observation(
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
        """Release a provider-owned SRBM session while retaining its close acknowledgement."""

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
        """Return standard SensorBus packets retained by a persistent source session."""

        return self._session(session_id).sensor_bus.packets(_NATIVE_SENSOR_ID, delivered=delivered)
        ####

    def _session(self, session_id: str) -> _Ads6SrbmSessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ValueError(f"ADS6 SRBM session {session_id!r} does not exist") from error
        ####


####


def _ads6_srbm_session_substeps(duration_s: float, source_step_s: float) -> int:
    """Validate one caller hold and return its exact source substep count."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("ADS6 SRBM session duration_s must be positive and finite")
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"ADS6 SRBM session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    return steps
    ####


def _ads6_srbm_sensor_bus(seed: int, cadence_s: float) -> SensorBus:
    """Build the native raw fixed-target track bus for one SRBM session."""

    tracker = RelativeStateTrackerSensor(
        RelativeStateTrackerConfig(target_id="ads6-srbm-target"),
        SensorBuildContext(_NATIVE_SENSOR_ID, seed),
    )
    bus = SensorBus()
    bus.register(
        SensorBinding(
            _NATIVE_SENSOR_ID,
            "ads6-srbm-1",
            SensorClockSpec(_NATIVE_SENSOR_ID, "tracking", cadence_s=cadence_s),
            tracker,
            provenance={
                "provider": "relative-state-track",
                "frame": _LOCAL_FRAME_ID,
                "execution": "persistent-source-session",
                "claim_boundary": "Raw committed target geometry only; source seeker enable and exo/endo phase gates remain CADAC behavior.",
            },
            rng_seed=seed,
        )
    )
    return bus
    ####


def _ads6_srbm_session_observation_schema() -> tuple[MissionCompositionSessionChannel, ...]:
    """Declare stable, source-owned ROCKET5 state and sensor observation ports."""

    scalar = {"topology": "continuous", "representation": "scalar"}
    vector3 = {"topology": "product", "representation": "vector3", "components": 3}
    opaque = {"topology": "opaque", "representation": "json"}
    return (
        MissionCompositionSessionChannel(
            id="position_ned_m",
            direction="observation",
            description="Committed ADS6 ROCKET5 local-NED position.",
            quantity=cadac_output_quantity("position_ned_m", "m"),
            unit="m",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="velocity_ned_mps",
            direction="observation",
            description="Committed ADS6 ROCKET5 local-NED velocity.",
            quantity=cadac_output_quantity("velocity_ned_mps", "m/s"),
            unit="m/s",
            shape=(3,),
            value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="altitude_m",
            direction="observation",
            description="Committed Flat3 altitude.",
            quantity=cadac_output_quantity("altitude_m", "m"),
            unit="m",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="speed_mps",
            direction="observation",
            description="Committed ROCKET5 speed.",
            quantity=cadac_output_quantity("speed_mps", "m/s"),
            unit="m/s",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="phase",
            direction="observation",
            description="Current source ascent, ballistic, or reentry phase.",
            data_type="string",
            sampling_semantics="discrete_sample",
            value_space={"topology": "finite_set", "representation": "string"},
        ),
        MissionCompositionSessionChannel(
            id="alpha_deg",
            direction="observation",
            description="Reduced-order source angle of attack response.",
            quantity=cadac_output_quantity("alpha_deg", "deg"),
            unit="deg",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="beta_deg",
            direction="observation",
            description="Reduced-order source sideslip response.",
            quantity=cadac_output_quantity("beta_deg", "deg"),
            unit="deg",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="normal_command_g",
            direction="observation",
            description="Source-owned ROCKET5 normal acceleration command.",
            quantity=cadac_output_quantity("normal_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_command_g",
            direction="observation",
            description="Source-owned ROCKET5 lateral acceleration command.",
            quantity=cadac_output_quantity("lateral_command_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="normal_acceleration_g",
            direction="observation",
            description="Committed ROCKET5 normal specific-force response.",
            quantity=cadac_output_quantity("normal_acceleration_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_acceleration_g",
            direction="observation",
            description="Committed ROCKET5 lateral specific-force response.",
            quantity=cadac_output_quantity("lateral_acceleration_g", "g"),
            unit="g",
            value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="source_sensor_state",
            direction="observation",
            description="Source seeker enable/phase state and its raw geometry values.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
        MissionCompositionSessionChannel(
            id="native_relative_state_track",
            direction="observation",
            description="Latest delivered Taoryx relative-state SensorBus packet at a committed source boundary.",
            data_type="json",
            sampling_semantics="discrete_sample",
            value_space=opaque,
        ),
    )
    ####


def _ads6_srbm_session_observation(
    *,
    session_id: str,
    sequence: int,
    source: Ads6SrbmSession,
    sensor_bus: SensorBus,
    lifecycle: str,
    events: tuple[str, ...] = (),
) -> MissionCompositionSessionObservation:
    """Create a finite session observation from committed ROCKET5 state."""

    sample = source.sample
    return MissionCompositionSessionObservation(
        session_id=session_id,
        sequence=sequence,
        time_s=source.sim_time_s,
        lifecycle=lifecycle,
        values={
            "position_ned_m": sample.position_ned_m,
            "velocity_ned_mps": sample.velocity_ned_mps,
            "altitude_m": sample.altitude_m,
            "speed_mps": sample.speed_mps,
            "phase": sample.phase,
            "alpha_deg": sample.alpha_deg,
            "beta_deg": sample.beta_deg,
            "normal_command_g": sample.normal_command_g,
            "lateral_command_g": sample.lateral_command_g,
            "normal_acceleration_g": sample.normal_acceleration_g,
            "lateral_acceleration_g": sample.lateral_acceleration_g,
            "source_sensor_state": {
                "source_seeker_enabled": bool(source.definition.guidance.seeker_mode),
                "exo_flag": sample.exo_flag,
                "range_to_target_m": sample.range_to_target_m,
                "closing_speed_mps": sample.closing_speed_mps,
                "time_to_go_s": sample.time_to_go_s,
                "target_displacement_ned_m": list(sample.target_displacement_ned_m),
                "target_unit_body": list(sample.target_unit_body),
                "line_of_sight_rate_body_rad_s": list(sample.line_of_sight_rate_body_rad_s),
            },
            "native_relative_state_track": _ads6_srbm_native_track_value(sensor_bus),
        },
        events=events,
    )
    ####


def _ads6_srbm_native_track_value(sensor_bus: SensorBus) -> dict[str, object]:
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


def _ads6_srbm_session_lifecycle(source: Ads6SrbmSession) -> str:
    if source.terminated_reason == "nonfinite_state":
        return "failed"
    if source.completed:
        return "completed"
    return "active"
    ####


def _ads6_srbm_session_events(
    transitions: tuple[Any, ...],
    source: Ads6SrbmSession,
) -> tuple[str, ...]:
    events = tuple(
        f"cadac-source-phase:{transition.previous_phase}:{transition.phase}:{transition.time_s:.17g}"
        for transition in transitions
    )
    if source.impact_result is not None:
        return (*events, f"cadac-source-terminal:{source.impact_result.kind}:{source.impact_result.time_s:.17g}")
    return events
    ####


def build_default_ads6_srbm_configuration(
    provider: CadacAds6SrbmMissionCompositionProvider,
    *,
    configuration_id: str = "ads6-srbm-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one exact source-response ADS6 SRBM configuration."""

    supplied = dict(overrides or {})
    routes = {
        "north_m": ("initialization", "north_m"),
        "east_m": ("initialization", "east_m"),
        "down_m": ("initialization", "down_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "heading_deg": ("initialization", "heading_deg"),
        "flight_path_deg": ("initialization", "flight_path_deg"),
        "alpha_deg": ("initialization", "alpha_deg"),
        "beta_deg": ("initialization", "beta_deg"),
        "target_north_m": ("guidance", "target_north_m"),
        "target_east_m": ("guidance", "target_east_m"),
        "target_down_m": ("guidance", "target_down_m"),
        "seeker_mode": ("guidance", "seeker_mode"),
        "guidance_mode": ("guidance", "guidance_mode"),
        "navigation_gain": ("guidance", "navigation_gain"),
        "maneuver_tgo_start_s": ("guidance", "maneuver_tgo_start_s"),
        "maneuver_initial_amplitude_g": ("guidance", "maneuver_initial_amplitude_g"),
        "maneuver_frequency_rad_s": ("guidance", "maneuver_frequency_rad_s"),
        "maneuver_tgo63_s": ("guidance", "maneuver_tgo63_s"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown ADS6 SRBM configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    values: dict[str, Any] = {"source_phase": ConfigurationParameterValue(value=ADS6_SRBM_PHASE_ID)}
    values.update(
        {
            group_id: ConfigurationGroupValue(
                values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in items.items()}
            )
            for group_id, items in grouped.items()
        }
    )
    schema = provider.get_model_schema(ADS6_SRBM_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=ADS6_SRBM_MODEL_ID,
        model_version=ADS6_SRBM_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=ADS6_SRBM_FIDELITY_ID,
        realization_id=ADS6_SRBM_REALIZATION_ID,
        mission_template_id=ADS6_SRBM_MISSION_ID,
        root=ConfigurationGroupValue(values=values),
    )


####


def register_ads6_srbm_mission_composition(
    provider: CadacAds6SrbmMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    registry.register(CADAC_PROVIDER_ID, ADS6_SRBM_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Ads6SrbmVehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    target = source.guidance.target_position_ned_m
    return TrajectoryConfigurationSchema(
        model_id=ADS6_SRBM_MODEL_ID,
        model_version=ADS6_SRBM_MODEL_VERSION,
        supported_fidelities=(ADS6_SRBM_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="ads6_srbm",
            label="ADS6 SRBM response-law vehicle",
            description="Source ROCKET5 translation, propulsion, aerodynamics, and reduced-order endo response states.",
            children=(
                _parameter(
                    "source_phase",
                    "Source phase",
                    "Exact installed source-model phase selector.",
                    default=ADS6_SRBM_PHASE_ID,
                    choices=(ADS6_SRBM_PHASE_ID,),
                    role="variant",
                ),
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter("north_m", "North", "Initial local north position.", default=initial.position_ned_m[0], unit="m"),
                        _parameter("east_m", "East", "Initial local east position.", default=initial.position_ned_m[1], unit="m"),
                        _parameter("down_m", "Down", "Initial local down position.", default=initial.position_ned_m[2], unit="m"),
                        _parameter("speed_mps", "Speed", "Initial Earth-relative speed.", default=initial.speed_mps, unit="m/s"),
                        _parameter("heading_deg", "Heading", "Initial heading angle.", default=initial.heading_deg, unit="deg"),
                        _parameter("flight_path_deg", "Flight path", "Initial flight-path angle.", default=initial.flight_path_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial source response-law alpha output.", default=initial.alpha_deg, unit="deg"),
                        _parameter("beta_deg", "Sideslip", "Initial source response-law beta output.", default=initial.beta_deg, unit="deg"),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="guidance",
                    label="Guidance and target",
                    children=(
                        _parameter("target_north_m", "Target north", "Fixed source target north coordinate.", default=target[0], unit="m"),
                        _parameter("target_east_m", "Target east", "Fixed source target east coordinate.", default=target[1], unit="m"),
                        _parameter("target_down_m", "Target down", "Fixed source target down coordinate.", default=target[2], unit="m"),
                        _parameter("seeker_mode", "Seeker mode", "0 disabled; 1 kinematic target sensor.", default=source.guidance.seeker_mode, role="variant"),
                        _parameter(
                            "guidance_mode", "Guidance mode", "Decimal |spiral|pro-nav| source mode.", default=source.guidance.guidance_mode, role="variant"
                        ),
                        _parameter("navigation_gain", "Navigation gain", "Source proportional-navigation gain.", default=source.guidance.navigation_gain),
                        _parameter(
                            "maneuver_tgo_start_s",
                            "Maneuver TGO",
                            "Time-to-go threshold for evasive spiral.",
                            default=source.guidance.maneuver_tgo_start_s,
                            unit="s",
                        ),
                        _parameter(
                            "maneuver_initial_amplitude_g",
                            "Maneuver amplitude",
                            "Initial spiral acceleration amplitude.",
                            default=source.guidance.maneuver_initial_amplitude_g,
                            unit="g",
                        ),
                        _parameter(
                            "maneuver_frequency_rad_s",
                            "Maneuver frequency",
                            "Spiral maneuver angular frequency.",
                            default=source.guidance.maneuver_frequency_rad_s,
                            unit="rad/s",
                        ),
                        _parameter(
                            "maneuver_tgo63_s",
                            "Maneuver decay",
                            "Source 63-percent time-to-go decay parameter.",
                            default=source.guidance.maneuver_tgo63_s,
                            unit="s",
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s", "End time", "Requested source-compatible propagation horizon.", default=source.end_time_s, unit="s", role="constraint"
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=source.trajectory_step_s or source.integration_step_s,
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=source.claim_boundary,
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    unit: str | None = None,
    role: str = "initialization",
    choices: tuple[object, ...] = (),
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        value_type="enum" if choices else "number",
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        choices=choices,
        role=role,
        compatible_fidelities=(ADS6_SRBM_FIDELITY_ID,),
        provenance="missiondesignsolutions/CADAC/ADS6 ROCKET5 source case",
    )


####


def _output(
    channel_id: str,
    label: str,
    description: str,
    *,
    unit: str | None = None,
    shape: tuple[int, ...] = (),
    frame: str | None = None,
    data_type: str = "float64",
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
        interpolation="step" if data_type in {"boolean", "string", "json"} else "linear",
        availability="guaranteed",
        compatible_fidelities=(ADS6_SRBM_FIDELITY_ID,),
        compatible_realizations=(ADS6_SRBM_REALIZATION_ID,),
        compatible_mission_templates=(ADS6_SRBM_MISSION_ID,),
        operations=("batch", "step"),
        provenance="CADAC ADS6 ROCKET5 source-grounded response-law reconstruction",
        claim_boundary="Available from the Python source-compatibility runtime; compiled golden parity remains a separate promotion gate.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_ned_m", "Position", "Flat-Earth local north/east/down position.", unit="m", shape=(3,), frame=_LOCAL_FRAME_ID),
        _output("velocity_ned_mps", "Velocity", "Flat-Earth local north/east/down velocity.", unit="m/s", shape=(3,), frame=_LOCAL_FRAME_ID),
    )
    flight = (
        _output("altitude_m", "Altitude", "Source flat-Earth altitude.", unit="m"),
        _output("speed_mps", "Speed", "Earth-relative speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Source heading angle.", unit="deg"),
        _output("flight_path_deg", "Flight path", "Source flight-path angle.", unit="deg"),
        _output("phase", "Source phase", "Endo-ascent, exo-ballistic, or endo-reentry phase.", data_type="string"),
        _output("exo_flag", "Exo flag", "Sticky source flag indicating that exo flight has occurred.", data_type="boolean"),
    )
    response = (
        _output("alpha_deg", "Angle of attack", "Reduced-order source alpha response.", unit="deg"),
        _output("beta_deg", "Sideslip", "Reduced-order source beta response.", unit="deg"),
        _output("pitch_response_rate_rad_s", "Pitch response rate", "Source reduced-order pitch-rate state.", unit="rad/s"),
        _output("yaw_response_rate_rad_s", "Yaw response rate", "Source reduced-order yaw-rate state.", unit="rad/s"),
        _output("normal_command_g", "Normal command", "Source normal acceleration command.", unit="g"),
        _output("lateral_command_g", "Lateral command", "Source lateral acceleration command.", unit="g"),
        _output("normal_acceleration_g", "Normal acceleration", "Realized normal specific force in source body axes.", unit="g"),
        _output("lateral_acceleration_g", "Lateral acceleration", "Realized lateral specific force in source body axes.", unit="g"),
    )
    environment = (
        _output("density_kg_m3", "Density", "NASA-Marshall US76 density.", unit="kg/m^3"),
        _output("pressure_pa", "Pressure", "NASA-Marshall US76 pressure.", unit="Pa"),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("mach", "Mach", "Source Mach number."),
    )
    propulsion = (
        _output("mass_kg", "Mass", "Current source mass.", unit="kg"),
        _output("thrust_n", "Thrust", "Pressure-corrected rocket thrust.", unit="N"),
        _output("propulsion_mode", "Propulsion mode", "Source rocket on/off mode."),
    )
    aerodynamics = (
        _output("lift_coefficient", "Lift coefficient", "Source table lift coefficient."),
        _output("drag_coefficient", "Drag coefficient", "Source table drag coefficient."),
        _output("axial_coefficient", "Axial coefficient", "Source axial body coefficient."),
        _output("side_coefficient", "Side coefficient", "Source side body coefficient."),
        _output("normal_coefficient", "Normal coefficient", "Source normal body coefficient."),
        _output("max_g", "Maximum maneuver load", "Alpha-limited source maneuver capability.", unit="g"),
        _output("specific_force_body_mps2", "Specific force", "Non-gravitational specific force in source rocket axes.", unit="m/s^2", shape=(3,)),
    )
    terminal = (
        _output("range_to_target_m", "Target range", "Range to configured fixed target coordinates.", unit="m"),
        _output("closing_speed_mps", "Closing speed", "Source signed target closing-speed quantity.", unit="m/s"),
        _output("time_to_go_s", "Time to go", "Range divided by absolute closing speed.", unit="s"),
        _output(
            "target_displacement_ned_m",
            "Target displacement NED",
            "Source seeker target displacement in the ADS6 local-NED frame.",
            unit="m",
            shape=(3,),
            frame=_LOCAL_FRAME_ID,
        ),
        _output(
            "target_unit_body",
            "Target unit line of sight",
            "Source seeker unit line-of-sight vector in ROCKET5 body axes.",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output(
            "line_of_sight_rate_body_rad_s",
            "Line-of-sight rate",
            "Source seeker line-of-sight angular-rate vector in ROCKET5 body axes.",
            unit="rad/s",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
    )
    telemetry = (*flight, *response, *environment, *propulsion, *aerodynamics, *terminal)
    return TrajectoryOutputSchema(
        model_id=ADS6_SRBM_MODEL_ID,
        model_version=ADS6_SRBM_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="flight_condition", label="Flight condition", description="Flat3 kinematics and phase state.", channel_ids=tuple(item.id for item in flight)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="response_law",
                label="Response law",
                description="Reduced-order alpha/beta and rate response states.",
                channel_ids=tuple(item.id for item in response),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="environment", label="Environment", description="NASA-Marshall atmosphere diagnostics.", channel_ids=tuple(item.id for item in environment)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="propulsion", label="Propulsion", description="Single-stage source rocket diagnostics.", channel_ids=tuple(item.id for item in propulsion)
            ),
            TrajectoryTelemetryGroupMetadata(
                id="aerodynamics",
                label="Aerodynamics",
                description="Table coefficients and force closure.",
                channel_ids=tuple(item.id for item in aerodynamics),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="terminal",
                label="Terminal geometry",
                description="Optional fixed-target seeker and guidance geometry.",
                channel_ids=tuple(item.id for item in terminal),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="ADS6 ROCKET5 translational truth plus reduced-order response-law telemetry; no rigid-body attitude channels are emitted.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=ADS6_SRBM_FIDELITY_ID,
        label="CADAC ADS6 SRBM pseudo-6DoF",
        rank=1,
        declared=True,
        dynamics_fidelity="pseudo_6dof",
        input_realization="provider_defined",
        runtime_fidelity="pseudo_6dof",
        control_realization="response_law",
        promotion_status="development",
        operations=("validate", "batch", "step"),
        profile_id="cadac.ads6.srbm.source-model",
        blockers=("compiled/source golden trajectory parity is not yet registered",),
        required_operations=("Flat3 translation", "SRBM propulsion and aerodynamics", "endo alpha/beta response law"),
        claim_boundary="Translation is integrated while alpha, beta, and pitch/yaw rates are reduced-order response states without rigid-body moment closure.",
    )
    realization = TrajectoryRealizationMetadata(
        id=ADS6_SRBM_REALIZATION_ID,
        label="CADAC ADS6 ROCKET5 response law",
        description="Source-ordered endo-ascent, exo-ballistic, and endo-reentry SRBM model.",
        status="available",
        dynamics_fidelities=("pseudo_6dof",),
        input_realization="provider_defined",
        controls=source_managed_control_advertisement(
            mission_ids=(ADS6_SRBM_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/ADS6/rocket_modules.cpp",),
            operations=("batch", "step"),
            claim_boundary="Source-program response-law control is internal to the persistent compatibility runtime; sessions expose no caller action channel.",
        ),
        fidelity_aliases=(ADS6_SRBM_FIDELITY_ID,),
        mission_template_ids=(ADS6_SRBM_MISSION_ID,),
        operations=("validate", "batch", "step"),
        native_factory_ids=(ADS6_SRBM_EXECUTOR_ID, ADS6_SRBM_SESSION_EXECUTOR_ID),
        source_refs=("missiondesignsolutions/CADAC/ADS6/rocket_modules.cpp",),
        blockers=(),
        claim_boundary="Executable ROCKET5 response-law plant; no physical attitude effectors or body-moment integration are claimed.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=ADS6_SRBM_MISSION_ID,
        name="ADS6 SRBM source flight",
        description="Execute the installed ROCKET5 ascent/ballistic/reentry source program.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("endo_ascent", "exo_ballistic", "endo_reentry"),
        compatible_fidelities=(ADS6_SRBM_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=ADS6_SRBM_FIDELITY_ID,
                realization_id=ADS6_SRBM_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation preserves the exact response-law fidelity boundary.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=ADS6_SRBM_FIDELITY_ID,
                realization_id=ADS6_SRBM_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatibility_runtime",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_SRBM_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed ADS6 SRBM plug-in.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=ADS6_SRBM_FIDELITY_ID,
                realization_id=ADS6_SRBM_REALIZATION_ID,
                operation="step",
                status="available",
                execution_mode="source_compatible_persistent_session",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_SRBM_SESSION_EXECUTOR_ID,
                claim_boundary="Step dispatch owns ROCKET5 source state and accepts only whole source-step holds with source-managed response-law control.",
            ),
        ),
        provenance="CADAC ADS6 Flat3 and ROCKET5 source modules",
        claim_boundary="Installed source ascent, ballistic exo, and reentry response-law mission only.",
    )
    return TrajectoryModelMetadata(
        id=ADS6_SRBM_MODEL_ID,
        name="CADAC ADS6 short-range ballistic missile",
        version=ADS6_SRBM_MODEL_VERSION,
        description="CADAC ADS6 ROCKET5/SRBM5 pseudo-6DoF target exposed as a Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC ADS6 SRBM",
            short_name="ADS6 SRBM",
            summary="Flat-Earth short-range ballistic missile with endo alpha/beta response and ballistic exo flight.",
            category="ballistic_missile",
            subcategory="short_range",
            sort_key="cadac-ads6-srbm",
            badges=("CADAC", "pseudo-6DoF", "SRBM", "response law"),
            default_fidelity_id=ADS6_SRBM_FIDELITY_ID,
            default_mission_template_id=ADS6_SRBM_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "alpha_deg", "beta_deg"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="ROCKET5",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/ADS6",),
                    provenance="CADAC ADS6 source package",
                    claim_boundary="Identity metadata only; SRBM5 is the semantic vehicle role while ROCKET5 is the source class token.",
                ),
            ),
        ),
        family_id=ADS6_SRBM_MODEL_ID,
        physical_family="short_range_ballistic_missile",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "ads6", "srbm", "pseudo_6dof", "response_law"),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("endo_ascent", "exo_ballistic", "endo_reentry"),
            termination_modes=("end_time", "ground_impact", "target_closest_approach", "nonfinite_state"),
            operations=("discover", "validate", "batch", "step"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=False,
        ),
        realizations=(realization,),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_FRAME_ID,
                name="CADAC ADS6 local level",
                description="Flat-Earth local north/east/down frame used by Flat3.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source local-level reference point",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/ADS6/flat3_modules.cpp",),
                provenance="CADAC ADS6 Flat3 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC ADS6 ROCKET5 body",
                description="Instantaneous ROCKET5 body frame used by the source seeker and response law.",
                frame_kind="body",
                axes=("forward", "right", "down"),
                handedness="right",
                origin="ROCKET5 source reference point",
                orientation="source Flat3 vehicle-to-local direction cosine matrix",
                source_refs=("missiondesignsolutions/CADAC/ADS6/rocket_modules.cpp",),
                provenance="CADAC ADS6 ROCKET5 source seeker convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        provenance="CADAC ADS6 ROCKET5 source-grounded Python reconstruction",
        claim_boundary=(
            "Executable pseudo-6DoF SRBM source vehicle. Alpha/beta and pitch/yaw-rate response states are one-way reduced-order dynamics; "
            "the result is not promoted to rigid-body 6DoF."
        ),
    )


####


def _overrides_from_resolved(resolved: Any) -> Ads6SrbmPluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("ADS6 SRBM resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    guidance = _group(resolved, "guidance")
    runtime = _group(resolved, "runtime")
    return Ads6SrbmPluginOverrides(
        position_ned_m=(float(initial["north_m"]), float(initial["east_m"]), float(initial["down_m"])),
        speed_mps=float(initial["speed_mps"]),
        heading_deg=float(initial["heading_deg"]),
        flight_path_deg=float(initial["flight_path_deg"]),
        alpha_deg=float(initial["alpha_deg"]),
        beta_deg=float(initial["beta_deg"]),
        target_position_ned_m=(
            float(guidance["target_north_m"]),
            float(guidance["target_east_m"]),
            float(guidance["target_down_m"]),
        ),
        seeker_mode=int(guidance["seeker_mode"]),
        guidance_mode=int(guidance["guidance_mode"]),
        navigation_gain=float(guidance["navigation_gain"]),
        maneuver_tgo_start_s=float(guidance["maneuver_tgo_start_s"]),
        maneuver_initial_amplitude_g=float(guidance["maneuver_initial_amplitude_g"]),
        maneuver_frequency_rad_s=float(guidance["maneuver_frequency_rad_s"]),
        maneuver_tgo63_s=float(guidance["maneuver_tgo63_s"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(resolved: Mapping[str, Any], group_id: str) -> Mapping[str, Any]:
    group = resolved.get(group_id)
    if not isinstance(group, Mapping):
        raise ValueError(f"ADS6 SRBM resolved configuration is missing group {group_id!r}")
    ####
    return group


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id.endswith("_m"):
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {"heading_deg", "flight_path_deg", "alpha_deg", "beta_deg"}:
        return "deg"
    ####
    if parameter_id in {"maneuver_tgo_start_s", "maneuver_tgo63_s", "end_time_s", "sample_step_s"}:
        return "s"
    ####
    if parameter_id == "maneuver_initial_amplitude_g":
        return "g"
    ####
    if parameter_id == "maneuver_frequency_rad_s":
        return "rad/s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Ads6SrbmRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=ADS6_SRBM_FIDELITY_ID,
        operation="batch",
        realization_id=ADS6_SRBM_REALIZATION_ID,
        mission_template_id=ADS6_SRBM_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    failed = run.terminated_reason == "nonfinite_state"
    terminated = run.terminated_reason not in {"end_time", "nonfinite_state"}
    object_result = TrajectoryObject(
        object_id="ads6-srbm-1",
        model_id=ADS6_SRBM_MODEL_ID,
        realization_id=ADS6_SRBM_REALIZATION_ID,
        name="CADAC ADS6 short-range ballistic missile",
        role="target",
        fidelity=ADS6_SRBM_FIDELITY_ID,
        status="failed" if failed else "terminated" if terminated else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC ADS6 ROCKET5 source-compatible pseudo-6DoF runner",
        claim_boundary=run.claim_boundary,
    )
    events: tuple[TrajectoryEvent, ...] = ()
    if request.output.include_events:
        phase_events = tuple(
            TrajectoryEvent(
                id=f"phase-{index + 1}",
                time_s=transition.time_s,
                category="custom",
                kind="cadac-source-phase-transition",
                object_id="ads6-srbm-1",
                detail=f"{transition.previous_phase} -> {transition.phase}",
                data={"previous_phase": transition.previous_phase, "phase": transition.phase, "altitude_m": transition.altitude_m},
            )
            for index, transition in enumerate(run.phase_transitions)
        )
        impact_events = ()
        if run.impact is not None:
            impact_events = (
                TrajectoryEvent(
                    id=run.impact.kind,
                    time_s=run.impact.time_s,
                    category="impact",
                    kind=run.impact.kind,
                    object_id="ads6-srbm-1",
                    detail=f"ADS6 SRBM terminated by {run.impact.kind.replace('_', ' ')}.",
                    data={"miss_distance_m": run.impact.miss_distance_m},
                ),
            )
        ####
        events = (*phase_events, *impact_events)
    ####
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-pseudo-6dof-response-law",
            message=(
                "ADS6 ROCKET5 integrates Flat3 translation while alpha, beta, and pitch/yaw response rates are reduced-order endo states; "
                "exo flight is ballistic and no rigid-body attitude or moment closure is present."
            ),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_SRBM_MODEL_ID,
            object_id="ads6-srbm-1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=ADS6_SRBM_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=ADS6_SRBM_MODEL_ID,
        primary_object_id="ads6-srbm-1",
        status="failed" if failed else "terminated" if terminated else "completed",
        objects=(object_result,),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact ADS6 ROCKET5 target returned through the common runner at pseudo-6DoF response-law fidelity.",
    )


####


def _runtime_channel(
    channel: TrajectoryOutputChannelMetadata,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryChannelMetadata:
    group_by_channel = {channel_id: group.id for group in output_schema.telemetry_groups for channel_id in group.channel_ids}
    return TrajectoryChannelMetadata(
        id=channel.id,
        channel_class="core_state" if channel in output_schema.core_channels else "telemetry",
        telemetry_group=group_by_channel.get(channel.id),
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


def _sample_value(channel_id: str, sample: Ads6SrbmSample) -> object:
    values: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "altitude_m": sample.altitude_m,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "alpha_deg": sample.alpha_deg,
        "beta_deg": sample.beta_deg,
        "pitch_response_rate_rad_s": sample.pitch_response_rate_rad_s,
        "yaw_response_rate_rad_s": sample.yaw_response_rate_rad_s,
        "phase": sample.phase,
        "exo_flag": sample.exo_flag,
        "density_kg_m3": sample.density_kg_m3,
        "pressure_pa": sample.pressure_pa,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "mach": sample.mach,
        "mass_kg": sample.mass_kg,
        "thrust_n": sample.thrust_n,
        "propulsion_mode": sample.propulsion_mode,
        "lift_coefficient": sample.lift_coefficient,
        "drag_coefficient": sample.drag_coefficient,
        "axial_coefficient": sample.axial_coefficient,
        "side_coefficient": sample.side_coefficient,
        "normal_coefficient": sample.normal_coefficient,
        "max_g": sample.max_g,
        "normal_command_g": sample.normal_command_g,
        "lateral_command_g": sample.lateral_command_g,
        "normal_acceleration_g": sample.normal_acceleration_g,
        "lateral_acceleration_g": sample.lateral_acceleration_g,
        "range_to_target_m": sample.range_to_target_m,
        "closing_speed_mps": sample.closing_speed_mps,
        "time_to_go_s": sample.time_to_go_s,
        "target_displacement_ned_m": sample.target_displacement_ned_m,
        "target_unit_body": sample.target_unit_body,
        "line_of_sight_rate_body_rad_s": sample.line_of_sight_rate_body_rad_s,
        "specific_force_body_mps2": sample.specific_force_body_mps2,
    }
    return values[channel_id]


####


__all__ = [
    "ADS6_SRBM_EXECUTOR_ID",
    "ADS6_SRBM_FIDELITY_ID",
    "ADS6_SRBM_MISSION_ID",
    "ADS6_SRBM_MODEL_ID",
    "ADS6_SRBM_MODEL_VERSION",
    "ADS6_SRBM_PHASE_ID",
    "ADS6_SRBM_REALIZATION_ID",
    "ADS6_SRBM_SESSION_EXECUTOR_ID",
    "CadacAds6SrbmMissionCompositionProvider",
    "build_default_ads6_srbm_configuration",
    "register_ads6_srbm_mission_composition",
]
