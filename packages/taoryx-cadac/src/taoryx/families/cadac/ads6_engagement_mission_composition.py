"""Taoryx Mission Composition bridge for source-ordered ADS6 engagements."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

from taoryx.runtime import SensorBinding, SensorBus, SensorClockSpec
from taoryx.sensor_api import MeasurementPacket, SensorBuildContext
from taoryx.sensor_plugins.relative_state import RelativeStateTrackerConfig, RelativeStateTrackerSensor
from taoryx.trajectory.configuration_contract import (
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
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

from .ads6_engagement import (
    Ads6EngagementRunConfig,
    Ads6EngagementRunResult,
    Ads6EngagementSample,
    Ads6EngagementSession,
    Ads6EngagementSourceDefinition,
)
from .ads6_engagement_plugin import Ads6EngagementPlugin
from .ads6_sam import _dcm_body_from_local, ads6_sam_initial_quaternion
from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import source_managed_control_advertisement
from .output_metadata import cadac_output_quantity
from .sensor_adapter import cadac_local_ned_sensor_context
from .session_authority import (
    CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
    CADAC_SOURCE_PROGRAM_PROFILE_ID,
    source_managed_session_authority_fields,
    source_managed_session_authority_state,
    validate_source_managed_session_request,
)

ADS6_ENGAGEMENT_MODEL_ID = "cadac.ads6.engagement"
ADS6_ENGAGEMENT_MODEL_VERSION = "0.13.0"
ADS6_ENGAGEMENT_PHASE_ID = "source_package"
ADS6_ENGAGEMENT_EXECUTOR_ID = "cadac.ads6.engagement.source_ordered.batch"
ADS6_ENGAGEMENT_SESSION_EXECUTOR_ID = "cadac.ads6.engagement.source_ordered.session"
_LOCAL_FRAME_ID = "cadac.ads6.local_ned"


@dataclass(slots=True)
class _Ads6EngagementSessionRecord:
    """Provider-owned ADS6 package and native raw-track state."""

    descriptor: MissionCompositionSessionDescriptor
    prepared: PreparedTrajectoryConfiguration
    definition: Ads6EngagementSourceDefinition
    source: Ads6EngagementSession
    sensor_bus: SensorBus
    sensor_ids: tuple[str, ...]
    seed: int
    sequence: int
    observation: MissionCompositionSessionObservation
    lifecycle: str = "ready"
    closed: bool = False


class CadacAds6EngagementMissionCompositionProvider:
    """Self-describing multi-root provider for one installed ADS6 package case."""

    def __init__(self, plugin: Ads6EngagementPlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish ADS6 engagement provider: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._definition = plugin.source_definition()
        self._fidelity_id = _sam_fidelity(self._definition.sam_phase)
        self._realization_id = f"cadac-ads6-package-{self._definition.sam_phase.replace('_', '-')}"
        self._mission_id = f"ads6_{self._definition.target_kind}_defense"
        self._schema = _build_configuration_schema(
            self._definition.target_kind,
            self._definition.sam_phase,
            self._definition.end_time_s,
            self._definition.trajectory_step_s or self._definition.integration_step_s,
            self._fidelity_id,
        )
        self._output_schema = _build_output_schema(
            self._fidelity_id,
            self._realization_id,
            self._mission_id,
        )
        self._sessions: dict[str, _Ads6EngagementSessionRecord] = {}
        self._model = _build_model_metadata(
            self._definition.target_kind,
            self._definition.sam_phase,
            self._fidelity_id,
            self._realization_id,
            self._mission_id,
            self._schema,
            self._output_schema,
            self._definition.claim_boundary,
        )

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != ADS6_ENGAGEMENT_MODEL_ID:
            raise KeyError(f"unknown ADS6 engagement model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != ADS6_ENGAGEMENT_MODEL_ID:
            raise KeyError(f"unknown ADS6 engagement model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != self._fidelity_id:
            raise ValueError(f"ADS6 package requires SAM envelope fidelity {self._fidelity_id!r}")
        ####
        if configuration.realization_id not in {None, self._realization_id}:
            raise ValueError(f"ADS6 package requires realization {self._realization_id!r}")
        ####
        if configuration.mission_template_id not in {None, self._mission_id}:
            raise ValueError(f"ADS6 package requires mission template {self._mission_id!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        resolved = prepared.resolved
        if not isinstance(resolved, Mapping):
            raise ValueError("ADS6 package resolved configuration must be a mapping")
        ####
        if resolved.get("source_phase") != ADS6_ENGAGEMENT_PHASE_ID:
            raise ValueError(f"ADS6 package requires source phase {ADS6_ENGAGEMENT_PHASE_ID!r}")
        ####
        package = _group(resolved, "package")
        if package.get("target_kind") != self._definition.target_kind:
            raise ValueError("ADS6 package target kind cannot differ from the installed source case")
        ####
        if package.get("sam_phase") != self._definition.sam_phase:
            raise ValueError("ADS6 package SAM realization cannot differ from the installed source case")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != ADS6_ENGAGEMENT_MODEL_ID:
            raise ValueError("ADS6 engagement executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 engagement prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_run_config(prepared.resolved))
        return _project_run_result(
            request,
            run,
            self._output_schema,
            self._realization_id,
            self._mission_id,
        )

    ####

    def open_session(
        self,
        request: MissionCompositionOpenSessionRequest,
        *,
        advertised_provider_version: str | None = None,
    ) -> MissionCompositionSessionDescriptor:
        """Open one persistent source-ordered ADS6 package session."""

        if request.session_id in self._sessions:
            raise ValueError(f"ADS6 engagement session {request.session_id!r} already exists")
        ####
        expected_version = ADS6_ENGAGEMENT_MODEL_VERSION if advertised_provider_version is None else advertised_provider_version
        if request.provider_id != CADAC_PROVIDER_ID or request.provider_version != expected_version:
            raise ValueError("ADS6 engagement session request names another provider version")
        if request.model_id != ADS6_ENGAGEMENT_MODEL_ID:
            raise ValueError("ADS6 engagement session request names another model")
        validate_source_managed_session_request(request.authority_profile_id, request.command_source_id)
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("ADS6 engagement session prepared configuration is stale")
        config = _run_config(prepared.resolved)
        if not math.isclose(request.integration_step_s, self._definition.integration_step_s, rel_tol=0.0, abs_tol=1.0e-12):
            raise ValueError(
                "ADS6 engagement session integration_step_s must equal the source package step "
                f"{self._definition.integration_step_s:.17g} s"
            )
        ####
        source = Ads6EngagementSession(self._definition, config)
        seed = 0 if request.seed is None else request.seed
        sensor_bus, sensor_ids = _engagement_sensor_bus(self._definition, seed)
        for index, sensor_id in enumerate(sensor_ids):
            context = _engagement_sensor_context(source, index)
            sensor_bus.accepted_context(sensor_id, context.host, context)
        ####
        observation = _engagement_session_observation(
            session_id=request.session_id,
            sequence=0,
            source=source,
            sensor_bus=sensor_bus,
            sensor_ids=sensor_ids,
            lifecycle="ready",
        )
        descriptor = MissionCompositionSessionDescriptor(
            session_id=request.session_id,
            provider_id=CADAC_PROVIDER_ID,
            provider_version=request.provider_version,
            model_id=ADS6_ENGAGEMENT_MODEL_ID,
            mission_template_id=self._mission_id,
            realization_id=self._realization_id,
            fidelity=self._fidelity_id,
            configuration_fingerprint=prepared.fingerprint,
            seed=request.seed,
            integration_step_s=self._definition.integration_step_s,
            supports_spawned_entities=False,
            action_schema=(),
            **source_managed_session_authority_fields(),
            observation_schema=_engagement_session_observation_schema(),
            initial_observation=observation,
            claim_boundary=(
                "The provider owns persistent vehicle-major SAM, target, RADAR0, source controller, launch-latch, "
                "source-packet, event, and stochastic state. Native raw relative-state packets are emitted after each "
                "complete package epoch and do not replace source RF/IR or RADAR0 behavior."
            ),
        )
        self._sessions[request.session_id] = _Ads6EngagementSessionRecord(
            descriptor=descriptor,
            prepared=prepared,
            definition=self._definition,
            source=source,
            sensor_bus=sensor_bus,
            sensor_ids=sensor_ids,
            seed=seed,
            sequence=0,
            observation=observation,
        )
        return descriptor
    ####

    create_session = open_session

    def inspect_session(self, request: MissionCompositionInspectSessionRequest | str) -> MissionCompositionSessionObservation:
        session_id = request.session_id if isinstance(request, MissionCompositionInspectSessionRequest) else request
        return self._session(session_id).observation
    ####

    def step_session(self, request: MissionCompositionSessionStepRequest) -> MissionCompositionSessionStepResult:
        """Advance one persistent ADS6 package without batch replay."""

        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("ADS6 engagement session is closed")
        if record.lifecycle in {"completed", "failed"}:
            raise ValueError("ADS6 engagement session is already terminal")
        if request.expected_sequence is not None and request.expected_sequence != record.sequence:
            raise ValueError(f"ADS6 engagement expected sequence {request.expected_sequence}, current sequence is {record.sequence}")
        if request.action:
            raise ValueError("ADS6 source-managed session does not accept external action channels")
        validate_source_managed_session_request(request.authority_profile_id, None)
        ####
        substeps = _engagement_session_substeps(request.duration_s, record.definition.integration_step_s)
        time_start_s = record.source.sim_time_s
        previous_contexts = tuple(_engagement_sensor_context(record.source, index) for index in range(len(record.sensor_ids)))
        events: list[str] = []
        for _ in range(substeps):
            epochs = record.source.advance(record.definition.integration_step_s)
            if not epochs:
                break
            ####
            for index, sensor_id in enumerate(record.sensor_ids):
                context = _engagement_sensor_context(record.source, index)
                record.sensor_bus.accepted_context(
                    sensor_id,
                    context.host,
                    context,
                    previous_truth=previous_contexts[index].host,
                    previous_context=previous_contexts[index],
                )
            ####
            previous_contexts = tuple(_engagement_sensor_context(record.source, index) for index in range(len(record.sensor_ids)))
            events.extend(_engagement_event_ids(epochs[-1].events))
            if record.source.completed:
                break
            ####
        ####
        record.sequence += 1
        record.lifecycle = _engagement_session_lifecycle(record.source)
        record.observation = _engagement_session_observation(
            session_id=request.session_id,
            sequence=record.sequence,
            source=record.source,
            sensor_bus=record.sensor_bus,
            sensor_ids=record.sensor_ids,
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
            authority_profile_id=CADAC_SOURCE_PROGRAM_PROFILE_ID,
            command_source_id=CADAC_SOURCE_PROGRAM_COMMAND_SOURCE_ID,
            lowering_evidence=source_managed_session_authority_state().model_dump(mode="json"),
        )
    ####

    def reset_session(self, request: MissionCompositionResetSessionRequest) -> MissionCompositionSessionObservation:
        record = self._session(request.session_id)
        if record.closed:
            raise ValueError("ADS6 engagement session is closed")
        selected_seed = record.seed if request.seed is None else request.seed
        record.source.reset()
        record.sensor_bus, record.sensor_ids = _engagement_sensor_bus(record.definition, selected_seed)
        for index, sensor_id in enumerate(record.sensor_ids):
            context = _engagement_sensor_context(record.source, index)
            record.sensor_bus.accepted_context(sensor_id, context.host, context)
        ####
        record.seed = selected_seed
        record.sequence = 0
        record.lifecycle = "ready"
        record.observation = _engagement_session_observation(
            session_id=request.session_id,
            sequence=0,
            source=record.source,
            sensor_bus=record.sensor_bus,
            sensor_ids=record.sensor_ids,
            lifecycle="ready",
        )
        return record.observation
    ####

    def close_session(self, request: MissionCompositionCloseSessionRequest | str) -> MissionCompositionClosedSession:
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
        return tuple(sorted(session_id for session_id, record in self._sessions.items() if not record.closed))
    ####

    def has_session(self, session_id: str) -> bool:
        return session_id in self._sessions
    ####

    def session_sensor_packets(self, session_id: str, *, delivered: bool = True) -> tuple[MeasurementPacket[Any], ...]:
        record = self._session(session_id)
        packets = [packet for sensor_id in record.sensor_ids for packet in record.sensor_bus.packets(sensor_id, delivered=delivered)]
        return tuple(sorted(packets, key=lambda packet: (packet.sampled_at_s, packet.sensor_id, packet.sequence)))
    ####

    def _session(self, session_id: str) -> _Ads6EngagementSessionRecord:
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ValueError(f"ADS6 engagement session {session_id!r} does not exist") from error
    ####


####


def _engagement_session_substeps(duration_s: float, source_step_s: float) -> int:
    """Validate a composition hold against the exact ADS6 package step."""

    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("ADS6 engagement session duration_s must be positive and finite")
    ####
    steps = round(duration_s / source_step_s)
    tolerance_s = max(1.0e-12, source_step_s * 1.0e-9)
    if steps <= 0 or not math.isclose(duration_s, steps * source_step_s, rel_tol=0.0, abs_tol=tolerance_s):
        raise ValueError(f"ADS6 engagement session duration_s must be an integral multiple of source step {source_step_s:.17g} s")
    ####
    return steps


####


def _engagement_sensor_bus(definition: Ads6EngagementSourceDefinition, seed: int) -> tuple[SensorBus, tuple[str, ...]]:
    """Build one native raw relative-state sensor per source SAM/target pair."""

    sam_bindings = tuple(item for item in definition.source_order if item.actor_kind == "sam")
    target_bindings = tuple(item for item in definition.source_order if item.actor_kind in {"aircraft", "srbm"})
    bus = SensorBus()
    sensor_ids: list[str] = []
    for index, (sam, target) in enumerate(zip(sam_bindings, target_bindings, strict=True)):
        sensor_id = f"ads6-engagement-{sam.actor_id}-native-relative-state"
        tracker = RelativeStateTrackerSensor(
            RelativeStateTrackerConfig(target_id=target.actor_id),
            SensorBuildContext(sensor_id, seed + index),
        )
        bus.register(
            SensorBinding(
                sensor_id,
                sam.actor_id,
                SensorClockSpec(sensor_id, "tracking", cadence_s=definition.integration_step_s),
                tracker,
                provenance={
                    "provider": "relative-state-track",
                    "frame": _LOCAL_FRAME_ID,
                    "execution": "persistent-source-package-session",
                    "claim_boundary": (
                        "Raw committed SAM/target geometry only; source RF/IR gimbal, acquisition, lock, "
                        "RADAR0 tracking, launch scheduling, and source packet lag remain package-owned state."
                    ),
                },
                rng_seed=seed + index,
            )
        )
        sensor_ids.append(sensor_id)
    ####
    return bus, tuple(sensor_ids)


####


def _engagement_sensor_context(source: Ads6EngagementSession, index: int):
    """Project one SAM/target pair from the committed source package into SensorBus truth."""

    sam_bindings = tuple(item for item in source.definition.source_order if item.actor_kind == "sam")
    target_bindings = tuple(item for item in source.definition.source_order if item.actor_kind in {"aircraft", "srbm"})
    sam_packet = source.packet(sam_bindings[index].actor_id)
    target_packet = source.packet(target_bindings[index].actor_id)
    if source.last_epoch is None:
        initial = source.definition.sams[index].initial_state
        quaternion = np.asarray(ads6_sam_initial_quaternion(initial), dtype=np.float64)
        body_rates = np.asarray(initial.body_rates_deg_s, dtype=np.float64) * math.pi / 180.0
    else:
        sample = source.last_epoch.sam_samples[index]
        quaternion = np.asarray(sample.quaternion_wxyz, dtype=np.float64)
        body_rates = np.asarray(sample.body_rates_rad_s, dtype=np.float64)
    ####
    return cadac_local_ned_sensor_context(
        time_s=source.sim_time_s,
        host_position_ned_m=np.asarray(sam_packet.position_ned_m, dtype=np.float64),
        host_velocity_ned_mps=np.asarray(sam_packet.velocity_ned_mps, dtype=np.float64),
        target_id=target_packet.actor_id,
        target_position_ned_m=np.asarray(target_packet.position_ned_m, dtype=np.float64),
        target_velocity_ned_mps=np.asarray(target_packet.velocity_ned_mps, dtype=np.float64),
        body_from_local=_dcm_body_from_local(quaternion),
        host_body_rate_rad_s=body_rates,
    )
    ####


####


def _engagement_session_observation_schema() -> tuple[MissionCompositionSessionChannel, ...]:
    """Declare primary-SAM control evidence plus all native raw track observations."""

    scalar = {"topology": "continuous", "representation": "scalar"}
    vector3 = {"topology": "product", "representation": "vector3", "components": 3}
    vector4 = {"topology": "product", "representation": "vector4", "components": 4}
    opaque = {"topology": "opaque", "representation": "json"}
    return (
        MissionCompositionSessionChannel(
            id="position_ned_m", direction="observation", description="Committed primary SAM local-NED position.",
            quantity=cadac_output_quantity("position_ned_m", "m"), unit="m", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="velocity_ned_mps", direction="observation", description="Committed primary SAM local-NED velocity.",
            quantity=cadac_output_quantity("velocity_ned_mps", "m/s"), unit="m/s", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="quaternion_wxyz", direction="observation", description="Committed primary SAM body-from-local quaternion.",
            unit="1", shape=(4,), value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="target_position_ned_m", direction="observation", description="Committed paired target local-NED position.",
            quantity=cadac_output_quantity("target_position_ned_m", "m"), unit="m", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="target_velocity_ned_mps", direction="observation", description="Committed paired target local-NED velocity.",
            quantity=cadac_output_quantity("target_velocity_ned_mps", "m/s"), unit="m/s", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_control_deg", direction="observation", description="Source-controller requested primary SAM roll, pitch, and yaw control.",
            unit="deg", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="achieved_control_deg", direction="observation", description="Primary SAM actuator-achieved roll, pitch, and yaw control.",
            unit="deg", shape=(3,), value_space=vector3,
        ),
        MissionCompositionSessionChannel(
            id="requested_fins_deg", direction="observation", description="Primary SAM requested physical fin angles.",
            unit="deg", shape=(4,), value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="achieved_fins_deg", direction="observation", description="Primary SAM achieved physical fin angles.",
            unit="deg", shape=(4,), value_space=vector4,
        ),
        MissionCompositionSessionChannel(
            id="normal_command_g", direction="observation", description="Source-controller normal acceleration command.",
            quantity=cadac_output_quantity("normal_command_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="lateral_command_g", direction="observation", description="Source-controller lateral acceleration command.",
            quantity=cadac_output_quantity("lateral_command_g", "g"), unit="g", value_space=scalar,
        ),
        MissionCompositionSessionChannel(
            id="achieved_lateral_normal_acceleration_g", direction="observation", description="Primary SAM achieved lateral and normal acceleration response.",
            unit="g", shape=(2,), value_space={"topology": "product", "representation": "vector2", "components": 2},
        ),
        MissionCompositionSessionChannel(
            id="source_controller_state", direction="observation", description="Source RF/IR, guidance, RADAR0, and packet-lag controller evidence.",
            data_type="json", sampling_semantics="discrete_sample", value_space=opaque,
        ),
        MissionCompositionSessionChannel(
            id="native_relative_state_tracks", direction="observation", description="Latest delivered Taoryx raw relative-state packets for all source SAM/target pairs.",
            data_type="json", sampling_semantics="discrete_sample", value_space=opaque,
        ),
    )
    ####


####


def _engagement_session_observation(
    *,
    session_id: str,
    sequence: int,
    source: Ads6EngagementSession,
    sensor_bus: SensorBus,
    sensor_ids: tuple[str, ...],
    lifecycle: str,
    events: tuple[str, ...] = (),
) -> MissionCompositionSessionObservation:
    """Return a committed primary-SAM observation plus package-level evidence."""

    sam_bindings = tuple(item for item in source.definition.source_order if item.actor_kind == "sam")
    target_bindings = tuple(item for item in source.definition.source_order if item.actor_kind in {"aircraft", "srbm"})
    primary = source.packet(sam_bindings[0].actor_id)
    target = source.packet(target_bindings[0].actor_id)
    if source.last_epoch is None:
        initial = source.definition.sams[0].initial_state
        quaternion = ads6_sam_initial_quaternion(initial)
        requested_control = (0.0, 0.0, 0.0)
        achieved_control = (0.0, 0.0, 0.0)
        requested_fins = (0.0, 0.0, 0.0, 0.0)
        achieved_fins = (0.0, 0.0, 0.0, 0.0)
        acceleration = (0.0, 0.0)
        controller: dict[str, object] = {"source_schedule": "vehicle_major_package", "state": "unadvanced"}
    else:
        sample = source.last_epoch.sam_samples[0]
        controller_sample = source.last_epoch.controller_samples[0]
        quaternion = sample.quaternion_wxyz
        requested_control = sample.requested_control_deg
        achieved_control = sample.achieved_control_deg
        requested_fins = sample.requested_fins_deg
        achieved_fins = sample.achieved_fins_deg
        acceleration = sample.achieved_lateral_normal_acceleration_g
        controller = {
            "source_schedule": "sam_then_target_then_radar",
            "control_mode": controller_sample.control_mode,
            "guidance_mode": controller_sample.guidance_mode,
            "sensor_mode": controller_sample.sensor_mode,
            "sensor_kind": controller_sample.sensor_kind,
            "target_actor_id": controller_sample.target_actor_id,
            "target_packet_epoch_s": controller_sample.target_packet_epoch_s,
            "target_range_m": controller_sample.target_range_m,
            "closing_speed_mps": controller_sample.closing_speed_mps,
            "radar_intercept_point_ned_m": controller_sample.radar_intercept_point_ned_m,
            "normal_lateral_command_g": controller_sample.normal_lateral_command_g,
        }
    ####
    return MissionCompositionSessionObservation(
        session_id=session_id,
        sequence=sequence,
        time_s=source.sim_time_s,
        lifecycle=lifecycle,
        values={
            "position_ned_m": primary.position_ned_m,
            "velocity_ned_mps": primary.velocity_ned_mps,
            "quaternion_wxyz": quaternion,
            "target_position_ned_m": target.position_ned_m,
            "target_velocity_ned_mps": target.velocity_ned_mps,
            "requested_control_deg": requested_control,
            "achieved_control_deg": achieved_control,
            "requested_fins_deg": requested_fins,
            "achieved_fins_deg": achieved_fins,
            "normal_command_g": controller.get("normal_lateral_command_g", (0.0, 0.0))[0],
            "lateral_command_g": controller.get("normal_lateral_command_g", (0.0, 0.0))[1],
            "achieved_lateral_normal_acceleration_g": acceleration,
            "source_controller_state": controller,
            "native_relative_state_tracks": {
                sensor_id: _engagement_native_track_value(sensor_bus, sensor_id) for sensor_id in sensor_ids
            },
        },
        events=events,
        control_authority=source_managed_session_authority_state(),
    )
    ####


####


def _engagement_native_track_value(sensor_bus: SensorBus, sensor_id: str) -> dict[str, object]:
    """Serialize one latest delivered standard SensorBus packet."""

    packet: MeasurementPacket[Any] = sensor_bus.packets(sensor_id)[-1]
    return {
        "sensor_id": packet.sensor_id,
        "sequence": packet.sequence,
        "sampled_at_s": packet.sampled_at_s,
        "available_at_s": packet.available_at_s,
        "valid": packet.valid,
        "invalid_reason": packet.invalid_reason,
        "payload": None if packet.payload is None else asdict(packet.payload),
    }
    ####


####


def _engagement_event_ids(events: tuple[Any, ...]) -> tuple[str, ...]:
    """Expose source package events through deterministic session identifiers."""

    return tuple(f"cadac-ads6-{event.kind}-{event.actor_id}-{event.time_s:.12g}" for event in events)
    ####


####


def _engagement_session_lifecycle(source: Ads6EngagementSession) -> str:
    """Map source package completion to the standard session lifecycle."""

    if source.run_result is not None and any(event.kind == "nonfinite_state" for event in source.run_result.events):
        return "failed"
    ####
    return "completed" if source.completed else "active"
    ####


####


def build_default_ads6_engagement_configuration(
    provider: CadacAds6EngagementMissionCompositionProvider,
    *,
    configuration_id: str = "ads6-engagement-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one exact installed ADS6 package configuration."""

    supplied = dict(overrides or {})
    routes = {
        "command_law": ("package", "command_law"),
        "pointing_gain": ("package", "pointing_gain"),
        "command_limit_deg": ("package", "command_limit_deg"),
        "radar_seed": ("package", "radar_seed"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown ADS6 engagement configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    values: dict[str, Any] = {
        "source_phase": ConfigurationParameterValue(value=ADS6_ENGAGEMENT_PHASE_ID),
    }
    values.update(
        {
            group_id: ConfigurationGroupValue(
                values={
                    parameter_id: ConfigurationParameterValue(
                        value=value,
                        unit=_configuration_unit(parameter_id),
                    )
                    for parameter_id, value in items.items()
                }
            )
            for group_id, items in grouped.items()
        }
    )
    schema = provider.get_model_schema(ADS6_ENGAGEMENT_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=ADS6_ENGAGEMENT_MODEL_ID,
        model_version=ADS6_ENGAGEMENT_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=provider._fidelity_id,
        realization_id=provider._realization_id,
        mission_template_id=provider._mission_id,
        root=ConfigurationGroupValue(values=values),
    )


####


def register_ads6_engagement_mission_composition(
    provider: CadacAds6EngagementMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    registry.register(CADAC_PROVIDER_ID, ADS6_ENGAGEMENT_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(
    target_kind: str,
    sam_phase: str,
    end_time_s: float,
    sample_step_s: float,
    fidelity_id: str,
) -> TrajectoryConfigurationSchema:
    return TrajectoryConfigurationSchema(
        model_id=ADS6_ENGAGEMENT_MODEL_ID,
        model_version=ADS6_ENGAGEMENT_MODEL_VERSION,
        supported_fidelities=(fidelity_id,),
        root=ConfigurationGroupSchema(
            id="ads6_engagement",
            label="ADS6 source package",
            description="Source-ordered SAM/target/RADAR0 package execution.",
            children=(
                _parameter(
                    "source_phase",
                    "Source phase",
                    "Exact installed package selector.",
                    default=ADS6_ENGAGEMENT_PHASE_ID,
                    choices=(ADS6_ENGAGEMENT_PHASE_ID,),
                    value_type="enum",
                    role="variant",
                    fidelity_id=fidelity_id,
                ),
                ConfigurationGroupSchema(
                    id="package",
                    label="Package scheduling and SAM controller",
                    children=(
                        _parameter(
                            "target_kind",
                            "Target kind",
                            "Installed source target family.",
                            default=target_kind,
                            choices=(target_kind,),
                            value_type="enum",
                            role="variant",
                            fidelity_id=fidelity_id,
                        ),
                        _parameter(
                            "sam_phase",
                            "SAM realization",
                            "Installed source SAM physical realization.",
                            default=sam_phase,
                            choices=(sam_phase,),
                            value_type="enum",
                            role="variant",
                            fidelity_id=fidelity_id,
                        ),
                        _parameter(
                            "command_law",
                            "Command law",
                            "Select the interleaved source controller or one legacy direct-command comparison seam.",
                            default="source_controller",
                            choices=("source_controller", "hold", "line_of_sight"),
                            value_type="enum",
                            role="variant",
                            fidelity_id=fidelity_id,
                        ),
                        _parameter(
                            "pointing_gain",
                            "Legacy pointing gain",
                            "Direct line-of-sight comparison gain; unused by the source-controller path.",
                            default=0.35,
                            fidelity_id=fidelity_id,
                        ),
                        _parameter(
                            "command_limit_deg",
                            "Legacy command limit",
                            "Symmetric direct comparison-command limit; unused by the source-controller path.",
                            default=20.0,
                            unit="deg",
                            role="constraint",
                            fidelity_id=fidelity_id,
                        ),
                        _parameter(
                            "radar_seed",
                            "Radar seed",
                            "Deterministic source-shaped measurement RNG seed.",
                            default=0,
                            value_type="integer",
                            role="variant",
                            fidelity_id=fidelity_id,
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s", "End time", "Package propagation horizon.", default=end_time_s, unit="s", role="constraint", fidelity_id=fidelity_id
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned multi-root trajectory cadence.",
                            default=sample_step_s,
                            unit="s",
                            role="constraint",
                            fidelity_id=fidelity_id,
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "Configuration selects only the installed source package, its fixed target/SAM realization, runtime horizon, "
            "radar seed, and controller path. It cannot substitute another CADAC actor or realization."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    fidelity_id: str,
    unit: str | None = None,
    role: str = "initialization",
    choices: tuple[str, ...] = (),
    value_type: ConfigurationValueType = "number",
) -> ConfigurationParameterSchema:
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
        choices=choices,
        role=role,
        compatible_fidelities=(fidelity_id,),
        provenance="installed ADS6 package source case",
    )


####


def _output(
    channel_id: str,
    label: str,
    description: str,
    *,
    fidelity_id: str,
    realization_id: str,
    mission_id: str,
    unit: str | None = None,
    shape: tuple[int | str, ...] = (),
    frame: str | None = None,
    data_type: str = "float64",
    availability: str = "conditional",
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
        availability=availability,
        compatible_fidelities=(fidelity_id,),
        compatible_realizations=(realization_id,),
        compatible_mission_templates=(mission_id,),
        operations=("batch", "step"),
        provenance="ADS6 source-ordered package projection",
        claim_boundary="Channel value is actor-specific; rigid-body channels are null for lower-fidelity roots.",
    )


####


def _build_output_schema(
    fidelity_id: str,
    realization_id: str,
    mission_id: str,
) -> TrajectoryOutputSchema:
    core = (
        _output(
            "position_ned_m",
            "Position",
            "Local North-East-Down position.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="m",
            shape=(3,),
            frame=_LOCAL_FRAME_ID,
            availability="guaranteed",
        ),
        _output(
            "velocity_ned_mps",
            "Velocity",
            "Local North-East-Down velocity.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="m/s",
            shape=(3,),
            frame=_LOCAL_FRAME_ID,
            availability="guaranteed",
        ),
    )
    telemetry = (
        _output(
            "quaternion_wxyz",
            "Quaternion",
            "Scalar-first body attitude for rigid-body SAM roots.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            shape=(4,),
            data_type="float64",
        ),
        _output(
            "body_rates_rad_s",
            "Body rates",
            "Body angular rates for rigid-body SAM roots.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="rad/s",
            shape=(3,),
        ),
        _output(
            "requested_control_deg",
            "Requested control",
            "Source-controller requested primary-SAM roll, pitch, and yaw control.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="deg",
            shape=(3,),
        ),
        _output(
            "achieved_control_deg",
            "Achieved control",
            "Primary-SAM actuator-achieved roll, pitch, and yaw control.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="deg",
            shape=(3,),
        ),
        _output(
            "requested_fins_deg",
            "Requested fin angles",
            "Primary-SAM requested physical fin deflections in source fin order.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="deg",
            shape=(4,),
        ),
        _output(
            "achieved_fins_deg",
            "Achieved fin angles",
            "Primary-SAM actuator-achieved physical fin deflections in source fin order.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="deg",
            shape=(4,),
        ),
        _output(
            "normal_command_g",
            "Normal command",
            "Source-controller normal acceleration command.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="g",
        ),
        _output(
            "lateral_command_g",
            "Lateral command",
            "Source-controller lateral acceleration command.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="g",
        ),
        _output(
            "achieved_lateral_acceleration_g",
            "Achieved lateral acceleration",
            "Primary-SAM achieved lateral acceleration response.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="g",
        ),
        _output(
            "achieved_normal_acceleration_g",
            "Achieved normal acceleration",
            "Primary-SAM achieved normal acceleration response.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            unit="g",
        ),
        _output(
            "actor_kind",
            "Actor kind",
            "SAM, aircraft, SRBM, or radar root classification.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="string",
        ),
        _output(
            "actor_fidelity",
            "Actor fidelity",
            "Per-root Taoryx fidelity claim.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="string",
        ),
        _output(
            "source_phase",
            "Source phase",
            "Per-root source phase or mode.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="string",
        ),
        _output(
            "held",
            "Held",
            "Actor is held at its initial state by source launch scheduling.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="boolean",
        ),
        _output(
            "alive",
            "Alive",
            "Source actor lifecycle state.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="boolean",
        ),
        _output(
            "telemetry",
            "Actor telemetry",
            "Actor-specific physical, response, or radar telemetry map.",
            fidelity_id=fidelity_id,
            realization_id=realization_id,
            mission_id=mission_id,
            data_type="json",
        ),
    )
    return TrajectoryOutputSchema(
        model_id=ADS6_ENGAGEMENT_MODEL_ID,
        model_version=ADS6_ENGAGEMENT_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="actor_state",
                label="Actor state",
                description="Per-root fidelity, phase, lifecycle, and actor-specific telemetry.",
                channel_ids=tuple(item.id for item in telemetry),
                default_selected=False,
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(
            supports_multiple_entities=True,
            supports_dynamic_spawning=False,
            supports_recursive_spawning=False,
            maximum_descendant_depth=0,
            relationship_kinds=(),
            child_output_schema_policy="same_as_parent",
            includes_spawn_initial_state=False,
            includes_lifecycle_events=True,
        ),
        claim_boundary=(
            "All source actors are independent roots. Position and velocity are guaranteed; rigid-body and actor-specific "
            "telemetry are conditional by root fidelity."
        ),
    )


####


def _build_model_metadata(
    target_kind: str,
    sam_phase: str,
    fidelity_id: str,
    realization_id: str,
    mission_id: str,
    schema: TrajectoryConfigurationSchema,
    output: TrajectoryOutputSchema,
    claim_boundary: str,
) -> TrajectoryModelMetadata:
    direct_wrench = fidelity_id == "rigid_body_6dof_direct_wrench"
    actuator_types = ("rcs",) if direct_wrench else (("thrust_vectoring",) if sam_phase == "tvc_control" else ("aerodynamic_surfaces",))
    realization = TrajectoryRealizationMetadata(
        id=realization_id,
        label=f"ADS6 package {sam_phase.replace('_', ' ')}",
        description="Source-ordered ADS6 package with persistent SAM, target, and RADAR0 actors.",
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="direct_wrench" if direct_wrench else "actuator_allocated",
        actuator_types=actuator_types,
        controls=source_managed_control_advertisement(
            mission_ids=(mission_id,),
            source_refs=("missiondesignsolutions/CADAC/ADS6",),
            claim_boundary="Source-order package control remains internal to the persistent compatibility runtime.",
        ),
        fidelity_aliases=(fidelity_id,),
        mission_template_ids=(mission_id,),
        operations=("validate", "batch", "step"),
        native_factory_ids=(ADS6_ENGAGEMENT_EXECUTOR_ID, ADS6_ENGAGEMENT_SESSION_EXECUTOR_ID),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        claim_boundary=claim_boundary,
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=mission_id,
        name=f"ADS6 {target_kind} defense",
        description="Source-order SAM/target/radar package composition.",
        status="development",
        initialization_variants=("source_package",),
        segment_sequence=("radar_track", "launch_schedule", "mixed_fidelity_propagation"),
        compatible_fidelities=(fidelity_id,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=fidelity_id,
                realization_id=realization_id,
                operation="batch",
                status="available",
                execution_mode="source_ordered_package",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_ENGAGEMENT_EXECUTOR_ID,
                claim_boundary=claim_boundary,
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=fidelity_id,
                realization_id=realization_id,
                operation="step",
                status="available",
                execution_mode="source_ordered_package",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=ADS6_ENGAGEMENT_SESSION_EXECUTOR_ID,
                claim_boundary="Persistent stepping retains the complete SAM/target/RADAR0 scheduler, source controller, packets, and launch latches.",
            ),
        ),
        provenance="ADS6 source package and Taoryx mixed-fidelity composition",
        claim_boundary=claim_boundary,
    )
    fidelity = TrajectoryFidelityMetadata(
        id=fidelity_id,
        label=f"ADS6 package SAM envelope ({fidelity_id})",
        rank=4 if not direct_wrench else 3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="direct_wrench" if direct_wrench else "actuator_allocated",
        actuator_types=actuator_types,
        runtime_fidelity="rigid_body_6dof",
        control_realization="direct_wrench" if direct_wrench else "effector_allocated",
        promotion_status="development",
        operations=("validate", "batch", "step"),
        required_operations=("source_order_scheduler", "radar_tracking", "launch_scheduling"),
        claim_boundary=(
            "Model fidelity describes the primary SAM envelope. Each returned target/radar root carries its own lower fidelity or static classification."
        ),
    )
    return TrajectoryModelMetadata(
        id=ADS6_ENGAGEMENT_MODEL_ID,
        name="ADS6 source-ordered engagement",
        version=ADS6_ENGAGEMENT_MODEL_VERSION,
        description="Mixed-fidelity ADS6 SAM, target, and RADAR0 package composition.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="ADS6 Engagement",
            short_name="ADS6",
            summary="Source-ordered multi-SAM defense composition with RADAR0 launch scheduling.",
            category="mission_composition",
            subcategory="air_defense",
            sort_key=ADS6_ENGAGEMENT_MODEL_ID,
            badges=("CADAC", "multi-root", "development"),
            default_fidelity_id=fidelity_id,
            default_mission_template_id=mission_id,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="target_kind",
                    label="Target kind",
                    description="Installed source package target actor family.",
                    semantic_role="implementation",
                    value_type="string",
                    value_kind="declared",
                    value=target_kind,
                    value_declared=True,
                    provenance="ADS6 RADAR0 mtrack and actor composition",
                    claim_boundary="Fixed by installed source case.",
                ),
            ),
        ),
        family_id="cadac.ads6",
        physical_family="cadac_ads6_air_defense",
        model_kind="mission_composition",
        status="development",
        tags=("cadac", "ads6", "multi-actor", "radar", target_kind),
        operations=("discover", "validate", "batch", "step"),
        common_runner_operations=("batch", "step"),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_package",),
            segment_types=(
                "source_actor_pass",
                "radar_track",
                "launch_schedule",
                "mixed_fidelity_propagation",
            ),
            termination_modes=("end_time", "actor_termination"),
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
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_FRAME_ID,
                name="ADS6 local North-East-Down",
                description="Shared flat-Earth local-level frame used by ADS6 package actors.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source package reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/ADS6",),
                provenance="ADS6 Flat6/Flat3/Flat0 source frames",
            ),
        ),
        output_schema=output,
        output_schema_id=output.schema_id,
        output_schema_fingerprint=output.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        source_refs=("missiondesignsolutions/CADAC/ADS6",),
        provenance="ADS6 source actor composition projected into Taoryx Mission Composition",
        claim_boundary=claim_boundary,
    )


####


def _run_config(resolved: Any) -> Ads6EngagementRunConfig:
    if not isinstance(resolved, Mapping):
        raise ValueError("ADS6 package resolved configuration must be a mapping")
    ####
    package = _group(resolved, "package")
    runtime = _group(resolved, "runtime")
    return Ads6EngagementRunConfig(
        command_law=str(package.get("command_law", "source_controller")),
        pointing_gain=float(package.get("pointing_gain", 0.35)),
        command_limit_deg=float(package.get("command_limit_deg", 20.0)),
        radar_seed=int(package.get("radar_seed", 0)),
        end_time_s=float(runtime["end_time_s"]) if "end_time_s" in runtime else None,
        sample_step_s=float(runtime["sample_step_s"]) if "sample_step_s" in runtime else None,
    )


####


def _group(resolved: Mapping[str, Any], group_id: str) -> Mapping[str, Any]:
    value = resolved.get(group_id, {})
    if not isinstance(value, Mapping):
        raise ValueError(f"ADS6 package configuration group {group_id!r} must resolve to a mapping")
    ####
    return value


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Ads6EngagementRunResult,
    output_schema: TrajectoryOutputSchema,
    realization_id: str,
    mission_id: str,
) -> MissionCompositionTrajectoryResult:
    configuration = request.prepared_configuration.configuration
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=configuration.fidelity,
        operation=request.operation,
        realization_id=realization_id,
        mission_template_id=mission_id,
    )
    objects = tuple(
        TrajectoryObject(
            object_id=item.actor_id,
            # The primary SAM root is the request's composite model; the
            # remaining independent source actors retain their own model IDs.
            model_id=(ADS6_ENGAGEMENT_MODEL_ID if item.actor_id == run.primary_actor_id else item.model_id),
            realization_id=_actor_realization(item.actor_kind, realization_id),
            name=item.source_role,
            role=item.actor_kind,
            fidelity=item.fidelity,
            status="completed" if item.samples[-1].alive else "terminated",
            # CADAC keeps every package actor alive from the initial source
            # pass; launch scheduling is represented by its held telemetry,
            # not by a later entity-spawn boundary.
            active_from_s=item.samples[0].time_s,
            active_to_s=item.samples[-1].time_s,
            terminal_disposition=None if item.samples[-1].alive else "source_terminated",
            channels=tuple(_runtime_channel(channel) for channel in _selected_actor_channels(selected, item.actor_kind)),
            samples=tuple(
                TrajectorySample(
                    time_s=sample.time_s,
                    values={channel.id: _sample_value(channel.id, sample) for channel in _selected_actor_channels(selected, item.actor_kind)},
                )
                for sample in item.samples
            ),
            provenance=f"{run.source_name}:{item.source_role}",
            claim_boundary=run.claim_boundary,
        )
        for item in run.objects
    )
    events = tuple(
        TrajectoryEvent(
            id=f"ads6-{index:06d}-{event.kind}",
            time_s=event.time_s,
            category="custom",
            kind=event.kind,
            object_id=event.actor_id,
            detail=event.kind.replace("_", " "),
            data={
                **event.details,
                **({"related_actor_id": event.related_actor_id} if event.related_actor_id is not None else {}),
            },
        )
        for index, event in enumerate(run.events)
        if request.output.include_events
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="warning",
            code="cadac-ads6-package-source-controller-boundary",
            message=(
                "ADS6 source-ordered actor propagation, RADAR0 launch scheduling, radar-uplink line guidance, "
                "deterministic RF/IR seeker acquisition and lock state, terminal proportional navigation, adaptive "
                "rate/acceleration control, and the physical SAM effector plant participate. Truth-aligned INS, RF "
                "glint/thermal-noise power modeling, complete IR focal-plane/aimpoint corruption, exact stochastic "
                "sequence parity, and bug-for-bug compiled executive parity remain outside this tranche."
            ),
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=ADS6_ENGAGEMENT_MODEL_ID,
            details={
                "packet_trace_count": len(run.packet_traces),
                "radar_track_count": len(run.radar_tracks),
                "launch_command_count": len(run.radar_launch_commands),
                "sam_controller_trace_count": len(run.sam_controller_traces),
                "sam_controller_sample_count": sum(len(trace.samples) for trace in run.sam_controller_traces),
                "sam_controller_event_count": sum(len(trace.source_events) for trace in run.sam_controller_traces),
                "source_controller_active": bool(run.sam_controller_traces),
                "ins_implementation": "truth_aligned",
                "rf_measurement_boundary": "deterministic_no_glint_or_thermal_noise",
                "ir_measurement_boundary": "deterministic_no_focal_plane_or_aimpoint_corruption",
                "launch_schedule_semantics": "documented_radar_latched_next_epoch",
            },
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=ADS6_ENGAGEMENT_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=ADS6_ENGAGEMENT_MODEL_ID,
        primary_object_id=run.primary_actor_id,
        status="completed" if run.terminated_reason == "end_time" else "terminated",
        objects=objects,
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary=run.claim_boundary,
    )


####


def _runtime_channel(channel: TrajectoryOutputChannelMetadata) -> TrajectoryChannelMetadata:
    return TrajectoryChannelMetadata(
        id=channel.id,
        channel_class="core_state" if channel.id in {"position_ned_m", "velocity_ned_mps"} else "telemetry",
        telemetry_group=None if channel.id in {"position_ned_m", "velocity_ned_mps"} else "actor_state",
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


def _sample_value(channel_id: str, sample: Ads6EngagementSample) -> object:
    values: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "quaternion_wxyz": sample.quaternion_wxyz,
        "body_rates_rad_s": sample.body_rates_rad_s,
        "requested_control_deg": sample.telemetry.get("requested_control_deg"),
        "achieved_control_deg": sample.telemetry.get("achieved_control_deg"),
        "requested_fins_deg": sample.telemetry.get("requested_fins_deg"),
        "achieved_fins_deg": sample.telemetry.get("achieved_fins_deg"),
        "normal_command_g": float(sample.telemetry.get("normal_command_g", 0.0)),
        "lateral_command_g": float(sample.telemetry.get("lateral_command_g", 0.0)),
        "achieved_lateral_acceleration_g": sample.telemetry.get("achieved_lateral_acceleration_g"),
        "achieved_normal_acceleration_g": sample.telemetry.get("achieved_normal_acceleration_g"),
        "actor_kind": sample.actor_kind,
        "actor_fidelity": sample.fidelity,
        "source_phase": sample.source_phase,
        "held": sample.held,
        "alive": sample.alive,
        "telemetry": sample.telemetry,
    }
    return values[channel_id]


####


def _selected_actor_channels(
    selected: tuple[TrajectoryOutputChannelMetadata, ...],
    actor_kind: str,
) -> tuple[TrajectoryOutputChannelMetadata, ...]:
    """Drop conditional rigid-body telemetry for lower-fidelity package roots."""

    if actor_kind == "sam":
        return selected
    ####
    sam_only_channel_ids = {
        "quaternion_wxyz",
        "body_rates_rad_s",
        "requested_control_deg",
        "achieved_control_deg",
        "requested_fins_deg",
        "achieved_fins_deg",
        "normal_command_g",
        "lateral_command_g",
        "achieved_lateral_acceleration_g",
        "achieved_normal_acceleration_g",
    }
    return tuple(channel for channel in selected if channel.id not in sam_only_channel_ids)


####


def _actor_realization(actor_kind: str, package_realization: str) -> str:
    return {
        "sam": package_realization,
        "aircraft": "cadac-ads6-aircraft-force-model",
        "srbm": "cadac-ads6-srbm-response-law",
        "radar": "cadac-ads6-static-radar",
    }[actor_kind]


####


def _sam_fidelity(phase: str) -> str:
    return "rigid_body_6dof_direct_wrench" if phase == "aggregate_rcs" else "rigid_body_6dof_surface_allocated"


####


def _configuration_unit(parameter_id: str) -> str | None:
    return {"command_limit_deg": "deg", "end_time_s": "s", "sample_step_s": "s"}.get(parameter_id)


####


__all__ = [
    "ADS6_ENGAGEMENT_EXECUTOR_ID",
    "ADS6_ENGAGEMENT_SESSION_EXECUTOR_ID",
    "ADS6_ENGAGEMENT_MODEL_ID",
    "ADS6_ENGAGEMENT_MODEL_VERSION",
    "ADS6_ENGAGEMENT_PHASE_ID",
    "CadacAds6EngagementMissionCompositionProvider",
    "build_default_ads6_engagement_configuration",
    "register_ads6_engagement_mission_composition",
]
