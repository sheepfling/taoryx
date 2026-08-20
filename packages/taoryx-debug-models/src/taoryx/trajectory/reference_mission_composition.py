"""Runnable analytical reference binding for the Mission Composition contract.

The repository's ballistic and constant-velocity waypoint fixtures predate the
typed configuration and multi-object execution envelopes.  This module binds
those deliberately simple native 3-DOF models to the current public contract.
It is an interface witness, not a historical TAOS or qualified vehicle claim.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING, Any

from pydantic import BaseModel, ConfigDict, Field, model_validator

from taoryx.composition_episode import EpisodeStatus

from ..fixture_composition_episode import FixtureCompositionEpisode, FixtureTransition
from .analytical_mission_composition import (
    ExampleMissionCompositionProvider,
    MissionCompositionOutputRequest,
    MissionCompositionParameter,
    MissionCompositionParameterValue,
    MissionCompositionSegmentRequest,
    MissionCompositionTrajectory,
    MissionCompositionTrajectoryRequest,
    MissionCompositionVehicle,
    analytical_initial_state,
    propagate_ballistic_state,
    propagate_constant_velocity_waypoint_state,
)
from .configuration_contract import (
    ConfigurationChoiceValue,
    ConfigurationGroupValue,
    ConfigurationParameterValue,
    ConfigurationSequenceValue,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryModelMetadata,
    TrajectoryOutputSchema,
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
)
from .execution_contract import (
    DiagnosticPhase,
    FailureCategory,
    MissionCompositionDiagnostic,
    MissionCompositionExecutionError,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    TrajectorySegmentResult,
    resolve_output_selection,
)
from .session_interface import build_session_interface_contract

if TYPE_CHECKING:
    from taoryx.plugins.discovery import PluginCatalog

_PROVIDER_ID = "taoryx.reference.mission-composition"
_DEFAULT_PROVIDER_VERSION = "0.1.0a0"

_BALLISTIC_MODEL_ID = "reference_ballistic_3dof"
_WAYPOINT_MODEL_ID = "reference_constant_velocity_waypoint_3dof"


class ReferenceBallisticLaunch(BaseModel):
    """Friendly SI launch state for the analytical ballistic fixture.

    ``heading_deg`` accepts any finite bearing and is stored in the canonical
    interval ``[0, 360)``.  This keeps the convenience API friendlier than the
    underlying portable configuration grammar while still authoring a
    canonical configuration tree.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    north_m: float = 0.0
    east_m: float = 0.0
    altitude_m: float = Field(ge=0.0)
    speed_m_s: float = Field(gt=0.001)
    heading_deg: float
    flight_path_angle_deg: float = Field(default=0.0, ge=-89.0, le=89.0)

    @model_validator(mode="after")
    def validate_launch(self) -> ReferenceBallisticLaunch:
        _validate_finite_state_values(
            north_m=self.north_m,
            east_m=self.east_m,
            altitude_m=self.altitude_m,
            speed_m_s=self.speed_m_s,
            heading_deg=self.heading_deg,
            flight_path_angle_deg=self.flight_path_angle_deg,
        )
        object.__setattr__(self, "heading_deg", self.heading_deg % 360.0)
        return self
        ####

    ####


class ReferenceWaypointCourseStart(BaseModel):
    """Friendly SI initial state for the analytical waypoint-course fixture."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    north_m: float = 0.0
    east_m: float = 0.0
    altitude_m: float = Field(ge=0.0)
    speed_m_s: float = Field(gt=0.001)
    heading_deg: float

    @model_validator(mode="after")
    def validate_initial_state(self) -> ReferenceWaypointCourseStart:
        _validate_finite_state_values(
            north_m=self.north_m,
            east_m=self.east_m,
            altitude_m=self.altitude_m,
            speed_m_s=self.speed_m_s,
            heading_deg=self.heading_deg,
        )
        object.__setattr__(self, "heading_deg", self.heading_deg % 360.0)
        return self
        ####

    ####


class ReferenceWaypoint(BaseModel):
    """One route waypoint for the analytical constant-velocity fixture.

    An omitted ``duration_s`` asks the builder to supply the direct
    point-to-point travel time at the course's configured cruise speed.  The
    result remains a normal portable sequence of ``waypoint_leg`` nodes.
    """

    model_config = ConfigDict(frozen=True, extra="forbid")

    north_m: float
    east_m: float
    altitude_m: float = Field(ge=0.0)
    arrival_tolerance_m: float = Field(default=25.0, ge=0.1)
    duration_s: float | None = Field(default=None, gt=0.0)
    instance_id: str | None = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def validate_waypoint(self) -> ReferenceWaypoint:
        for name, value in {
            "north_m": self.north_m,
            "east_m": self.east_m,
            "altitude_m": self.altitude_m,
            "arrival_tolerance_m": self.arrival_tolerance_m,
        }.items():
            _require_finite(value, name)
        if self.duration_s is not None:
            _require_finite(self.duration_s, "duration_s")
        if self.instance_id is not None and not self.instance_id.strip():
            raise ValueError("instance_id must contain non-whitespace text")
        return self
        ####

    ####


def _require_finite(value: float, name: str) -> None:
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    ####


def _validate_finite_state_values(**values: float) -> None:
    for name, value in values.items():
        _require_finite(value, name)
    ####


class ReferenceMissionCompositionProvider:
    """Two-model reference provider with a registered common-runner binding."""

    def __init__(
        self,
        *,
        provider_version: str = _DEFAULT_PROVIDER_VERSION,
        plugin_catalog: PluginCatalog | None = None,
    ) -> None:
        if not provider_version.strip():
            raise ValueError("analytical reference provider version must not be empty")
        self._analytical = ExampleMissionCompositionProvider()
        self._models = self._analytical.list_models()
        self._plugin_catalog = plugin_catalog
        self._metadata = TrajectoryProviderMetadata(
            id=_PROVIDER_ID,
            name="TAORYX Analytical Mission Composition Reference",
            version=provider_version,
            description=(
                "Runnable native point-mass examples for the common ballistic and "
                "constant-velocity waypoint Mission Composition workflows."
            ),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="TAORYX Analytical Mission Composition Reference",
                short_name="Analytical Reference",
                summary="Two deterministic 3-DOF models for learning and exercising the common provider contract.",
                organization="TAORYX",
                categories=("Reference", "Development"),
            ),
            status="runnable_reference",
            tags=("mission-composition", "analytical-reference", "native-3dof"),
            execution_contract="taoryx.reference-analytical-execution/v1",
            model_count=len(self._models),
            provenance="repository analytical interface fixtures",
            claim_boundary=(
                "Interface and lifecycle witness only. These analytical fixtures are not source-grounded "
                "vehicle models, historical runtime compatibility, or qualification evidence."
            ),
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return the current provider-independent publication."""

        return self._metadata
        ####

    @property
    def plugin_catalog(self) -> PluginCatalog | None:
        """Expose the selected plug-in scope retained by the focused provider."""

        return self._plugin_catalog
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Return both runnable analytical model advertisements."""

        return self._models
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return one portable configuration tree."""

        return self._analytical.get_model_schema(model_id)
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return one portable core-state and telemetry schema."""

        return self._analytical.get_model_output_schema(model_id)
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate and fingerprint one portable configuration instance."""

        return self._analytical.validate_configuration(configuration)
        ####

    def build_model_default_configuration(
        self,
        model_id: str,
        *,
        configuration_id: str,
    ) -> TrajectoryConfigurationInstance:
        """Return a small deterministic example for either analytical fixture.

        The values are onboarding inputs for exercising the interface.  They
        do not represent a physical launch condition or a performance claim.
        """

        if model_id == _BALLISTIC_MODEL_ID:
            return self.build_ballistic_configuration(
                ReferenceBallisticLaunch(
                    altitude_m=1_000.0,
                    speed_m_s=150.0,
                    heading_deg=45.0,
                    flight_path_angle_deg=10.0,
                ),
                coast_durations_s=15.0,
                configuration_id=configuration_id,
            )
        if model_id == _WAYPOINT_MODEL_ID:
            return self.build_waypoint_course_configuration(
                ReferenceWaypointCourseStart(
                    altitude_m=1_000.0,
                    speed_m_s=120.0,
                    heading_deg=0.0,
                ),
                (
                    ReferenceWaypoint(
                        north_m=3_000.0,
                        east_m=1_000.0,
                        altitude_m=1_200.0,
                    ),
                ),
                configuration_id=configuration_id,
            )
        raise KeyError(f"unknown analytical reference model {model_id!r}")
        ####

    def open_session_episode(
        self,
        prepared: PreparedTrajectoryConfiguration,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> FixtureCompositionEpisode:
        """Open a stateful analytical episode using the batch transition itself."""

        del integration_step_s
        model = next((item for item in self._models if item.id == prepared.configuration.model_id), None)
        if model is None:
            raise KeyError(f"unknown analytical reference model {prepared.configuration.model_id!r}")
        return _open_reference_session_episode(model, prepared, seed=seed)
        ####

    def build_ballistic_configuration(
        self,
        launch: ReferenceBallisticLaunch,
        coast_durations_s: float | Sequence[float],
        *,
        configuration_id: str = "reference-ballistic",
    ) -> TrajectoryConfigurationInstance:
        """Build a canonical, repeatable ballistic-coast composition.

        All inputs use SI units.  ``coast_durations_s`` accepts one duration or
        an ordered sequence, so the common single- and multi-coast cases do
        not require callers to construct nested ``Group``/``Choice`` values.
        """

        durations = _positive_durations(coast_durations_s, field="coast_durations_s")
        initialization = ConfigurationChoiceValue(
            selected="launch_state",
            value=ConfigurationGroupValue(
                values={
                    "north_m": ConfigurationParameterValue(value=launch.north_m, unit="m"),
                    "east_m": ConfigurationParameterValue(value=launch.east_m, unit="m"),
                    "altitude_m": ConfigurationParameterValue(value=launch.altitude_m, unit="m"),
                    "speed_m_s": ConfigurationParameterValue(value=launch.speed_m_s, unit="m/s"),
                    "heading_deg": ConfigurationParameterValue(value=launch.heading_deg, unit="deg"),
                    "flight_path_angle_deg": ConfigurationParameterValue(value=launch.flight_path_angle_deg, unit="deg"),
                }
            ),
        )
        segments = ConfigurationSequenceValue(
            items=tuple(
                ConfigurationChoiceValue(
                    selected="ballistic_coast",
                    instance_id=f"coast-{index:02d}",
                    value=ConfigurationGroupValue(
                        values={"duration_s": ConfigurationParameterValue(value=duration_s, unit="s")}
                    ),
                )
                for index, duration_s in enumerate(durations, start=1)
            )
        )
        return self._configuration(
            model_id=_BALLISTIC_MODEL_ID,
            configuration_id=configuration_id,
            initialization=initialization,
            segments=segments,
        )
        ####

    def prepare_ballistic(
        self,
        launch: ReferenceBallisticLaunch,
        coast_durations_s: float | Sequence[float],
        *,
        configuration_id: str = "reference-ballistic",
    ) -> PreparedTrajectoryConfiguration:
        """Build and validate a friendly ballistic composition in one call."""

        return self.validate_configuration(
            self.build_ballistic_configuration(
                launch,
                coast_durations_s,
                configuration_id=configuration_id,
            )
        )
        ####

    def build_waypoint_course_configuration(
        self,
        initial: ReferenceWaypointCourseStart,
        waypoints: Sequence[ReferenceWaypoint],
        *,
        configuration_id: str = "reference-waypoint-course",
    ) -> TrajectoryConfigurationInstance:
        """Build a straight-line, constant-cruise-speed waypoint course.

        Omitted leg durations are derived from the current planned position and
        the configured cruise speed.  An explicitly timed short leg therefore
        changes the starting point used to derive the following automatic leg.
        This is a deterministic fixture convenience, not a turn-performance or
        guidance-performance prediction.
        """

        if not isinstance(waypoints, Sequence) or isinstance(waypoints, str | bytes):
            raise TypeError("waypoints must be an ordered sequence of ReferenceWaypoint values")
        if not waypoints:
            raise ValueError("waypoints must contain at least one waypoint")
        initial_values = ConfigurationChoiceValue(
            selected="initial_state",
            value=ConfigurationGroupValue(
                values={
                    "north_m": ConfigurationParameterValue(value=initial.north_m, unit="m"),
                    "east_m": ConfigurationParameterValue(value=initial.east_m, unit="m"),
                    "altitude_m": ConfigurationParameterValue(value=initial.altitude_m, unit="m"),
                    "speed_m_s": ConfigurationParameterValue(value=initial.speed_m_s, unit="m/s"),
                    "heading_deg": ConfigurationParameterValue(value=initial.heading_deg, unit="deg"),
                }
            ),
        )
        planned_position = (initial.north_m, initial.east_m, initial.altitude_m)
        segment_items: list[ConfigurationChoiceValue] = []
        for index, waypoint in enumerate(waypoints, start=1):
            if not isinstance(waypoint, ReferenceWaypoint):
                raise TypeError("waypoints must contain only ReferenceWaypoint values")
            target_position = (waypoint.north_m, waypoint.east_m, waypoint.altitude_m)
            duration_s = waypoint.duration_s
            if duration_s is None:
                duration_s = _direct_leg_duration(planned_position, target_position, initial.speed_m_s)
            segment_items.append(
                ConfigurationChoiceValue(
                    selected="waypoint_leg",
                    instance_id=waypoint.instance_id or f"waypoint-{index:02d}",
                    value=ConfigurationGroupValue(
                        values={
                            "duration_s": ConfigurationParameterValue(value=duration_s, unit="s"),
                            "waypoint_north_m": ConfigurationParameterValue(value=waypoint.north_m, unit="m"),
                            "waypoint_east_m": ConfigurationParameterValue(value=waypoint.east_m, unit="m"),
                            "waypoint_altitude_m": ConfigurationParameterValue(value=waypoint.altitude_m, unit="m"),
                            "arrival_tolerance_m": ConfigurationParameterValue(value=waypoint.arrival_tolerance_m, unit="m"),
                        }
                    ),
                )
            )
            planned_position = _planned_position_after_leg(
                planned_position,
                target_position,
                initial.speed_m_s,
                duration_s,
            )
        return self._configuration(
            model_id=_WAYPOINT_MODEL_ID,
            configuration_id=configuration_id,
            initialization=initial_values,
            segments=ConfigurationSequenceValue(items=tuple(segment_items)),
        )
        ####

    def prepare_waypoint_course(
        self,
        initial: ReferenceWaypointCourseStart,
        waypoints: Sequence[ReferenceWaypoint],
        *,
        configuration_id: str = "reference-waypoint-course",
    ) -> PreparedTrajectoryConfiguration:
        """Build and validate a friendly waypoint-course composition in one call."""

        return self.validate_configuration(
            self.build_waypoint_course_configuration(
                initial,
                waypoints,
                configuration_id=configuration_id,
            )
        )
        ####

    def _configuration(
        self,
        *,
        model_id: str,
        configuration_id: str,
        initialization: ConfigurationChoiceValue,
        segments: ConfigurationSequenceValue,
    ) -> TrajectoryConfigurationInstance:
        schema = self.get_model_schema(model_id)
        return TrajectoryConfigurationInstance(
            configuration_id=configuration_id,
            model_id=model_id,
            model_version=schema.model_version,
            schema_fingerprint=schema.fingerprint,
            fidelity="point_mass_3dof",
            realization_id="analytical_point_mass",
            mission_template_id=f"{model_id}_repeatable_sequence_v1",
            root=ConfigurationGroupValue(values={"initialization": initialization, "segments": segments}),
        )
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Return a common runner with both advertised executors registered."""

        return MissionCompositionRunnerRegistry(
            {
                (self.metadata.id, model.id): self._execute
                for model in self.list_models()
            }
        )
        ####

    def _execute(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.operation != "batch":
            raise _execution_error(
                "operation-not-supported",
                "The analytical reference provider advertises batch execution only.",
                phase="preflight",
                path="/operation",
                provider_id=request.provider_id,
                model_id=request.model_id,
                category="unsupported",
            )
        if request.provider_id != self.metadata.id or request.provider_version != self.metadata.version:
            raise _execution_error(
                "provider-identity-mismatch",
                "The request names another provider or provider version.",
                phase="preflight",
                path="/provider_id",
                provider_id=request.provider_id,
                model_id=request.model_id,
                category="invalid_request",
            )
        analytical_request = self._analytical_request(request)
        analytical_result = self._analytical.run(self._analytical.prepare(analytical_request))
        return self._standard_result(request, analytical_result)
        ####

    def _analytical_request(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryRequest:
        resolved = _mapping(request.prepared_configuration.resolved, path="/prepared_configuration/resolved")
        initialization = _choice(resolved.get("initialization"), path="/prepared_configuration/resolved/initialization")
        segment_values = _sequence(resolved.get("segments"), path="/prepared_configuration/resolved/segments")
        vehicle = self._analytical.metadata.vehicle(request.model_id)
        initialization_descriptor = vehicle.initialization(initialization[0])
        initialization_parameters = _parameter_values(
            initialization_descriptor.parameters,
            initialization[1],
            path="/prepared_configuration/resolved/initialization/value",
        )
        segments: list[MissionCompositionSegmentRequest] = []
        for index, item in enumerate(segment_values, start=1):
            segment_id, values = _choice(item, path=f"/prepared_configuration/resolved/segments/{index - 1}")
            descriptor = vehicle.segment(segment_id)
            instance_id = item.get("instance_id") if isinstance(item, Mapping) else None
            segments.append(
                MissionCompositionSegmentRequest(
                    id=segment_id,
                    instance_id=instance_id if isinstance(instance_id, str) else f"{index:02d}-{segment_id}",
                    parameters=_parameter_values(
                        descriptor.parameters,
                        values,
                        path=f"/prepared_configuration/resolved/segments/{index - 1}/value",
                    ),
                )
            )
        maximum_samples = request.output.maximum_samples_per_object
        if maximum_samples == 1:
            raise _execution_error(
                "output-selection-not-supported",
                "The analytical reference executor cannot return only the initial sample.",
                phase="preflight",
                path="/output/maximum_samples_per_object",
                provider_id=request.provider_id,
                model_id=request.model_id,
                category="unsupported",
            )
        try:
            selected_output = resolve_output_selection(
                self.get_model_output_schema(request.model_id),
                request.output,
                fidelity=request.prepared_configuration.configuration.fidelity,
                operation="batch",
            )
        except ValueError as error:
            raise _execution_error(
                "invalid-output-selection",
                str(error),
                phase="preflight",
                path="/output",
                provider_id=request.provider_id,
                model_id=request.model_id,
                category="unsupported",
            ) from error
        return MissionCompositionTrajectoryRequest(
            request_id=request.request_id,
            vehicle_id=request.model_id,
            fidelity=request.prepared_configuration.configuration.fidelity,
            initialization_id=initialization[0],
            initialization=initialization_parameters,
            segments=tuple(segments),
            output=MissionCompositionOutputRequest(
                cadence_s=request.output.cadence_s or 0.1,
                channels=tuple(item.id for item in selected_output),
                include_events=request.output.include_events,
                max_samples=maximum_samples,
            ),
        )
        ####

    def _standard_result(
        self,
        request: MissionCompositionRunRequest,
        result: MissionCompositionTrajectory,
    ) -> MissionCompositionTrajectoryResult:
        vehicle = self._analytical.metadata.vehicle(result.vehicle_id)
        primary_object_id = f"{request.request_id}:primary"
        channels = _channel_metadata(vehicle, result, self.get_model_output_schema(request.model_id))
        events = _events(primary_object_id, result)
        event_ids_by_segment_kind = {
            (item.segment_instance_id, item.kind): item.id
            for item in events
            if item.segment_instance_id is not None
        }
        segments = tuple(
            TrajectorySegmentResult(
                id=item.id,
                instance_id=item.instance_id,
                object_id=primary_object_id,
                start_time_s=item.start_time_s,
                end_time_s=item.end_time_s,
                status=item.status,
                event_ids=tuple(
                    event_ids_by_segment_kind[(item.instance_id, kind)]
                    for kind in item.events
                    if (item.instance_id, kind) in event_ids_by_segment_kind
                ),
            )
            for item in result.segments
        ) if request.output.include_segments else ()
        diagnostics = tuple(
            MissionCompositionDiagnostic(
                severity="info",
                code="provider-note",
                message=message,
                phase="projection",
                recoverability="degraded",
                provider_id=request.provider_id,
                model_id=request.model_id,
                object_id=primary_object_id,
            )
            for message in result.diagnostics
        )
        return MissionCompositionTrajectoryResult(
            provider_id=request.provider_id,
            provider_version=request.provider_version,
            request_id=request.request_id,
            configuration_fingerprint=request.prepared_configuration.fingerprint,
            primary_model_id=request.model_id,
            primary_object_id=primary_object_id,
            status=result.status,
            objects=(
                TrajectoryObject(
                    object_id=primary_object_id,
                    model_id=result.vehicle_id,
                    realization_id="analytical_point_mass",
                    name=vehicle.display_name,
                    role="primary_vehicle",
                    fidelity=result.fidelity,
                    status=result.status,
                    active_from_s=result.samples[0].time_s,
                    active_to_s=result.samples[-1].time_s,
                    terminal_disposition=result.status,
                    channels=channels,
                    samples=tuple(
                        TrajectorySample(
                            time_s=item.time_s,
                            values=item.values,
                            segment_instance_id=item.segment_instance_id,
                        )
                        for item in result.samples
                    ),
                    segments=segments,
                    provenance=vehicle.provenance,
                    claim_boundary=result.claim_boundary,
                ),
            ),
            events=events,
            diagnostics=diagnostics,
            claim_boundary=result.claim_boundary,
        )
        ####

    ####


def _open_reference_session_episode(
    model: TrajectoryModelMetadata,
    prepared: PreparedTrajectoryConfiguration,
    *,
    seed: int | None,
) -> FixtureCompositionEpisode:
    """Create the ballistic or waypoint lifecycle around shared transitions."""

    resolved = _mapping(prepared.resolved, path="/prepared_configuration/resolved")
    _, initialization = _choice(
        resolved.get("initialization"),
        path="/prepared_configuration/resolved/initialization",
    )
    segment_payloads = _sequence(
        resolved.get("segments"),
        path="/prepared_configuration/resolved/segments",
    )
    segments: tuple[dict[str, Any], ...] = tuple(
        {
            **dict(_choice(item, path=f"/prepared_configuration/resolved/segments/{index}")[1]),
            "segment_id": _choice(item, path=f"/prepared_configuration/resolved/segments/{index}")[0],
            "instance_id": (
                item.get("instance_id", f"segment-{index + 1:02d}")
                if isinstance(item, Mapping)
                else f"segment-{index + 1:02d}"
            ),
        }
        for index, item in enumerate(segment_payloads)
    )
    realization_id = prepared.configuration.realization_id or "analytical_point_mass"
    contract, observation_schema = build_session_interface_contract(
        model,
        realization_id=realization_id,
        fidelity=prepared.configuration.fidelity,
        family_id="analytical_reference",
        physical_family="analytical_fixture",
        claim_boundary=(
            "Stateful analytical reference fixture. It reuses the batch transition and makes no "
            "historical-runtime, controller, actuator, atmosphere, or qualification claim."
        ),
    )

    def initial_state_factory(_: int | None) -> Mapping[str, object]:
        state: dict[str, Any] = dict(analytical_initial_state(initialization))
        state["configured_segments"] = [dict(item) for item in segments]
        state["configured_segment_index"] = 0
        state["configured_segment_elapsed_s"] = 0.0
        state["held_kinematic"] = {}
        state["held_live_waypoint"] = {}
        return state
        ####

    transition = _ballistic_session_transition if model.id == _BALLISTIC_MODEL_ID else _waypoint_session_transition
    return FixtureCompositionEpisode(
        interface_contract=contract,
        observation_schema=observation_schema,
        initial_state_factory=initial_state_factory,
        observation_factory=_analytical_session_observation,
        transition=transition,
        authority_selection_hook=_analytical_authority_handoff,
        claim_boundary=contract.claim_boundary,
        seed=seed,
    )
    ####


def _numeric_analytical_state(state: Mapping[str, Any]) -> dict[str, float]:
    return {
        identifier: float(state[identifier])
        for identifier in (
            "time_s",
            "north_m",
            "east_m",
            "altitude_m",
            "speed_m_s",
            "cruise_speed_m_s",
            "heading_deg",
            "flight_path_angle_deg",
            "load_factor_g",
        )
    }
    ####


def _merge_numeric_state(state: Mapping[str, Any], numeric: Mapping[str, float]) -> dict[str, object]:
    merged = dict(state)
    merged.update(numeric)
    return merged
    ####


def _ballistic_session_transition(
    state: Mapping[str, Any],
    profile_id: str,
    action: Mapping[str, Any],
    duration_s: float,
    _: float,
) -> FixtureTransition:
    if profile_id != "open_loop_coast" or action:
        raise ValueError("ballistic sessions accept only the zero-action open_loop_coast profile")
    numeric = _numeric_analytical_state(state)
    propagate_ballistic_state(numeric, duration_s)
    events: tuple[str, ...] = ()
    status: EpisodeStatus = "active"
    diagnostics: tuple[str, ...] = ()
    if numeric["altitude_m"] <= 0.0:
        numeric["altitude_m"] = 0.0
        events = ("impact",)
        diagnostics = ("analytical ballistic fixture reached the local altitude floor",)
        status = "completed"
    return FixtureTransition(
        state=_merge_numeric_state(state, numeric),
        applied_action={},
        applied_semantic_action={},
        lowering_evidence={
            "authority_profile_id": profile_id,
            "semantic_action": {},
            "native_transition": "propagate_ballistic_state",
            "gravity_m_s2": 9.80665,
        },
        events=events,
        diagnostics=diagnostics,
        status=status,
    )
    ####


def _waypoint_session_transition(
    state: Mapping[str, Any],
    profile_id: str,
    action: Mapping[str, Any],
    duration_s: float,
    _: float,
) -> FixtureTransition:
    next_state = dict(state)
    numeric = _numeric_analytical_state(state)
    events: list[str] = []
    applied_semantic: dict[str, object]
    native: dict[str, object]

    if profile_id == "configured_waypoint_guidance":
        if action:
            raise ValueError("configured waypoint guidance is provider-owned and accepts no caller action")
        segments = next_state.get("configured_segments")
        if not isinstance(segments, list) or not segments:
            raise ValueError("configured waypoint session has no prepared segments")
        remaining = duration_s
        index = int(next_state.get("configured_segment_index", 0))
        elapsed = float(next_state.get("configured_segment_elapsed_s", 0.0))
        while remaining > 1.0e-12:
            if index >= len(segments):
                numeric["speed_m_s"] = 0.0
                break
            target = segments[index]
            if not isinstance(target, dict):
                raise ValueError("configured waypoint segment is malformed")
            segment_duration = float(target["duration_s"])
            step_s = min(remaining, max(segment_duration - elapsed, 0.0))
            if step_s <= 1.0e-12:
                events.append(f"segment_completed:{target.get('instance_id', index)}")
                index += 1
                elapsed = 0.0
                if index < len(segments):
                    numeric["speed_m_s"] = numeric["cruise_speed_m_s"]
                continue
            propagate_constant_velocity_waypoint_state(numeric, target, step_s)
            remaining -= step_s
            elapsed += step_s
            if _waypoint_distance(numeric, target) <= float(target.get("arrival_tolerance_m", 25.0)):
                marker = f"waypoint_captured:{target.get('instance_id', index)}"
                if marker not in events:
                    events.append(marker)
            if elapsed >= segment_duration - 1.0e-12:
                events.append(f"segment_completed:{target.get('instance_id', index)}")
                index += 1
                elapsed = 0.0
                if index < len(segments):
                    numeric["speed_m_s"] = numeric["cruise_speed_m_s"]
        next_state["configured_segment_index"] = index
        next_state["configured_segment_elapsed_s"] = elapsed
        applied_semantic = {}
        native = {
            "configured_segment_index": index,
            "configured_target": None if index >= len(segments) else segments[index],
        }
        lowering = ("configured_waypoint_sequence", "propagate_constant_velocity_waypoint_state")
    elif profile_id == "kinematic_velocity_command":
        held = dict(next_state.get("held_kinematic", {}))
        held.update(action)
        held.setdefault("guidance.speed.command", numeric["speed_m_s"])
        held.setdefault("guidance.heading.command", numeric["heading_deg"])
        held.setdefault("guidance.flight_path_angle.command", numeric["flight_path_angle_deg"])
        speed = float(held["guidance.speed.command"])
        heading_deg = float(held["guidance.heading.command"]) % 360.0
        path_deg = float(held["guidance.flight_path_angle.command"])
        horizontal = speed * math.cos(math.radians(path_deg))
        target = {
            "waypoint_north_m": numeric["north_m"] + horizontal * math.cos(math.radians(heading_deg)) * duration_s,
            "waypoint_east_m": numeric["east_m"] + horizontal * math.sin(math.radians(heading_deg)) * duration_s,
            "waypoint_altitude_m": max(0.0, numeric["altitude_m"] + speed * math.sin(math.radians(path_deg)) * duration_s),
        }
        numeric["speed_m_s"] = speed
        propagate_constant_velocity_waypoint_state(numeric, target, duration_s)
        # The shared waypoint propagator stops at a captured target. This
        # synthetic target is exactly one hold interval away, so restore the
        # commanded kinematic state for the next boundary instead of
        # misreporting every accepted velocity command as a stop.
        numeric["speed_m_s"] = speed
        numeric["heading_deg"] = heading_deg
        numeric["flight_path_angle_deg"] = path_deg
        next_state["held_kinematic"] = held
        applied_semantic = held
        native = {"kinematic_target": target, "speed_m_s": speed}
        lowering = ("kinematic_command_hold", "propagate_constant_velocity_waypoint_state")
    elif profile_id == "live_waypoint_guidance":
        held = dict(next_state.get("held_live_waypoint", {}))
        held.update(action)
        required = {
            "navigation.waypoint.north.command",
            "navigation.waypoint.east.command",
            "navigation.waypoint.altitude.command",
        }
        missing = sorted(required - set(held))
        if missing:
            raise ValueError(f"first live-waypoint action must supply {missing!r}")
        held.setdefault("navigation.waypoint.capture_radius.command", 25.0)
        held.setdefault("navigation.waypoint.speed.command", numeric["cruise_speed_m_s"])
        target = {
            "waypoint_north_m": float(held["navigation.waypoint.north.command"]),
            "waypoint_east_m": float(held["navigation.waypoint.east.command"]),
            "waypoint_altitude_m": float(held["navigation.waypoint.altitude.command"]),
        }
        numeric["speed_m_s"] = float(held["navigation.waypoint.speed.command"])
        propagate_constant_velocity_waypoint_state(numeric, target, duration_s)
        if _waypoint_distance(numeric, target) <= float(held["navigation.waypoint.capture_radius.command"]):
            events.append("live_waypoint_captured")
        next_state["held_live_waypoint"] = held
        applied_semantic = held
        native = {**target, "speed_m_s": float(held["navigation.waypoint.speed.command"])}
        lowering = ("live_waypoint_hold", "propagate_constant_velocity_waypoint_state")
    else:
        raise ValueError(f"unsupported analytical waypoint authority {profile_id!r}")

    next_state.update(numeric)
    return FixtureTransition(
        state=next_state,
        applied_action=native,
        applied_semantic_action=applied_semantic,
        lowering_evidence={
            "authority_profile_id": profile_id,
            "lowering_chain": list(lowering),
            "native_transition": "propagate_constant_velocity_waypoint_state",
            "native_action": native,
        },
        events=tuple(events),
        status="active",
    )
    ####


def _waypoint_distance(state: Mapping[str, float], target: Mapping[str, Any]) -> float:
    return math.sqrt(
        (float(target["waypoint_north_m"]) - state["north_m"]) ** 2
        + (float(target["waypoint_east_m"]) - state["east_m"]) ** 2
        + (float(target["waypoint_altitude_m"]) - state["altitude_m"]) ** 2
    )
    ####


def _analytical_authority_handoff(
    state: Mapping[str, Any],
    _: str,
    selected: str,
) -> Mapping[str, object]:
    """Seed newly selected holds from the accepted state for bumpless transfer."""

    next_state = dict(state)
    if selected == "kinematic_velocity_command":
        next_state["held_kinematic"] = {
            "guidance.speed.command": float(state["speed_m_s"]),
            "guidance.heading.command": float(state["heading_deg"]),
            "guidance.flight_path_angle.command": float(state["flight_path_angle_deg"]),
        }
    return next_state
    ####


def _analytical_session_observation(
    state: Mapping[str, Any],
    _: float,
    __: str,
) -> Mapping[str, object]:
    numeric = _numeric_analytical_state(state)
    heading = math.radians(numeric["heading_deg"])
    path = math.radians(numeric["flight_path_angle_deg"])
    horizontal = numeric["speed_m_s"] * math.cos(path)
    return {
        "position.north_m": numeric["north_m"],
        "position.east_m": numeric["east_m"],
        "position.altitude_m": numeric["altitude_m"],
        "velocity.north_m_s": horizontal * math.cos(heading),
        "velocity.east_m_s": horizontal * math.sin(heading),
        "velocity.vertical_m_s": numeric["speed_m_s"] * math.sin(path),
        "velocity.speed_m_s": numeric["speed_m_s"],
        "attitude.heading_deg": numeric["heading_deg"] % 360.0,
        "attitude.flight_path_angle_deg": numeric["flight_path_angle_deg"],
        "maneuver.load_factor_g": numeric["load_factor_g"],
    }
    ####


def _positive_durations(value: float | Sequence[float], *, field: str) -> tuple[float, ...]:
    if isinstance(value, bool):
        raise TypeError(f"{field} must be a positive duration or sequence of positive durations")
    durations: tuple[float, ...]
    if isinstance(value, int | float):
        durations = (float(value),)
    elif isinstance(value, Sequence) and not isinstance(value, str | bytes):
        durations = tuple(float(item) for item in value)
    else:
        raise TypeError(f"{field} must be a positive duration or sequence of positive durations")
    if not durations:
        raise ValueError(f"{field} must contain at least one duration")
    for duration_s in durations:
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise ValueError(f"{field} must contain only finite positive durations")
    return durations
    ####


def _direct_leg_duration(
    origin: tuple[float, float, float],
    target: tuple[float, float, float],
    speed_m_s: float,
) -> float:
    return max(math.dist(origin, target) / speed_m_s, 0.001)
    ####


def _planned_position_after_leg(
    origin: tuple[float, float, float],
    target: tuple[float, float, float],
    speed_m_s: float,
    duration_s: float,
) -> tuple[float, float, float]:
    distance = math.dist(origin, target)
    if distance <= 1.0e-12:
        return target
    fraction = min(speed_m_s * duration_s / distance, 1.0)
    return (
        origin[0] + (target[0] - origin[0]) * fraction,
        origin[1] + (target[1] - origin[1]) * fraction,
        origin[2] + (target[2] - origin[2]) * fraction,
    )
    ####


def _mapping(value: object, *, path: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise _execution_error(
            "invalid-prepared-configuration",
            "Expected an object at this configuration path.",
            phase="configuration",
            path=path,
            category="invalid_request",
        )
    return value
    ####


def _choice(value: object, *, path: str) -> tuple[str, Mapping[str, Any]]:
    payload = _mapping(value, path=path)
    selected = payload.get("selected")
    if not isinstance(selected, str) or not selected:
        raise _execution_error(
            "invalid-prepared-choice",
            "Expected a non-empty selected variant.",
            phase="configuration",
            path=f"{path}/selected",
            category="invalid_request",
        )
    return selected, _mapping(payload.get("value"), path=f"{path}/value")
    ####


def _sequence(value: object, *, path: str) -> Sequence[object]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise _execution_error(
            "invalid-prepared-sequence",
            "Expected an ordered configuration sequence.",
            phase="configuration",
            path=path,
            category="invalid_request",
        )
    return value
    ####


def _parameter_values(
    descriptors: Sequence[MissionCompositionParameter],
    values: Mapping[str, Any],
    *,
    path: str,
) -> dict[str, MissionCompositionParameterValue]:
    descriptor_by_id = {item.id: item for item in descriptors}
    unknown = sorted(set(values) - set(descriptor_by_id))
    if unknown:
        raise _execution_error(
            "invalid-prepared-parameter",
            f"Prepared values contain parameters absent from the model schema: {unknown!r}.",
            phase="configuration",
            path=path,
            category="invalid_request",
        )
    return {
        key: MissionCompositionParameterValue(value=value, unit=descriptor_by_id[key].canonical_unit)
        for key, value in values.items()
    }
    ####


def _channel_metadata(
    vehicle: MissionCompositionVehicle,
    result: MissionCompositionTrajectory,
    output_schema: TrajectoryOutputSchema,
) -> tuple[TrajectoryChannelMetadata, ...]:
    descriptors = {item.id: item for item in vehicle.channels}
    telemetry_group_by_channel = {
        channel_id: group.id
        for group in output_schema.telemetry_groups
        for channel_id in group.channel_ids
    }
    quantity_by_unit = {
        "m": "length",
        "m/s": "speed",
        "m/s^2": "acceleration",
        "deg": "angle",
        "rad": "angle",
        "g0": "load_factor",
    }
    return tuple(
        TrajectoryChannelMetadata(
            id=channel_id,
            channel_class="telemetry" if channel_id in telemetry_group_by_channel else "core_state",
            telemetry_group=telemetry_group_by_channel.get(channel_id),
            quantity=quantity_by_unit.get(unit or ""),
            unit=unit,
            data_type="float64",
            shape=(),
            frame=descriptors[channel_id].frame,
            sampling_semantics="continuous_sample",
            interpolation="periodic" if channel_id.endswith("heading_deg") else "linear",
            description=descriptors[channel_id].description,
        )
        for channel_id, unit in result.channel_units.items()
    )
    ####


def _events(primary_object_id: str, result: MissionCompositionTrajectory) -> tuple[TrajectoryEvent, ...]:
    return tuple(
        TrajectoryEvent(
            id=f"event-{index:04d}-{item.kind.replace('_', '-')}",
            time_s=item.time_s,
            category="impact" if item.kind == "impact" else "segment",
            kind=item.kind,
            object_id=primary_object_id,
            segment_instance_id=item.segment_instance_id,
            detail=item.detail,
        )
        for index, item in enumerate(result.events, start=1)
    )
    ####


def _execution_error(
    code: str,
    message: str,
    *,
    phase: DiagnosticPhase,
    path: str,
    category: FailureCategory,
    provider_id: str | None = None,
    model_id: str | None = None,
) -> MissionCompositionExecutionError:
    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message,
            phase=phase,
            recoverability="correctable",
            path=path,
            provider_id=provider_id,
            model_id=model_id,
        ),
        category=category,
    )
    ####


__all__ = ["ReferenceMissionCompositionProvider"]
