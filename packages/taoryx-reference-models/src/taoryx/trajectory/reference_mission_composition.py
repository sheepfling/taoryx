"""Runnable analytical reference binding for the Mission Composition contract.

The repository's ballistic and constant-velocity waypoint fixtures predate the
typed configuration and multi-object execution envelopes.  This module binds
those deliberately simple native 3-DOF models to the current public contract.
It is an interface witness, not a historical TAOS or qualified vehicle claim.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

from pydantic import BaseModel, ConfigDict, Field, model_validator

from .analytical_mission_composition import (
    ExampleMissionCompositionProvider,
    MissionCompositionOutputRequest,
    MissionCompositionParameter,
    MissionCompositionParameterValue,
    MissionCompositionSegmentRequest,
    MissionCompositionTrajectory,
    MissionCompositionTrajectoryRequest,
    MissionCompositionVehicle,
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

_PROVIDER_ID = "taoryx.reference.mission-composition"
_PROVIDER_VERSION = "1.0.0"

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

    def __init__(self) -> None:
        self._analytical = ExampleMissionCompositionProvider()
        self._models = tuple(
            item.model_copy(update={"common_runner_operations": ("batch",)})
            for item in self._analytical.list_models()
        )
        self._metadata = TrajectoryProviderMetadata(
            id=_PROVIDER_ID,
            name="TAORYX Analytical Mission Composition Reference",
            version=_PROVIDER_VERSION,
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
