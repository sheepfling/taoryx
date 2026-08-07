"""Runnable analytical reference binding for the Mission Composition contract.

The repository's ballistic and constant-velocity waypoint fixtures predate the
typed configuration and multi-object execution envelopes.  This module binds
those deliberately simple native 3-DOF models to the current public contract.
It is an interface witness, not a historical TAOS or qualified vehicle claim.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

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
            segments.append(
                MissionCompositionSegmentRequest(
                    id=segment_id,
                    instance_id=f"{index:02d}-{segment_id}",
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
