"""Common Mission Composition bridge for the existing native batch bindings.

The portable configuration tree is the public request contract.  Existing
vehicle-composition artifacts remain the semantic handoff used by the native
family adapters.  This module is the only bridge between those two shapes and
normalizes every native batch result into the common multi-entity trajectory
contract.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, Literal, cast

from ..fidelity_contracts import FidelityTier
from ..language.grammar_contracts import GrammarProfile
from ..outputs import RunArtifact, TelemetryChannel, VehicleTelemetry
from ..runtime.runner import run_files
from ..vehicle_batch_execution import VehicleBatchExecution, execute_vehicle_composition_batch
from ..vehicle_composition import (
    CompiledVehicleComposition,
    CompositionValue,
    InitializationSelection,
    MissionSelection,
    SegmentSelection,
    VariantSelection,
    VehicleCompositionRequest,
    compile_vehicle_composition,
)
from .configuration_contract import (
    ConfigurationChoiceSchema,
    ConfigurationChoiceValue,
    ConfigurationGroupSchema,
    ConfigurationGroupValue,
    ConfigurationNode,
    ConfigurationNodeValue,
    ConfigurationOptionalSchema,
    ConfigurationOptionalValue,
    ConfigurationParameterSchema,
    ConfigurationParameterValue,
    ConfigurationSequenceSchema,
    ConfigurationSequenceValue,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryModelMetadata,
    TrajectoryOutputChannelMetadata,
)
from .dual_launch import render_dual_launch_problems
from .dual_launch_mission_composition import (
    DUAL_LAUNCH_MODEL_ID,
    build_dual_launch_prepared_case,
)
from .execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionExecutionError,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
    TrajectoryChannelMetadata,
    TrajectoryEntityRelationship,
    TrajectoryEvent,
    TrajectoryObject,
    TrajectorySample,
    TrajectorySegmentResult,
    TrajectoryStateSnapshot,
    resolve_output_selection,
)
from .native_output_contract import NativeOutputBinding, extract_native_channel, native_output_bindings
from .registry_mission_composition import RegistryMissionCompositionProvider
from .runtime_mission_composition import RuntimeArtifactProjection, trajectory_result_from_run_artifact
from .simple_aero_mission_composition import SIMPLE_AERO_MODEL_ID, build_simple_aero_prepared_configuration

_EXECUTOR_ID = "taoryx.registry.native-batch.v1"


def configuration_instance_from_vehicle_request(
    provider: RegistryMissionCompositionProvider,
    request: VehicleCompositionRequest,
    *,
    realization_id: str | None = None,
) -> TrajectoryConfigurationInstance:
    """Project one checked-in native request into the portable typed tree.

    This is intentionally public: the execution-witness audit uses it to prove
    that every native tuple is reachable through the exact same configuration
    grammar presented to a provider-neutral consumer.
    """

    if request.mission_graph is not None:
        raise ValueError("portable registry configuration does not yet expose authored mission graphs")
    if request.observation.profile_id != "truth_debug" or request.observation.declared_sensor is not None:
        raise ValueError("portable registry configuration does not yet expose native observation profiles")
    schema = provider.get_model_schema(request.vehicle)
    root_schema = _require_group(schema.root, "configuration root")
    initialization_schema = _require_choice(_child(root_schema, "initialization"), "initialization")
    segments_schema = _require_sequence(_child(root_schema, "segments"), "segments")
    segment_choice = _require_choice(segments_schema.item, "segment item")

    initialization_variant = _variant(initialization_schema, request.initialization.id)
    initialization_value = ConfigurationChoiceValue(
        selected=request.initialization.id,
        value=_value_for_parameter_tree(initialization_variant.node, request.initialization.inputs),
    )
    segment_values: list[ConfigurationNodeValue] = []
    for selected in request.segments:
        variant = _variant(segment_choice, selected.id)
        segment_values.append(
            ConfigurationChoiceValue(
                selected=selected.id,
                instance_id=selected.instance_id,
                value=_value_for_parameter_tree(variant.node, selected.inputs),
            )
        )

    values: dict[str, ConfigurationNodeValue] = {
        "initialization": initialization_value,
        "segments": ConfigurationSequenceValue(items=tuple(segment_values)),
    }
    variant_schema = next((item for item in root_schema.children if item.id == "variant_configuration"), None)
    if variant_schema is not None:
        if not isinstance(variant_schema, ConfigurationOptionalSchema):
            raise TypeError("variant_configuration must be an optional schema")
        values[variant_schema.id] = _variant_value(variant_schema, request.variant.inputs)
    elif request.variant.inputs:
        raise ValueError(f"model {request.vehicle!r} does not advertise vehicle-variant inputs")

    return TrajectoryConfigurationInstance(
        configuration_id=request.id,
        model_id=schema.model_id,
        model_version=schema.model_version,
        schema_fingerprint=schema.fingerprint,
        fidelity=request.fidelity,
        realization_id=realization_id or request.fidelity,
        mission_template_id=request.mission.id,
        root=ConfigurationGroupValue(values=values),
    )
    ####


def compile_prepared_vehicle_composition(
    provider: RegistryMissionCompositionProvider,
    prepared: PreparedTrajectoryConfiguration,
) -> CompiledVehicleComposition:
    """Compile a validated portable configuration into the native semantic IR."""

    configuration = prepared.configuration
    if provider.model(configuration.model_id).model_kind != "canonical_vehicle_family":
        raise _execution_error(
            "native-model-not-supported",
            "The selected workflow does not use the canonical native vehicle-composition compiler.",
            phase="preflight",
            category="unsupported",
            model_id=configuration.model_id,
        )
    validated = provider.validate_configuration(configuration)
    if validated.fingerprint != prepared.fingerprint or validated.resolved != prepared.resolved:
        raise _execution_error(
            "prepared-configuration-stale",
            "The prepared configuration does not match the provider's current schema validation result.",
            phase="configuration",
            category="invalid_request",
            model_id=configuration.model_id,
        )
    schema = provider.get_model_schema(configuration.model_id)
    root_schema = _require_group(schema.root, "configuration root")
    root = _require_mapping(prepared.resolved, "prepared configuration root")

    initialization_schema = _require_choice(_child(root_schema, "initialization"), "initialization")
    initialization_resolved = _selected_resolution(root.get("initialization"), "initialization")
    initialization_variant = _variant(initialization_schema, initialization_resolved[0])
    initialization_inputs = _composition_values(initialization_variant.node, initialization_resolved[1])

    segments_schema = _require_sequence(_child(root_schema, "segments"), "segments")
    segment_choice = _require_choice(segments_schema.item, "segment item")
    raw_segments = root.get("segments")
    if not isinstance(raw_segments, list):
        raise ValueError("prepared configuration segments must be a list")
    segments: list[SegmentSelection] = []
    segment_ids: list[str] = []
    for raw in raw_segments:
        selected_id, selected_values = _selected_resolution(raw, "segment")
        variant = _variant(segment_choice, selected_id)
        segment_ids.append(selected_id)
        segments.append(
            SegmentSelection(
                id=selected_id,
                instance_id=_selected_instance_id(raw),
                inputs=_composition_values(variant.node, selected_values),
            )
        )

    mission_id = _resolve_mission_id(
        provider.model(configuration.model_id),
        configuration.mission_template_id,
        configuration.fidelity,
        tuple(segment_ids),
    )
    variant_inputs: dict[str, CompositionValue] = {}
    variant_schema = next((item for item in root_schema.children if item.id == "variant_configuration"), None)
    if variant_schema is not None and root.get(variant_schema.id) is not None:
        variant_inputs = _composition_values(variant_schema, root[variant_schema.id], role="variant")

    native_request = VehicleCompositionRequest(
        id=configuration.configuration_id,
        vehicle=configuration.model_id,
        fidelity=cast(FidelityTier, configuration.fidelity),
        initialization=InitializationSelection(id=initialization_resolved[0], inputs=initialization_inputs),
        mission=MissionSelection(id=mission_id),
        segments=tuple(segments),
        variant=VariantSelection(inputs=variant_inputs),
    )
    return compile_vehicle_composition(native_request)
    ####


def build_registry_mission_composition_runner(
    provider: RegistryMissionCompositionProvider | None = None,
) -> MissionCompositionRunnerRegistry:
    """Register every canonical model with at least one common native batch tuple."""

    selected_provider = provider or RegistryMissionCompositionProvider()
    runner = MissionCompositionRunnerRegistry()
    for model in selected_provider.list_models():
        if "batch" not in model.common_runner_operations:
            continue

        def execute(request: MissionCompositionRunRequest, *, _model_id: str = model.id) -> MissionCompositionTrajectoryResult:
            if request.model_id != _model_id:
                raise _execution_error(
                    "executor-model-mismatch",
                    "The registered executor received a request for another model.",
                    phase="preflight",
                    category="provider_error",
                    model_id=request.model_id,
                )
            return execute_registry_batch_request(selected_provider, request)
            ####

        runner.register(selected_provider.metadata.id, model.id, execute)
    return runner
    ####


def execute_registry_batch_request(
    provider: RegistryMissionCompositionProvider,
    request: MissionCompositionRunRequest,
) -> MissionCompositionTrajectoryResult:
    """Execute one exact portable registry request through the native seam."""

    if request.provider_id != provider.metadata.id or request.provider_version != provider.metadata.version:
        raise _execution_error(
            "provider-version-mismatch",
            "The request provider identity does not match this Mission Composition provider.",
            phase="preflight",
            category="invalid_request",
            model_id=request.model_id,
        )
    if request.operation != "batch":
        raise _execution_error(
            "operation-not-supported",
            "Use the stateful Mission Composition session contract for interactive execution.",
            phase="preflight",
            category="unsupported",
            model_id=request.model_id,
        )
    model = provider.model(request.model_id)
    if model.id == SIMPLE_AERO_MODEL_ID:
        return _execute_simple_aero_batch(provider, model, request)
    if model.id == DUAL_LAUNCH_MODEL_ID:
        return _execute_dual_launch_batch(provider, model, request)
    composition = compile_prepared_vehicle_composition(provider, request.prepared_configuration)
    realization_id = request.prepared_configuration.configuration.realization_id or composition.fidelity
    _require_registered_operation(model, composition.mission, composition.fidelity, realization_id, "batch")

    try:
        with TemporaryDirectory(prefix="taoryx-mission-composition-") as temporary:
            batch = execute_vehicle_composition_batch(composition, Path(temporary))
            return _project_batch_result(provider, model, request, composition, realization_id, batch)
    except MissionCompositionExecutionError:
        raise
    except (KeyError, TypeError, ValueError) as error:
        raise _execution_error(
            "native-batch-execution-failed",
            str(error),
            phase="execution",
            category="execution_failed",
            model_id=request.model_id,
            details={"exception_type": type(error).__name__},
        ) from error
    ####


def _execute_simple_aero_batch(
    provider: RegistryMissionCompositionProvider,
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
) -> MissionCompositionTrajectoryResult:
    prepared = provider.validate_configuration(request.prepared_configuration.configuration)
    if prepared.fingerprint != request.prepared_configuration.fingerprint:
        raise _execution_error(
            "prepared-configuration-stale",
            "The prepared Simple Aero configuration no longer matches the provider schema.",
            phase="configuration",
            category="invalid_request",
            model_id=model.id,
        )
    build = build_simple_aero_prepared_configuration(prepared)
    mission_id = request.prepared_configuration.configuration.mission_template_id or "fixed_ld_baseline"
    realization_id = request.prepared_configuration.configuration.realization_id or "fixed_ld_point_mass"
    _require_registered_operation(model, mission_id, "point_mass_3dof", realization_id, "batch")
    with TemporaryDirectory(prefix="taoryx-simple-aero-") as temporary:
        root = Path(temporary)
        problem = root / "mission.prb"
        build.write(problem)
        expected_steps = math.ceil(build.derived.total_duration_s / build.parameters.time_step_s) + 1000
        report = run_files(
            problem,
            output_dir=root / "run",
            max_steps=max(expected_steps, 1000),
            profile=GrammarProfile.TAORYX,
        )
        if report.exit_code != 0 or not report.artifacts:
            raise _execution_error(
                "simple-aero-runtime-failed",
                "The generated Simple Aero problem did not complete in Simulation Runtime.",
                phase="execution",
                category="execution_failed",
                model_id=model.id,
                details={
                    "exit_code": report.exit_code,
                    "diagnostics": [item.model_dump(mode="json") for item in report.diagnostics],
                },
            )
        artifact = _simple_aero_artifact(
            report.artifacts[0],
            request,
            build.parameters.vehicle_id,
            model,
        )
    object_id = next(iter(artifact.vehicles))
    return trajectory_result_from_run_artifact(
        artifact,
        RuntimeArtifactProjection(
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            request_id=request.request_id,
            configuration_fingerprint=prepared.fingerprint,
            primary_model_id=model.id,
            primary_object_id=object_id,
            default_fidelity="point_mass_3dof",
            default_realization_id=realization_id,
            operation="batch",
            status="completed",
            model_ids={object_id: model.id},
            roles={object_id: "primary_vehicle"},
            object_statuses={object_id: "completed"},
            provenance="generated Simple Aero problem executed by Simulation Runtime",
            claim_boundary=model.claim_boundary,
        ),
        output_schema=model.output_schema,
        output_selection=request.output,
    )
    ####


def _simple_aero_artifact(
    artifact: RunArtifact,
    request: MissionCompositionRunRequest,
    vehicle_name: str,
    model: TrajectoryModelMetadata,
) -> RunArtifact:
    if not artifact.vehicles:
        raise ValueError("Simple Aero runtime artifact contains no vehicles")
    source = next(
        (item for item in artifact.vehicles.values() if item.name == vehicle_name),
        next(iter(artifact.vehicles.values())),
    )
    source_ids = {
        "position.geodetic.altitude": "position.altitude.geodetic",
        "position.geodetic.latitude": "position.latitude.geodetic",
        "position.geodetic.longitude": "position.longitude",
        "velocity.speed": "taos.vel",
        "mass.total": "mass.total",
        "propulsion.mass_flow": "taos.mdot",
        "propulsion.thrust": "propulsion.thrust",
        "propulsion.throttle_command": "propulsion.throttle_command",
        "aerodynamics.dynamic_pressure": "aerodynamics.dynamic_pressure",
        "aerodynamics.angle_of_attack": "aerodynamics.angle_of_attack",
    }
    metadata = {item.id: item for item in model.output_schema.channels}
    indexes = _sample_indices(source.times, request.output.cadence_s, request.output.maximum_samples_per_object)
    channels: dict[str, TelemetryChannel] = {}
    for channel_id, source_id in source_ids.items():
        try:
            native = source.channels[source_id]
        except KeyError as error:
            raise ValueError(f"Simple Aero runtime omitted advertised channel {channel_id!r}") from error
        descriptor = metadata[channel_id]
        channels[channel_id] = TelemetryChannel(
            source_name=native.source_name,
            semantic_name=channel_id,
            unit=descriptor.canonical_unit,
            interpolation=(
                "angle" if descriptor.interpolation == "periodic" else cast(Literal["linear", "angle", "step", "slerp", "event"], descriptor.interpolation)
            ),
            values=[native.values[index] for index in indexes],
        )
    object_id = f"{request.request_id}:primary"
    primary = VehicleTelemetry(
        vehicle_id=object_id,
        name=vehicle_name,
        model_id=model.id,
        kind=source.kind,
        dynamics=source.dynamics,
        attitude_source=source.attitude_source,
        times=[source.times[index] for index in indexes],
        channels=channels,
        segments=source.segments,
        events=source.events,
    )
    return artifact.model_copy(update={"vehicles": {object_id: primary}})
    ####


def _execute_dual_launch_batch(
    provider: RegistryMissionCompositionProvider,
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
) -> MissionCompositionTrajectoryResult:
    """Execute one source-generated dual-launch form through the common runner.

    The native Alpha 2 source models a continuous primary trajectory.  Its
    attached-booster transition is reported as an event, not as a spawned
    child, because the runtime does not independently propagate a released
    glider history.
    """

    prepared = provider.validate_configuration(request.prepared_configuration.configuration)
    if prepared.fingerprint != request.prepared_configuration.fingerprint or prepared.resolved != request.prepared_configuration.resolved:
        raise _execution_error(
            "prepared-configuration-stale",
            "The prepared dual-launch configuration no longer matches the provider schema.",
            phase="configuration",
            category="invalid_request",
            model_id=model.id,
        )
    configuration = prepared.configuration
    mission_id = configuration.mission_template_id
    realization_id = configuration.realization_id or "generated_native_problem"
    if mission_id is None:
        raise _execution_error(
            "mission-template-required",
            "Dual-launch native execution requires one exact launch-form mission template.",
            phase="configuration",
            category="invalid_request",
            model_id=model.id,
        )
    _require_registered_operation(model, mission_id, configuration.fidelity, realization_id, "batch")
    case = build_dual_launch_prepared_case(prepared)
    launch_mode = case.extensions.get("launch_mode")
    if launch_mode not in {"air_release", "attached_booster"}:
        raise _execution_error(
            "dual-launch-mode-unresolved",
            "The resolved dual-launch case did not declare a supported launch mode.",
            phase="configuration",
            category="provider_error",
            model_id=model.id,
        )
    try:
        with TemporaryDirectory(prefix="taoryx-dual-launch-") as temporary:
            root = Path(temporary)
            problem = next(item for item in render_dual_launch_problems(case, root / "problems") if item.launch_mode == launch_mode)
            report = run_files(
                problem.path,
                output_dir=root / "run",
                max_steps=10_000,
                profile=GrammarProfile.TAORYX,
            )
            if report.exit_code != 0 or not report.artifacts:
                raise _execution_error(
                    "dual-launch-runtime-failed",
                    "The generated dual-launch problem did not complete in Simulation Runtime.",
                    phase="execution",
                    category="execution_failed",
                    model_id=model.id,
                    details={
                        "exit_code": report.exit_code,
                        "launch_mode": launch_mode,
                        "diagnostics": [item.model_dump(mode="json") for item in report.diagnostics],
                    },
                )
            artifact = _dual_launch_artifact(report.artifacts[0], request, model, launch_mode)
    except MissionCompositionExecutionError:
        raise
    except (KeyError, StopIteration, TypeError, ValueError) as error:
        raise _execution_error(
            "dual-launch-native-projection-failed",
            str(error),
            phase="projection",
            category="provider_error",
            model_id=model.id,
            details={"exception_type": type(error).__name__, "launch_mode": launch_mode},
        ) from error
    object_id = next(iter(artifact.vehicles))
    return trajectory_result_from_run_artifact(
        artifact,
        RuntimeArtifactProjection(
            provider_id=provider.metadata.id,
            provider_version=provider.metadata.version,
            request_id=request.request_id,
            configuration_fingerprint=prepared.fingerprint,
            primary_model_id=model.id,
            primary_object_id=object_id,
            default_fidelity="point_mass_3dof",
            default_realization_id=realization_id,
            operation="batch",
            status="completed",
            model_ids={object_id: model.id},
            roles={object_id: "primary_vehicle"},
            object_statuses={object_id: "completed"},
            provenance="generated Alpha 2 dual-launch native point-mass problem executed by Simulation Runtime",
            claim_boundary=model.claim_boundary,
        ),
        output_schema=model.output_schema,
        output_selection=request.output,
    )
    ####


def _dual_launch_artifact(
    artifact: RunArtifact,
    request: MissionCompositionRunRequest,
    model: TrajectoryModelMetadata,
    launch_mode: Literal["air_release", "attached_booster"],
) -> RunArtifact:
    """Project the source primary history onto the advertised dual-launch schema."""

    try:
        source = artifact.vehicles["1"]
    except KeyError as error:
        raise ValueError("dual-launch runtime artifact omitted its primary trajectory") from error
    indexes = _sample_indices(source.times, request.output.cadence_s, request.output.maximum_samples_per_object)
    native_ids = {
        "position.local.north": "taos.north",
        "position.local.east": "taos.east",
        "mass.total": "mass.total",
        "propulsion.thrust": "propulsion.thrust",
        "propulsion.mass_flow": "taos.mdot",
        "control.bank.commanded": "taos.command.bank",
        "control.throttle.commanded": "propulsion.throttle_command",
        "aerodynamics.dynamic_pressure": "aerodynamics.dynamic_pressure",
        "diagnostics.segment_index": "phase.segment",
    }
    sampled_times = [source.times[index] for index in indexes]
    values: dict[str, list[float | None]] = {
        channel_id: _optional_channel_values(_dual_launch_native_values(source, native_id, indexes))
        for channel_id, native_id in native_ids.items()
    }
    all_indexes = tuple(range(len(source.times)))
    full_position = {
        "north": _dual_launch_native_values(source, "taos.north", all_indexes),
        "east": _dual_launch_native_values(source, "taos.east", all_indexes),
    }
    altitude = _dual_launch_native_values(source, "position.altitude.geodetic", all_indexes)
    full_position["down"] = [altitude[0] - value for value in altitude]
    values["position.local.down"] = _optional_channel_values(full_position["down"][index] for index in indexes)
    for axis, position in full_position.items():
        velocity = _finite_difference(source.times, position)
        values[f"velocity.local.{axis}"] = _optional_channel_values(velocity[index] for index in indexes)
    metadata = {item.id: item for item in model.output_schema.channels}
    source_for_channel = {
        **native_ids,
        "position.local.down": "position.altitude.geodetic",
        "velocity.local.north": "finite_difference(taos.north,time)",
        "velocity.local.east": "finite_difference(taos.east,time)",
        "velocity.local.down": "finite_difference(position.altitude.geodetic,time)",
    }
    channels = {
        channel_id: TelemetryChannel(
            source_name=source_for_channel[channel_id],
            semantic_name=channel_id,
            unit=metadata[channel_id].canonical_unit,
            interpolation=("step" if metadata[channel_id].interpolation == "step" else "linear"),
            values=channel_values,
        )
        for channel_id, channel_values in values.items()
    }
    object_id = f"{request.request_id}:primary"
    primary = VehicleTelemetry(
        vehicle_id=object_id,
        name=f"dual-launch-{launch_mode}",
        model_id=model.id,
        kind=source.kind,
        dynamics=source.dynamics,
        attitude_source=source.attitude_source,
        times=sampled_times,
        channels=channels,
        segments=source.segments,
        events=source.events,
    )
    events = [_dual_launch_event(payload, object_id, launch_mode) for payload in artifact.events if payload.get("vehicle") == "1"]
    return artifact.model_copy(update={"vehicles": {object_id: primary}, "events": events})
    ####


def _dual_launch_native_values(
    source: VehicleTelemetry,
    channel_id: str,
    indexes: Sequence[int],
) -> list[float]:
    try:
        raw = source.channels[channel_id].values
    except KeyError as error:
        raise ValueError(f"dual-launch runtime omitted advertised source channel {channel_id!r}") from error
    values: list[float] = []
    for index in indexes:
        value = raw[index]
        if value is None or not math.isfinite(value):
            raise ValueError(f"dual-launch source channel {channel_id!r} contains a non-finite accepted sample")
        values.append(float(value))
    return values
    ####


def _optional_channel_values(values: Iterable[float]) -> list[float | None]:
    """Widen finite samples to the optional telemetry channel value contract."""

    return [float(value) for value in values]
    ####


def _finite_difference(times: Sequence[float], values: Sequence[float]) -> list[float]:
    """Differentiate one finite sampled signal without pretending it is native state."""

    if len(times) != len(values) or len(times) < 2:
        raise ValueError("finite-difference projection requires at least two aligned samples")
    result: list[float] = []
    for index, value in enumerate(values):
        left = index - 1 if index > 0 else 0
        right = index + 1 if index + 1 < len(values) else len(values) - 1
        delta_time = times[right] - times[left]
        if delta_time <= 0.0:
            raise ValueError("dual-launch selected samples must have strictly increasing time for velocity projection")
        derivative = (values[right] - values[left]) / delta_time
        if not math.isfinite(derivative):
            raise ValueError("dual-launch finite-difference velocity is non-finite")
        result.append(float(derivative))
    return result
    ####


def _dual_launch_event(
    payload: dict[str, object],
    object_id: str,
    launch_mode: Literal["air_release", "attached_booster"],
) -> dict[str, object]:
    """Keep source events scoped to the returned primary, naming only the real handoff."""

    event = {**payload, "vehicle": object_id}
    if launch_mode == "attached_booster" and payload.get("segment_from") == 1 and payload.get("segment_to") == 2 and payload.get("action") == "goto":
        event.update(
            {
                "name": "booster-glider-separation",
                "event_id": "booster-glider-separation",
                "action": "handoff",
                "status": "committed",
            }
        )
    return event
    ####


def _project_batch_result(
    provider: RegistryMissionCompositionProvider,
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
    composition: CompiledVehicleComposition,
    realization_id: str,
    batch: VehicleBatchExecution,
) -> MissionCompositionTrajectoryResult:
    rows = _read_truth_rows(batch.output_dir)
    selected_metadata = resolve_output_selection(
        model.output_schema,
        request.output,
        fidelity=composition.fidelity,
        operation="batch",
        realization_id=realization_id,
        mission_template_id=composition.mission,
    )
    bindings = _applicable_bindings(model.id, composition.fidelity, composition.mission)
    binding_by_id = {item.id: item for item in bindings}
    missing_bindings = sorted(item.id for item in selected_metadata if item.id not in binding_by_id)
    if missing_bindings:
        raise _execution_error(
            "output-projector-missing",
            f"No native projector exists for advertised output channels {missing_bindings!r}.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    sampled_rows = _sample_rows(rows, request.output.cadence_s, request.output.maximum_samples_per_object)
    samples = tuple(
        TrajectorySample(
            time_s=_row_time(row),
            values={item.id: extract_native_channel(row, binding_by_id[item.id]) for item in selected_metadata},
            segment_instance_id=_segment_marker(row, composition),
        )
        for row in sampled_rows
    )
    channels = tuple(_result_channel(item, binding_by_id[item.id]) for item in selected_metadata)
    root_id = f"{request.request_id}:primary"
    primary_segments = _segment_results(root_id, samples) if request.output.include_segments else ()
    diagnostics: list[MissionCompositionDiagnostic] = []
    if not batch.passed:
        diagnostics.append(
            MissionCompositionDiagnostic(
                severity="warning",
                code="native-mission-disposition-failed",
                message="The native executor completed but its mission-level pass disposition was false.",
                phase="execution",
                recoverability="degraded",
                provider_id=provider.metadata.id,
                model_id=model.id,
            )
        )
    primary = TrajectoryObject(
        object_id=root_id,
        model_id=model.id,
        realization_id=realization_id,
        name=model.name,
        role="primary_vehicle",
        fidelity=composition.fidelity,
        status="completed" if batch.passed else "terminated",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition="mission_complete" if batch.passed else "native_mission_disposition_failed",
        channels=channels,
        samples=samples,
        segments=primary_segments,
        provenance=f"native factory {batch.binding.factory_id}",
        claim_boundary=str(batch.as_dict().get("claim_boundary") or model.claim_boundary),
    )

    objects: list[TrajectoryObject] = [primary]
    events: list[TrajectoryEvent] = []
    relationships: list[TrajectoryEntityRelationship] = []
    native_trajectory = getattr(batch.execution, "trajectory", None)
    if native_trajectory is not None and hasattr(native_trajectory, "spawned_bodies") and hasattr(native_trajectory, "mission_events"):
        if request.output.include_events:
            events.extend(_mission_events(root_id, native_trajectory, composition))
        if request.output.include_spawned_objects:
            maximum_children = None
            if request.output.maximum_objects is not None:
                maximum_children = max(request.output.maximum_objects - 1, 0)
            children = native_trajectory.spawned_bodies[:maximum_children]
            for index, child in enumerate(children, start=1):
                child_object, event, relationship = _spawned_child_result(
                    model,
                    request,
                    composition,
                    root_id,
                    child,
                    index,
                )
                objects.append(child_object)
                events.append(event)
                relationships.append(relationship)
            if maximum_children is not None and len(native_trajectory.spawned_bodies) > maximum_children:
                diagnostics.append(
                    MissionCompositionDiagnostic(
                        severity="warning",
                        code="spawned-object-limit-applied",
                        message="Spawned objects were omitted by maximum_objects.",
                        phase="projection",
                        recoverability="degraded",
                        provider_id=provider.metadata.id,
                        model_id=model.id,
                        details={
                            "available_children": len(native_trajectory.spawned_bodies),
                            "returned_children": maximum_children,
                        },
                    )
                )
    events = _deduplicate_events(events)
    return MissionCompositionTrajectoryResult(
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=model.id,
        primary_object_id=root_id,
        status="completed" if batch.passed else "terminated",
        objects=tuple(objects),
        events=tuple(sorted(events, key=lambda item: (item.time_s, item.id))),
        relationships=tuple(relationships),
        diagnostics=tuple(diagnostics),
        claim_boundary=(
            "The common result is a lossless semantic projection of the exact native truth channels selected by the "
            "request. Native qualification and model claim boundaries remain unchanged."
        ),
    )
    ####


def _spawned_child_result(
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
    composition: CompiledVehicleComposition,
    parent_object_id: str,
    child: Any,
    index: int,
) -> tuple[TrajectoryObject, TrajectoryEvent, TrajectoryEntityRelationship]:
    deployment = next(
        (item for item in model.deployments if item.lifecycle == "independently_propagated" and composition.fidelity in item.compatible_fidelities),
        None,
    )
    if deployment is None:
        raise _execution_error(
            "spawned-child-not-advertised",
            "The native executor returned a child trajectory that discovery did not advertise.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    rows = tuple(dict(item) for item in child.telemetry)
    if not rows:
        raise ValueError(f"spawned child {child.body_id!r} returned no telemetry")
    sampled_rows = _sample_rows(rows, request.output.cadence_s, request.output.maximum_samples_per_object)
    core_specs = (
        ("position.local.x", "Local X Position", "length", "m", "local_cartesian", "position_m[0]"),
        ("position.local.y", "Local Y Position", "length", "m", "local_cartesian", "position_m[1]"),
        ("position.local.z", "Local Z Position", "length", "m", "local_cartesian", "position_m[2]"),
        ("velocity.local.x", "Local X Velocity", "speed", "m/s", "local_cartesian", "velocity_m_s[0]"),
        ("velocity.local.y", "Local Y Velocity", "speed", "m/s", "local_cartesian", "velocity_m_s[1]"),
        ("velocity.local.z", "Local Z Velocity", "speed", "m/s", "local_cartesian", "velocity_m_s[2]"),
    )
    telemetry_specs = (
        ("mass.total", "Total Mass", "mass", "kg", None, "mass_kg", "resources"),
        ("aerodynamics.drag_force", "Drag Force", "force", "N", "local_cartesian", "drag_force_n", "aerodynamics"),
        ("aerodynamics.projected_area", "Projected Area", "area", "m^2", "body", "projected_area_m2", "aerodynamics"),
        ("angular_rate.norm", "Angular-Rate Norm", "angular_rate", "rad/s", "body", "angular_rate_norm_rad_s", "dynamics"),
    )
    bindings: list[NativeOutputBinding] = [
        NativeOutputBinding(identifier, label, f"Spawned-child {label.casefold()}.", "core_state", quantity, unit, frame, (path,), (), ())
        for identifier, label, quantity, unit, frame, path in core_specs
    ]
    selected_groups = set(request.output.telemetry_groups)
    selected_ids = set(request.output.channels)
    if request.output.mode == "all":
        include_telemetry = lambda _identifier, _group: True
    elif request.output.mode == "selected":
        include_telemetry = lambda identifier, group: identifier in selected_ids or group in selected_groups
    else:
        include_telemetry = lambda _identifier, _group: False
    bindings.extend(
        NativeOutputBinding(
            identifier,
            label,
            f"Spawned-child {label.casefold()}.",
            "telemetry",
            quantity,
            unit,
            frame,
            (path,),
            (),
            (),
            group,
        )
        for identifier, label, quantity, unit, frame, path, group in telemetry_specs
        if include_telemetry(identifier, group)
    )
    channel_metadata = tuple(
        TrajectoryChannelMetadata(
            id=item.id,
            channel_class=item.channel_class,
            telemetry_group=item.telemetry_group,
            quantity=item.quantity,
            unit=item.unit,
            data_type="float64",
            shape=(),
            frame=item.frame,
            sampling_semantics="continuous_sample",
            interpolation=item.interpolation,
            description=item.description,
        )
        for item in bindings
    )
    samples = tuple(
        TrajectorySample(
            time_s=_row_time(row),
            values={item.id: extract_native_channel(row, item) for item in bindings},
            segment_instance_id="spawned-ballistic",
        )
        for row in sampled_rows
    )
    object_id = f"{request.request_id}:child:{index}:{child.body_id}"
    event_id = f"{request.request_id}:spawn:{index}:{child.parent_event_id}"
    kind = _relationship_kind(deployment.id, deployment.trigger_event_kinds)
    event = TrajectoryEvent(
        id=event_id,
        time_s=samples[0].time_s,
        category=kind,
        kind=child.parent_event_id,
        object_id=object_id,
        parent_object_id=parent_object_id,
        deployment_id=deployment.id,
        detail=f"Spawned {deployment.child_role} from {model.id}.",
        data={
            "native_child_model_id": child.body_id,
            "shape": child.shape,
            "requested_fidelity": None if child.requested_fidelity is None else child.requested_fidelity.value,
            "realized_fidelity": child.fidelity.value,
        },
    )
    child_object = TrajectoryObject(
        object_id=object_id,
        model_id=deployment.child_model_id or child.body_id,
        realization_id=child.body_id,
        name=child.body_id,
        role=deployment.child_role,
        fidelity=child.fidelity.value,
        status="completed" if child.classification == "impact" else "terminated" if child.classification == "timeout" else "failed",
        parent_object_id=parent_object_id,
        deployment_id=deployment.id,
        spawn_event_id=event_id,
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=child.classification,
        channels=channel_metadata,
        samples=samples,
        segments=(
            TrajectorySegmentResult(
                id="spawned_ballistic",
                instance_id="spawned-ballistic",
                object_id=object_id,
                start_time_s=samples[0].time_s,
                end_time_s=samples[-1].time_s,
                status="completed" if child.classification == "impact" else "terminated",
            ),
        )
        if request.output.include_segments
        else (),
        provenance="native detached-body propagation",
        claim_boundary=deployment.claim_boundary,
    )
    relationship = TrajectoryEntityRelationship(
        id=f"{request.request_id}:relationship:{index}",
        kind=kind,
        parent_object_id=parent_object_id,
        child_object_id=object_id,
        deployment_id=deployment.id,
        event_id=event_id,
        created_at_s=samples[0].time_s,
        state_transfer=deployment.state_initialization,
        initial_state=TrajectoryStateSnapshot(time_s=samples[0].time_s, values=samples[0].values),
        provenance=deployment.provenance,
    )
    return child_object, event, relationship
    ####


def _read_truth_rows(output_dir: Path) -> tuple[dict[str, object], ...]:
    csv_path = output_dir / "truth_telemetry.csv"
    json_path = output_dir / "truth_telemetry.json"
    if csv_path.is_file():
        with csv_path.open(newline="", encoding="utf-8") as handle:
            rows = tuple(dict(row) for row in csv.DictReader(handle))
    elif json_path.is_file():
        payload = json.loads(json_path.read_text(encoding="utf-8"))
        if not isinstance(payload, list) or not all(isinstance(item, dict) for item in payload):
            raise ValueError("native truth_telemetry.json must contain a list of objects")
        rows = tuple(dict(item) for item in payload)
    else:
        raise ValueError("native executor emitted neither truth_telemetry.csv nor truth_telemetry.json")
    if not rows:
        raise ValueError("native executor emitted an empty truth trajectory")
    times = tuple(_row_time(item) for item in rows)
    if any(later < earlier for earlier, later in zip(times, times[1:], strict=False)):
        raise ValueError("native truth trajectory is not monotonic")
    return rows
    ####


def _sample_rows(
    rows: Sequence[dict[str, object]],
    cadence_s: float | None,
    maximum_samples: int | None,
) -> tuple[dict[str, object], ...]:
    indexes = _sample_indices(tuple(_row_time(item) for item in rows), cadence_s, maximum_samples)
    return tuple(rows[index] for index in indexes)
    ####


def _sample_indices(
    times: Sequence[float],
    cadence_s: float | None,
    maximum_samples: int | None,
) -> tuple[int, ...]:
    if not times:
        raise ValueError("cannot sample an empty trajectory")
    selected = list(range(len(times)))
    if cadence_s is not None and len(selected) > 1:
        cadence_indexes = [selected[0]]
        last_time = times[selected[0]]
        for index in selected[1:-1]:
            if times[index] >= last_time + cadence_s - 1.0e-12:
                cadence_indexes.append(index)
                last_time = times[index]
        if selected[-1] != cadence_indexes[-1]:
            cadence_indexes.append(selected[-1])
        selected = cadence_indexes
    if maximum_samples is not None and len(selected) > maximum_samples:
        if maximum_samples == 1:
            selected = [selected[-1]]
        else:
            selected = [selected[round(index * (len(selected) - 1) / (maximum_samples - 1))] for index in range(maximum_samples)]
    return tuple(selected)
    ####


def _applicable_bindings(model_id: str, fidelity: str, mission: str) -> tuple[NativeOutputBinding, ...]:
    return tuple(
        item
        for item in native_output_bindings(model_id)
        if (not item.fidelities or fidelity in item.fidelities) and (not item.mission_templates or mission in item.mission_templates)
    )
    ####


def _result_channel(metadata: TrajectoryOutputChannelMetadata, binding: NativeOutputBinding) -> TrajectoryChannelMetadata:
    return TrajectoryChannelMetadata(
        id=metadata.id,
        channel_class=binding.channel_class,
        telemetry_group=binding.telemetry_group,
        quantity=metadata.quantity,
        unit=metadata.canonical_unit,
        data_type=metadata.data_type,
        shape=metadata.shape,
        frame=metadata.frame,
        sampling_semantics=metadata.sampling_semantics,
        interpolation=metadata.interpolation,
        description=metadata.description,
    )
    ####


def _row_time(row: Mapping[str, object]) -> float:
    for key in ("time_s", "time"):
        raw = row.get(key)
        if isinstance(raw, str):
            try:
                raw = float(raw)
            except ValueError:
                continue
        if isinstance(raw, int | float) and not isinstance(raw, bool) and math.isfinite(float(raw)):
            value = float(raw)
            if value < 0.0:
                raise ValueError("native truth time cannot be negative")
            return value
    raise ValueError("native truth row does not contain a finite time_s")
    ####


def _segment_marker(row: Mapping[str, object], composition: CompiledVehicleComposition) -> str | None:
    for key in ("segment_instance_id", "mission_segment", "phase"):
        raw = row.get(key)
        if not isinstance(raw, str) or not raw:
            continue
        exact = next((item.instance_id for item in composition.segments if item.instance_id == raw), None)
        if exact is not None:
            return exact
        matches = tuple(item.instance_id for item in composition.segments if item.id == raw)
        if len(matches) == 1:
            return matches[0]
        return raw
    return None
    ####


def _segment_results(object_id: str, samples: tuple[TrajectorySample, ...]) -> tuple[TrajectorySegmentResult, ...]:
    spans: list[TrajectorySegmentResult] = []
    start = 0
    occurrence: dict[str, int] = {}
    while start < len(samples):
        marker = samples[start].segment_instance_id
        if marker is None:
            start += 1
            continue
        end = start
        while end + 1 < len(samples) and samples[end + 1].segment_instance_id == marker:
            end += 1
        occurrence[marker] = occurrence.get(marker, 0) + 1
        instance_id = marker if occurrence[marker] == 1 else f"{marker}:{occurrence[marker]}"
        spans.append(
            TrajectorySegmentResult(
                id=marker,
                instance_id=instance_id,
                object_id=object_id,
                start_time_s=samples[start].time_s,
                end_time_s=samples[end].time_s,
                status="completed",
            )
        )
        start = end + 1
    return tuple(spans)
    ####


def _mission_events(
    root_id: str,
    trajectory: Any,
    composition: CompiledVehicleComposition,
) -> list[TrajectoryEvent]:
    result: list[TrajectoryEvent] = []
    for index, raw in enumerate(trajectory.mission_events, start=1):
        accepted = raw.get("accepted_time_s", raw.get("time_s"))
        if isinstance(accepted, str):
            accepted = float(accepted)
        if not isinstance(accepted, int | float) or isinstance(accepted, bool) or not math.isfinite(float(accepted)):
            continue
        kind = str(raw.get("kind") or raw.get("event_id") or "mission-event")
        segment = raw.get("segment")
        result.append(
            TrajectoryEvent(
                id=f"{root_id}:mission-event:{index}",
                time_s=float(accepted),
                category="segment",
                kind=kind,
                object_id=root_id,
                segment_instance_id=(
                    next((item.instance_id for item in composition.segments if item.id == segment), None) if isinstance(segment, str) else None
                ),
                detail="Native accepted mission event.",
                data=dict(raw),
            )
        )
    return result
    ####


def _deduplicate_events(events: Sequence[TrajectoryEvent]) -> list[TrajectoryEvent]:
    result: list[TrajectoryEvent] = []
    seen: set[str] = set()
    for event in events:
        identifier = event.id
        if identifier in seen:
            suffix = 2
            while f"{identifier}:{suffix}" in seen:
                suffix += 1
            event = event.model_copy(update={"id": f"{identifier}:{suffix}"})
        seen.add(event.id)
        result.append(event)
    return result
    ####


def _relationship_kind(
    deployment_id: str,
    trigger_kinds: Sequence[str],
) -> Literal["release", "separation", "deployment"]:
    text = " ".join((deployment_id, *trigger_kinds)).casefold()
    if "separation" in text:
        return "separation"
    if "release" in text:
        return "release"
    return "deployment"
    ####


def _require_registered_operation(
    model: TrajectoryModelMetadata,
    mission_id: str,
    fidelity: str,
    realization_id: str,
    operation: Literal["batch", "step"],
) -> None:
    mission = next((item for item in model.mission_templates if item.id == mission_id), None)
    if mission is None:
        raise _execution_error(
            "mission-not-advertised",
            f"Mission {mission_id!r} is not advertised for model {model.id!r}.",
            phase="preflight",
            category="unsupported",
            model_id=model.id,
        )
    exact = next(
        (item for item in mission.operations if item.fidelity == fidelity and item.operation == operation and item.realization_id in {None, realization_id}),
        None,
    )
    if exact is None or exact.status != "available" or exact.common_runner_status != "registered":
        blockers = () if exact is None else exact.blockers
        raise _execution_error(
            "operation-not-available",
            f"The exact {model.id}/{mission_id}/{fidelity}/{realization_id}/{operation} tuple is not available.",
            phase="preflight",
            category="unsupported",
            model_id=model.id,
            details={"blockers": list(blockers)},
        )
    ####


def _resolve_mission_id(
    model: TrajectoryModelMetadata,
    requested: str | None,
    fidelity: str,
    segment_ids: tuple[str, ...],
) -> str:
    candidates = tuple(item for item in model.mission_templates if item.segment_sequence == segment_ids and fidelity in item.compatible_fidelities)
    if requested is not None:
        selected = next((item for item in candidates if item.id == requested), None)
        if selected is None:
            raise ValueError(f"mission_template_id {requested!r} does not match the configured sequence and fidelity")
        return selected.id
    if len(candidates) != 1:
        raise ValueError("mission_template_id is required because the configured sequence does not identify exactly one mission")
    return candidates[0].id
    ####


def _composition_values(
    schema: ConfigurationNode,
    resolved: object,
    *,
    role: str | None = None,
) -> dict[str, CompositionValue]:
    result: dict[str, CompositionValue] = {}

    def visit(node: ConfigurationNode, value: object) -> None:
        if isinstance(node, ConfigurationParameterSchema):
            if role is None or node.role == role:
                if node.id in result:
                    raise ValueError(f"configuration parameter {node.id!r} occurs more than once")
                result[node.id] = CompositionValue(value=value, unit=node.canonical_unit)
            return
        if isinstance(node, ConfigurationGroupSchema):
            mapping = _require_mapping(value, node.id)
            for child_schema in node.children:
                if child_schema.id in mapping:
                    visit(child_schema, mapping[child_schema.id])
            return
        if isinstance(node, ConfigurationChoiceSchema):
            selected_id, selected_value = _selected_resolution(value, node.id)
            visit(_variant(node, selected_id).node, selected_value)
            return
        if isinstance(node, ConfigurationSequenceSchema):
            if not isinstance(value, list):
                raise TypeError(f"{node.id} must resolve to a list")
            for item in value:
                visit(node.item, item)
            return
        if isinstance(node, ConfigurationOptionalSchema):
            if value is not None:
                visit(node.item, value)
            return
        raise TypeError(f"unsupported configuration node {type(node).__name__}")
        ####

    visit(schema, resolved)
    return result
    ####


def _value_for_parameter_tree(
    schema: ConfigurationNode,
    inputs: Mapping[str, CompositionValue],
) -> ConfigurationNodeValue:
    if isinstance(schema, ConfigurationParameterSchema):
        supplied = inputs.get(schema.id)
        if supplied is None:
            if schema.default_declared:
                return ConfigurationParameterValue(value=schema.default, unit=schema.canonical_unit)
            raise ValueError(f"native request does not supply required parameter {schema.id!r}")
        return ConfigurationParameterValue(value=supplied.value, unit=supplied.unit)
    if isinstance(schema, ConfigurationGroupSchema):
        values: dict[str, ConfigurationNodeValue] = {}
        for child_schema in schema.children:
            if _subtree_parameter_ids(child_schema) & set(inputs):
                values[child_schema.id] = _value_for_parameter_tree(child_schema, inputs)
            elif isinstance(child_schema, ConfigurationParameterSchema) and child_schema.default_declared:
                values[child_schema.id] = ConfigurationParameterValue(
                    value=child_schema.default,
                    unit=child_schema.canonical_unit,
                )
            elif isinstance(child_schema, ConfigurationOptionalSchema):
                values[child_schema.id] = ConfigurationOptionalValue(enabled=False)
        return ConfigurationGroupValue(values=values)
    raise TypeError("native initialization and segment variants must resolve to parameter groups")
    ####


def _variant_value(
    schema: ConfigurationOptionalSchema,
    inputs: Mapping[str, CompositionValue],
) -> ConfigurationOptionalValue:
    unknown = sorted(set(inputs) - _subtree_parameter_ids(schema))
    if unknown:
        raise ValueError(f"native request supplies unknown variant parameters {unknown!r}")
    if not inputs:
        return ConfigurationOptionalValue(enabled=False)
    return ConfigurationOptionalValue(enabled=True, value=_variant_subtree_value(schema.item, inputs))
    ####


def _variant_subtree_value(
    schema: ConfigurationNode,
    inputs: Mapping[str, CompositionValue],
) -> ConfigurationNodeValue:
    present = _subtree_parameter_ids(schema) & set(inputs)
    if isinstance(schema, ConfigurationParameterSchema):
        supplied = inputs[schema.id]
        return ConfigurationParameterValue(value=supplied.value, unit=supplied.unit)
    if isinstance(schema, ConfigurationGroupSchema):
        values: dict[str, ConfigurationNodeValue] = {}
        for child in schema.children:
            if _subtree_parameter_ids(child) & present:
                values[child.id] = _variant_subtree_value(child, inputs)
            elif isinstance(child, ConfigurationOptionalSchema):
                values[child.id] = ConfigurationOptionalValue(enabled=False)
        return ConfigurationGroupValue(values=values)
    if isinstance(schema, ConfigurationChoiceSchema):
        variants = tuple(item for item in schema.variants if _subtree_parameter_ids(item.node) & present)
        if len(variants) != 1:
            raise ValueError(f"variant inputs do not select exactly one {schema.id!r} choice")
        selected = variants[0]
        return ConfigurationChoiceValue(
            selected=selected.id,
            value=_variant_subtree_value(selected.node, inputs),
        )
    if isinstance(schema, ConfigurationOptionalSchema):
        if not present:
            return ConfigurationOptionalValue(enabled=False)
        return ConfigurationOptionalValue(enabled=True, value=_variant_subtree_value(schema.item, inputs))
    raise TypeError(f"unsupported variant configuration node {type(schema).__name__}")
    ####


def _subtree_parameter_ids(schema: ConfigurationNode) -> set[str]:
    if isinstance(schema, ConfigurationParameterSchema):
        return {schema.id}
    if isinstance(schema, ConfigurationGroupSchema):
        return set().union(*(_subtree_parameter_ids(item) for item in schema.children)) if schema.children else set()
    if isinstance(schema, ConfigurationChoiceSchema):
        return set().union(*(_subtree_parameter_ids(item.node) for item in schema.variants))
    if isinstance(schema, ConfigurationSequenceSchema):
        return _subtree_parameter_ids(schema.item)
    if isinstance(schema, ConfigurationOptionalSchema):
        return _subtree_parameter_ids(schema.item)
    raise TypeError(type(schema).__name__)
    ####


def _selected_resolution(value: object, name: str) -> tuple[str, object]:
    mapping = _require_mapping(value, name)
    selected = mapping.get("selected")
    if not isinstance(selected, str):
        raise TypeError(f"{name} choice has no selected identifier")
    return selected, mapping.get("value")
    ####


def _selected_instance_id(value: object) -> str | None:
    mapping = _require_mapping(value, "choice")
    instance_id = mapping.get("instance_id")
    if instance_id is not None and not isinstance(instance_id, str):
        raise TypeError("choice instance_id must be a string")
    return instance_id
    ####


def _child(group: ConfigurationGroupSchema, identifier: str) -> ConfigurationNode:
    try:
        return next(item for item in group.children if item.id == identifier)
    except StopIteration as error:
        raise KeyError(f"group {group.id!r} has no child {identifier!r}") from error
    ####


def _variant(choice: ConfigurationChoiceSchema, identifier: str) -> Any:
    try:
        return next(item for item in choice.variants if item.id == identifier)
    except StopIteration as error:
        raise KeyError(f"choice {choice.id!r} has no variant {identifier!r}") from error
    ####


def _require_group(node: ConfigurationNode, name: str) -> ConfigurationGroupSchema:
    if not isinstance(node, ConfigurationGroupSchema):
        raise TypeError(f"{name} must be a group")
    return node
    ####


def _require_choice(node: ConfigurationNode, name: str) -> ConfigurationChoiceSchema:
    if not isinstance(node, ConfigurationChoiceSchema):
        raise TypeError(f"{name} must be a choice")
    return node
    ####


def _require_sequence(node: ConfigurationNode, name: str) -> ConfigurationSequenceSchema:
    if not isinstance(node, ConfigurationSequenceSchema):
        raise TypeError(f"{name} must be a sequence")
    return node
    ####


def _require_mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping")
    return cast(Mapping[str, object], value)
    ####


def _execution_error(
    code: str,
    message: str,
    *,
    phase: Literal["discovery", "configuration", "preflight", "execution", "projection"],
    category: Literal["invalid_request", "unsupported", "unavailable", "execution_failed", "provider_error"],
    model_id: str,
    details: dict[str, Any] | None = None,
) -> MissionCompositionExecutionError:
    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message or "Native Mission Composition bridge failed.",
            phase=phase,
            recoverability="correctable" if category in {"invalid_request", "unsupported"} else "fatal",
            model_id=model_id,
            details=details or {},
        ),
        category=category,
    )
    ####


__all__ = [
    "build_registry_mission_composition_runner",
    "compile_prepared_vehicle_composition",
    "configuration_instance_from_vehicle_request",
    "execute_registry_batch_request",
]
