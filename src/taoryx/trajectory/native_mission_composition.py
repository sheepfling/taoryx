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
from collections.abc import Mapping, Sequence
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any, Literal, cast

from ..deployment import DeploymentBinding, DeploymentChildExecution
from ..fidelity_contracts import FidelityTier
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

_EXECUTOR_ID = "taoryx.registry.native-batch.v1"
_SIMPLE_AERO_MODEL_ID = "simple_aero"
_DUAL_LAUNCH_MODEL_ID = "dual_launch_glider"


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
    if request.deployment_bindings:
        raise ValueError(
            "portable registry configuration keeps child selection at the MissionCompositionRunRequest boundary; pass deployment_bindings with the run request"
        )
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
    *,
    deployment_bindings: tuple[DeploymentBinding, ...] = (),
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
        deployment_bindings=deployment_bindings,
    )
    return compile_vehicle_composition(native_request, plugins=provider.plugin_catalog)
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
    if model.id == _SIMPLE_AERO_MODEL_ID:
        return _execute_simple_aero_batch(provider, model, request)
    if model.id == _DUAL_LAUNCH_MODEL_ID:
        return _execute_dual_launch_batch(provider, model, request)
    _validate_selected_deployment_bindings(model, request)
    composition = compile_prepared_vehicle_composition(
        provider,
        request.prepared_configuration,
        deployment_bindings=request.deployment_bindings,
    )
    realization_id = request.prepared_configuration.configuration.realization_id or composition.fidelity
    _require_registered_operation(model, composition.mission, composition.fidelity, realization_id, "batch")

    try:
        with TemporaryDirectory(prefix="taoryx-mission-composition-") as temporary:
            batch = execute_vehicle_composition_batch(
                composition,
                Path(temporary),
                plugins=provider.plugin_catalog,
            )
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


def _validate_selected_deployment_bindings(
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
) -> None:
    """Require a common-run child selection to match parent advertisement.

    The portable configuration tree selects the parent model.  The run request
    carries optional, independently installed children so the same prepared
    parent configuration can be run with or without a deployment.  A binding
    is never ignored: its deployment, event, external ownership, and child
    model must agree with the selected parent's public metadata.
    """

    if not request.deployment_bindings:
        return
    by_id = {item.id: item for item in model.deployments}
    for binding in request.deployment_bindings:
        deployment = by_id.get(binding.deployment_id)
        if deployment is None:
            raise _execution_error(
                "deployment-not-advertised",
                f"Model {model.id!r} does not advertise deployment {binding.deployment_id!r}.",
                phase="preflight",
                category="invalid_request",
                model_id=model.id,
            )
        if deployment.status != "available" or "batch" not in deployment.operations:
            raise _execution_error(
                "deployment-not-executable",
                f"Deployment {binding.deployment_id!r} is not available for common batch execution.",
                phase="preflight",
                category="unavailable",
                model_id=model.id,
            )
        if binding.release_event_id not in deployment.trigger_event_kinds:
            raise _execution_error(
                "deployment-event-not-advertised",
                f"Deployment {binding.deployment_id!r} does not advertise release event {binding.release_event_id!r}.",
                phase="preflight",
                category="invalid_request",
                model_id=model.id,
            )
        if deployment.child_model_scope != "external":
            raise _execution_error(
                "deployment-binding-not-external",
                f"Deployment {binding.deployment_id!r} does not accept a cross-plug-in child binding.",
                phase="preflight",
                category="invalid_request",
                model_id=model.id,
            )
        advertised_child = (
            deployment.child_plugin_id,
            deployment.child_runtime_id,
            deployment.child_model_id,
        )
        selected_child = (
            binding.child.plugin_id,
            binding.child.runtime_id,
            binding.child.model_id,
        )
        if selected_child != advertised_child:
            raise _execution_error(
                "deployment-child-not-advertised",
                f"Deployment {binding.deployment_id!r} requires child {advertised_child!r}, not {selected_child!r}.",
                phase="preflight",
                category="invalid_request",
                model_id=model.id,
            )
    ####


def _execute_simple_aero_batch(
    provider: RegistryMissionCompositionProvider,
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
) -> MissionCompositionTrajectoryResult:
    """Forward the legacy aggregate route to the package-owned implementation."""

    from taoryx_simple_aero.mission_composition import execute_simple_aero_batch

    return execute_simple_aero_batch(
        request,
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        model=model,
        validate_configuration=provider.validate_configuration,
    )
    ####


def _execute_dual_launch_batch(
    provider: RegistryMissionCompositionProvider,
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
) -> MissionCompositionTrajectoryResult:
    """Forward the legacy aggregate route to the package-owned implementation."""

    from taoryx_dual_launch.mission_composition import execute_dual_launch_batch

    return execute_dual_launch_batch(
        request,
        provider_id=provider.metadata.id,
        provider_version=provider.metadata.version,
        model=model,
        validate_configuration=provider.validate_configuration,
    )
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
    deployment_children = getattr(batch.execution, "deployment_children", ())
    if deployment_children:
        if not isinstance(deployment_children, tuple) or not all(isinstance(item, DeploymentChildExecution) for item in deployment_children):
            raise _execution_error(
                "deployment-child-receipt-invalid",
                "The native executor returned an invalid cross-plug-in child receipt collection.",
                phase="projection",
                category="provider_error",
                model_id=model.id,
            )
        available_children = len(deployment_children)
        if request.output.include_spawned_objects:
            remaining_children = available_children
            if request.output.maximum_objects is not None:
                remaining_children = max(request.output.maximum_objects - len(objects), 0)
            for index, child in enumerate(deployment_children[:remaining_children], start=1):
                child_object, event, relationship = _deployment_child_result(
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
            if remaining_children < available_children:
                diagnostics.append(
                    MissionCompositionDiagnostic(
                        severity="warning",
                        code="spawned-object-limit-applied",
                        message="Cross-plug-in deployment children were omitted by maximum_objects.",
                        phase="projection",
                        recoverability="degraded",
                        provider_id=provider.metadata.id,
                        model_id=model.id,
                        details={
                            "available_children": available_children,
                            "returned_children": remaining_children,
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
    bindings = _passive_child_bindings(request, description_prefix="Spawned-child")
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


def _deployment_child_result(
    model: TrajectoryModelMetadata,
    request: MissionCompositionRunRequest,
    composition: CompiledVehicleComposition,
    parent_object_id: str,
    child: DeploymentChildExecution,
    index: int,
) -> tuple[TrajectoryObject, TrajectoryEvent, TrajectoryEntityRelationship]:
    """Project one independently installed child receipt into common lineage."""

    deployment = next((item for item in model.deployments if item.id == child.deployment_id), None)
    if deployment is None or deployment.child_model_scope != "external":
        raise _execution_error(
            "deployment-child-not-advertised",
            f"The native executor returned an unadvertised external child for deployment {child.deployment_id!r}.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    advertised_child = (
        deployment.child_plugin_id,
        deployment.child_runtime_id,
        deployment.child_model_id,
    )
    returned_child = (
        child.runtime_plugin_id,
        child.runtime_id,
        child.child_model_id,
    )
    if returned_child != advertised_child:
        raise _execution_error(
            "deployment-child-identity-mismatch",
            f"Deployment {child.deployment_id!r} returned child {returned_child!r}, expected {advertised_child!r}.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    parent_identity = (
        child.parent_model_id,
        child.parent_family_id,
        child.parent_composition_id,
    )
    expected_parent_identity = (model.id, composition.family_id, composition.id)
    if parent_identity != expected_parent_identity:
        raise _execution_error(
            "deployment-child-parent-mismatch",
            f"Deployment child receipt names parent {parent_identity!r}, expected {expected_parent_identity!r}.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    rows = tuple(dict(item) for item in child.telemetry)
    if not rows:
        raise _execution_error(
            "deployment-child-telemetry-missing",
            f"Deployment child {child.child_object_id!r} returned no telemetry.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    sampled_rows = _sample_rows(rows, request.output.cadence_s, request.output.maximum_samples_per_object)
    bindings = _passive_child_bindings(request, description_prefix="Deployment-child")
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
            segment_instance_id="external-deployment-child",
        )
        for row in sampled_rows
    )
    if not math.isclose(samples[0].time_s, child.release_state.time_s, rel_tol=0.0, abs_tol=1.0e-9):
        raise _execution_error(
            "deployment-child-release-time-mismatch",
            f"Deployment child {child.child_object_id!r} does not begin at its accepted release time.",
            phase="projection",
            category="provider_error",
            model_id=model.id,
        )
    object_id = f"{request.request_id}:deployment-child:{index}:{child.child_object_id}"
    event_id = f"{request.request_id}:deployment:{index}:{child.parent_event_id}"
    status, terminal_disposition = _deployment_child_status(child)
    event = TrajectoryEvent(
        id=event_id,
        time_s=samples[0].time_s,
        category=child.relationship_kind,
        kind=child.parent_event_id,
        object_id=object_id,
        parent_object_id=parent_object_id,
        deployment_id=child.deployment_id,
        detail=f"Spawned externally owned {deployment.child_role} from {model.id}.",
        data={
            "binding_id": child.binding_id,
            "state_transfer": child.state_transfer,
            "runtime_plugin_id": child.runtime_plugin_id,
            "runtime_id": child.runtime_id,
            "requested_fidelity": child.requested_fidelity,
            "realized_fidelity": child.realized_fidelity,
            "source_parent_object_id": child.parent_object_id,
            "termination": child.termination,
        },
    )
    child_object = TrajectoryObject(
        object_id=object_id,
        model_id=child.child_model_id,
        realization_id=child.runtime_id,
        name=child.child_model_id,
        role=deployment.child_role,
        fidelity=child.realized_fidelity,
        status=status,
        parent_object_id=parent_object_id,
        deployment_id=child.deployment_id,
        spawn_event_id=event_id,
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=terminal_disposition,
        channels=channel_metadata,
        samples=samples,
        segments=(
            TrajectorySegmentResult(
                id="external_deployment_child",
                instance_id="external-deployment-child",
                object_id=object_id,
                start_time_s=samples[0].time_s,
                end_time_s=samples[-1].time_s,
                status=status,
            ),
        )
        if request.output.include_segments
        else (),
        provenance=(f"cross-plug-in runtime {child.runtime_plugin_id}/{child.runtime_id}; {child.provenance.get('child_fixture', 'provider-defined child')}"),
        claim_boundary=child.claim_boundary,
    )
    relationship = TrajectoryEntityRelationship(
        id=f"{request.request_id}:deployment-relationship:{index}",
        kind=child.relationship_kind,
        parent_object_id=parent_object_id,
        child_object_id=object_id,
        deployment_id=child.deployment_id,
        event_id=event_id,
        created_at_s=samples[0].time_s,
        state_transfer=child.state_transfer,
        initial_state=TrajectoryStateSnapshot(time_s=samples[0].time_s, values=samples[0].values),
        provenance=(f"parent deployment binding {child.binding_id}; child runtime {child.runtime_plugin_id}/{child.runtime_id}"),
    )
    return child_object, event, relationship
    ####


def _deployment_child_status(child: DeploymentChildExecution) -> tuple[Literal["completed", "terminated", "failed"], str]:
    """Map portable child receipt status into the common trajectory taxonomy."""

    if child.status == "invalid":
        return ("failed", child.termination)
    if child.status == "terminated" or child.termination == "horizon":
        return ("terminated", child.termination)
    return ("completed", child.termination)
    ####


def _passive_child_bindings(
    request: MissionCompositionRunRequest,
    *,
    description_prefix: str,
) -> tuple[NativeOutputBinding, ...]:
    """Return the standardized local passive-child output surface.

    Both legacy spawned bodies and the independent passive-body plug-in emit
    this deliberately small local-frame projection. It keeps core state
    available in every output mode while applying the caller's normal
    telemetry selection to optional mass, aerodynamics, and tumble-rate data.
    """

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
        NativeOutputBinding(
            identifier,
            label,
            f"{description_prefix} {label.casefold()}.",
            "core_state",
            quantity,
            unit,
            frame,
            (path,),
            (),
            (),
        )
        for identifier, label, quantity, unit, frame, path in core_specs
    ]
    selected_groups = set(request.output.telemetry_groups)
    selected_ids = set(request.output.channels)
    for identifier, label, quantity, unit, frame, path, group in telemetry_specs:
        include_telemetry = request.output.mode == "all" or (request.output.mode == "selected" and (identifier in selected_ids or group in selected_groups))
        if include_telemetry:
            bindings.append(
                NativeOutputBinding(
                    identifier,
                    label,
                    f"{description_prefix} {label.casefold()}.",
                    "telemetry",
                    quantity,
                    unit,
                    frame,
                    (path,),
                    (),
                    (),
                    group,
                )
            )
    return tuple(bindings)
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
