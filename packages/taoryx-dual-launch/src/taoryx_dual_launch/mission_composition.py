"""Focused Mission Composition provider for the dual-launch glider workflow.

This package owns the source-shaped point-mass lowering and result projection.
The legacy aggregate can call :func:`execute_dual_launch_batch` with its own
provider identity, but it does not retain this model's implementation or data.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Iterable, Mapping, Sequence
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any, Literal, cast

from taoryx.trajectory.dual_launch import render_dual_launch_problems
from taoryx.trajectory.dual_launch_mission_composition import (
    DUAL_LAUNCH_MODEL_ID,
    build_dual_launch_prepared_case,
    dual_launch_configuration_schema,
    dual_launch_model_metadata,
)

from taoryx.outputs import RunArtifact, RunLifecycleEvent, TelemetryChannel, VehicleTelemetry
from taoryx.trajectory.configuration_contract import (
    ConfigurationContractError,
    PreparedTrajectoryConfiguration,
    TrajectoryConfigurationInstance,
    TrajectoryConfigurationSchema,
    TrajectoryModelMetadata,
    TrajectoryOutputSchema,
    TrajectoryProviderMetadata,
    TrajectoryProviderPresentationMetadata,
    validate_configuration_instance,
)
from taoryx.trajectory.execution_contract import (
    MissionCompositionDiagnostic,
    MissionCompositionExecutionError,
    MissionCompositionRunnerRegistry,
    MissionCompositionRunRequest,
    MissionCompositionTrajectoryResult,
)
from taoryx.trajectory.runtime_mission_composition import (
    RuntimeArtifactProjection,
    trajectory_result_from_run_artifact,
)

if TYPE_CHECKING:
    from taoryx.plugins.discovery import PluginCatalog


DUAL_LAUNCH_PROVIDER_ID = "taoryx.dual-launch.mission-composition"
_FIDELITY_ID = "point_mass_3dof"
_REALIZATION_ID = "generated_native_problem"
LaunchMode = Literal["air_release", "attached_booster"]


class DualLaunchMissionCompositionProvider:
    """One-model focused provider for the installed dual-launch workflow."""

    def __init__(
        self,
        *,
        provider_version: str,
        plugin_catalog: PluginCatalog | None = None,
    ) -> None:
        """Construct the package-owned advertisement only after selection."""

        if not provider_version.strip():
            raise ValueError("Dual-launch provider version must not be empty")
        self._plugin_catalog = plugin_catalog
        self._schema = dual_launch_configuration_schema()
        self._model = dual_launch_model_metadata(self._schema)
        self._metadata = TrajectoryProviderMetadata(
            id=DUAL_LAUNCH_PROVIDER_ID,
            name="TAORYX Dual-Launch Mission Composition",
            version=provider_version,
            description=("Source-generated point-mass dual-launch glider workflow with air-release and attached-booster configuration forms."),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="Dual-Launch Glider",
                short_name="Dual Launch",
                summary="Synthetic point-mass launch-form and waypoint workflow with an event-only separation boundary.",
                organization="TAORYX",
                categories=("Trajectory Workflow", "Low Fidelity", "Synthetic"),
            ),
            status="runnable_workflow",
            tags=("dual-launch", "workflow", "point-mass", "core-replay-session", "synthetic"),
            execution_contract="taoryx.dual-launch.common-batch/v1",
            model_count=1,
            provenance=("packages/taoryx-dual-launch; verification/dual_launch_family_catalog.yaml; src/taoryx/trajectory/dual_launch.py"),
            claim_boundary=(
                "This provider owns a synthetic source-generated point-mass workflow. Core exposes its exact batch "
                "result through a read-only session, but it does not qualify a glider, booster, controller, "
                "higher-fidelity dynamics, live state-responsive stepping, or independently propagated child."
            ),
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return the focused provider identity and package version."""

        return self._metadata
        ####

    @property
    def plugin_catalog(self) -> PluginCatalog | None:
        """Expose the selected catalog without widening discovery during execution."""

        return self._plugin_catalog
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Advertise only the package-owned dual-launch model."""

        return (self._model,)
        ####

    def model(self, model_id: str) -> TrajectoryModelMetadata:
        """Resolve the package's single stable model identity."""

        if model_id != DUAL_LAUNCH_MODEL_ID:
            raise KeyError(f"unknown dual-launch model {model_id!r}")
        return self._model
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return the complete launch-form grammar for the selected model."""

        self.model(model_id)
        return self._schema
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return the normalized state, resource, control, and event surface."""

        return self.model(model_id).output_schema
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate exactly one executable launch form without inferring unsupported modes."""

        if configuration.model_id != DUAL_LAUNCH_MODEL_ID:
            raise ConfigurationContractError(
                "unknown-model",
                f"expected dual-launch model {DUAL_LAUNCH_MODEL_ID!r}",
                path="configuration.model_id",
            )
        prepared = validate_configuration_instance(self._schema, configuration)
        self._validate_selection(configuration)
        return prepared
        ####

    def build_model_default_configuration(
        self,
        model_id: str,
        *,
        configuration_id: str,
    ) -> TrajectoryConfigurationInstance:
        """Return the package-owned air-release onboarding configuration."""

        self.model(model_id)
        from taoryx.trajectory.dual_launch_mission_composition import (
            build_dual_launch_example_configuration,
        )

        return build_dual_launch_example_configuration(schema=self._schema).model_copy(
            update={"configuration_id": configuration_id}
        )
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Register only the declared point-mass batch executor."""

        return MissionCompositionRunnerRegistry({(self.metadata.id, DUAL_LAUNCH_MODEL_ID): self._execute_batch})
        ####

    def _execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Execute this package's source-generated batch route."""

        return execute_dual_launch_batch(
            request,
            provider_id=self.metadata.id,
            provider_version=self.metadata.version,
            model=self._model,
            validate_configuration=self.validate_configuration,
        )
        ####

    def _validate_selection(self, configuration: TrajectoryConfigurationInstance) -> None:
        """Keep fidelity, realization, and mission selection inside the runnable matrix."""

        if configuration.fidelity != _FIDELITY_ID:
            raise ConfigurationContractError(
                "fidelity-unavailable",
                f"Dual launch supports only {_FIDELITY_ID!r} in its runnable focused provider.",
                path="configuration.fidelity",
            )
        if configuration.realization_id not in {None, _REALIZATION_ID}:
            raise ConfigurationContractError(
                "unknown-realization",
                f"Dual launch supports only realization {_REALIZATION_ID!r}",
                path="configuration.realization_id",
            )
        mission_id = configuration.mission_template_id
        mission = next((item for item in self._model.mission_templates if item.id == mission_id), None)
        if mission is None:
            raise ConfigurationContractError(
                "unknown-mission-template",
                f"expected one of {sorted(item.id for item in self._model.mission_templates)!r}",
                path="configuration.mission_template_id",
            )
        if _FIDELITY_ID not in mission.compatible_fidelities:
            raise ConfigurationContractError(
                "mission-fidelity-incompatible",
                f"mission {mission.id!r} does not support {_FIDELITY_ID!r}",
                path="configuration.mission_template_id",
            )
        if mission.id not in self._model.realizations[0].mission_template_ids:
            raise ConfigurationContractError(
                "realization-mission-incompatible",
                f"realization {_REALIZATION_ID!r} does not support mission {mission.id!r}",
                path="configuration.realization_id",
            )
        ####

    ####


def execute_dual_launch_batch(
    request: MissionCompositionRunRequest,
    *,
    provider_id: str,
    provider_version: str,
    model: TrajectoryModelMetadata,
    validate_configuration: Callable[[TrajectoryConfigurationInstance], PreparedTrajectoryConfiguration],
) -> MissionCompositionTrajectoryResult:
    """Run the package-owned lowering and project its normal batch result.

    The compatibility aggregate may call this function with its own provider
    identity. The schema, lowering, runtime source problem, and output
    projection remain solely in the dual-launch package.
    """

    if request.provider_id != provider_id or request.provider_version != provider_version:
        raise _execution_error(
            "provider-version-mismatch",
            "The request provider identity does not match this dual-launch provider.",
            phase="preflight",
            category="invalid_request",
            model_id=model.id,
        )
    if request.model_id != DUAL_LAUNCH_MODEL_ID:
        raise _execution_error(
            "executor-model-mismatch",
            "The dual-launch executor received a request for another model.",
            phase="preflight",
            category="provider_error",
            model_id=request.model_id,
        )
    if request.operation != "batch":
        raise _execution_error(
            "operation-not-supported",
            "Dual launch has no interactive Mission Composition session binding.",
            phase="preflight",
            category="unsupported",
            model_id=model.id,
        )

    prepared = validate_configuration(request.prepared_configuration.configuration)
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
    realization_id = configuration.realization_id or _REALIZATION_ID
    if mission_id is None:
        raise _execution_error(
            "mission-template-required",
            "Dual-launch native execution requires one exact launch-form mission template.",
            phase="configuration",
            category="invalid_request",
            model_id=model.id,
        )
    _require_available_batch(model, mission_id, configuration.fidelity, realization_id)
    case = build_dual_launch_prepared_case(prepared)
    raw_launch_mode = case.extensions.get("launch_mode")
    if raw_launch_mode not in {"air_release", "attached_booster"}:
        raise _execution_error(
            "dual-launch-mode-unresolved",
            "The resolved dual-launch case did not declare a supported launch mode.",
            phase="configuration",
            category="provider_error",
            model_id=model.id,
        )
    launch_mode = cast(LaunchMode, raw_launch_mode)
    try:
        from taoryx.language.grammar_contracts import GrammarProfile
        from taoryx.runtime.runner import run_files

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
            provider_id=provider_id,
            provider_version=provider_version,
            request_id=request.request_id,
            configuration_fingerprint=prepared.fingerprint,
            primary_model_id=model.id,
            primary_object_id=object_id,
            default_fidelity=_FIDELITY_ID,
            default_realization_id=realization_id,
            operation="batch",
            status="completed",
            model_ids={object_id: model.id},
            roles={object_id: "primary_vehicle"},
            object_statuses={object_id: "completed"},
            provenance="package-owned generated dual-launch point-mass problem executed by Simulation Runtime",
            claim_boundary=model.claim_boundary,
        ),
        output_schema=model.output_schema,
        output_selection=request.output,
    )
    ####


def _require_available_batch(
    model: TrajectoryModelMetadata,
    mission_id: str,
    fidelity: str,
    realization_id: str,
) -> None:
    """Require the exact advertised package batch tuple before execution."""

    mission = next((item for item in model.mission_templates if item.id == mission_id), None)
    operation = (
        next(
            (item for item in mission.operations if item.fidelity == fidelity and item.realization_id in {None, realization_id} and item.operation == "batch"),
            None,
        )
        if mission is not None
        else None
    )
    if operation is None or operation.status != "available" or operation.common_runner_status != "registered":
        blockers = () if operation is None else operation.blockers
        raise _execution_error(
            "operation-not-available",
            f"Dual launch has no available common batch operation for {mission_id!r}/{fidelity!r}/{realization_id!r}.",
            phase="preflight",
            category="unavailable",
            model_id=model.id,
            details={"blockers": list(blockers)},
        )
    ####


def _dual_launch_artifact(
    artifact: RunArtifact,
    request: MissionCompositionRunRequest,
    model: TrajectoryModelMetadata,
    launch_mode: LaunchMode,
) -> RunArtifact:
    """Project the source primary history onto the advertised output schema."""

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
        channel_id: _optional_channel_values(_dual_launch_native_values(source, native_id, indexes)) for channel_id, native_id in native_ids.items()
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
    events = [
        RunLifecycleEvent.model_validate(_dual_launch_event(payload, object_id, launch_mode)) for payload in artifact.events if payload.get("vehicle") == "1"
    ]
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
    """Widen committed finite samples to the optional telemetry value contract."""

    return [float(value) for value in values]
    ####


def _finite_difference(times: Sequence[float], values: Sequence[float]) -> list[float]:
    """Differentiate a sampled state without presenting it as a native channel."""

    if len(times) != len(values) or len(times) < 2:
        raise ValueError("finite-difference projection requires at least two aligned samples")
    result: list[float] = []
    for index, _value in enumerate(values):
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


def _sample_indices(
    times: Sequence[float],
    cadence_s: float | None,
    maximum_samples: int | None,
) -> tuple[int, ...]:
    """Apply the common output sampling selection to source runtime samples."""

    if not times:
        raise ValueError("cannot sample an empty dual-launch trajectory")
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


def _dual_launch_event(
    payload: Mapping[str, object],
    object_id: str,
    launch_mode: LaunchMode,
) -> dict[str, object]:
    """Scope source events to the primary and name only the real handoff."""

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


def _execution_error(
    code: str,
    message: str,
    *,
    phase: Literal["discovery", "configuration", "preflight", "execution", "projection"],
    category: Literal["invalid_request", "unsupported", "unavailable", "execution_failed", "provider_error"],
    model_id: str,
    details: dict[str, Any] | None = None,
) -> MissionCompositionExecutionError:
    """Build one package-authored structured error for the common runner."""

    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message or "Dual-launch Mission Composition execution failed.",
            phase=phase,
            recoverability="correctable" if category in {"invalid_request", "unsupported"} else "fatal",
            model_id=model_id,
            details=details or {},
        ),
        category=category,
    )
    ####


__all__ = [
    "DUAL_LAUNCH_PROVIDER_ID",
    "DualLaunchMissionCompositionProvider",
    "execute_dual_launch_batch",
]
