"""Focused Mission Composition provider for the Simple Aero workflow.

The Simple Aero package owns this non-physical point-mass workflow end to end:
its configuration schema, generated-problem batch execution, and the existing
generated-command/direct-throttle session.  The compatibility aggregate may
continue to expose the historical model separately, but focused callers do not
need that aggregate to discover or execute this workflow.
"""

from __future__ import annotations

import math
from collections.abc import Callable
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any, Literal, cast

from taoryx.language.grammar_contracts import GrammarProfile
from taoryx.outputs import RunArtifact, TelemetryChannel, VehicleTelemetry
from taoryx.runtime.runner import run_files
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


SIMPLE_AERO_PROVIDER_ID = "taoryx.simple-aero.mission-composition"
SIMPLE_AERO_MODEL_ID = "simple_aero"
_FIDELITY_ID = "point_mass_3dof"
_REALIZATION_ID = "fixed_ld_point_mass"


class SimpleAeroMissionCompositionProvider:
    """One-model focused provider for the installed Simple Aero workflow."""

    def __init__(
        self,
        *,
        provider_version: str,
        plugin_catalog: PluginCatalog | None = None,
    ) -> None:
        """Construct the lightweight public model surface on first selection."""

        if not provider_version.strip():
            raise ValueError("Simple Aero provider version must not be empty")
        from taoryx.trajectory.simple_aero_mission_composition import (
            simple_aero_configuration_schema,
            simple_aero_model_metadata,
        )

        self._plugin_catalog = plugin_catalog
        self._schema = simple_aero_configuration_schema()
        self._model = simple_aero_model_metadata(self._schema)
        self._metadata = TrajectoryProviderMetadata(
            id=SIMPLE_AERO_PROVIDER_ID,
            name="TAORYX Simple Aero Mission Composition",
            version=provider_version,
            description=(
                "Package-owned fixed-L/D point-mass workflow with generated batch commands and a bounded "
                "direct-throttle session mode."
            ),
            presentation=TrajectoryProviderPresentationMetadata(
                display_name="Simple Aero",
                short_name="Simple Aero",
                summary="Synthetic fixed-L/D point-mass workflow for low-fidelity composition and UI integration.",
                organization="TAORYX",
                categories=("Trajectory Workflow", "Low Fidelity", "Synthetic"),
            ),
            status="runnable_workflow",
            tags=("simple-aero", "workflow", "point-mass", "fixed-ld", "synthetic"),
            execution_contract="taoryx.simple-aero.common-batch/v1",
            model_count=1,
            provenance=(
                "packages/taoryx-simple-aero; verification/alpha2_family_catalog.yaml; "
                "verification/simple_aero_segment_catalog.yaml"
            ),
            claim_boundary=(
                "This provider owns a synthetic fixed-L/D workflow, not a physical vehicle, controller qualification, "
                "or flight-performance claim. Generated bank is telemetry-only in the interactive point-mass session."
            ),
        )
        ####

    @property
    def metadata(self) -> TrajectoryProviderMetadata:
        """Return the versioned focused provider identity."""

        return self._metadata
        ####

    @property
    def plugin_catalog(self) -> PluginCatalog | None:
        """Expose the selected catalog without widening discovery during execution."""

        return self._plugin_catalog
        ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        """Advertise only the package-owned workflow model."""

        return (self._model,)
        ####

    def model(self, model_id: str) -> TrajectoryModelMetadata:
        """Resolve the single focused workflow model."""

        if model_id != SIMPLE_AERO_MODEL_ID:
            raise KeyError(f"unknown Simple Aero model {model_id!r}")
        return self._model
        ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        """Return the complete package-owned configuration grammar."""

        self.model(model_id)
        return self._schema
        ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        """Return the declared normalized output surface."""

        return self.model(model_id).output_schema
        ####

    def validate_configuration(
        self,
        configuration: TrajectoryConfigurationInstance,
    ) -> PreparedTrajectoryConfiguration:
        """Validate one exact fixed-L/D workflow selection and its typed values."""

        if configuration.model_id != SIMPLE_AERO_MODEL_ID:
            raise ConfigurationContractError(
                "unknown-model",
                f"expected Simple Aero model {SIMPLE_AERO_MODEL_ID!r}",
                path="configuration.model_id",
            )
        prepared = validate_configuration_instance(self._schema, configuration)
        self._validate_selection(configuration)
        return prepared
        ####

    def open_session_episode(
        self,
        prepared: PreparedTrajectoryConfiguration,
        *,
        seed: int | None = None,
        integration_step_s: float = 0.02,
    ) -> object:
        """Open the advertised generated-command/direct-throttle point-mass session."""

        validated = self._validate_prepared(prepared)
        from taoryx.trajectory.simple_aero_mission_composition import open_simple_aero_session_episode

        return open_simple_aero_session_episode(
            self._model,
            validated,
            seed=seed,
            integration_step_s=integration_step_s,
        )
        ####

    def build_runner(self) -> MissionCompositionRunnerRegistry:
        """Register the package-owned normal batch path under this provider ID."""

        return MissionCompositionRunnerRegistry(
            {(self.metadata.id, SIMPLE_AERO_MODEL_ID): self._execute_batch}
        )
        ####

    def _execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        """Execute exactly this provider's fixed-L/D batch workflow."""

        return execute_simple_aero_batch(
            request,
            provider_id=self.metadata.id,
            provider_version=self.metadata.version,
            model=self._model,
            validate_configuration=self.validate_configuration,
        )
        ####

    def _validate_prepared(self, prepared: PreparedTrajectoryConfiguration) -> PreparedTrajectoryConfiguration:
        """Reject a prepared configuration from another schema revision or model."""

        validated = self.validate_configuration(prepared.configuration)
        if validated.fingerprint != prepared.fingerprint:
            raise ConfigurationContractError(
                "prepared-configuration-stale",
                "The prepared Simple Aero configuration no longer matches this provider schema.",
                path="prepared_configuration",
            )
        return validated
        ####

    def _validate_selection(self, configuration: TrajectoryConfigurationInstance) -> None:
        """Keep realization, fidelity, and template selection inside the advertised matrix."""

        if configuration.fidelity != _FIDELITY_ID:
            raise ConfigurationContractError(
                "fidelity-unavailable",
                f"Simple Aero supports only {_FIDELITY_ID!r} in its runnable focused provider.",
                path="configuration.fidelity",
            )
        if configuration.realization_id not in {None, _REALIZATION_ID}:
            raise ConfigurationContractError(
                "unknown-realization",
                f"Simple Aero supports only realization {_REALIZATION_ID!r}",
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


def execute_simple_aero_batch(
    request: MissionCompositionRunRequest,
    *,
    provider_id: str,
    provider_version: str,
    model: TrajectoryModelMetadata,
    validate_configuration: Callable[[TrajectoryConfigurationInstance], PreparedTrajectoryConfiguration],
) -> MissionCompositionTrajectoryResult:
    """Run the package-owned Simple Aero lowering and project its normal result.

    The compatibility aggregate calls this function too, so the only retained
    aggregate behavior is an alternate provider identity.  The source-shaped
    lowering, runtime artifact selection, and output projection remain owned
    by this package.
    """

    if request.provider_id != provider_id or request.provider_version != provider_version:
        raise _execution_error(
            "provider-version-mismatch",
            "The request provider identity does not match this Simple Aero provider.",
            phase="preflight",
            category="invalid_request",
            model_id=model.id,
        )
    if request.model_id != SIMPLE_AERO_MODEL_ID:
        raise _execution_error(
            "executor-model-mismatch",
            "The Simple Aero executor received a request for another model.",
            phase="preflight",
            category="provider_error",
            model_id=request.model_id,
        )
    if request.operation != "batch":
        raise _execution_error(
            "operation-not-supported",
            "Use the stateful Simple Aero session contract for interactive execution.",
            phase="preflight",
            category="unsupported",
            model_id=model.id,
        )

    prepared = validate_configuration(request.prepared_configuration.configuration)
    if prepared.fingerprint != request.prepared_configuration.fingerprint:
        raise _execution_error(
            "prepared-configuration-stale",
            "The prepared Simple Aero configuration no longer matches the provider schema.",
            phase="configuration",
            category="invalid_request",
            model_id=model.id,
        )
    from taoryx.trajectory.simple_aero_mission_composition import build_simple_aero_prepared_configuration

    build = build_simple_aero_prepared_configuration(prepared)
    mission_id = request.prepared_configuration.configuration.mission_template_id or "fixed_ld_baseline"
    realization_id = request.prepared_configuration.configuration.realization_id or _REALIZATION_ID
    _require_available_batch(model, mission_id, _FIDELITY_ID, realization_id)
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
            provenance="package-owned generated Simple Aero problem executed by Simulation Runtime",
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
    """Require the exact advertised common batch operation before execution."""

    mission = next((item for item in model.mission_templates if item.id == mission_id), None)
    operation = (
        next(
            (
                item
                for item in mission.operations
                if item.fidelity == fidelity and item.realization_id == realization_id and item.operation == "batch"
            ),
            None,
        )
        if mission is not None
        else None
    )
    if operation is None or operation.status != "available" or operation.common_runner_status != "registered":
        raise _execution_error(
            "operation-not-advertised",
            f"Simple Aero has no available common batch operation for {mission_id!r}/{fidelity!r}/{realization_id!r}.",
            phase="preflight",
            category="unavailable",
            model_id=model.id,
        )
    ####


def _simple_aero_artifact(
    artifact: RunArtifact,
    request: MissionCompositionRunRequest,
    vehicle_name: str,
    model: TrajectoryModelMetadata,
) -> RunArtifact:
    """Project exact Simple Aero runtime channels into the advertised names."""

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
                "angle"
                if descriptor.interpolation == "periodic"
                else cast(Literal["linear", "angle", "step", "slerp", "event"], descriptor.interpolation)
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


def _sample_indices(
    times: list[float],
    cadence_s: float | None,
    maximum_samples: int | None,
) -> list[int]:
    """Preserve runtime sampling while honoring the common output selection."""

    if not times:
        raise ValueError("Simple Aero runtime artifact has no samples")
    indexes = list(range(len(times)))
    if cadence_s is not None:
        selected = [0]
        next_time = times[0] + cadence_s
        for index, time_s in enumerate(times[1:], start=1):
            if time_s + 1.0e-12 >= next_time:
                selected.append(index)
                next_time = time_s + cadence_s
        if selected[-1] != len(times) - 1:
            selected.append(len(times) - 1)
        indexes = selected
    if maximum_samples is not None and len(indexes) > maximum_samples:
        if maximum_samples == 1:
            return [indexes[-1]]
        stride = (len(indexes) - 1) / (maximum_samples - 1)
        indexes = [indexes[round(index * stride)] for index in range(maximum_samples)]
    return indexes
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
    """Build one provider-authored structured failure for the common runner."""

    return MissionCompositionExecutionError(
        MissionCompositionDiagnostic(
            severity="error",
            code=code,
            message=message,
            phase=phase,
            recoverability="correctable" if category in {"invalid_request", "unsupported"} else "fatal",
            model_id=model_id,
            details=details or {},
        ),
        category=category,
    )
    ####


__all__ = [
    "SIMPLE_AERO_MODEL_ID",
    "SIMPLE_AERO_PROVIDER_ID",
    "SimpleAeroMissionCompositionProvider",
    "execute_simple_aero_batch",
]
