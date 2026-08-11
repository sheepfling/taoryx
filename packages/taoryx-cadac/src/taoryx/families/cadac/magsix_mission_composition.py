"""Taoryx Mission Composition bridge for the source-compatible MAGSIX trajectory plug-in."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

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

from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import blocked_control_advertisement, source_managed_control_advertisement
from .magsix import MagsixRunResult, MagsixSample
from .magsix_plugin import MagsixPluginOverrides, MagsixVehiclePlugin
from .output_metadata import cadac_output_quantity

MAGSIX_MODEL_ID = "cadac.magsix.vehicle"
MAGSIX_MODEL_VERSION = "0.5.0"
MAGSIX_TRAJECTORY_FIDELITY_ID = "point_mass_3dof"
MAGSIX_ATTITUDE_FIDELITY_ID = "pseudo_6dof"
MAGSIX_TRAJECTORY_REALIZATION_ID = "cadac-source-phase.trajectory_only"
MAGSIX_ATTITUDE_REALIZATION_ID = "cadac-source-phase.restricted_attitude"
MAGSIX_TRAJECTORY_PHASE_ID = "trajectory_only"
MAGSIX_ATTITUDE_PHASE_ID = "restricted_attitude"
MAGSIX_MISSION_ID = "trajectory_only"
MAGSIX_EXECUTOR_ID = "cadac.magsix.trajectory.batch"
_LOCAL_FRAME_ID = "cadac.magsix.local_ned"


class CadacMagsixMissionCompositionProvider:
    """Self-describing provider for one installed MAGSIX trajectory-only source case."""

    def __init__(self, plugin: MagsixVehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish MAGSIX provider with an incomplete installation: " + "; ".join(blockers))
        ####
        self._plugin = plugin
        self._schema = _build_configuration_schema(plugin)
        self._output_schema = _build_output_schema()
        self._model = _build_model_metadata(self._schema, self._output_schema)

    ####

    def list_models(self) -> tuple[TrajectoryModelMetadata, ...]:
        return (self._model,)

    ####

    def get_model_schema(self, model_id: str) -> TrajectoryConfigurationSchema:
        if model_id != MAGSIX_MODEL_ID:
            raise KeyError(f"unknown MAGSIX model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != MAGSIX_MODEL_ID:
            raise KeyError(f"unknown MAGSIX model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        configuration = materialize_configuration_defaults(self._schema, configuration)
        prepared = validate_configuration_instance(self._schema, configuration)
        resolved = prepared.resolved
        if not isinstance(resolved, Mapping):
            raise ValueError("MAGSIX resolved configuration root must be a mapping")
        ####
        phase = resolved.get("source_phase")
        if phase == MAGSIX_TRAJECTORY_PHASE_ID:
            if configuration.fidelity != MAGSIX_TRAJECTORY_FIDELITY_ID:
                raise ValueError(f"MAGSIX trajectory-only phase requires fidelity {MAGSIX_TRAJECTORY_FIDELITY_ID!r}")
            ####
            if configuration.realization_id not in {None, MAGSIX_TRAJECTORY_REALIZATION_ID}:
                raise ValueError(f"MAGSIX trajectory-only phase requires realization {MAGSIX_TRAJECTORY_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id not in {None, MAGSIX_MISSION_ID}:
                raise ValueError(f"MAGSIX trajectory-only phase supports only mission template {MAGSIX_MISSION_ID!r}")
            ####
        elif phase == MAGSIX_ATTITUDE_PHASE_ID:
            if configuration.fidelity != MAGSIX_ATTITUDE_FIDELITY_ID:
                raise ValueError(f"MAGSIX restricted-attitude phase requires fidelity {MAGSIX_ATTITUDE_FIDELITY_ID!r}")
            ####
            if configuration.realization_id not in {None, MAGSIX_ATTITUDE_REALIZATION_ID}:
                raise ValueError(f"MAGSIX restricted-attitude phase requires realization {MAGSIX_ATTITUDE_REALIZATION_ID!r}")
            ####
            if configuration.mission_template_id is not None:
                raise ValueError("MAGSIX restricted-attitude validation does not advertise an executable mission template")
            ####
        else:
            raise ValueError(f"unsupported MAGSIX source phase {phase!r}")
        ####
        return prepared

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != MAGSIX_MODEL_ID:
            raise ValueError("MAGSIX executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("MAGSIX prepared configuration does not match provider validation")
        ####
        if prepared.configuration.fidelity != MAGSIX_TRAJECTORY_FIDELITY_ID:
            raise ValueError("MAGSIX restricted-attitude phase is validation-only until the attitude perturbation runtime is installed")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_magsix_configuration(
    provider: CadacMagsixMissionCompositionProvider,
    *,
    configuration_id: str = "magsix-default",
    phase_id: str = MAGSIX_TRAJECTORY_PHASE_ID,
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create trajectory-only or validate-only restricted-attitude configuration from source defaults."""

    if phase_id not in {MAGSIX_TRAJECTORY_PHASE_ID, MAGSIX_ATTITUDE_PHASE_ID}:
        raise ValueError(f"unknown MAGSIX phase {phase_id!r}")
    ####
    supplied = dict(overrides or {})
    routes = {
        "north_m": ("initialization", "north_m"),
        "east_m": ("initialization", "east_m"),
        "altitude_m": ("initialization", "altitude_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "heading_deg": ("initialization", "heading_deg"),
        "flight_path_deg": ("initialization", "flight_path_deg"),
        "spin_rpm": ("initialization", "spin_rpm"),
        "end_time_dnt": ("runtime", "end_time_dnt"),
        "sample_step_dnt": ("runtime", "sample_step_dnt"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown MAGSIX configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    values: dict[str, Any] = {"source_phase": ConfigurationParameterValue(value=phase_id)}
    values.update(
        {
            group_id: ConfigurationGroupValue(
                values={parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in items.items()}
            )
            for group_id, items in grouped.items()
        }
    )
    schema = provider.get_model_schema(MAGSIX_MODEL_ID)
    trajectory_only = phase_id == MAGSIX_TRAJECTORY_PHASE_ID
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=MAGSIX_MODEL_ID,
        model_version=MAGSIX_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID if trajectory_only else MAGSIX_ATTITUDE_FIDELITY_ID,
        realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID if trajectory_only else MAGSIX_ATTITUDE_REALIZATION_ID,
        mission_template_id=MAGSIX_MISSION_ID if trajectory_only else None,
        root=ConfigurationGroupValue(values=values),
    )


####


def register_magsix_mission_composition(
    provider: CadacMagsixMissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model MAGSIX trajectory-only batch executor."""

    registry.register(CADAC_PROVIDER_ID, MAGSIX_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: MagsixVehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    compatible = (MAGSIX_TRAJECTORY_FIDELITY_ID, MAGSIX_ATTITUDE_FIDELITY_ID)
    return TrajectoryConfigurationSchema(
        model_id=MAGSIX_MODEL_ID,
        model_version=MAGSIX_MODEL_VERSION,
        supported_fidelities=compatible,
        root=ConfigurationGroupSchema(
            id="magsix",
            label="MAGSIX source vehicle",
            description="Independent planar trajectory phase with an explicitly classified but validate-only restricted-attitude phase.",
            children=(
                ConfigurationParameterSchema(
                    id="source_phase",
                    label="Source phase",
                    description="Select the independently runnable trajectory or the restricted attitude-perturbation phase.",
                    value_type="enum",
                    required=False,
                    default=MAGSIX_TRAJECTORY_PHASE_ID,
                    default_declared=True,
                    choices=(MAGSIX_TRAJECTORY_PHASE_ID, MAGSIX_ATTITUDE_PHASE_ID),
                    role="variant",
                    compatible_fidelities=compatible,
                    provenance="missiondesignsolutions/CADAC/MAGSIX",
                ),
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter(
                            "north_m",
                            "North",
                            "Initial north displacement in source local-level coordinates.",
                            default=initial.north_m,
                            unit="m",
                            compatible=compatible,
                        ),
                        _parameter(
                            "east_m",
                            "East",
                            "Initial east displacement in source local-level coordinates.",
                            default=initial.east_m,
                            unit="m",
                            compatible=compatible,
                        ),
                        _parameter("altitude_m", "Altitude", "Initial height above sea level.", default=initial.altitude_m, unit="m", compatible=compatible),
                        _parameter("speed_mps", "Speed", "Initial rotor speed.", default=initial.speed_mps, unit="m/s", compatible=compatible),
                        _parameter("heading_deg", "Heading", "Fixed trajectory heading angle.", default=initial.heading_deg, unit="deg", compatible=compatible),
                        _parameter(
                            "flight_path_deg",
                            "Glide angle",
                            "Initial vertical flight-path angle.",
                            default=initial.flight_path_deg,
                            unit="deg",
                            compatible=compatible,
                        ),
                        _parameter("spin_rpm", "Spin", "Initial Magnus-rotor spin rate.", default=initial.spin_rpm, unit="rpm", compatible=compatible),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_dnt",
                            "End time",
                            "Requested source Dynamic Normalized Time horizon.",
                            default=source.end_time_dnt,
                            role="constraint",
                            compatible=compatible,
                        ),
                        _parameter(
                            "sample_step_dnt",
                            "Sample cadence",
                            "Returned source Dynamic Normalized Time cadence.",
                            default=source.plot_step_dnt or source.integration_step_dnt,
                            role="constraint",
                            compatible=compatible,
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary=(
            "The executable phase is exactly the source trajectory-only operation. The restricted-attitude phase is classified as pseudo-6DoF "
            "but remains validation-only and cannot be silently substituted by the T1 runner."
        ),
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    compatible: tuple[str, ...],
    unit: str | None = None,
    role: str = "initialization",
) -> ConfigurationParameterSchema:
    return ConfigurationParameterSchema(
        id=parameter_id,
        label=label,
        description=description,
        canonical_unit=unit,
        display_unit=unit,
        required=False,
        default=default,
        default_declared=True,
        role=role,
        compatible_fidelities=compatible,
        provenance="missiondesignsolutions/CADAC/MAGSIX/input.asc",
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
) -> TrajectoryOutputChannelMetadata:
    return TrajectoryOutputChannelMetadata(
        id=channel_id,
        label=label,
        description=description,
        quantity=cadac_output_quantity(channel_id, unit),
        canonical_unit=unit,
        display_unit=unit,
        shape=shape,
        frame=frame,
        availability="guaranteed",
        compatible_fidelities=(MAGSIX_TRAJECTORY_FIDELITY_ID,),
        compatible_realizations=(MAGSIX_TRAJECTORY_REALIZATION_ID,),
        compatible_mission_templates=(MAGSIX_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC MAGSIX trajectory-only source reconstruction",
        claim_boundary="Available only from the independently runnable source trajectory equations; restricted attitude is not inferred.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_ned_m", "Position", "Local north/east/down trajectory position.", unit="m", shape=(3,), frame=_LOCAL_FRAME_ID),
        _output("velocity_ned_mps", "Velocity", "Local north/east/down trajectory velocity.", unit="m/s", shape=(3,), frame=_LOCAL_FRAME_ID),
    )
    trajectory = (
        _output("altitude_m", "Altitude", "Height above sea level.", unit="m"),
        _output("speed_mps", "Speed", "Rotor trajectory speed.", unit="m/s"),
        _output("heading_deg", "Heading", "Fixed source heading angle.", unit="deg"),
        _output("flight_path_deg", "Glide angle", "Source vertical flight-path angle.", unit="deg"),
        _output("spin_rpm", "Spin", "Magnus-rotor spin rate.", unit="rpm"),
        _output("tip_speed_ratio", "Tip-speed ratio", "Rotor tip-speed ratio."),
        _output("source_time_dnt", "Source time", "Source Dynamic Normalized Time coordinate."),
        _output("dynamic_time_scale_s", "Dynamic time scale", "Current DNT-to-seconds time scale tau.", unit="s"),
    )
    environment = (
        _output("density_kg_m3", "Density", "Source US76 atmospheric density.", unit="kg/m^3"),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source aerodynamic dynamic pressure.", unit="Pa"),
        _output("mach", "Mach", "Rotor Mach number."),
    )
    telemetry = (*trajectory, *environment)
    return TrajectoryOutputSchema(
        model_id=MAGSIX_MODEL_ID,
        model_version=MAGSIX_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="trajectory",
                label="Trajectory",
                description="Planar Magnus-rotor trajectory and spin diagnostics.",
                channel_ids=tuple(item.id for item in trajectory),
            ),
            TrajectoryTelemetryGroupMetadata(
                id="environment",
                label="Environment",
                description="Atmosphere and dynamic-pressure diagnostics.",
                channel_ids=tuple(item.id for item in environment),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="MAGSIX trajectory-only center-of-mass/spin truth; no restricted-attitude channels are emitted.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    trajectory_fidelity = TrajectoryFidelityMetadata(
        id=MAGSIX_TRAJECTORY_FIDELITY_ID,
        label="CADAC MAGSIX trajectory 3-DoF",
        rank=0,
        declared=True,
        dynamics_fidelity="point_mass_3dof",
        input_realization="provider_defined",
        runtime_fidelity="point_mass_3dof",
        control_realization="force_model",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.magsix.trajectory_only",
        blockers=("compiled/source golden trajectory parity is not registered",),
        required_operations=("environment", "trajectory"),
        claim_boundary="Planar trajectory and spin-state equations only; source attitude perturbations are excluded.",
    )
    attitude_fidelity = TrajectoryFidelityMetadata(
        id=MAGSIX_ATTITUDE_FIDELITY_ID,
        label="CADAC MAGSIX restricted attitude",
        rank=1,
        declared=True,
        dynamics_fidelity="pseudo_6dof",
        input_realization="provider_defined",
        runtime_fidelity="pseudo_6dof",
        control_realization="response_law",
        promotion_status="planned",
        operations=("validate",),
        profile_id="cadac.magsix.restricted_attitude",
        blockers=("attitude perturbation equations and one-way trajectory coupling are not yet installed in the Taoryx runtime",),
        required_operations=("environment", "trajectory", "attitude"),
        claim_boundary="Restricted attitude is one-way coupled from trajectory and is not a full rigid-body control realization.",
    )
    trajectory_realization = TrajectoryRealizationMetadata(
        id=MAGSIX_TRAJECTORY_REALIZATION_ID,
        label="MAGSIX trajectory only",
        description="Source-compatible independently runnable planar trajectory and spin equations.",
        status="available",
        dynamics_fidelities=("point_mass_3dof",),
        input_realization="provider_defined",
        controls=source_managed_control_advertisement(
            mission_ids=(MAGSIX_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/MAGSIX",),
            claim_boundary="The MAGSIX source trajectory is internally propagated in batch only.",
        ),
        fidelity_aliases=(MAGSIX_TRAJECTORY_FIDELITY_ID,),
        mission_template_ids=(MAGSIX_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(MAGSIX_EXECUTOR_ID,),
        blockers=(),
        source_refs=("missiondesignsolutions/CADAC/MAGSIX",),
        claim_boundary="Exact trajectory-only source phase; does not synthesize attitude truth.",
    )
    attitude_realization = TrajectoryRealizationMetadata(
        id=MAGSIX_ATTITUDE_REALIZATION_ID,
        label="MAGSIX restricted attitude",
        description="Source trajectory-driven attitude perturbation realization.",
        status="blocked",
        dynamics_fidelities=("pseudo_6dof",),
        input_realization="provider_defined",
        controls=blocked_control_advertisement(
            claim_boundary="The restricted-attitude phase has no installed control runtime.",
        ),
        fidelity_aliases=(MAGSIX_ATTITUDE_FIDELITY_ID,),
        mission_template_ids=(),
        operations=("validate",),
        native_factory_ids=(),
        blockers=("restricted attitude runtime is not installed",),
        source_refs=("missiondesignsolutions/CADAC/MAGSIX/attitude.cpp",),
        claim_boundary="Classified source phase only; no batch execution is advertised.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=MAGSIX_MISSION_ID,
        name="MAGSIX trajectory-only propagation",
        description="Propagate the independently runnable planar Magnus-rotor trajectory until horizon or ground impact.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("trajectory_only",),
        compatible_fidelities=(MAGSIX_TRAJECTORY_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID,
                realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation preserves the source trajectory-only operation.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID,
                realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_compatibility_runtime",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=MAGSIX_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed MAGSIX trajectory-only plug-in.",
            ),
        ),
        provenance="CADAC MAGSIX environment and planar trajectory equations",
        claim_boundary="Trajectory-only source operation; restricted attitude is not part of this mission template.",
    )
    return TrajectoryModelMetadata(
        id=MAGSIX_MODEL_ID,
        name="CADAC MAGSIX Magnus rotor",
        version=MAGSIX_MODEL_VERSION,
        description="CADAC MAGSIX independently runnable planar Magnus-rotor trajectory exposed as a Taoryx plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC MAGSIX",
            short_name="MAGSIX",
            summary="Planar Magnus-rotor trajectory with a separately classified restricted-attitude phase.",
            category="rotor_body",
            subcategory="magnus_rotor",
            sort_key="cadac-magsix",
            badges=("CADAC", "3-DoF", "Magnus rotor", "restricted-6DoF source"),
            default_fidelity_id=MAGSIX_TRAJECTORY_FIDELITY_ID,
            default_mission_template_id=MAGSIX_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "spin_rpm"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="ROTOR",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/MAGSIX",),
                    provenance="CADAC MAGSIX source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=MAGSIX_MODEL_ID,
        physical_family="magnus_rotor",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "magnus_rotor", "point_mass_3dof", "pseudo_6dof_planned"),
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("trajectory_only", "restricted_attitude"),
            termination_modes=("end_time", "ground_impact"),
            operations=("discover", "validate", "batch"),
            supports_custom_segments=False,
            supports_deployment=False,
            supports_staging=False,
            supports_dynamic_child_generation=False,
            supports_multiple_stages=False,
            supports_submodels=False,
        ),
        realizations=(trajectory_realization, attitude_realization),
        mission_templates=(mission,),
        deployments=(),
        reference_frames=(
            TrajectoryReferenceFrameMetadata(
                id=_LOCAL_FRAME_ID,
                name="CADAC MAGSIX local level",
                description="Flat-Earth local north/east/down trajectory coordinates.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source initial local-level reference point",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/MAGSIX/trajectory.cpp",),
                provenance="CADAC MAGSIX convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(trajectory_fidelity, attitude_fidelity),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/MAGSIX",),
        provenance="CADAC MAGSIX source-grounded Python trajectory reconstruction",
        claim_boundary=(
            "The common runner executes only the independently runnable trajectory phase. The restricted-attitude source phase remains visible "
            "and validate-only, preventing a trajectory result from being mislabeled as pseudo-6DoF."
        ),
    )


####


def _overrides_from_resolved(resolved: Any) -> MagsixPluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("MAGSIX resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    runtime = _group(resolved, "runtime")
    return MagsixPluginOverrides(
        north_m=float(initial["north_m"]),
        east_m=float(initial["east_m"]),
        altitude_m=float(initial["altitude_m"]),
        speed_mps=float(initial["speed_mps"]),
        heading_deg=float(initial["heading_deg"]),
        flight_path_deg=float(initial["flight_path_deg"]),
        spin_rpm=float(initial["spin_rpm"]),
        end_time_dnt=float(runtime["end_time_dnt"]),
        sample_step_dnt=float(runtime["sample_step_dnt"]),
    )


####


def _group(resolved: Mapping[str, Any], group_id: str) -> Mapping[str, Any]:
    group = resolved.get(group_id)
    if not isinstance(group, Mapping):
        raise ValueError(f"MAGSIX resolved configuration is missing group {group_id!r}")
    ####
    return group


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id in {"north_m", "east_m", "altitude_m"}:
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {"heading_deg", "flight_path_deg"}:
        return "deg"
    ####
    if parameter_id == "spin_rpm":
        return "rpm"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: MagsixRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID,
        operation="batch",
        realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID,
        mission_template_id=MAGSIX_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    terminated = run.status == "terminated"
    events = ()
    if request.output.include_events and terminated and run.impact_time_s is not None:
        events = (
            TrajectoryEvent(
                id="ground-impact",
                time_s=run.impact_time_s,
                category="impact",
                kind="ground-impact",
                object_id="rotor1",
                detail="MAGSIX source trajectory crossed the configured ground altitude.",
            ),
        )
    ####
    object_result = TrajectoryObject(
        object_id="rotor1",
        model_id=MAGSIX_MODEL_ID,
        realization_id=MAGSIX_TRAJECTORY_REALIZATION_ID,
        name="CADAC MAGSIX Magnus rotor",
        role="vehicle",
        fidelity=MAGSIX_TRAJECTORY_FIDELITY_ID,
        status="terminated" if terminated else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition="ground_impact" if terminated else "end_time",
        channels=channels,
        samples=samples,
        provenance="CADAC MAGSIX source-compatible trajectory-only runner",
        claim_boundary="Center-of-mass/spin trajectory truth only; restricted attitude is not emitted.",
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-trajectory-only-classification",
            message="MAGSIX trajectory runs as point-mass 3-DoF because the source explicitly decouples trajectory from attitude perturbations.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=MAGSIX_MODEL_ID,
            object_id="rotor1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=MAGSIX_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=MAGSIX_MODEL_ID,
        primary_object_id="rotor1",
        status="terminated" if terminated else "completed",
        objects=(object_result,),
        events=events,
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact MAGSIX trajectory-only source vehicle returned through the common runner without pseudo-6DoF promotion.",
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


def _sample_value(channel_id: str, sample: MagsixSample) -> object:
    values: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "altitude_m": sample.altitude_m,
        "speed_mps": sample.speed_mps,
        "heading_deg": sample.heading_deg,
        "flight_path_deg": sample.flight_path_deg,
        "spin_rpm": sample.spin_rpm,
        "tip_speed_ratio": sample.tip_speed_ratio,
        "source_time_dnt": sample.source_time_dnt,
        "dynamic_time_scale_s": sample.dynamic_time_scale_s,
        "density_kg_m3": sample.density_kg_m3,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "mach": sample.mach,
    }
    return values[channel_id]


####


__all__ = [
    "MAGSIX_ATTITUDE_FIDELITY_ID",
    "MAGSIX_ATTITUDE_PHASE_ID",
    "MAGSIX_ATTITUDE_REALIZATION_ID",
    "MAGSIX_EXECUTOR_ID",
    "MAGSIX_MISSION_ID",
    "MAGSIX_MODEL_ID",
    "MAGSIX_MODEL_VERSION",
    "MAGSIX_TRAJECTORY_FIDELITY_ID",
    "MAGSIX_TRAJECTORY_PHASE_ID",
    "MAGSIX_TRAJECTORY_REALIZATION_ID",
    "CadacMagsixMissionCompositionProvider",
    "build_default_magsix_configuration",
    "register_magsix_mission_composition",
]
