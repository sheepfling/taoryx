"""Taoryx Mission Composition bridge for the FALCON6 physical-plant plug-in."""

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
    TrajectoryObject,
    TrajectorySample,
    resolve_output_selection,
)

from .aim5_mission_composition import CADAC_PROVIDER_ID
from .configuration_defaults import materialize_configuration_defaults
from .control_metadata import CadacControlOutputEvidence, batch_configuration_control_advertisement
from .falcon6 import Falcon6PlantRunResult, Falcon6PlantSample
from .falcon6_plugin import Falcon6PluginOverrides, Falcon6VehiclePlugin
from .output_metadata import cadac_output_quantity

FALCON6_MODEL_ID = "cadac.falcon6.aircraft"
FALCON6_MODEL_VERSION = "0.5.0"
FALCON6_FIDELITY_ID = "rigid_body_6dof_surface_allocated"
FALCON6_REALIZATION_ID = "cadac-source-physical-surfaces"
FALCON6_MISSION_ID = "direct_surface_physical_plant"
FALCON6_EXECUTOR_ID = "cadac.falcon6.physical_surface.batch"
_LOCAL_NED_FRAME_ID = "cadac.local_ned"
_BODY_FRAME_ID = "cadac.body_xyz"


class CadacFalcon6MissionCompositionProvider:
    """Self-describing batch provider for one installed FALCON6 physical plant."""

    def __init__(self, plugin: Falcon6VehiclePlugin) -> None:
        blockers = plugin.validate_installation()
        if blockers:
            raise ValueError("cannot publish FALCON6 provider with an incomplete installation: " + "; ".join(blockers))
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
        if model_id != FALCON6_MODEL_ID:
            raise KeyError(f"unknown FALCON6 model {model_id!r}")
        ####
        return self._schema

    ####

    def get_model_output_schema(self, model_id: str) -> TrajectoryOutputSchema:
        if model_id != FALCON6_MODEL_ID:
            raise KeyError(f"unknown FALCON6 model {model_id!r}")
        ####
        return self._output_schema

    ####

    def validate_configuration(self, configuration: TrajectoryConfigurationInstance) -> PreparedTrajectoryConfiguration:
        if configuration.fidelity != FALCON6_FIDELITY_ID:
            raise ValueError(f"FALCON6 physical plant requires fidelity {FALCON6_FIDELITY_ID!r}")
        ####
        if configuration.realization_id not in {None, FALCON6_REALIZATION_ID}:
            raise ValueError(f"FALCON6 supports only realization {FALCON6_REALIZATION_ID!r}")
        ####
        if configuration.mission_template_id not in {None, FALCON6_MISSION_ID}:
            raise ValueError(f"FALCON6 supports only mission template {FALCON6_MISSION_ID!r}")
        ####
        configuration = materialize_configuration_defaults(self._schema, configuration)
        return validate_configuration_instance(self._schema, configuration)

    ####

    def execute_batch(self, request: MissionCompositionRunRequest) -> MissionCompositionTrajectoryResult:
        if request.provider_id != CADAC_PROVIDER_ID or request.model_id != FALCON6_MODEL_ID:
            raise ValueError("FALCON6 executor received a request for another provider/model")
        ####
        prepared = self.validate_configuration(request.prepared_configuration.configuration)
        if prepared.fingerprint != request.prepared_configuration.fingerprint:
            raise ValueError("FALCON6 prepared configuration does not match provider validation")
        ####
        run = self._plugin.run_batch(_overrides_from_resolved(prepared.resolved))
        return _project_run_result(request, run, self._output_schema)

    ####


####


def build_default_falcon6_configuration(
    provider: CadacFalcon6MissionCompositionProvider,
    *,
    configuration_id: str = "falcon6-default",
    overrides: Mapping[str, object] | None = None,
) -> TrajectoryConfigurationInstance:
    """Create one direct-surface FALCON6 configuration from source defaults plus overrides."""

    supplied = dict(overrides or {})
    routes = {
        "position_ned_m": ("initialization", "position_ned_m"),
        "speed_mps": ("initialization", "speed_mps"),
        "yaw_deg": ("initialization", "yaw_deg"),
        "pitch_deg": ("initialization", "pitch_deg"),
        "roll_deg": ("initialization", "roll_deg"),
        "alpha_deg": ("initialization", "alpha_deg"),
        "beta_deg": ("initialization", "beta_deg"),
        "body_rates_deg_s": ("initialization", "body_rates_deg_s"),
        "aileron_command_deg": ("physical_controls", "aileron_command_deg"),
        "elevator_command_deg": ("physical_controls", "elevator_command_deg"),
        "rudder_command_deg": ("physical_controls", "rudder_command_deg"),
        "end_time_s": ("runtime", "end_time_s"),
        "sample_step_s": ("runtime", "sample_step_s"),
    }
    unknown = sorted(set(supplied) - set(routes))
    if unknown:
        raise ValueError(f"unknown FALCON6 configuration overrides: {unknown!r}")
    ####
    grouped: dict[str, dict[str, object]] = {}
    for source_name, value in supplied.items():
        group_id, parameter_id = routes[source_name]
        grouped.setdefault(group_id, {})[parameter_id] = value
    ####
    root = ConfigurationGroupValue(
        values={
            group_id: ConfigurationGroupValue(
                values={
                    parameter_id: ConfigurationParameterValue(value=value, unit=_configuration_unit(parameter_id)) for parameter_id, value in values.items()
                }
            )
            for group_id, values in grouped.items()
        }
    )
    schema = provider.get_model_schema(FALCON6_MODEL_ID)
    return TrajectoryConfigurationInstance(
        configuration_id=configuration_id,
        model_id=FALCON6_MODEL_ID,
        model_version=FALCON6_MODEL_VERSION,
        schema_fingerprint=schema.fingerprint,
        fidelity=FALCON6_FIDELITY_ID,
        realization_id=FALCON6_REALIZATION_ID,
        mission_template_id=FALCON6_MISSION_ID,
        root=root,
    )


####


def register_falcon6_mission_composition(
    provider: CadacFalcon6MissionCompositionProvider,
    registry: MissionCompositionRunnerRegistry,
) -> None:
    """Register the exact provider/model FALCON6 batch executor."""

    registry.register(CADAC_PROVIDER_ID, FALCON6_MODEL_ID, provider.execute_batch)


####


def _build_configuration_schema(plugin: Falcon6VehiclePlugin) -> TrajectoryConfigurationSchema:
    source = plugin.source_definition()
    initial = source.initial_state
    actuator_limit = source.actuator.position_limit_deg
    return TrajectoryConfigurationSchema(
        model_id=FALCON6_MODEL_ID,
        model_version=FALCON6_MODEL_VERSION,
        supported_fidelities=(FALCON6_FIDELITY_ID,),
        root=ConfigurationGroupSchema(
            id="falcon6",
            label="FALCON6 physical plant",
            description="Direct physical-surface plant configuration over the installed source case.",
            children=(
                ConfigurationGroupSchema(
                    id="initialization",
                    label="Initial truth",
                    children=(
                        _parameter(
                            "position_ned_m",
                            "Position",
                            "Initial local-NED position.",
                            default=initial.position_ned_m,
                            value_type="vector3",
                            unit="m",
                            frame=_LOCAL_NED_FRAME_ID,
                        ),
                        _parameter("speed_mps", "Speed", "Initial vehicle speed.", default=initial.speed_mps, unit="m/s"),
                        _parameter("yaw_deg", "Yaw", "Initial body yaw angle.", default=initial.yaw_deg, unit="deg"),
                        _parameter("pitch_deg", "Pitch", "Initial body pitch angle.", default=initial.pitch_deg, unit="deg"),
                        _parameter("roll_deg", "Roll", "Initial body roll angle.", default=initial.roll_deg, unit="deg"),
                        _parameter("alpha_deg", "Angle of attack", "Initial angle of attack.", default=initial.alpha_deg, unit="deg"),
                        _parameter("beta_deg", "Sideslip", "Initial sideslip angle.", default=initial.beta_deg, unit="deg"),
                        _parameter(
                            "body_rates_deg_s",
                            "Body rates",
                            "Initial roll/pitch/yaw body rates.",
                            default=initial.body_rates_deg_s,
                            value_type="vector3",
                            unit="deg/s",
                            frame=_BODY_FRAME_ID,
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="physical_controls",
                    label="Physical surface commands",
                    description="Requested positions entering the source actuator module.",
                    children=(
                        _parameter(
                            "aileron_command_deg",
                            "Aileron",
                            f"Requested aileron position; source actuator position limit is ±{actuator_limit:g} deg.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "elevator_command_deg",
                            "Elevator",
                            f"Requested elevator position; source actuator position limit is ±{actuator_limit:g} deg.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                        _parameter(
                            "rudder_command_deg",
                            "Rudder",
                            f"Requested rudder position; source actuator position limit is ±{actuator_limit:g} deg.",
                            default=0.0,
                            unit="deg",
                            role="segment",
                        ),
                    ),
                ),
                ConfigurationGroupSchema(
                    id="runtime",
                    label="Runtime",
                    children=(
                        _parameter(
                            "end_time_s",
                            "End time",
                            "Requested direct-plant propagation horizon.",
                            default=min(source.end_time_s, 30.0),
                            unit="s",
                            role="constraint",
                        ),
                        _parameter(
                            "sample_step_s",
                            "Sample cadence",
                            "Returned trajectory cadence.",
                            default=max(source.integration_step_s, 0.02),
                            unit="s",
                            role="constraint",
                        ),
                    ),
                ),
            ),
        ),
        claim_boundary="Configuration exposes source initial truth plus direct physical-surface commands; CADAC guidance/autopilot command generation is not part of this realization.",
    )


####


def _parameter(
    parameter_id: str,
    label: str,
    description: str,
    *,
    default: object,
    value_type: str = "number",
    unit: str | None = None,
    frame: str | None = None,
    role: str = "initialization",
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
        role=role,
        compatible_fidelities=(FALCON6_FIDELITY_ID,),
        frame=frame,
        provenance="missiondesignsolutions/CADAC/FALCON6/input.asc",
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
        compatible_fidelities=(FALCON6_FIDELITY_ID,),
        compatible_realizations=(FALCON6_REALIZATION_ID,),
        compatible_mission_templates=(FALCON6_MISSION_ID,),
        operations=("batch",),
        provenance="CADAC FALCON6 source-grounded physical plant",
        claim_boundary="Available from the Python physical-plant reconstruction; compiled-CADAC parity remains pending.",
    )


####


def _build_output_schema() -> TrajectoryOutputSchema:
    core = (
        _output("position_ned_m", "Position", "Local-NED position.", unit="m", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output("velocity_ned_mps", "Velocity", "Local-NED velocity.", unit="m/s", shape=(3,), frame=_LOCAL_NED_FRAME_ID),
        _output("quaternion_wxyz", "Attitude quaternion", "Scalar-first body-from-local quaternion.", shape=(4,)),
        _output("body_rates_rad_s", "Body rates", "Roll/pitch/yaw angular velocity in body axes.", unit="rad/s", shape=(3,), frame=_BODY_FRAME_ID),
    )
    telemetry = (
        _output("alpha_deg", "Angle of attack", "Source aerodynamic angle of attack.", unit="deg"),
        _output("beta_deg", "Sideslip", "Source aerodynamic sideslip angle.", unit="deg"),
        _output("mach", "Mach", "Source atmosphere-relative Mach number."),
        _output("dynamic_pressure_pa", "Dynamic pressure", "Source dynamic pressure.", unit="Pa"),
        _output("altitude_m", "Altitude", "Flat-Earth altitude.", unit="m"),
        _output("requested_surfaces_deg", "Requested surfaces", "Requested aileron/elevator/rudder positions.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID),
        _output(
            "achieved_surfaces_deg", "Achieved surfaces", "Actuator-achieved aileron/elevator/rudder positions.", unit="deg", shape=(3,), frame=_BODY_FRAME_ID
        ),
        _output(
            "aero_surfaces_deg",
            "Aerodynamic surfaces",
            "Surface positions consumed by aerodynamics at this source epoch.",
            unit="deg",
            shape=(3,),
            frame=_BODY_FRAME_ID,
        ),
        _output("throttle", "Throttle", "Resolved source throttle command."),
        _output("thrust_n", "Thrust", "Source turbojet thrust.", unit="N"),
        _output("force_body_n", "Body force", "Non-gravitational force in body axes.", unit="N", shape=(3,), frame=_BODY_FRAME_ID),
        _output("moment_body_nm", "Body moment", "Aerodynamic moment in body axes.", unit="N*m", shape=(3,), frame=_BODY_FRAME_ID),
    )
    return TrajectoryOutputSchema(
        model_id=FALCON6_MODEL_ID,
        model_version=FALCON6_MODEL_VERSION,
        core_channels=core,
        telemetry_channels=telemetry,
        telemetry_groups=(
            TrajectoryTelemetryGroupMetadata(
                id="physical_plant",
                label="Physical plant",
                description="Aerodynamic, propulsion, and physical-effector diagnostics.",
                channel_ids=tuple(item.id for item in telemetry),
            ),
        ),
        entity_output=TrajectoryEntityOutputMetadata(),
        claim_boundary="Rigid-body truth and physical-effector telemetry from the direct-surface FALCON6 plant realization.",
    )


####


def _build_model_metadata(
    schema: TrajectoryConfigurationSchema,
    output_schema: TrajectoryOutputSchema,
) -> TrajectoryModelMetadata:
    fidelity = TrajectoryFidelityMetadata(
        id=FALCON6_FIDELITY_ID,
        label="CADAC FALCON6 physical-surface 6-DoF",
        rank=3,
        declared=True,
        dynamics_fidelity="rigid_body_6dof",
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        compatibility_aliases=("rigid_body_6dof_effector_allocated",),
        runtime_fidelity="rigid_body_6dof",
        control_realization="surface_allocated",
        promotion_status="development",
        operations=("validate", "batch"),
        profile_id="cadac.falcon6.physical-surface-plant",
        blockers=("compiled CADAC numerical golden parity is not yet registered",),
        required_operations=("rigid-body propagation", "physical actuator realization", "aerodynamic force/moment closure"),
        claim_boundary="Full rigid-body plant and physical surface dynamics participate; CADAC autopilot/guidance command generation is outside this direct-surface realization.",
    )
    realization = TrajectoryRealizationMetadata(
        id=FALCON6_REALIZATION_ID,
        label="CADAC physical surface plant",
        description="Source-grounded FALCON6 rigid-body plant commanded directly at the aileron/elevator/rudder actuator boundary.",
        status="available",
        dynamics_fidelities=("rigid_body_6dof",),
        input_realization="actuator_allocated",
        actuator_types=("aerodynamic_surfaces",),
        controls=batch_configuration_control_advertisement(
            schema,
            channels={
                "actuator.aileron.deflection": ("physical_controls", "aileron_command_deg"),
                "actuator.elevator.deflection": ("physical_controls", "elevator_command_deg"),
                "actuator.rudder.deflection": ("physical_controls", "rudder_command_deg"),
            },
            output_evidence={
                "actuator.aileron.deflection": CadacControlOutputEvidence("requested_surfaces_deg", ("achieved_surfaces_deg",), 0, (0,)),
                "actuator.elevator.deflection": CadacControlOutputEvidence("requested_surfaces_deg", ("achieved_surfaces_deg",), 1, (1,)),
                "actuator.rudder.deflection": CadacControlOutputEvidence("requested_surfaces_deg", ("achieved_surfaces_deg",), 2, (2,)),
            },
            authority_id="physical_surface_commands",
            authority="effector",
            authority_description="Caller-owned aileron, elevator, and rudder positions fixed for one FALCON6 batch run.",
            intent_id="direct_surface_control",
            intent_label="Direct Surface Control",
            intent_description="Command the FALCON6 physical aileron, elevator, and rudder boundary for one batch.",
            mission_ids=(FALCON6_MISSION_ID,),
            source_refs=("missiondesignsolutions/CADAC/FALCON6",),
            claim_boundary="The caller supplies fixed direct surface positions through the batch configuration. No common-runner step/session control is registered.",
        ),
        fidelity_aliases=(FALCON6_FIDELITY_ID, "rigid_body_6dof_effector_allocated"),
        mission_template_ids=(FALCON6_MISSION_ID,),
        operations=("validate", "batch"),
        native_factory_ids=(FALCON6_EXECUTOR_ID,),
        source_refs=("missiondesignsolutions/CADAC/FALCON6",),
        blockers=("source waypoint guidance/autopilot is not included in this realization",),
        claim_boundary="Available as a direct physical-surface plant, not as a source-autopilot mission replay.",
    )
    mission = TrajectoryMissionTemplateMetadata(
        id=FALCON6_MISSION_ID,
        name="FALCON6 direct physical-surface propagation",
        description="Propagate the source rigid-body aircraft with constant direct aileron/elevator/rudder commands.",
        status="development",
        initialization_variants=("source_case_override",),
        segment_sequence=("direct_surface",),
        compatible_fidelities=(FALCON6_FIDELITY_ID,),
        operations=(
            TrajectoryMissionOperationMetadata(
                fidelity=FALCON6_FIDELITY_ID,
                realization_id=FALCON6_REALIZATION_ID,
                operation="validate",
                status="available",
                execution_mode="source_grounded_validation",
                common_runner_status="not_available",
                claim_boundary="Validation prepares direct physical-surface plant inputs.",
            ),
            TrajectoryMissionOperationMetadata(
                fidelity=FALCON6_FIDELITY_ID,
                realization_id=FALCON6_REALIZATION_ID,
                operation="batch",
                status="available",
                execution_mode="source_grounded_physical_plant",
                availability_scope="native_runtime_binding",
                common_runner_status="registered",
                executor_id=FALCON6_EXECUTOR_ID,
                claim_boundary="Batch dispatch reaches the exact installed FALCON6 physical-plant plug-in.",
            ),
        ),
        provenance="CADAC FALCON6 rigid-body, actuator, aerodynamics, propulsion, Euler, and Newton modules",
        claim_boundary="Direct-surface mission only; source waypoint guidance and autopilot remain a later realization.",
    )
    return TrajectoryModelMetadata(
        id=FALCON6_MODEL_ID,
        name="CADAC FALCON6 aircraft",
        version=FALCON6_MODEL_VERSION,
        description="CADAC FALCON6 rigid-body aircraft exposed as a physical-surface Taoryx vehicle plug-in.",
        presentation=TrajectoryModelPresentationMetadata(
            display_name="CADAC FALCON6",
            short_name="FALCON6",
            summary="Source-grounded rigid-body aircraft with physical aileron, elevator, and rudder actuator dynamics.",
            category="aircraft",
            subcategory="fighter",
            sort_key="cadac-falcon6",
            badges=("CADAC", "6-DoF", "physical-surfaces", "development"),
            default_fidelity_id=FALCON6_FIDELITY_ID,
            default_mission_template_id=FALCON6_MISSION_ID,
            default_output_channel_ids=("position_ned_m", "velocity_ned_mps", "quaternion_wxyz", "body_rates_rad_s"),
            properties=(
                TrajectoryModelPropertyMetadata(
                    id="source_model",
                    label="Source model",
                    description="Upstream CADAC actor identifier.",
                    semantic_role="identity",
                    value_type="string",
                    value_kind="declared",
                    value="PLANE6",
                    value_declared=True,
                    source_refs=("missiondesignsolutions/CADAC/FALCON6",),
                    provenance="CADAC FALCON6 source package",
                    claim_boundary="Identity metadata only.",
                ),
            ),
        ),
        family_id=FALCON6_MODEL_ID,
        physical_family="fixed_wing_fighter",
        model_kind="vehicle_plugin",
        status="development",
        tags=("cadac", "aircraft", "rigid_body_6dof", "physical_surfaces"),
        operations=("discover", "validate", "batch"),
        common_runner_operations=("batch",),
        capabilities=TrajectoryModelCapabilities(
            initialization_modes=("source_case_override",),
            segment_types=("direct_surface",),
            termination_modes=("end_time", "nonfinite_state"),
            operations=("discover", "validate", "batch"),
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
                id=_LOCAL_NED_FRAME_ID,
                name="CADAC flat-Earth local NED",
                description="Local North-East-Down frame used by FALCON6 Flat6.",
                frame_kind="local_tangent",
                axes=("north", "east", "down"),
                handedness="right",
                origin="source-case local reference point E",
                orientation="north-east-down",
                source_refs=("missiondesignsolutions/CADAC/FALCON6/newton.cpp",),
                provenance="CADAC Flat6 convention",
            ),
            TrajectoryReferenceFrameMetadata(
                id=_BODY_FRAME_ID,
                name="CADAC aircraft body axes",
                description="Aircraft body-fixed x/y/z axes used by forces, moments, rates, and effectors.",
                frame_kind="body",
                axes=("x", "y", "z"),
                handedness="right",
                origin="aircraft reference point",
                orientation="source FALCON6 body convention",
                source_refs=("missiondesignsolutions/CADAC/FALCON6",),
                provenance="CADAC FALCON6 body convention",
            ),
        ),
        output_schema=output_schema,
        output_schema_id=output_schema.schema_id,
        output_schema_fingerprint=output_schema.fingerprint,
        configuration_schema_id=schema.schema_id,
        configuration_schema_fingerprint=schema.fingerprint,
        fidelities=(fidelity,),
        fidelity_transitions=(),
        source_refs=("missiondesignsolutions/CADAC/FALCON6",),
        provenance="CADAC FALCON6 source-grounded Python physical-plant reconstruction",
        claim_boundary="Executable physical plant at development status; source autopilot and compiled numerical parity are separate promotion gates.",
    )


####


def _overrides_from_resolved(resolved: Any) -> Falcon6PluginOverrides:
    if not isinstance(resolved, Mapping):
        raise ValueError("FALCON6 resolved configuration root must be a mapping")
    ####
    initial = _group(resolved, "initialization")
    controls = _group(resolved, "physical_controls")
    runtime = _group(resolved, "runtime")
    return Falcon6PluginOverrides(
        position_ned_m=_vector3(initial["position_ned_m"]),
        speed_mps=float(initial["speed_mps"]),
        yaw_deg=float(initial["yaw_deg"]),
        pitch_deg=float(initial["pitch_deg"]),
        roll_deg=float(initial["roll_deg"]),
        alpha_deg=float(initial["alpha_deg"]),
        beta_deg=float(initial["beta_deg"]),
        body_rates_deg_s=_vector3(initial["body_rates_deg_s"]),
        aileron_command_deg=float(controls["aileron_command_deg"]),
        elevator_command_deg=float(controls["elevator_command_deg"]),
        rudder_command_deg=float(controls["rudder_command_deg"]),
        end_time_s=float(runtime["end_time_s"]),
        sample_step_s=float(runtime["sample_step_s"]),
    )


####


def _group(root: Mapping[str, object], name: str) -> Mapping[str, object]:
    value = root.get(name)
    if not isinstance(value, Mapping):
        raise ValueError(f"FALCON6 resolved configuration requires group {name!r}")
    ####
    return value


####


def _vector3(value: object) -> tuple[float, float, float]:
    if not isinstance(value, (tuple, list)) or len(value) != 3:
        raise ValueError("FALCON6 vector configuration requires exactly three components")
    ####
    return (float(value[0]), float(value[1]), float(value[2]))


####


def _configuration_unit(parameter_id: str) -> str | None:
    if parameter_id == "position_ned_m":
        return "m"
    ####
    if parameter_id == "speed_mps":
        return "m/s"
    ####
    if parameter_id in {"yaw_deg", "pitch_deg", "roll_deg", "alpha_deg", "beta_deg", "aileron_command_deg", "elevator_command_deg", "rudder_command_deg"}:
        return "deg"
    ####
    if parameter_id == "body_rates_deg_s":
        return "deg/s"
    ####
    if parameter_id in {"end_time_s", "sample_step_s"}:
        return "s"
    ####
    return None


####


def _project_run_result(
    request: MissionCompositionRunRequest,
    run: Falcon6PlantRunResult,
    output_schema: TrajectoryOutputSchema,
) -> MissionCompositionTrajectoryResult:
    selected = resolve_output_selection(
        output_schema,
        request.output,
        fidelity=FALCON6_FIDELITY_ID,
        operation="batch",
        realization_id=FALCON6_REALIZATION_ID,
        mission_template_id=FALCON6_MISSION_ID,
    )
    channels = tuple(_runtime_channel(item, output_schema) for item in selected)
    samples = tuple(
        TrajectorySample(
            time_s=sample.time_s,
            values={channel.id: _sample_value(channel.id, sample) for channel in selected},
        )
        for sample in run.samples
    )
    failed = run.terminated_reason != "end_time"
    object_result = TrajectoryObject(
        object_id="aircraft1",
        model_id=FALCON6_MODEL_ID,
        realization_id=FALCON6_REALIZATION_ID,
        name="CADAC FALCON6 aircraft",
        role="vehicle",
        fidelity=FALCON6_FIDELITY_ID,
        status="failed" if failed else "completed",
        active_from_s=samples[0].time_s,
        active_to_s=samples[-1].time_s,
        terminal_disposition=run.terminated_reason,
        channels=channels,
        samples=samples,
        provenance="CADAC FALCON6 source-grounded direct physical-surface plant",
        claim_boundary="Physical plant truth participates; source guidance/autopilot is not inferred from direct surface commands.",
    )
    diagnostics = (
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-parity-pending",
            message="FALCON6 executes through the source-grounded Python physical plant; compiled-CADAC numerical parity is not yet promoted.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=FALCON6_MODEL_ID,
            object_id="aircraft1",
        ),
        MissionCompositionDiagnostic(
            severity="info",
            code="cadac-direct-surface-realization",
            message="This realization injects commands at the physical-surface actuator boundary and does not execute CADAC waypoint guidance/autopilot modules.",
            phase="execution",
            recoverability="degraded",
            provider_id=CADAC_PROVIDER_ID,
            model_id=FALCON6_MODEL_ID,
            object_id="aircraft1",
        ),
    )
    return MissionCompositionTrajectoryResult(
        provider_id=CADAC_PROVIDER_ID,
        provider_version=FALCON6_MODEL_VERSION,
        request_id=request.request_id,
        configuration_fingerprint=request.prepared_configuration.fingerprint,
        primary_model_id=FALCON6_MODEL_ID,
        primary_object_id="aircraft1",
        status="failed" if failed else "completed",
        objects=(object_result,),
        events=(),
        relationships=(),
        diagnostics=diagnostics,
        claim_boundary="One exact FALCON6 physical-surface plant object returned through the common runner; no source-autopilot claim is implied.",
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


def _sample_value(channel_id: str, sample: Falcon6PlantSample) -> object:
    values: dict[str, object] = {
        "position_ned_m": sample.position_ned_m,
        "velocity_ned_mps": sample.velocity_ned_mps,
        "quaternion_wxyz": sample.quaternion_wxyz,
        "body_rates_rad_s": sample.body_rates_rad_s,
        "alpha_deg": sample.alpha_deg,
        "beta_deg": sample.beta_deg,
        "mach": sample.mach,
        "dynamic_pressure_pa": sample.dynamic_pressure_pa,
        "altitude_m": sample.altitude_m,
        "requested_surfaces_deg": sample.requested_surfaces_deg,
        "achieved_surfaces_deg": sample.achieved_surfaces_deg,
        "aero_surfaces_deg": sample.aero_surfaces_deg,
        "throttle": sample.throttle,
        "thrust_n": sample.thrust_n,
        "force_body_n": sample.force_body_n,
        "moment_body_nm": sample.moment_body_nm,
    }
    return values[channel_id]


####


__all__ = [
    "FALCON6_EXECUTOR_ID",
    "FALCON6_FIDELITY_ID",
    "FALCON6_MISSION_ID",
    "FALCON6_MODEL_ID",
    "FALCON6_MODEL_VERSION",
    "FALCON6_REALIZATION_ID",
    "CadacFalcon6MissionCompositionProvider",
    "build_default_falcon6_configuration",
    "register_falcon6_mission_composition",
]
